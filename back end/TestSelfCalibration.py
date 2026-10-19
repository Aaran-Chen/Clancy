"""Synthetic end-to-end tests for SelfCalibration.py. No hardware needed.

A ground-truth camera is placed in the physical (model) frame. The real pose
generator and the real forward kinematics drive a marker taped to the gripper
through the calibration poses. Corners are projected with the ground truth,
corrupted like the real thing (pixel noise, joint-angle noise, marker facing
away, dropped frames), and handed to the solver in the display frame the web
app sends. The solved model must then map table pixels back to the right
display-frame coordinates.

Run:  python TestSelfCalibration.py
"""

from __future__ import annotations

import base64
import copy
import math
import os
import sys

import cv2
import numpy as np

import arm_geometry as Geo
import bridge_server as Bridge
import RecordedCaptures as RC
import SelfCalibration as SC

# Same workspace reference as the app and calibration pose planner.
DisplayOffset = np.array(list(Bridge.controller.WorkspaceOffset().values()))
Width, Height = 1280, 720
TableZDisplay = SC.DefaultTableZCm
TableZModel = TableZDisplay - DisplayOffset[2]

# Diagnostic frames from these synthetic runs must not land in the real
# captures folder, where they would be mistaken for camera evidence.
import tempfile
from pathlib import Path
TestRoot = Path(__file__).resolve().parent / "TestArtifacts"
TestRoot.mkdir(exist_ok=True)
SC.CapturesDir = TestRoot / "SyntheticCaptures"
SC.SessionPath = TestRoot / "SyntheticSession.json"

Controller = Bridge.controller
Geometry = Controller.arm_geometry
Failures: list[str] = []
CachedPoses = None


def GetTestPoses():
    global CachedPoses
    if CachedPoses is None:
        # These synthetic cameras face the arm from the front (MakeScene).
        CachedPoses = Controller.selfcal_poses("Facing")["poses"]
    return CachedPoses


def Recorded(*Paths, What: str) -> bool:
    """True when a replay's recorded inputs are on disk. Else the replay FAILS, RECORDING MISSING (RecordedCaptures: it
    used to print [SKIP] and pass), or with SO101_MISSING_RECORDINGS=skip it is a skip that says so."""
    Absent = RC.Missing(*Paths)
    if not Absent:
        return True
    Text = RC.MissingText(What, Absent)
    if os.environ.get("SO101_MISSING_RECORDINGS") == "skip":
        print(f"  [SKIP] {Text}")
    else:
        Check(f"recorded {What}", False, Text)
    return False


def Check(Name: str, Condition: bool, Detail: str) -> None:
    print(f"  [{'PASS' if Condition else 'FAIL'}] {Name}: {Detail}")
    if not Condition:
        Failures.append(Name)


def ModelToDisplay(Points: np.ndarray, Mirrored: bool) -> np.ndarray:
    Out = Points.copy()
    if Mirrored:
        Out[..., 0] = -Out[..., 0]
    return Out + DisplayOffset


def LookAt(Eye: np.ndarray, Target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """World-to-camera rotation and translation (OpenCV: +z forward, +y down)."""
    Forward = Target - Eye
    Forward /= np.linalg.norm(Forward)
    Right = np.cross(Forward, [0.0, 0.0, 1.0])
    Right /= np.linalg.norm(Right)
    Down = np.cross(Forward, Right)
    Rotation = np.vstack([Right, Down, Forward])
    return Rotation, -Rotation @ Eye


def MakeScene(Seed: int, Mirrored: bool, Distortion: tuple[float, float]):
    Rng = np.random.default_rng(Seed)
    Eye = np.array([Rng.uniform(-18, 18), Rng.uniform(50, 62), Rng.uniform(45, 60)])
    Rotation, Translation = LookAt(Eye, np.array([Rng.uniform(-4, 4), 21.0, TableZModel]))
    Truth = {
        "f": Rng.uniform(820, 1050), "cx": Rng.uniform(610, 670), "cy": Rng.uniform(340, 380),
        "dist": np.array([Distortion[0], Distortion[1], 0.0, 0.0]),
        "R": Rotation, "t": Translation, "eye": Eye, "mirrored": Mirrored,
    }
    # Marker on top of the gripper, 4 cm back from the tip, 35 mm square, rotated 25 deg.
    Half, Angle = 1.75, math.radians(25)
    # Physical ArUco corner order seen from the printed face (normal +z_g):
    # TL, TR, BR, BL = (-,+), (+,+), (+,-), (-,-). The reverse order describes a
    # mirror-image marker, which the chiral ArUco code cannot be read from.
    InPlane = np.array([[-Half, Half], [Half, Half], [Half, -Half], [-Half, -Half]])
    Spin = np.array([[math.cos(Angle), -math.sin(Angle)], [math.sin(Angle), math.cos(Angle)]])
    Corners2D = InPlane @ Spin.T
    Truth["q"] = np.column_stack([Corners2D[:, 0] - 4.0, Corners2D[:, 1], np.full(4, 2.5)])
    return Truth, Rng


def ProjectTruth(Truth, PointsModel: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    CameraMatrix = np.array([[Truth["f"], 0, Truth["cx"]], [0, Truth["f"], Truth["cy"]], [0, 0, 1]])
    Rvec, _ = cv2.Rodrigues(Truth["R"])
    Pixels, _ = cv2.projectPoints(PointsModel.reshape(-1, 1, 3), Rvec, Truth["t"], CameraMatrix, Truth["dist"])
    Depth = (PointsModel.reshape(-1, 3) @ Truth["R"].T + Truth["t"])[:, 2]
    return Pixels.reshape(-1, 2), Depth


def PoseObservations(Truth, Rng, PixelNoise: float, AngleNoiseDeg: float):
    """Drive the real pose list through FK; return (observations, detection count)."""
    Poses = GetTestPoses()
    Limits = Controller._ik_joint_limits()
    Records = []
    for Index, Pose in enumerate(Poses):
        # Use exactly the branch/angles that the bridge sends for this plan.
        Angles = Controller.CalibrationPlans[Pose["plan_id"]]["angles"]
        TrueJoints = np.array([[P["x"], P["y"], P["z"]] for P in Geo.forward_joint_positions(Geometry, Angles)])
        Tip, Frame = SC.GripperFrame(TrueJoints)
        CornersModel = Tip[None, :] + (Frame @ Truth["q"].T).T  # (4, 3)
        Pixels, Depth = ProjectTruth(Truth, CornersModel)
        Normal = Frame @ np.array([0.0, 0.0, 1.0])
        ToCamera = Truth["eye"] - CornersModel.mean(axis=0)
        Facing = float(Normal @ ToCamera / np.linalg.norm(ToCamera))
        InImage = (Depth > 0).all() and (Pixels[:, 0] > 2).all() and (Pixels[:, 0] < Width - 3).all() \
            and (Pixels[:, 1] > 2).all() and (Pixels[:, 1] < Height - 3).all()
        if not InImage or Facing < math.cos(math.radians(72)) or Rng.random() < 0.15:
            continue
        # What the bridge would report: FK of slightly wrong joint readings.
        Noisy = {K: V + Rng.normal(0, AngleNoiseDeg) for K, V in Angles.items()}
        ReportedJoints = np.array([[P["x"], P["y"], P["z"]] for P in Geo.forward_joint_positions(Geometry, Noisy)])
        DisplayJoints = ModelToDisplay(ReportedJoints, Truth["mirrored"])
        Records.append((Index, Pixels + Rng.normal(0, PixelNoise, Pixels.shape), DisplayJoints))
    return Records, len(Poses)


def InjectNumeric(Records):
    SC.ResetSession()
    with SC._Lock:
        SC._Session["image_size"] = [Width, Height]
        SC._Session["mirrored"] = True
        for Index, Pixels, DisplayJoints in Records:
            Tip, Frame = SC.GripperFrame(DisplayJoints)
            SC._Session["observations"].append({
                "pose_index": Index, "corners": Pixels.tolist(),
                "tip": Tip.tolist(), "frame": Frame.tolist(), "frames_detected": 3,
            })


def TablePlaneError(Truth, Model) -> tuple[float, float]:
    """Project true table points; query PixelToPlane like /locate would; compare."""
    Xs, Ys = np.meshgrid(np.linspace(-15, 15, 11), np.linspace(12, 32, 9))
    TrueModel = np.column_stack([Xs.ravel(), Ys.ravel(), np.full(Xs.size, TableZModel)])
    Pixels, Depth = ProjectTruth(Truth, TrueModel)
    Errors = []
    for P, (U, V), D in zip(TrueModel, Pixels, Depth):
        if D <= 0 or not (0 <= U <= Width - 1 and 0 <= V <= Height - 1):
            continue
        # Native OpenCV pixel -> mirrored continuous capture space (what the browser sends).
        UQuery, VQuery = Width - (U + 0.5), V + 0.5
        Mapped = SC.PixelToPlane(Model, UQuery, VQuery)
        if not Mapped["ok"]:
            Errors.append(99.0)
            continue
        Expected = ModelToDisplay(P, Truth["mirrored"])
        Errors.append(math.hypot(Mapped["x"] - Expected[0], Mapped["y"] - Expected[1]))
    Errors = np.array(Errors)
    return float(np.sqrt(np.mean(Errors ** 2))), float(Errors.max())


def RunNumericCase(Name, Seed, Mirrored, Distortion, PixelNoise, AngleNoiseDeg, MinDetections=10, CameraTolCm=3.0, FocalTol=0.05):
    print(f"\n== {Name}")
    Truth, Rng = MakeScene(Seed, Mirrored, Distortion)
    Records, Total = PoseObservations(Truth, Rng, PixelNoise, AngleNoiseDeg)
    print(f"  marker seen at {len(Records)} of {Total} poses; camera at {np.round(Truth['eye'], 1)} (model frame)")
    if len(Records) < MinDetections:
        Check("enough detections", False, f"{len(Records)}")
        return
    InjectNumeric(Records)
    Report = SC.Solve(TableZ=TableZDisplay, Save=False)
    Model = Report["model"]
    print(f"  fit {Report['fit_rms_px']} px, hold-out {Report['holdout_rms_px']} px, predicted table "
          f"{Report['plane_rms_cm']} cm, f {Report['focal_px']} (true {Truth['f']:.1f}), "
          f"marker {Report['marker_edge_mm']} mm (true 35), mirror {Report['mirror_x']}")
    Check("accepted without warnings", Report["ok"] and Report["quality_ok"], "; ".join(Report["problems"]) or "no problems")
    Check("handedness", Report["mirror_x"] == Mirrored, f"solved {Report['mirror_x']}, true {Mirrored}")
    Check("focal length", abs(Report["focal_px"] - Truth["f"]) / Truth["f"] < FocalTol,
          f"{Report['focal_px']} vs {Truth['f']:.1f} (limit {FocalTol:.0%})")
    TrueEyeDisplay = ModelToDisplay(Truth["eye"], Mirrored)
    EyeError = float(np.linalg.norm(np.array(Report["camera_position_cm"]) - TrueEyeDisplay))
    # Under heavy noise, focal length and camera distance trade off along a flat
    # valley; the table mapping (checked below) is what the system relies on.
    Check("camera position", EyeError < CameraTolCm, f"{EyeError:.2f} cm off (limit {CameraTolCm} cm)")
    Check("marker size", abs(Report["marker_edge_mm"] - 35.0) < 4.0, f"{Report['marker_edge_mm']} mm")
    Rms, Worst = TablePlaneError(Truth, Model)
    Check("table mapping", Rms < 0.6 and Worst < 1.0, f"RMS {Rms:.3f} cm, worst {Worst:.3f} cm")


def RenderedPipelineCase():
    """Render the real marker into frames, mirror them like the browser, detect, solve."""
    print("\n== Rendered frames through AddObservation (detection + mirror handling)")
    Truth, Rng = MakeScene(7, True, (0.0, 0.0))
    Dictionary = cv2.aruco.getPredefinedDictionary(SC.ArucoDictionaryId)
    Side, Pad = 240, 40
    Marker = cv2.aruco.generateImageMarker(Dictionary, SC.MarkerId, Side)
    Tile = np.full((Side + 2 * Pad, Side + 2 * Pad), 255, np.uint8)
    Tile[Pad:Pad + Side, Pad:Pad + Side] = Marker
    # Detector corner k is marker-image corner k: TL, TR, BR, BL of the black square.
    Source = np.float32([[Pad, Pad], [Pad + Side, Pad], [Pad + Side, Pad + Side], [Pad, Pad + Side]]) - 0.5
    Records, _ = PoseObservations(Truth, Rng, 0.0, 0.05)
    SC.ResetSession()
    Detected, CornerErrors = 0, []
    for Index, Pixels, DisplayJoints in Records:
        Background = np.full((Height, Width, 3), (96, 120, 140), np.uint8)
        H = cv2.getPerspectiveTransform(Source, Pixels.astype(np.float32))
        Warped = cv2.warpPerspective(cv2.cvtColor(Tile, cv2.COLOR_GRAY2BGR), H, (Width, Height),
                                     flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT, dst=Background)
        Frames = []
        for _ in range(3):
            Noisy = np.clip(cv2.GaussianBlur(Warped, (3, 3), 0.6).astype(np.int16) + Rng.normal(0, 3, Warped.shape), 0, 255).astype(np.uint8)
            Mirrored = cv2.flip(Noisy, 1)
            Ok, Jpeg = cv2.imencode(".jpg", Mirrored, [cv2.IMWRITE_JPEG_QUALITY, 90])
            Frames.append("data:image/jpeg;base64," + base64.b64encode(Jpeg.tobytes()).decode())
        Joints = [{"x": float(P[0]), "y": float(P[1]), "z": float(P[2])} for P in DisplayJoints]
        Result = SC.AddObservation(Frames, Joints, True, Index)
        if Result.get("detected"):
            Detected += 1
            Stored = np.array(SC._Session["observations"][-1]["corners"])
            CornerErrors.append(float(np.max(np.linalg.norm(Stored - Pixels, axis=1))))
    print(f"  detected in {Detected} of {len(Records)} rendered poses")
    Check("rendered detection rate", Detected >= 0.8 * len(Records), f"{Detected}/{len(Records)}")
    if CornerErrors:
        Check("corner accuracy after mirror round-trip", float(np.median(CornerErrors)) < 1.0,
              f"median worst-corner error {np.median(CornerErrors):.2f} px")
    Report = SC.Solve(TableZ=TableZDisplay, Save=False)
    if not Report.get("model"):
        Check("rendered solve", False, Report.get("error", "no model"))
        return
    Rms, Worst = TablePlaneError(Truth, Report["model"])
    print(f"  fit {Report['fit_rms_px']} px, hold-out {Report['holdout_rms_px']} px")
    Check("rendered solve accepted without warnings", Report["ok"] and Report["quality_ok"], "; ".join(Report["problems"]) or "no problems")
    Check("rendered table mapping", Rms < 0.6 and Worst < 1.5, f"RMS {Rms:.3f} cm, worst {Worst:.3f} cm")


def RedTapeDistractorCase():
    """Red tape among decoys: the tracker locks onto the servo wiring halfway through.

    Every pose also offers a stationary red object and a wire blob that moves
    with the elbow. Solving must pick the tape by camera consistency.
    """
    print("\n== Red tape with a stationary red object and wiring the tracker locks onto")
    Truth, Rng = MakeScene(8, True, (-0.06, 0.01))
    Poses = GetTestPoses()
    SC.ResetSession()
    TrueTape, Tracked, Decoys = 0, 0, 0
    with SC._Lock:
        SC._Session["image_size"] = [Width, Height]
        SC._Session["mirrored"] = True
        for Index, Pose in enumerate(Poses):
            Angles = Controller.CalibrationPlans[Pose["plan_id"]]["angles"]
            Joints = np.array([[P["x"], P["y"], P["z"]] for P in Geo.forward_joint_positions(Geometry, Angles)])
            Tip, Frame = SC.GripperFrame(Joints)
            Tape = Tip + Frame @ np.array([-3.0, 0.5, 2.0])
            Wire = Joints[2] + np.array([1.5, -1.0, 2.5])
            Pixels, Depth = ProjectTruth(Truth, np.array([Tape, Wire]))
            if (Depth <= 0).any() or not (5 < Pixels[0, 0] < Width - 5 and 5 < Pixels[0, 1] < Height - 5):
                continue
            Noisy = {K: V + Rng.normal(0, 0.3) for K, V in Angles.items()}
            Display = ModelToDisplay(np.array([[P["x"], P["y"], P["z"]] for P in Geo.forward_joint_positions(Geometry, Noisy)]),
                                     Truth["mirrored"])
            DisplayTip, DisplayFrame = SC.GripperFrame(Display)
            TapePixel = Pixels[0] + Rng.normal(0, 1.5, 2)
            Candidates = [{"centroid": TapePixel.tolist(), "area": 900.0},
                          {"centroid": [1010.0, 140.0], "area": 3000.0},
                          {"centroid": (Pixels[1] + Rng.normal(0, 1.5, 2)).tolist(), "area": 450.0}]
            Candidates += [{"centroid": Rng.uniform([0, 0], [Width, Height]).tolist(), "area": 120.0} for _ in range(2)]
            SC._Session.setdefault("red_poses", []).append({"pose_index": Index, "tip": DisplayTip.tolist(),
                                                            "frame": DisplayFrame.tolist(), "candidates": Candidates})
            TrueTape += 1
            # The live tracker follows the tape for the first poses, then the wiring.
            Chosen = TapePixel if Index < len(Poses) * 0.4 else Candidates[2]["centroid"]
            Decoys += Index >= len(Poses) * 0.4
            SC._Session["observations"].append({"pose_index": Index, "corners": [list(Chosen)], "tip": DisplayTip.tolist(),
                                                "frame": DisplayFrame.tolist(), "method": "red"})
            Tracked += 1
    print(f"  tape in view at {TrueTape} poses; the tracker followed the wiring at {Decoys}")
    with SC._Lock:
        Oracle = copy.deepcopy(SC._Session)
    Report = SC.Solve(TableZ=TableZDisplay, Save=False)
    Assignment = Report.get("tape_assignment") or {}
    print(f"  assigned {Assignment.get('assigned')} poses, {Assignment.get('changed_from_tracker')} changed from the tracker; "
          f"fit {Report.get('fit_rms_cm')} cm ({Report.get('fit_rms_px')} px)")
    Check("distractor solve accepted without warnings", Report["ok"] and Report.get("quality_ok"), "; ".join(Report.get("problems", [])) or Report.get("error", "no problems"))
    Check("tape chosen over the decoys", (Assignment.get("assigned") or 0) >= 0.8 * TrueTape,
          f"{Assignment.get('assigned')} of {TrueTape}")
    if Report.get("model"):
        Rms, Worst = TablePlaneError(Truth, Report["model"])
        # The same poses solved with the true tape pixel only: the accuracy a
        # single noisy point per pose allows, which the decoys must not spoil.
        with SC._Lock:
            SC._Session.clear()
            SC._Session.update(Oracle)
            SC._Session["observations"] = [{**O, "corners": [next(C["centroid"] for C in P["candidates"])]}
                                           for O, P in zip(Oracle["observations"], Oracle["red_poses"])]
            SC._Session.pop("red_poses")
        Ideal = SC.Solve(TableZ=TableZDisplay, Save=False)
        IdealRms, _ = TablePlaneError(Truth, Ideal["model"])
        Check("distractor table mapping", Rms < 1.0 and Worst < 1.5 and Rms <= 1.1 * IdealRms + 0.05,
              f"RMS {Rms:.3f} cm, worst {Worst:.3f} cm; with only the true tape {IdealRms:.3f} cm")


def RejectionCases():
    """Nothing is rejected on its own (Aaran, 2026-10-10): a bad fit is kept with its problems as warnings for the
    user to judge, and only a run that computes no camera at all keeps the current calibration."""
    print("\n== Bad fits are kept with warnings; nothing computed keeps the current calibration")
    SC.ResetSession()
    Report = SC.Solve(TableZ=TableZDisplay, Save=False)
    Check("no observations: not computed", not Report["ok"] and Report["computed"] is False
          and Report["error"].startswith("Could not compute a calibration: only 0 valid tip poses"), Report["error"][:80])
    # Garbage: pixels unrelated to the poses.
    Truth, Rng = MakeScene(3, True, (0.0, 0.0))
    Records, _ = PoseObservations(Truth, Rng, 0.3, 0.1)
    Scrambled = [(I, Rng.uniform([0, 0], [Width, Height], (4, 2)), J) for I, _, J in Records]
    InjectNumeric(Scrambled)
    Report = SC.Solve(TableZ=TableZDisplay, Save=False)
    Check("random pixels: kept with warnings, or nothing computed",
          (Report["ok"] and not Report["quality_ok"] and any("fit error" in Problem for Problem in Report["problems"])
           and Report["model"]["quality_problems"] == Report["problems"]) or (not Report["ok"] and Report["computed"] is False),
          "; ".join(Report.get("problems") or [Report.get("error", "")])[:120])
    # Camera bumped halfway through.
    Truth2, _ = MakeScene(4, True, (0.0, 0.0))
    Moved = dict(Truth2)
    Moved["R"], Moved["t"] = LookAt(Truth2["eye"] + np.array([6.0, -4.0, 3.0]), np.array([0.0, 21.0, TableZModel]))
    RecordsA, _ = PoseObservations(Truth2, np.random.default_rng(11), 0.3, 0.1)
    RecordsB, _ = PoseObservations(Moved, np.random.default_rng(12), 0.3, 0.1)
    Half = len(RecordsA) // 2
    InjectNumeric(RecordsA[:Half] + RecordsB[Half:])
    Report = SC.Solve(TableZ=TableZDisplay, Save=False)
    Check("camera moved mid-run: kept with its fit errors as warnings", Report["ok"] and not Report["quality_ok"] and bool(Report["problems"]),
          "; ".join(Report.get("problems") or [Report.get("error", "")])[:120])


def FitMotionCases():
    """The desk normal, from depth at every pose, must not turn during the fit (CameraDrift.MotionDuringFit)."""
    print("\n== Camera motion during the fit")
    import CameraDrift as CD
    Desk = np.array([0.0023, -0.4564, -0.8897])  # the 15:04 fit's desk normal, camera frame
    Desk /= np.linalg.norm(Desk)

    def Planes(CreepDeg, Seed, NoiseDeg=0.06):
        """24 poses x 3 frames of fit-time desk planes: steady creep about the camera's x axis plus
        per-frame noise per axis (0.06 deg; the 15:04 fit measured 0.05 including its creep), and
        one frame 0.22 deg off, the worst seen then."""
        Rng = np.random.default_rng(Seed)
        Rows = []
        for Pose in range(24):
            for Frame in range(3):
                Tilt = np.radians([CreepDeg * Pose / 23 + Rng.normal(0, NoiseDeg), Rng.normal(0, NoiseDeg), 0.0])
                if (Pose, Frame) == (5, 1):
                    Tilt[1] += np.radians(0.22)
                Rows.append({"pose_index": Pose, "received_at": Pose * 4.0 + Frame * 0.1,
                             "n": (cv2.Rodrigues(Tilt)[0] @ Desk).tolist(), "d": -47.1 + Rng.normal(0, 0.04)})
        return Rows

    Still = [CD.MotionDuringFit(Planes(0.0, Seed)) for Seed in range(300)]
    Check("a still camera is never rejected", not any(Result["moved"] for Result in Still),
          f"300 noisy runs, worst {max(Result['angle_deg'] for Result in Still):.3f} deg (limit {CD.MaxFitMotionDeg})")
    for Creep, Needed in ((0.39, 300), (0.3, 294)):
        Caught = sum(CD.MotionDuringFit(Planes(Creep, Seed))["moved"] for Seed in range(300))
        Check(f"{Creep} deg of creep during the fit is rejected", Caught >= Needed, f"{Caught} of 300 runs rejected")
    # One pose whose plane locked onto another surface (the bin floor, 6 cm lower) is not motion,
    # in the first third or the last; steady creep with such a pose is still caught.
    Outliers = []
    for Seed in range(100):
        for Pose in (2, 21):
            Rows = Planes(0.0, Seed)
            for Row in Rows:
                if Row["pose_index"] == Pose:
                    Row["d"] -= 6.0
            Outliers.append(CD.MotionDuringFit(Rows))
    Check("a pose on another surface does not reject a still camera", not any(Result["moved"] for Result in Outliers)
          and all(Result["outlier_poses"] == 1 for Result in Outliers),
          f"worst {max(abs(Result['offset_cm']) for Result in Outliers):.3f} cm (limit {CD.MaxFitMotionCm})")
    Creeping = Planes(0.39, 0)
    for Row in Creeping:
        if Row["pose_index"] == 2:
            Row["d"] -= 6.0
    Motion = CD.MotionDuringFit(Creeping)
    Check("creep with a pose on another surface is still rejected", Motion["moved"], f"{Motion['angle_deg']:.3f} deg")
    Run = RC.Day("2026-09-28") / "selfcal" / "runs" / "20260928_150208_0a49ed111d2b"
    if not Recorded(Run / "observations", What="the 2026-09-28 15:04 fit's raw depth"):
        return
    import json as Json
    Real = []
    for Capture in sorted((Run / "observations").glob("*_pose*.json")):
        Record = Json.loads(Capture.read_text(encoding="utf-8"))
        for Name in Record["depth_frames_png"] or []:
            Plane = CD.DeskPlane(cv2.imread(str(Capture.parent / Name), cv2.IMREAD_UNCHANGED), Record["depth_intrinsics"])
            if Plane is not None:
                Real.append(dict(Plane, pose_index=Record["pose_index"], received_at=Record["received_at"]))
    Motion = CD.MotionDuringFit(Real)
    Check("the real 15:04 fit (camera turned about 0.3 deg) is rejected", Motion["moved"] and len(Real) == 72,
          f"{len(Real)} frames, first vs last third {Motion['angle_deg']:.3f} deg / {Motion['offset_cm']:+.3f} cm")


# Recorded runs to replay, read only, where RecordedCaptures finds them (in a git worktree, the main checkout's).
ReplayCaptures = RC.Root()


def DriftBaselineReplayCases():
    """2026-10-01: four tape fits of 0.45-0.51 cm over 21-24 poses were rejected by the drift baseline.

    The camera looked down at a bin under the arm beside a bookshelf, over a floor too far away to
    count; every frame's recorded "desk" was the bookshelf's side (90 deg from the desk), so the
    baseline found 0-5 usable frames and the per-frame planes read 7-15 deg of motion. Replayed
    from each run's own Session.json and fit report, the baseline must now be built on the bin and
    find the camera still, /locate's check must pass on the run's own frames, clear-desk runs must
    keep the very baseline they had, and the real 0.3 deg turn of 2026-09-28 15:04 must still read
    as motion on the desk's own cells.
    """
    print("\n== Drift baseline replayed on recorded runs (read only)")
    import json as Json
    import CameraDrift as CD
    Runs = ReplayCaptures / "2026-10-01" / "selfcal" / "runs"
    Names = ("20261001_192446_eaa8b7580ae7", "20261001_193148_dccb1d13e651", "20261001_193545_641f1d3f0987", "20261001_193803_40734e7ea5a1")
    if not Recorded(*(Runs / Name for Name in Names), What="the 2026-10-01 bin-and-bookshelf calibration runs"):
        return
    Drop = ("drift_reference", "fit_desk_planes", "fit_camera_motion", "fit_desk_planes_redetected")

    def Replay(Run):
        Report = Json.loads(sorted((Run / "records").glob("*FitReport*"))[-1].read_text(encoding="utf-8"))
        Report = Report.get("payload", Report)
        Model = {Key: Value for Key, Value in Report["model"].items() if Key not in Drop}
        Session = Json.loads((Run / "Session.json").read_text(encoding="utf-8"))
        return Report, Model, Session, SC._DriftBaseline(Session, Model, None)

    for Name in Names:
        Report, Model, Session, Problems = Replay(Runs / Name)
        Motion, Reference = Model["fit_camera_motion"], Model.get("drift_reference")
        Was = Report.get("fit_camera_motion") or {}
        Check(f"{Name[9:15]} drift baseline passes", not Problems and Reference is not None and Motion.get("method") == "desk_map",
              f"was {Was.get('angle_deg')} deg of motion; now {Motion['angle_deg']} deg / {Motion['offset_cm']:+.3f} cm, baseline on "
              f"{Reference and Reference['frames']} frames, planes {Model['fit_desk_planes_redetected']}; {Problems or 'no problems'}")
        if Reference is None:
            continue
        Frames = [Plane for Plane in Session["desk_planes"]][::3]
        Passed = sum(CD.CheckDesk(CD.PlaneDepth(Plane), Session["depth_intrinsics"], Model)["ok"] for Plane in Frames)
        Check(f"{Name[9:15]} /locate's desk check passes on the run's own frames", Passed == len(Frames), f"{Passed} of {len(Frames)}")
    for Day, Name in (("2026-09-28", "20260928_224357_b1e21f8c4fba"), ("2026-09-30", "20260930_164640_b334b436f2cb")):
        Run = ReplayCaptures / Day / "selfcal" / "runs" / Name
        if not Recorded(Run, What=f"the {Day} clear-desk calibration run"):
            continue
        Report, Model, Session, Problems = Replay(Run)
        Before = Report["model"]["drift_reference"]
        After = Model.get("drift_reference") or {}
        Check(f"{Name[:15]} (clear desk) keeps its baseline", not Problems and Model["fit_desk_planes_redetected"]["found_again"] == 0
              and After.get("desk_n") == Before["desk_n"] and After.get("desk_d") == Before["desk_d"],
              f"{Model['fit_desk_planes_redetected']}, motion {Model['fit_camera_motion']['angle_deg']} deg")
    Run = ReplayCaptures / "2026-09-28" / "selfcal" / "runs" / "20260928_150208_0a49ed111d2b"
    if Recorded(Run / "observations", What="the 2026-09-28 15:04 fit's raw depth"):
        Planes = []
        for Capture in sorted((Run / "observations").glob("*_pose*.json")):
            Record = Json.loads(Capture.read_text(encoding="utf-8"))
            for Frame, Name in enumerate(Record["depth_frames_png"] or []):
                Plane = CD.DeskPlane(cv2.imread(str(Capture.parent / Name), cv2.IMREAD_UNCHANGED), Record["depth_intrinsics"])
                if Plane is not None:
                    Planes.append(dict(Plane, pose_index=Record["pose_index"], received_at=Record["received_at"],
                                       archive=str(Capture), frame=Frame))
        Normal = CD._Unit(np.mean([Plane["n"] for Plane in Planes], axis=0))
        Motion = CD.MotionFromDeskMap(Planes, Normal, float(np.mean([Plane["d"] for Plane in Planes])), Record["depth_intrinsics"])
        Check("the real 15:04 turn (about 0.3 deg) still reads as motion on the desk's cells", bool(Motion and Motion["moved"]),
              f"{Motion and Motion['angle_deg']} deg / {Motion and Motion['offset_cm']} cm (limit {CD.MaxFitMotionDeg} deg)")


RealRun = RC.Day("2026-09-28") / "selfcal" / "runs" / "20260928_224357_b1e21f8c4fba"


def RealRunRegionCase():
    """The 22:43 depth run re-solved offline stores where its tips were, for /locate's reliability flag."""
    print("\n== The 2026-09-28 22:43 run re-solved: the fit keeps its calibrated tips' region")
    if not Recorded(RealRun / "Session.json", What="the 2026-09-28 22:43 calibration run"):
        return
    import json as Json
    Session = Json.loads((RealRun / "Session.json").read_text(encoding="utf-8"))
    with SC._Lock:
        SC._Session.clear()
        SC._Session.update(copy.deepcopy(Session))
        SC._Session.pop("archive_directory", None)
    Report = SC.Solve(Save=False)
    Model = Report.get("model") or {}
    Tips = np.array([Pose["tip"] for Pose in Session["red_poses"]])
    Check("real run accepted without warnings", Report.get("ok") and Report.get("quality_ok") and Report.get("observations_used") == 24,
          f"{Report.get('observations_used')} poses, fit {Report.get('fit_rms_cm')} cm, depth {Report.get('depth_rms_cm')} cm")
    Bounds = Model.get("observed_tip_bounds_cm")
    Check("tip bounds stored (display frame)", Bounds == SC.TipRegion(Tips)["observed_tip_bounds_cm"], f"{Bounds}")
    Check("tip hull stored", Model.get("observed_tip_hull_cm") == SC.TipRegion(Tips)["observed_tip_hull_cm"]
          and SC.TipRegionDistance(Model, *Tips[:, :2].mean(axis=0)) == 0.0,
          f"{len(Model.get('observed_tip_hull_cm') or [])} hull vertices")


def ArchiveFramesCase():
    """Archived poses say which frame each value is in; an old archive still replays through AddObservation."""
    print("\n== Calibration archives label their frames")
    if not Recorded(RealRun / "observations", What="the 2026-09-28 22:43 calibration run"):
        return
    Captures = sorted((RealRun / "observations").glob("*_pose0.json"))
    import json as Json
    Old = Json.loads(Captures[0].read_text(encoding="utf-8"))
    Check("an old archive has no frame labels", "frames" not in Old, "format_version 2 without labels")
    SC.ResetSession()
    Result = SC.AddObservation(Old["images"], Old["joints"], Old["mirrored"], Old["pose_index"], Method="red", Metadata=Old["metadata"])
    Check("old archive replays", Result.get("ok") and bool(Result.get("capture_archive")), str(Result.get("error") or "archived"))
    New = Json.loads(Path(Result["capture_archive"]).read_text(encoding="utf-8"))
    Session = Json.loads((Path(SC._Session["archive_directory"]) / "Session.json").read_text(encoding="utf-8"))
    Check("frames labelled in the pose archive and the session", New.get("frames") == SC.ArchiveFrames == Session.get("frames"),
          ", ".join(SC.ArchiveFrames))
    Tip = np.array([Old["joints"][-1][Axis] for Axis in "xyz"])
    Visual = Old["metadata"]["requested_pose"]["visual_target"]
    Target = New.get("requested_display_target") or {}
    Reached = float(np.linalg.norm(np.array([Target.get(Axis, np.nan) for Axis in "xyz"]) - Tip))
    Misread = float(np.linalg.norm(np.array([Visual[Axis] for Axis in "xyz"]) - Tip))
    # Pose 0 asked for model x -2.401 = visual 10.9 = display -10.9; the tip was at -11.2.
    Check("the display-frame request lands on the tip", Reached < 1.1 and Misread > 20,
          f"{Reached:.2f} cm from the tip (the raw visual_target read as display: {Misread:.1f} cm)")


if __name__ == "__main__":
    FitMotionCases()
    DriftBaselineReplayCases()
    RunNumericCase("Mirrored display frame (as in the app), mild distortion", 1, True, (-0.08, 0.02), 0.3, 0.1)
    RunNumericCase("Non-mirrored display frame", 2, False, (-0.05, 0.01), 0.3, 0.1)
    # Focal length and distance slide along a flat valley at this noise level; the
    # table mapping is the requirement.
    RunNumericCase("Noisier: 0.8 px corners, 0.3 deg joints", 5, True, (-0.10, 0.03), 0.8, 0.3, CameraTolCm=6.5, FocalTol=0.10)
    RunNumericCase("Strong barrel distortion", 6, True, (-0.25, 0.08), 0.3, 0.1)
    RenderedPipelineCase()
    RedTapeDistractorCase()
    RejectionCases()
    RealRunRegionCase()
    ArchiveFramesCase()
    SC.ResetSession()
    print(f"\n{'ALL PASSED' if not Failures else 'FAILED: ' + ', '.join(Failures)}")
    sys.exit(1 if Failures else 0)
