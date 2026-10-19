"""Is the camera still where it was calibrated, with no calibrated desk in view? Checked against the robot.

Why
---
On 2026-10-06 the cart was rolled from the desk to a bookshelf cabinet, and at 15:01:01Z "grab the pill
bottle" was refused (camera_check_failed): the camera check could only compare the camera with the old desk
(its plane search latched onto the paper in the robot's own basket, 6.16 deg off), and the gripper-tape backup
was switched off by its wrist-roll rule (roll 1420 against the calibration's 1570). The camera was in fact still
where it was calibrated: the red tip tape sat 0.52 cm from where the arm puts it, through every joint. At about
15:02:25Z a hand on the lens turned the head 4.16 deg; from then on the tape reads 2.66-2.88 cm off.

Policy (Aaran, 2026-10-06: assume the camera is at the same place; no desk needed; check the calibration
against the home position of the tip)
------
When the calibrated desk is not in view (CameraDrift.CheckCapture: no desk plane near it, or fewer than
CameraDrift.MinMapCells of its calibrated cells seen as desk), the desk is not required. The camera is assumed
fixed where it was calibrated, and the gripper's red tip tape checks it (TapeCheck) whenever the arm is
connected, torque on and still; tasks start at Home, so the first capture of a task checks it at Home:
- the tape within TapeVerifiedCm of where the arm puts it: verified, the capture passes;
- the tape positively identified farther than that: camera_moved, blocked (recalibrate);
- the tape not judgeable (arm moving, limp or not connected, jaw off the calibration's, the tape hidden or out
  of frame): the capture passes as "assumed", with a one-line warning naming why.
The basket rim (RimCheck: the depth profile of its top edge against the calibration's own frames, and
RobotCameraReference's RGB keyframe) is computed and logged as advisory fields only: it never blocks and is
never required. Nothing here saves anything, and the camera model is an input throughout, so a head motor can
supply each capture's pose later.

Roll-aware tape
---------------
The calibration stores the tape's offset Q in the frame SelfCalibration.GripperFrame builds from the joints
(x from the roll joint to the tip, y horizontal across the arm). That frame turns with wrist roll only through
the tip's offset from the roll axis, so Q holds only at the calibration's roll (CameraDrift.TapeRollToleranceRaw).
The tape is rigid with the gripper link instead: q, its place in the gripper_link frame, is Q taken at the same
pose with the wrist roll at the calibration's (CaptureArmPose's "reference"), and the tape at the capture's roll
is gripper_link * q. At the calibration's roll that is Q exactly. One q reproduces the 24 poses of the 2026-10-02
calibration to 0.003 cm rms; at the 15:01 pose (roll 1420, 13 deg from 1570) the tape moved 0.29 cm, 0.14 cm from
where the pseudo frame puts it.

Which red is the tape
---------------------
As CameraDrift.TapeCheck, but off a desk the only reference for height is the camera model itself, and a camera
turned on the robot moves the tape's apparent height: about 0.45 cm per degree of droop (the 15:01 frame
re-rendered drooped 6 deg reads the tape 4.1 cm off, 2.6 cm of it in height). The desk path's 2 cm height filter
would drop the real tape at droops of about 4.5-7 deg and pass a moved camera as "assumed". So within
CameraDrift.TapeNearCm in 3-D the nearest tape-sized blob is the tape at any height; beyond, out to
CameraDrift.TapeSearchCm, only a lone tape-sized blob within CameraDrift.TapeHeightToleranceCm of the tape's
display height (a pan keeps it). Tape-sized: the calibration's areas when it has them (CameraDrift.TapeAreaRange,
TapeAreaFactor), else DefaultTapeAreaCm2: the tape measures 2.0 cm2 at Home, the red marks on the basket paper
0.08-0.43 cm2 (all 12-16 cm from it), arm wiring 0.05-0.3, the desk's eraser 9. Size cannot tell the basket rim's
red strip (0.46-0.75 cm2) from the tape, which reads 0.15-0.46 cm2 edge-on in the 2026-10-10 clicks, so the strip is
masked where the camera model puts it instead (SelfCalibration.BasketStripExclusion), and the lower bound stays 0.6.
"""

from __future__ import annotations

import json
import math
import time
import warnings
from pathlib import Path
from typing import Any

import cv2
import numpy as np

import CameraDrift
import DepthGeometry

# The tape at most this far (3-D) from where the arm puts it verifies the camera; farther, positively
# identified, blocks as moved. The cart-move limit of the desk path (CameraDrift's module notes): a 1 deg
# droop of the real 21:46 frame read 0.82 cm; the 15:01 frame, camera right, 0.52 cm.
TapeVerifiedCm = CameraDrift.CartTapeMaxErrorCm
# Tape-sized when the calibration recorded no tape areas (manual clicks keep none): cm^2.
DefaultTapeAreaCm2 = (0.6, 6.0)
# Basket rim (advisory). Its top edge per 1 cm of display X across the front, inside a window around the
# calibration's edge: Y +-2.5 cm, Z 4 cm below to 1.5 cm above (the solid band is 2.5 cm tall); widened by
# RimWidenCm before the rim counts as not found. The change against the calibration's profile is fitted as
# dY = shift_y + yaw * X and dZ = shift_z + roll * X; the rim roll is not used (a bag on it read 2.5 deg).
RimBinCm = 1.0
RimXRangeCm = (-14.0, 14.0)
RimWindowYCm = 2.5
RimWindowZCm = (-4.0, 1.5)
RimWidenCm = 6.0
RimTopPercentile = 85
RimMinBinPoints = 3
RimArmClearanceCm = 5.0
RimMinBins = 20
RimMinAgreeing = 15
RimAgreeCm = 0.3
RimMaxYawDeg = 1.25
RimMaxShiftCm = 0.75
# The calibration's profile: its first poses (the arm sweeps over the rim later in the run), each with at
# least this many clear bins (2026-10-02: poses 0, 3 and 4 of the first 8).
RimBaselinePoses = 8
RimBaselineMinBins = 24
Resolution = 4096


def _Steps(A, B) -> int:
    Difference = abs(int(A) - int(B)) % Resolution
    return min(Difference, Resolution - Difference)


def _Clock(FittedAt: Any) -> str:
    return CameraDrift._Clock(FittedAt)


def _Matrix(Value) -> np.ndarray | None:
    try:
        Matrix = np.asarray(Value, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    return Matrix if Matrix.shape == (4, 4) and np.isfinite(Matrix).all() else None


def ReferenceRaw(Model: dict[str, Any] | None) -> dict[str, int] | None:
    """The raw steps CaptureArmPose evaluates each capture's pose at for the tape's place on the gripper:
    the calibration's wrist roll, or None without a verified calibration that recorded it."""
    Reference, _ = CameraDrift.VerifiedReference(Model)
    try:
        return {"wrist_roll": int(Reference["tape_pose"]["wrist_roll_raw"])}
    except (KeyError, TypeError, ValueError):  # no calibration, no tape pose, or no roll in it
        return None


def LinkOffset(Q, Reference: dict[str, Any]) -> np.ndarray | None:
    """q: the tape in the gripper_link frame, from the calibration's offset Q (pseudo gripper frame) at
    Reference, the capture's pose with the calibration's wrist roll ({"joints", "gripper_link"})."""
    import SelfCalibration  # local import: SelfCalibration imports CameraDrift at fit time

    Link = _Matrix(Reference.get("gripper_link"))
    if Link is None or not Reference.get("joints"):
        return None
    try:
        Tip, Frame = SelfCalibration.GripperFrame(np.asarray(Reference["joints"], dtype=np.float64))
    except ValueError:
        return None
    # The display frame mirrors the CAD's X, so the link's 3x3 is a reflection: solve, never transpose.
    try:
        return np.linalg.solve(Link[:3, :3], Tip + Frame @ np.asarray(Q, dtype=np.float64) - Link[:3, 3])
    except np.linalg.LinAlgError:
        return None


def _TapeAreas(Reference: dict[str, Any]) -> tuple[float, float]:
    Areas = Reference.get("tape_area_cm2")
    if Areas:
        return float(Areas[0]) / CameraDrift.TapeAreaFactor, float(Areas[1]) * CameraDrift.TapeAreaFactor
    return DefaultTapeAreaCm2


def TapeCheck(ImageBgr: np.ndarray | None, DepthMm: np.ndarray | None, Intrinsics: dict[str, Any] | None,
              Model: dict[str, Any] | None, Arm: dict[str, Any] | None) -> dict[str, Any]:
    """The gripper's red tip tape, measured in depth, against forward kinematics through every joint.

    ImageBgr is the capture in native (unmirrored) orientation; Arm the bridge's CaptureArmPose with
    "gripper_link" and "reference" (at the calibration's wrist roll, ReferenceRaw). CameraDrift.TapeCheck's
    contract: tape_check "ok" (within TapeVerifiedCm), "moved" (farther), or why it could not judge
    ("unavailable", "pose_differs": the jaw, "not_visible", "ambiguous"); plus tape_model "roll_aware",
    tape_roll_change_deg (the wrist roll against the calibration's) and tape_link_offset_cm (q).
    """
    import SelfCalibration  # local import, as LinkOffset

    Result: dict[str, Any] = {"tape_check": "unavailable", "tape_error_cm": None, "tape_model": "roll_aware"}
    Reference, Why = CameraDrift.VerifiedReference(Model)
    Tape = (Model or {}).get("marker_corners_gripper_cm") or []
    if Reference is None:
        Result["tape_reason"] = Why
        return Result
    if len(Tape) != 1:
        Result["tape_reason"] = "the calibration has no tape offset on the gripper"
        return Result
    if ImageBgr is None or DepthMm is None or Intrinsics is None:
        Result["tape_reason"] = "no colour image or depth frame for this capture"
        return Result
    if not Arm or not Arm.get("ok"):
        Result["tape_reason"] = (Arm or {}).get("reason", "no arm pose for this capture")
        return Result
    Pose = Reference.get("tape_pose") or {}
    Raw = Arm.get("servos_raw") or {}
    # Only the jaw is gated: the tape may sit on the moving jaw. The wrist roll is in the kinematics.
    if Pose.get("gripper_raw") is not None and "gripper" in Raw:
        Steps = _Steps(Raw["gripper"], Pose["gripper_raw"])
        if Steps > CameraDrift.TapeGripperToleranceRaw:
            Result.update(tape_check="pose_differs", tape_reason=f"gripper differs from the calibration's by {Steps} steps")
            return Result
    # The reference pose is this capture's with only the wrist roll at the calibration's.
    Link, Pinned = _Matrix(Arm.get("gripper_link")), Arm.get("reference") or {}
    PinnedRaw, Roll = Pinned.get("servos_raw") or {}, Pose.get("wrist_roll_raw")
    Offset = None
    if (Link is not None and Roll is not None and PinnedRaw.get("wrist_roll") is not None and int(PinnedRaw["wrist_roll"]) == int(Roll)
            and all(PinnedRaw.get(Name) == Value for Name, Value in Raw.items() if Name != "wrist_roll")):
        Offset = LinkOffset(Tape[0], Pinned)
    if Offset is None:
        Result["tape_reason"] = "no gripper frame at the calibration's wrist roll for this capture"
        return Result
    # The link turned against the calibration's roll: both 3x3s carry the display mirror, which cancels here.
    Turn = np.linalg.solve(_Matrix(Pinned["gripper_link"])[:3, :3], Link[:3, :3])
    Result.update(tape_link_offset_cm=[round(float(Value), 3) for Value in Offset],
                  tape_roll_change_deg=round(math.degrees(math.acos(float(np.clip((np.trace(Turn) - 1.0) / 2.0, -1.0, 1.0)))), 2))
    if "wrist_roll" in Raw:
        Result["tape_wrist_roll_raw"] = int(Raw["wrist_roll"])
    PredictedDisplay = Link[:3, :3] @ Offset + Link[:3, 3]
    Predicted = DepthGeometry.DisplayToCamera(PredictedDisplay, Model)[0]
    Height, Width = DepthMm.shape[:2]
    Pixel = DepthGeometry.Project(Predicted, Intrinsics)[0]
    Result["tape_predicted_px"] = [round(float(Value), 1) for Value in Pixel]
    if Predicted[2] <= 5.0 or not (10 <= Pixel[0] < Width - 10 and 10 <= Pixel[1] < Height - 10):
        Result.update(tape_check="not_visible", tape_reason="the tape is predicted outside the image")
        return Result
    Low, High = _TapeAreas(Reference)
    Scored, Others = [], 0
    # The basket rim's red strip is never the tape (SelfCalibration.BasketStripExclusion).
    Detection = SelfCalibration.DetectRedDetailed(ImageBgr, Exclude=SelfCalibration.BasketStripExclusion(Model, Pixel, Intrinsics, (Width, Height)))
    if Detection["excluded"]:
        Result["tape_basket_strip_excluded"] = Detection["excluded"]
    for Candidate in Detection.get("options", []):
        Point = SelfCalibration._BlobCameraPoint(DepthMm, Candidate, Intrinsics)
        Depth = float(Point[2]) if Point is not None else float(Predicted[2])
        Area = float(Candidate["area"]) * (Depth / float(Intrinsics["fx"])) ** 2
        if not Low <= Area <= High:
            Others += 1  # arm wiring, the basket paper's marks, the eraser: not the tape
            continue
        if Point is not None:
            Error = float(np.linalg.norm(Point - Predicted))
            Rise = float(DepthGeometry.CameraToDisplay(Point, Model)[0][2] - PredictedDisplay[2])
        else:  # no depth on the blob: lateral error only
            Error = float(np.linalg.norm(np.asarray(Candidate["centroid"]) - Pixel)) * float(Predicted[2]) / float(Intrinsics["fx"])
            Rise = None
        Scored.append((Error, Rise, Candidate))
    Scored.sort(key=lambda Item: Item[0])
    Result["tape_other_blobs"] = Others
    Near = [Item for Item in Scored if Item[0] <= CameraDrift.TapeNearCm]
    # Beyond the near search only a blob measured in depth at the tape's height can be the tape seen elsewhere.
    Far = [Item for Item in Scored if Item[1] is not None and abs(Item[1]) <= CameraDrift.TapeHeightToleranceCm
           and Item[0] <= CameraDrift.TapeSearchCm]
    if Near:
        Error, _, Best = Near[0]
    elif len(Far) == 1:
        Error, _, Best = Far[0]  # one tape, seen where the arm is not: the camera turned or slid
    else:
        Result.update(tape_check="ambiguous" if Far else "not_visible",
                      tape_reason=("several tape-sized red blobs, none where the arm puts the tape" if Far else
                                   "no tape-sized red blob near where the arm puts the tape"),
                      tape_nearest_cm=round(Scored[0][0], 2) if Scored else None)
        return Result
    Result.update(tape_error_cm=round(Error, 2), tape_measured_px=[round(float(Value), 1) for Value in Best["centroid"]])
    # Measured - predicted in the display frame, as CameraDrift.TapeCheck (GraspPlan.ts TapeCorrection).
    Measured = SelfCalibration._BlobCameraPoint(DepthMm, Best, Intrinsics)
    if Measured is not None:
        Residual = DepthGeometry.CameraToDisplay(Measured, Model)[0] - PredictedDisplay
        Result["tape_residual_cm"] = [round(float(Value), 2) for Value in Residual]
    if len(Near) > 1 and Near[1][0] - Error < CameraDrift.TapeRivalMarginCm:
        Result.update(tape_check="ambiguous", tape_reason="more than one red blob near the predicted tape")
        return Result
    Result["tape_check"] = "moved" if Error > TapeVerifiedCm else "ok"
    return Result


def TapeAdvice(Tape: dict[str, Any]) -> str:
    """Why the tape could not judge the camera, as a clause for the "assumed" warning."""
    State, Reason = Tape.get("tape_check"), str(Tape.get("tape_reason") or "")
    if State == "pose_differs":
        return f"the jaw is not where it was at calibration ({Reason})"
    if State == "not_visible":
        return ("the red tape is outside the camera's view at this arm pose" if "outside the image" in Reason
                else "the red tape on the gripper is not seen where the arm puts it: covered, turned away or out of view")
    if State == "ambiguous":
        return "more than one red thing near the gripper looks like the tape"
    if Reason == "arm not connected":
        return "the arm is not connected"
    if Reason == "torque off":
        return "the arm's torque is off"
    if "mov" in Reason or "joint record" in Reason:
        return "the arm was moving around the capture"
    return Reason or "the tape could not be looked for"


# --------------------------------------------------------------------------
# Basket rim (advisory)
# --------------------------------------------------------------------------

def _RimBins() -> np.ndarray:
    return np.arange(RimXRangeCm[0], RimXRangeCm[1], RimBinCm)


def RimProfile(DepthMm: np.ndarray, Intrinsics: dict[str, Any], Model: dict[str, Any], Joints=None,
               Edge: tuple[float, float] | None = None, WidenCm: float = 0.0) -> np.ndarray:
    """The rim's top edge in the display frame: per RimBinCm of X, the mean (Y, Z) of the highest points
    (above the RimTopPercentile) inside the window around Edge (the calibration's edge Y, Z; the band
    Y 19.5-24.5, Z -3.5..2 without one), the arm's links and RimArmClearanceCm around them left out. NaN rows
    where a bin has too few points."""
    Bins = _RimBins()
    Profile = np.full((len(Bins), 2), np.nan)
    Points = CameraDrift.CameraPoints(DepthMm, Intrinsics, 2)
    Points = Points[(Points[:, 2] > 5.0) & (Points[:, 2] < 120.0)]
    if not len(Points):
        return Profile
    Display = DepthGeometry.CameraToDisplay(Points, Model)
    EdgeY, EdgeZ = Edge if Edge is not None else (22.0, 0.5)
    Keep = ((np.abs(Display[:, 1] - EdgeY) <= RimWindowYCm + WidenCm) & (Display[:, 2] >= EdgeZ + RimWindowZCm[0] - WidenCm)
            & (Display[:, 2] <= EdgeZ + RimWindowZCm[1] + WidenCm) & (Display[:, 0] >= Bins[0]) & (Display[:, 0] < Bins[-1] + RimBinCm))
    Display = Display[Keep]
    if Joints is not None and len(Display):
        Display = Display[~DepthGeometry.ArmExclusion(Joints, RimArmClearanceCm)(Display)]
    for Index, Start in enumerate(Bins):
        Column = Display[(Display[:, 0] >= Start) & (Display[:, 0] < Start + RimBinCm)]
        if len(Column) < RimMinBinPoints:
            continue
        Top = Column[Column[:, 2] >= np.percentile(Column[:, 2], RimTopPercentile)]
        Profile[Index] = [float(np.mean(Top[:, 1])), float(np.mean(Top[:, 2]))]
    return Profile


def RimChange(Profile: np.ndarray, Baseline: np.ndarray) -> dict[str, Any] | None:
    """The rim against the calibration's: {"yaw_deg", "shift_y_cm", "shift_z_cm", "roll_deg", "bins",
    "agreeing"}, or None with fewer than RimMinBins bins seen in both. Each of dY and dZ along X is fitted
    by least median of squares through bin pairs, then by least squares on the bins within RimAgreeCm."""
    X = _RimBins() + RimBinCm / 2.0
    Change = np.asarray(Profile, dtype=np.float64) - np.asarray(Baseline, dtype=np.float64)
    Seen = np.isfinite(Change).all(axis=1)
    if int(Seen.sum()) < RimMinBins:
        return None
    Fits = {}
    Pairs = [(First, Second) for First in range(int(Seen.sum())) for Second in range(First + 1, int(Seen.sum()))]
    for Column, Name in ((0, "y"), (1, "z")):
        Along, Values = X[Seen], Change[Seen, Column]
        Best = None
        for First, Second in Pairs:
            if abs(Along[Second] - Along[First]) < 3.0:
                continue
            Slope = (Values[Second] - Values[First]) / (Along[Second] - Along[First])
            Intercept = Values[First] - Slope * Along[First]
            Median = float(np.median(np.abs(Values - Intercept - Slope * Along)))
            if Best is None or Median < Best[0]:
                Best = (Median, Slope, Intercept)
        _, Slope, Intercept = Best
        Agree = np.abs(Values - Intercept - Slope * Along) <= RimAgreeCm
        if int(Agree.sum()) >= 2:
            Intercept, Slope = np.linalg.lstsq(np.column_stack([np.ones(int(Agree.sum())), Along[Agree]]), Values[Agree], rcond=None)[0]
        Fits[Name] = (float(Intercept), math.degrees(math.atan(float(Slope))), int(Agree.sum()))
    return {"yaw_deg": round(Fits["y"][1], 3), "shift_y_cm": round(Fits["y"][0], 3), "shift_z_cm": round(Fits["z"][0], 3),
            "roll_deg": round(Fits["z"][1], 3), "bins": int(Seen.sum()), "agreeing": min(Fits["y"][2], Fits["z"][2])}


_RimBaselines: dict[tuple, dict[str, Any] | None] = {}


def _RimKey(Model: dict[str, Any], Reference: dict[str, Any]) -> tuple:
    # The calibration's run and camera pose; a desk re-measured after a cart move changes neither.
    return (Model.get("fitted_at"), str(Reference.get("keyframe")), tuple(Model.get("rvec") or ()), tuple(Model.get("tvec") or ()),
            bool(Model.get("mirror_x")))


def RimBaseline(Model: dict[str, Any] | None) -> dict[str, Any] | None:
    """The rim profile of the calibration's own frames, built from its run archive (the run of the drift
    reference's keyframe) and kept per calibration: {"profile", "edge", "poses", "source"}, or None when the
    archive is not on disk or too little of the rim was clear in it."""
    Reference, _ = CameraDrift.VerifiedReference(Model)
    if not (Reference or {}).get("keyframe"):
        return None
    Key = _RimKey(Model, Reference)
    if Key not in _RimBaselines:
        _RimBaselines[Key] = _BuildRimBaseline(Path(Reference["keyframe"]).parent, Model)
    return _RimBaselines[Key]


def Status(Model: dict[str, Any] | None) -> dict[str, Any]:
    """What a capture with no calibrated desk in view is checked against (GET /calibration/status):
    robot_reference "present" when the gripper tape can check the camera (a verified calibration with a
    tape offset and the wrist roll it was fitted at), else "none"; and the advisory rim's profile once a
    capture has built it from the calibration's run."""
    Reference, Why = CameraDrift.VerifiedReference(Model)
    Tape = (Model or {}).get("marker_corners_gripper_cm") or []
    Roll = ReferenceRaw(Model)
    Present = Reference is not None and len(Tape) == 1 and Roll is not None
    Rim = _RimBaselines.get(_RimKey(Model, Reference), "not built yet") if Reference is not None and Reference.get("keyframe") else None
    return {"robot_reference": "present" if Present else "none",
            "robot_reference_detail": {
                "tape": ({"offset_cm": [round(float(Value), 3) for Value in Tape[0]], "tape_pose": Reference.get("tape_pose"),
                          "tape_area_cm2": Reference.get("tape_area_cm2"), "limit_cm": TapeVerifiedCm} if Present
                         else Why or "the calibration has no tape offset, or no wrist roll recorded with it"),
                "rim": Rim if isinstance(Rim, str) or Rim is None else {"poses": Rim["poses"], "edge_cm": [round(Value, 2) for Value in Rim["edge"]],
                                                                        "advisory": True}}}


def _NanMedian(Rows: list[np.ndarray]) -> np.ndarray:
    """The median profile over frames, bin by bin (NaN where no frame saw a bin)."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN bins stay NaN
        return np.nanmedian(np.stack(Rows), axis=0)


def _BuildRimBaseline(Folder: Path, Model: dict[str, Any]) -> dict[str, Any] | None:
    Intrinsics = {Key: Model[Key] for Key in ("fx", "fy", "cx", "cy", "dist")} | dict(zip(("width", "height"), Model["image_size"]))
    Records = []
    try:
        for Path_ in Folder.glob("*_pose*.json"):
            Records.append((int(Path_.stem.rsplit("_pose", 1)[1]), Path_))
    except (OSError, ValueError):
        return None
    Profiles, Used = [], []
    for PoseIndex, Path_ in sorted(Records)[:RimBaselinePoses]:
        try:
            Record = json.loads(Path_.read_text(encoding="utf-8"))
            Joints = np.asarray([[P["x"], P["y"], P["z"]] for P in Record["joints"]], dtype=np.float64)
            Frames = [cv2.imread(str(Path_.parent / Name), cv2.IMREAD_UNCHANGED) for Name in Record.get("depth_frames_png") or []]
        except (OSError, KeyError, TypeError, ValueError):
            continue
        Rows = [RimProfile(Depth, Record.get("depth_intrinsics") or Intrinsics, Model, Joints)
                for Depth in Frames if Depth is not None and Depth.dtype == np.uint16]
        if not Rows:
            continue
        Pose = _NanMedian(Rows)
        if int(np.isfinite(Pose).all(axis=1).sum()) >= RimBaselineMinBins:
            Profiles.append(Pose)
            Used.append(PoseIndex)
    if not Profiles:
        return None
    Profile = _NanMedian(Profiles)
    Seen = np.isfinite(Profile).all(axis=1)
    return {"profile": Profile, "edge": (float(np.median(Profile[Seen, 0])), float(np.median(Profile[Seen, 1]))),
            "poses": Used, "source": str(Folder)}


def RimCheck(ImageBgr, DepthMm, Intrinsics, Model, Arm) -> dict[str, Any]:
    """The basket rim against the calibration (advisory: logged, never deciding). rim_check "ok" (within
    RimMaxYawDeg and RimMaxShiftCm), "moved", "not_judgeable" (too little of the rim clear), or "unavailable"
    (no depth, or no calibration frames to compare with); rim_keyframe: RobotCameraReference's RGB/depth
    keyframe match (None when it found too few features)."""
    import RobotCameraReference  # the fixed-rim keyframe registration (HEAD 0f58fdc), advisory here

    Started = time.perf_counter()
    Result: dict[str, Any] = {"rim_check": "unavailable"}
    Joints = np.asarray(Arm["joints"], dtype=np.float64) if Arm and Arm.get("ok") and Arm.get("joints") else None
    try:
        Baseline = RimBaseline(Model) if DepthMm is not None and Intrinsics else None
        if Baseline is None:
            Result["rim_reason"] = "no depth frame" if DepthMm is None else "no calibration frames of the rim to compare with"
        else:
            Change = None
            for Widen in (0.0, RimWidenCm):
                Change = RimChange(RimProfile(DepthMm, Intrinsics, Model, Joints, Baseline["edge"], Widen), Baseline["profile"])
                if Change is not None:
                    break
            if Change is None or Change["agreeing"] < RimMinAgreeing:
                Result.update(rim_check="not_judgeable", rim_bins=Change["bins"] if Change else None,
                              rim_reason=f"only {Change['bins'] if Change else 'under ' + str(RimMinBins)} of {len(_RimBins())} cm of the rim are clear"
                              + (f", {Change['agreeing']} agreeing" if Change else ""))
            else:
                Moved = abs(Change["yaw_deg"]) > RimMaxYawDeg or max(abs(Change["shift_y_cm"]), abs(Change["shift_z_cm"])) > RimMaxShiftCm
                Result.update(rim_check="moved" if Moved else "ok", rim_yaw_deg=round(Change["yaw_deg"], 2),
                              rim_shift_cm=[round(Change["shift_y_cm"], 2), round(Change["shift_z_cm"], 2)],
                              rim_bins=Change["bins"], rim_agreeing_bins=Change["agreeing"])
        Keyframe = RobotCameraReference.Check(ImageBgr, DepthMm, Intrinsics, Model, Arm)
        Result["rim_keyframe"] = None if Keyframe is None else {
            "ok": Keyframe["ok"], "rotation_deg": Keyframe.get("robot_rotation_deg"), "translation_cm": Keyframe.get("robot_translation_cm"),
            "rotation_bound_deg": Keyframe.get("robot_rotation_bound_deg"), "translation_bound_cm": Keyframe.get("robot_translation_bound_cm"),
            "inliers": Keyframe.get("robot_inliers")}
    except Exception as Exc:  # advisory: never let it stop a capture
        Result.update(rim_check="unavailable", rim_reason=f"{type(Exc).__name__}: {Exc}")
    Result["rim_ms"] = round((time.perf_counter() - Started) * 1000, 1)
    return Result


# --------------------------------------------------------------------------
# The verdict
# --------------------------------------------------------------------------

def Check(ImageBgr, DepthMm, Intrinsics, Model, Arm, Seq=None, DeskCheck: dict[str, Any] | None = None) -> dict[str, Any]:
    """The camera_check of a capture with no calibrated desk in view (CameraDrift.CheckCapture): the camera is
    assumed where it was calibrated and the gripper tape checks it (the module notes). robot_check "verified"
    or "assumed" pass (an assumed one carries "warning", one sentence); "moved" blocks as camera_moved, and
    "failed" (the check itself raised) as camera_check_failed. DeskCheck, the desk path's own result, rides
    along under "desk_check"; the rim's reading under rim_*."""
    Started = time.perf_counter()
    Reference, _ = CameraDrift.VerifiedReference(Model)
    FittedAt = (Reference or {}).get("fitted_at") or (Model or {}).get("fitted_at")
    Clock = _Clock(FittedAt)
    Result: dict[str, Any] = {"ok": True, "angle_deg": None, "offset_cm": None, "reason": "", "reference_fitted_at": FittedAt,
                              "seq": Seq, "alignment_source": "robot"}
    try:
        Tape = TapeCheck(ImageBgr, DepthMm, Intrinsics, Model, Arm)
    except Exception as Exc:  # a fault in the check itself is no verdict on the camera: fail closed
        Tape = {"tape_check": "failed", "tape_error_cm": None, "tape_reason": f"{type(Exc).__name__}: {Exc}"}
    if Tape["tape_check"] == "failed":
        Result.update(ok=False, robot_check="failed", error_code="camera_check_failed",
                      reason=f"no calibrated desk in view, and the gripper tape check failed ({Tape['tape_reason']})",
                      error="The camera could not be checked against the gripper tip: the check itself failed (the reason is "
                            "in the bridge's camera check). Send the task again.")
    elif Tape["tape_check"] == "ok":
        Result.update(robot_check="verified",
                      reason=f"no calibrated desk in view; the gripper tape is {Tape['tape_error_cm']:.2f} cm from where the arm puts it "
                             f"(limit {TapeVerifiedCm} cm, through every joint): the camera is where it was calibrated")
    elif Tape["tape_check"] == "moved":
        Result.update(ok=False, robot_check="moved", error_code="camera_moved",
                      reason=f"no calibrated desk in view; the gripper tape is {Tape['tape_error_cm']:.2f} cm from where the arm puts it "
                             f"(limit {TapeVerifiedCm} cm, through every joint)",
                      error=f"The gripper tip is {Tape['tape_error_cm']:.1f} cm from where the calibration at {Clock} puts it, so the camera "
                            "moved on the robot: recalibrate (Automatic calibration).")
    else:
        Why = TapeAdvice(Tape)
        Result.update(robot_check="assumed",
                      reason=f"no calibrated desk in view; the camera is assumed where it was calibrated: the gripper tape could not "
                             f"check it ({Tape.get('tape_reason') or Tape['tape_check']})",
                      warning=f"Camera assumed where it was calibrated at {Clock}: the gripper tip could not be checked ({Why}).")
    Result.update(Tape)
    Result.update(RimCheck(ImageBgr, DepthMm, Intrinsics, Model, Arm))
    if DeskCheck is not None:
        Result["desk_check"] = DeskCheck
    # The desk path's time and this one's: what the capture waited for.
    Result["check_ms"] = round((time.perf_counter() - Started) * 1000 + float((DeskCheck or {}).get("check_ms") or 0.0), 1)
    return Result


ShadowKey = "robot_shadow"


def WithShadow(Result: dict[str, Any], ImageBgr, DepthMm, Intrinsics, Model, Arm) -> dict[str, Any]:
    """The desk path's camera_check as it stands, with the robot's reading beside it under ShadowKey (the
    roll-aware tape and the rim; logged for comparison, it never changes ok, error_code, error or reason)."""
    if DepthMm is None or Intrinsics is None or CameraDrift.VerifiedReference(Model)[0] is None:
        return Result
    try:
        Shadow = Check(ImageBgr, DepthMm, Intrinsics, Model, Arm)
        Result[ShadowKey] = {Key: Value for Key, Value in Shadow.items()
                             if Key not in ("ok", "angle_deg", "offset_cm", "reason", "reference_fitted_at", "seq", "alignment_source", "error")}
    except Exception as Exc:  # a shadow must never stop a capture the desk passed
        Result[ShadowKey] = {"robot_check": "unavailable", "error": f"{type(Exc).__name__}: {Exc}"}
    return Result
