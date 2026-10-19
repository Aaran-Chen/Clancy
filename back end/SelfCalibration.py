"""Automatic camera self-calibration: the arm shows a marker to the camera.

Why
---
Manual calibration (touch the tip to the desk, click it in the image) has to be
redone every time the camera or the arm moves, which does not scale. Here the
arm does it itself. It visits a set of poses while a printed ArUco marker taped
to the gripper is photographed at each one. The joints say exactly where the
gripper is, the image says where the marker appears, and one least-squares fit
recovers the whole camera: focal length, principal point, radial distortion,
and where the camera sits. No clicks, no ruler, no separate lens step.

Model (display frame, cm: the frame the AI, the panels and manual probes use)
-----
For observation i and marker corner k:

    P_ik     = Tip_i + G_i @ Q_k
    pixel_ik = Project(f, cx, cy, k1, k2, R, t, S @ P_ik)

* Tip_i and G_i come from the live joint positions. G_i is an orthonormal frame
  built from the distal link direction and the pan direction, so it moves
  rigidly with the gripper while wrist roll is held still. IK never commands
  roll and the routine never touches the gripper jaw.
* Q_k (four 3-vectors) are FREE parameters. The marker's printed size, where it
  is taped and how it is rotated never need measuring.
* S is identity or a mirror in X. The display frame is a reflection of the
  physical world in this app, and a camera with a proper rotation cannot fit a
  reflected world, so both are fitted and the far better one is kept. Nothing
  about handedness is assumed.
* Red tape (the marker used on the hardware) gives one centroid per pose and a
  single Q, the tape's offset from the tip. Every pose's red blobs are kept and
  RobustRedAssignment picks the tape by camera consistency, because the live
  tracker can follow the servo wiring. Fit quality is judged in cm at the marker,
  where the arm's joint accuracy (a few mm) sets the floor.

Depth camera
------------
With the Orbbec depth camera the frames come with metric depth aligned to the
color image and factory intrinsics. Every red blob then also records its 3D
camera point, and Solve fixes the intrinsics and fits only the camera pose and
the tape offset, to the tape pixels and the depth points together
(DepthGeometry.FitDepthCamera). The saved model is "pinhole_depth_v1". It also keeps
where the tips were (observed_tip_bounds_cm, observed_tip_hull_cm): /locate trusts a
depth-measured object within TipRegionMarginCm of that hull.

Pixel conventions
-----------------
Frames arrive from the browser mirrored left-right (CAMERA_MIRRORED_VS_ARM in
App.tsx). ArUco codes are chiral, so frames are flipped back before detection
and the fit lives in native OpenCV pixel coordinates (pixel centres on
integers). Queries from /locate arrive in the mirrored continuous space the VLM
and the manual probe clicks use, and PixelToPlane converts them back.
"""

from __future__ import annotations

import base64
import copy
import json
import math
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from scipy.optimize import least_squares
import CalibrationArchive
import CameraDrift
import DepthGeometry

BackendRoot = Path(__file__).resolve().parent
SessionPath = BackendRoot / "selfcal_session.json"
CapturesDir = BackendRoot / "captures"

ArucoDictionaryId = cv2.aruco.DICT_4X4_50
MarkerId = 0

# Quality limits. A fit that misses any of them is still saved and applied (Aaran, 2026-10-10: "never
# reject a calibration unless I explicitly reject a calibration"): each miss is a quality problem in
# the report and in the saved model (quality_problems), shown to the user with its numbers, and only
# the user rejects a calibration (calibration.reject_camera_model restores the one it replaced).
# Fit errors are judged in cm at the marker (pixel error x depth / focal
# length): the arm's own joint accuracy, a few millimetres at the tip, sets that
# floor whatever the camera's distance or resolution. On the hardware a correct
# red-tape run fits to about 0.35 cm (11 px with this camera), so the old 2.5 px
# limit rejected every real run.
MinObservations = 10  # recommended; a fit from fewer poses is saved with a warning
MaxFitRmsCm = 0.6
MaxHoldoutRmsCm = 0.9
MaxPlaneRmsCm = 1.0
MinCameraHeightAboveTableCm = 5.0
# The fewest poses from which a camera is computed at all. Below it a run computes nothing and the
# current calibration stays: the only outcome that keeps it without the user rejecting the new one.
# The colour-only red-tape fit has 14 unknowns (focal length, principal point, two distortion terms,
# the pose, the tape offset) against 2 residuals per pose plus 4 prior terms, and the tape search
# (RobustRedAssignment) samples 5 poses per hypothesis: 6 poses is the first count that fixes them.
MinSolvableObservations = 6
# What a calibration saved without a drift baseline means for the camera check (/locate, CameraDrift.CheckCapture
# answers camera_not_verified, and the app stops every task that acts on an object).
NoBaselineEffect = ("no camera-check baseline was saved with it, so /locate cannot verify the camera: every task that "
                    "acts on an object stops (camera not verified) until a calibration with a baseline is saved or this one "
                    "is rejected; descriptions, relative jogs and gripper commands still work")

# Table height in the display frame. Measured by touching the gripper tip to
# the desk during the 2026-07-24 manual calibration. The arm still stands on
# the same desk, so its height relative to the arm base is unchanged. Callers
# may override it per solve.
DefaultTableZCm = float(json.loads((BackendRoot / "WorkspaceFrame.json").read_text(encoding="utf-8"))["TableZCm"])

# Several frames are captured per pose; they must agree or the arm was moving.
FrameAgreePx = 3.0
# An observation whose worst corner error exceeds this multiple of the median
# is treated as a bad detection and dropped before the final fit.
OutlierFactor = 4.0
OutlierFloorPx = 3.0

# Weak prior on radial distortion: k = 0.1 costs about as much as 1 px of
# error, so distortion is only used when the data actually asks for it.
DistortionPriorWeight = 10.0
# Weak prior keeping the principal point near the image centre: a tip volume
# only a few centimetres thick cannot separate it from camera rotation, and the
# real run's first robust fit slid it to the image corner. An offset of 5% of
# the width costs about as much as 2 px of error. Both priors grow with the
# data's own error beyond PriorNoiseReferencePx (see _FitHypothesis).
PrincipalPointPriorPx = 2.0
PrincipalPointPriorFraction = 0.05
PriorNoiseReferencePx = 1.0

# Red tape contributes one centroid per pose to the 3D camera fit, including
# an unknown rigid offset from the gripper tip. A selected blob establishes
# identity; unrelated stationary red objects are excluded from tracking.
# Seen edge-on or from across the desk the tip tape is only 20-60 px (2026-09-28 runs: most
# poses), so the size floors stay small; identity comes from tracking and camera consistency.
RedMinAreaPx = 12.0
RedBlobMinAreaPx = 8.0
RedMaxAreaFraction = 0.05
RedMinCompactness = 0.45
RedAgreePx = 8.0
RedSelectionMinAreaPx = 40.0
RedTrackingMinAreaPx = 12.0
# Foreshortening changes the tape's area by more than 10x between poses.
RedTrackingAreaRatio = 12.0
# Solving picks the tape among every pose's red blobs by camera consistency
# (RobustRedAssignment). Sizes are for a 1280x720 frame and scale with it.
RedCandidateMinAreaPx = 12.0
RedStaticRadiusPx = 15.0
RedStaticRecurrence = 0.4
RedRansacSamples = 1200
RedRansacGateFraction = 0.05


def _BasketStripDisplay() -> np.ndarray | None:
    try:
        Point = json.loads((BackendRoot / "WorkspaceFrame.json").read_text(encoding="utf-8"))["BasketRedStrip"]["DisplayCm"]
        return np.array([float(Point[Axis]) for Axis in ("x", "y", "z")])
    except (OSError, KeyError, TypeError, ValueError):
        return None


# The basket's far rim carries a small red strip that passes every red-tape test (hue, red over green,
# shape, and the white basket behind it reads as the light housing): 2026-10-12, 45-130 px at 662,485
# in all 10 poses, 0.46 cm2 at 44 cm; 2026-10-10 (another mount) 92-147 px at 664,507 in 5 of 24, too
# few for RobustRedAssignment's recurrence rule. It is fixed to the robot, so its place in the display
# frame (WorkspaceFrame.json BasketRedStrip) projected through the current camera model is masked
# (BasketStripExclusion): red within RedStaticRadiusPx of it is dropped, unless the arm's tape is
# predicted within BasketStripGuardPx of it (the real tape passing in front of it must stay).
BasketStripDisplayCm = _BasketStripDisplay()
BasketStripGuardPx = 40.0

# Depth-camera fits: the 3D tape points must agree with the fitted camera.
MaxDepthRmsCm = 1.0
# At one gripper tilt the tape offset's vertical part and the camera height trade off exactly
# (residuals cannot tell them apart), so a depth fit needs the tilt to vary across poses.
MinGripperTiltSpreadDeg = 5.0
DepthOutlierFloorCm = 1.5

# A depth-measured object is outside the calibrated area when its XY lies more than this far
# outside the hull of the calibrated tips (TipRegionDistance). Depth measures the object itself;
# what limits it is the fitted camera-to-arm transform, good where the tape was and worse with
# distance. Spatial holdouts of the 2026-09-28 22:43 run (refit on part of its tips, tested on the
# rest) came back within 0.1-1.3 cm up to 10 cm past the training band (one pose 1.9 cm, in the
# narrowest, 15 cm band); 11-16 cm past it, errors reached 2.4 cm (1.5-2.4 cm in that band). The
# live tape check read 0.72-0.75 cm. The 9-16 cm drop from the tape to the desk adds little:
# bootstrap refits put the desk under the tips at 0.63 cm median, as at the tips. The old pixel
# test flagged that desk too, and so every object of 2026-09-29.
TipRegionMarginCm = 10.0

# The frames of what each archived calibration pose holds. requested_pose.x/y/z (model) read against
# a tip (display) looked like a 22 cm miss on 2026-09-29, while every pose was reached within 1 cm.
ArchiveFrames = {
    "metadata.requested_pose x, y, z": "model: the bridge's IK frame, relative to the arm base (visual x + workspace offset x)",
    "metadata.requested_pose.visual_target": "visual: the bridge/panel workspace frame, X = -display X",
    "requested_display_target": "display: visual_target with X negated, comparable to the tip",
    "joints, tip, frame, camera model": "display: the AI and app frame (X+ = the arm's left, home offset applied)",
}

_Lock = threading.Lock()
_Session: dict[str, Any] = {"image_size": None, "mirrored": None, "observations": []}


# --------------------------------------------------------------------------
# Session
# --------------------------------------------------------------------------

def _PersistSession() -> None:
    if _Session.get("archive_directory"):
        CalibrationArchive.WriteJson(Path(_Session["archive_directory"]) / "Session.json", _Session)
    CalibrationArchive.WriteJson(SessionPath, _Session)


def ResetSession(Context: dict | None = None) -> dict[str, Any]:
    Directory = CalibrationArchive.CreateRun(CapturesDir, Context)
    with _Lock:
        _Session.clear()
        _Session["image_size"] = None
        _Session["mirrored"] = None
        _Session["observations"] = []
        _Session["archive_directory"] = str(Directory)
        _Session["run_id"] = Directory.name
        _Session["frames"] = ArchiveFrames
        _PersistSession()
    return {"ok": True, "observations": 0, "run_id": Directory.name, "archive_directory": str(Directory)}


def RecordEvent(Kind, Payload):
    Directory = _Session.get("archive_directory")
    return CalibrationArchive.Record(Directory, Kind, Payload) if Directory else None


def RunDirectory() -> str | None:
    """The current calibration run's archive (ResetSession), where a saved camera records what it replaced."""
    with _Lock:
        return _Session.get("archive_directory")


def LoadSessionFromDisk() -> dict[str, Any]:
    """Reload the last session (for offline re-solves and debugging)."""
    with _Lock:
        try:
            Data = json.loads(SessionPath.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {"ok": False, "error": "No saved session."}
        _Session.update(Data)
        return {"ok": True, "observations": len(_Session["observations"])}


# --------------------------------------------------------------------------
# Detection and gripper geometry
# --------------------------------------------------------------------------

def _Detector(Relaxed: bool = False) -> cv2.aruco.ArucoDetector:
    Dictionary = cv2.aruco.getPredefinedDictionary(ArucoDictionaryId)
    Params = cv2.aruco.DetectorParameters()
    Params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    Params.cornerRefinementWinSize = 5
    Params.cornerRefinementMaxIterations = 50
    Params.cornerRefinementMinAccuracy = 0.01
    if Relaxed:
        # For small, soft or dim markers: wider thresholding windows, smaller
        # accepted markers, looser square test, more bit-error correction.
        Params.adaptiveThreshWinSizeMin = 3
        Params.adaptiveThreshWinSizeMax = 53
        Params.adaptiveThreshWinSizeStep = 4
        Params.minMarkerPerimeterRate = 0.01
        Params.polygonalApproxAccuracyRate = 0.06
        Params.errorCorrectionRate = 0.8
        Params.perspectiveRemovePixelPerCell = 8
        Params.perspectiveRemoveIgnoredMarginPerCell = 0.15
        Params.minCornerDistanceRate = 0.02
    return cv2.aruco.ArucoDetector(Dictionary, Params)


def DecodeDataUrl(DataUrl: str) -> np.ndarray | None:
    try:
        _, Payload = DataUrl.split(",", 1)
        Buffer = np.frombuffer(base64.b64decode(Payload), dtype=np.uint8)
        return cv2.imdecode(Buffer, cv2.IMREAD_COLOR)
    except (ValueError, TypeError, cv2.error):
        return None


def _FindInGray(Gray: np.ndarray, Relaxed: bool):
    Corners, Ids, Rejected = _Detector(Relaxed).detectMarkers(Gray)
    Found = None
    OtherIds: list[int] = []
    if Ids is not None:
        for MarkerCorners, Id in zip(Corners, Ids.flatten()):
            if int(Id) == MarkerId:
                Found = MarkerCorners.reshape(4, 2).astype(np.float64)
            else:
                OtherIds.append(int(Id))
    RejectedList = [R.reshape(4, 2).astype(np.float64) for R in (Rejected or [])]
    return Found, RejectedList, OtherIds


def DetectMarkerDetailed(Image: np.ndarray) -> dict[str, Any]:
    """Find marker MarkerId, trying progressively more forgiving stages.

    Stages: plain; local contrast (CLAHE) for dim scenes; 2x upscale for small
    markers; relaxed detector parameters. Returns the corners (native OpenCV
    pixels) plus diagnostics that explain a miss.
    """
    Gray = cv2.cvtColor(Image, cv2.COLOR_BGR2GRAY) if Image.ndim == 3 else Image
    Contrast = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(Gray)
    Stages = (
        ("plain", Gray, 1.0, False),
        ("contrast", Contrast, 1.0, False),
        ("upscaled", cv2.resize(Contrast, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC), 2.0, False),
        ("relaxed", Contrast, 1.0, True),
    )
    MostRejected: list[np.ndarray] = []
    OtherIds: set[int] = set()
    for Name, Source, Scale, Relaxed in Stages:
        Found, Rejected, Others = _FindInGray(Source, Relaxed)
        OtherIds.update(Others)
        if Scale != 1.0:
            # Pixel centres map as x_orig = (x_up + 0.5) / scale - 0.5.
            Unscale = lambda Pts: (Pts + 0.5) / Scale - 0.5  # noqa: E731
            Found = None if Found is None else Unscale(Found)
            Rejected = [Unscale(R) for R in Rejected]
        if len(Rejected) > len(MostRejected):
            MostRejected = Rejected
        if Found is not None:
            Edge = float(np.mean(np.linalg.norm(Found - np.roll(Found, 1, axis=0), axis=1)))
            return {"corners": Found, "stage": Name, "edge_px": Edge, "rejected": MostRejected,
                    "other_ids": sorted(OtherIds), "brightness": float(Gray.mean())}
    return {"corners": None, "stage": None, "edge_px": None, "rejected": MostRejected,
            "other_ids": sorted(OtherIds), "brightness": float(Gray.mean())}


def DetectMarker(Image: np.ndarray) -> np.ndarray | None:
    """Return the 4x2 corners of marker MarkerId in native OpenCV pixels, or None."""
    return DetectMarkerDetailed(Image)["corners"]


def MissReason(Detection: dict[str, Any]) -> str:
    """Plain-language explanation of why a frame had no usable marker."""
    if Detection["other_ids"]:
        return f"found ArUco id {Detection['other_ids']} but not id {MarkerId}; print the marker sheet from Downloads"
    if Detection["brightness"] < 45:
        return f"frame too dark (mean brightness {Detection['brightness']:.0f}/255)"
    Count = len(Detection["rejected"])
    if Count:
        return (f"{Count} marker-like shape(s) seen but none readable: usually blur, glare, a bent "
                "sheet, tape over the black squares, or the marker is too small or too oblique")
    return "nothing marker-like in view: marker out of frame, too small, or facing away"


def SaveDiagnosticFrame(Image: np.ndarray, Detection: dict[str, Any], Tag: str) -> str | None:
    """Archive a native-orientation frame with the detection drawn on it."""
    try:
        Canvas = Image.copy()
        for Rejected in Detection["rejected"][:40]:
            cv2.polylines(Canvas, [np.round(Rejected).astype(np.int32)], True, (60, 60, 230), 1, cv2.LINE_AA)
        if Detection["corners"] is not None:
            Pts = np.round(Detection["corners"]).astype(np.int32)
            cv2.polylines(Canvas, [Pts], True, (60, 220, 60), 2, cv2.LINE_AA)
            cv2.circle(Canvas, tuple(int(v) for v in Pts[0]), 5, (60, 220, 60), -1, cv2.LINE_AA)
            Label = f"marker {Detection['edge_px']:.0f}px ({Detection['stage']})"
        else:
            Label = "no marker: " + MissReason(Detection)[:90]
        cv2.putText(Canvas, Label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(Canvas, Label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
        Folder = CapturesDir / time.strftime("%Y-%m-%d") / "selfcal"
        Folder.mkdir(parents=True, exist_ok=True)
        Name = f"{time.strftime('%H%M%S')}_{int(time.time() * 1000) % 1000:03d}_{Tag}.jpg"
        cv2.imwrite(str(Folder / Name), Canvas, [cv2.IMWRITE_JPEG_QUALITY, 88])
        return str(Folder / Name)
    except (OSError, cv2.error):
        return None


_RedTracker: dict[str, Any] = {}


def SavedCameraModel() -> dict[str, Any] | None:
    """The saved camera model (calibration.camera_model), or None when there is none or it cannot be read."""
    try:
        import calibration  # local import: calibration imports this module for queries
        return calibration.camera_model()
    except Exception:  # noqa: BLE001 - the mask is optional: an unreadable file means no mask
        return None


def ModelPixel(Model: dict[str, Any] | None, Point, Intrinsics: dict[str, Any] | None = None,
               ImageSize=None, Mirrored: bool = False) -> np.ndarray | None:
    """Where Model sees a display-frame point, in native OpenCV pixels (flipped like the image when
    Mirrored), or None: no depth-camera model, a model fitted at another image size, the point behind
    the camera. Intrinsics default to the ones the depth model was fitted with."""
    if not Model or Model.get("type") != "pinhole_depth_v1" or Point is None:
        return None
    Fitted = Model.get("image_size")
    if ImageSize is not None and Fitted and list(Fitted) != [int(ImageSize[0]), int(ImageSize[1])]:
        return None
    try:
        Camera = DepthGeometry.DisplayToCamera(np.asarray(Point, dtype=np.float64), Model)[0]
        if Camera[2] <= 5.0:
            return None
        Pixel = DepthGeometry.Project(Camera, Intrinsics or Model)[0]
        Width = int((ImageSize or Fitted)[0]) if Mirrored else 0
    except (KeyError, TypeError, ValueError, IndexError, cv2.error):
        return None
    if not np.isfinite(Pixel).all():
        return None
    return np.array([Width - 1 - Pixel[0], Pixel[1]]) if Mirrored else Pixel


def BasketStripExclusion(Model: dict[str, Any] | None, TapePixel, Intrinsics: dict[str, Any] | None = None,
                         ImageSize=None, Mirrored: bool = False) -> list[list[float]]:
    """DetectRedDetailed's Exclude for the basket rim's red strip: its pixel under Model, or [] when there is
    no camera model to project it with, no predicted tape pixel (TapePixel, in the same pixels), or the
    tape is predicted within BasketStripGuardPx of it."""
    Strip = ModelPixel(Model, BasketStripDisplayCm, Intrinsics, ImageSize, Mirrored)
    if Strip is None or TapePixel is None:
        return []
    Scale = float((ImageSize or Model.get("image_size") or [1280])[0]) / 1280.0
    if float(np.linalg.norm(Strip - np.asarray(TapePixel, dtype=np.float64))) <= BasketStripGuardPx * Scale:
        return []
    return [Strip.tolist()]


def DetectRedDetailed(Image: np.ndarray, Hint: dict | None = None, Track: bool = False,
                      MaxJumpFrac: float = 0.12, Exclude=None) -> dict[str, Any]:
    """Detect orange/red tape, retaining identity instead of choosing the largest red object.

    Exclude: pixels (BasketStripExclusion) where red is known not to be the tape; a blob whose
    centroid is within RedStaticRadiusPx of one is dropped before anything is chosen."""
    Height, Width = Image.shape[:2]
    AreaScale = Width * Height / (1280.0 * 720.0)
    Hsv = cv2.cvtColor(Image, cv2.COLOR_BGR2HSV)
    Mask = cv2.inRange(Hsv, np.array([0, 85, 65]), np.array([18, 255, 255]))
    Mask |= cv2.inRange(Hsv, np.array([168, 85, 65]), np.array([179, 255, 255]))
    Blue, Green, Red = cv2.split(Image.astype(np.float32))
    # Warm wood can have the same hue as orange tape; red over green is what separates them
    # (wood, skin and the servo wires never reach 1.5x). Shadowed tape turns pinkish, about
    # 1.4x red over blue, so blue only needs a looser margin.
    Dominant = (Red > 1.5 * Green) & (Red > 1.25 * Blue) & (Red - Green > 30)
    Mask[~Dominant] = 0
    # Small kernel keeps a narrow or foreshortened piece of tape intact.
    Mask = cv2.morphologyEx(Mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    Mask = cv2.morphologyEx(Mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    Contours, _ = cv2.findContours(Mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    Candidates = []
    Clipped = False
    for Contour in Contours:
        Area = float(cv2.contourArea(Contour))
        if not max(4.0, RedBlobMinAreaPx * AreaScale) <= Area <= Width * Height * 0.012:
            continue
        (_, _), (A, B), _ = cv2.minAreaRect(Contour)
        if min(A, B) < 2 or max(A, B) / max(min(A, B), 1) > 6:
            continue
        HullArea = float(cv2.contourArea(cv2.convexHull(Contour)))
        if Area / max(HullArea, 1) < 0.5:
            continue
        Moments = cv2.moments(Contour)
        if Moments["m00"] <= 1e-6:
            continue
        Center = np.array([Moments["m10"] / Moments["m00"], Moments["m01"] / Moments["m00"]])
        # A cropped patch still has an interior centroid, but that centroid is
        # no longer the same physical point. Check the entire contour bounds.
        Left, Top, BoxWidth, BoxHeight = cv2.boundingRect(Contour)
        EdgeMargin = max(4, round(min(Width, Height) * 0.005))
        if (Left <= EdgeMargin or Top <= EdgeMargin
                or Left + BoxWidth >= Width - EdgeMargin or Top + BoxHeight >= Height - EdgeMargin):
            Clipped = True
            continue
        Radius = int(max(45 * Width / 1280, math.sqrt(Area) * 3.5))
        Cx, Cy = np.round(Center).astype(int)
        Surrounding = Hsv[max(0, Cy - Radius):min(Height, Cy + Radius), max(0, Cx - Radius):min(Width, Cx + Radius)]
        # This arm has a light gripper housing. A red region on a wooden wall
        # or isolated cable lacks the nearby light housing around the tip.
        LightHousing = (Surrounding[..., 1] < 75) & (Surrounding[..., 2] > 155)
        if int(LightHousing.sum()) < max(40, Area * 0.3):
            continue
        Candidates.append({"centroid": Center, "area": Area, "contour": Contour,
                           "bounds": [Left, Top, BoxWidth, BoxHeight]})
    Excluded = 0
    if Exclude:
        Radius = RedStaticRadiusPx * Width / 1280.0
        Kept = [C for C in Candidates
                if all(np.linalg.norm(C["centroid"] - np.asarray(Point, dtype=np.float64)) > Radius for Point in Exclude)]
        Excluded, Candidates = len(Candidates) - len(Kept), Kept
    # Highlights can split the wrapped tape into a large face and a small side
    # sliver. Nearest-centroid tracking used to jump to that sliver. Exclude
    # small fragments beside a larger patch before applying temporal gates, so
    # rejecting the main patch cannot make its fragment the replacement tip.
    Margin = max(4, round(Width / 128))
    def IsSideFragment(Candidate):
        X, Y, W, H = Candidate["bounds"]
        for Other in Candidates:
            if Other["area"] < 3 * Candidate["area"]:
                continue
            Ox, Oy, Ow, Oh = Other["bounds"]
            GapX = max(0, X - (Ox + Ow), Ox - (X + W))
            GapY = max(0, Y - (Oy + Oh), Oy - (Y + H))
            if math.hypot(GapX, GapY) <= Margin:
                return True
        return False
    Candidates = [Candidate for Candidate in Candidates if not IsSideFragment(Candidate)]
    Result = {"centroid": None, "area": None, "red_px": int(cv2.countNonZero(Mask)),
              "candidates": len(Candidates), "brightness": float(Hsv[:, :, 2].mean()),
              "options": Candidates, "excluded": Excluded,
              "reason": "Click the red tape on the gripper tip in the check image."}
    if Hint is not None:
        Point = _HintPoint(Hint, Width, Height)
        Tolerance = _HintTolerancePx(Width)
        Ranked = sorted(Candidates, key=lambda C: abs(cv2.pointPolygonTest(C["contour"], Point, True))) if Point else []
        Selected = next((C for C in Ranked if cv2.pointPolygonTest(C["contour"], Point, True) >= -Tolerance), None) if Point else None
        if Selected is not None and Selected["area"] < max(20.0, RedSelectionMinAreaPx * AreaScale):
            Result["reason"] = ("The selected red region is too small to identify as the tip tape reliably. "
                                "Make the entire tip patch visible and click it, away from the orange wiring.")
        elif Selected is not None:
            Result.update(Selected, reason=None)
        elif Point is None:
            Result["reason"] = "That click did not land inside the image; click the red tape itself."
    elif Track and _RedTracker.get("center") is not None:
        Previous = np.array(_RedTracker["center"])
        # A rolling area alone can shrink repeatedly until an orange wire
        # fragment becomes the marker. Keep a floor relative to the original
        # selection as well as the image resolution.
        MinimumArea = max(4.0, RedTrackingMinAreaPx * AreaScale,
                          float(_RedTracker.get("seed_area", 0.0)) / RedTrackingAreaRatio / 2)
        Candidates = [C for C in Candidates if C["area"] >= MinimumArea]
        Candidates = [C for C in Candidates
                      if 1 / RedTrackingAreaRatio <= C["area"] / _RedTracker["area"] <= RedTrackingAreaRatio]
        Candidates = [C for C in Candidates if not any(
            np.linalg.norm(C["centroid"] - np.array(Other)) < 18 for Other in _RedTracker.get("background", []))]
        Ranked = sorted(Candidates, key=lambda C: np.linalg.norm(C["centroid"] - Previous))
        if Ranked and np.linalg.norm(Ranked[0]["centroid"] - Previous) <= MaxJumpFrac * Width:
            Distance = np.linalg.norm(Ranked[0]["centroid"] - Previous)
            if len(Ranked) == 1 or np.linalg.norm(Ranked[1]["centroid"] - Previous) > Distance + 30:
                Result.update(Ranked[0], reason=None)
            else:
                Result["reason"] = "Multiple red blobs could be the tip; stop and select the tip again."
        else:
            Result["reason"] = "The selected tip tape is out of view, occluded, or moved too far to track."
    elif not Candidates:
        Result["reason"] = ("The tape is cut off by the image edge. Keep the entire patch in view."
                            if Clipped else "No usable red/orange tip blob in view. Bring the tip into the image.")
    if Clipped and not Result["options"] and Result["centroid"] is None:
        Result["reason"] = "A red region is cut off by the image edge; the selected tip was not identified."
    return Result


def _HintPoint(Hint: dict | None, Width: int, Height: int) -> tuple[float, float] | None:
    """User click as native detection pixels. Accepts fractions or raw pixels.

    Returns None for a missing or malformed hint instead of raising, so a bad
    click degrades to a guided miss rather than a 500.
    """
    if not isinstance(Hint, dict):
        return None
    try:
        Uh, Vh = float(Hint.get("u", -1)), float(Hint.get("v", -1))
    except (TypeError, ValueError):
        return None
    if max(Uh, Vh) > 1.5:
        Uh, Vh = Uh / max(Width, 1), Vh / max(Height, 1)
    if not (0.0 <= Uh <= 1.0 and 0.0 <= Vh <= 1.0):
        return None
    return (Uh * Width, Vh * Height)


def _HintTolerancePx(Width: int) -> float:
    # A finger click on a sidebar-scaled preview lands tens of native pixels
    # off; scale tolerance with frame width instead of a fixed 12 px.
    return max(12.0, 0.025 * float(Width))


def RedMissReason(RedPx: int, Candidates: int, Brightness: float, FrameArea: float) -> str:
    if Brightness < 45:
        return f"frame too dark (mean brightness {Brightness:.0f}/255)"
    if RedPx < RedMinAreaPx:
        return "no red tape in view: tip tape out of frame, too small, or facing away"
    if RedPx > RedMaxAreaFraction * FrameArea * 4:
        return "too much red in view: rim tape or red object dominates, cover the rim strip during calibration"
    if Candidates:
        return "red seen but no compact tip blob: rim strip likely, cover it and keep only the small tip tape visible"
    return "no red blob found: check light and that the tip tape is matte red and unwrinkled"


def SaveRedDiagnosticFrame(Image: np.ndarray, Detection: dict[str, Any], Tag: str) -> str | None:
    try:
        Canvas = Image.copy()
        Contour = Detection.get("contour")
        if Contour is not None:
            cv2.drawContours(Canvas, [Contour], -1, (60, 220, 60), 2, cv2.LINE_AA)
        Centroid = Detection.get("centroid")
        if Centroid is not None:
            cv2.circle(Canvas, (int(Centroid[0]), int(Centroid[1])), 6, (60, 220, 60), -1, cv2.LINE_AA)
            Label = f"red tape {float(Detection.get('area', 0.0)):.0f}px"
        else:
            Label = "no red: " + str(Detection.get("reason", "miss"))[:90]
        cv2.putText(Canvas, Label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(Canvas, Label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
        Folder = CapturesDir / time.strftime("%Y-%m-%d") / "selfcal"
        Folder.mkdir(parents=True, exist_ok=True)
        Name = f"{time.strftime('%H%M%S')}_{int(time.time() * 1000) % 1000:03d}_{Tag}.jpg"
        cv2.imwrite(str(Folder / Name), Canvas, [cv2.IMWRITE_JPEG_QUALITY, 88])
        return str(Folder / Name)
    except (OSError, cv2.error):
        return None


def _BlobCameraPoint(DepthMm: np.ndarray | None, Candidate: dict[str, Any], Intrinsics: dict[str, Any] | None):
    """Camera-frame point (cm) of a red blob from aligned depth, or None."""
    if DepthMm is None or Intrinsics is None or Candidate.get("centroid") is None:
        return None
    Depth = DepthGeometry.DepthAtBlob(DepthMm, Candidate.get("contour"), Candidate["centroid"], Candidate.get("area"))
    if Depth is None:
        return None
    U, V = (float(Value) for Value in Candidate["centroid"])
    return DepthGeometry.Deproject(U, V, Depth, Intrinsics)[0]


def _UsableDepth(DepthFrames, Index: int, Image: np.ndarray) -> np.ndarray | None:
    """The depth frame paired with image Index, if it is aligned to that image."""
    if not DepthFrames or Index >= len(DepthFrames) or DepthFrames[Index] is None:
        return None
    Depth = DepthFrames[Index]
    return Depth if Depth.shape[:2] == Image.shape[:2] else None


def CheckFrames(Images: list[str], Mirrored: bool, Hint: dict | None = None, Initialize: bool = False,
                DepthFrames: list[np.ndarray | None] | None = None,
                DepthIntrinsics: dict[str, Any] | None = None) -> dict[str, Any]:
    Results, RawImages, TapePoints = [], [], []
    for Index, DataUrl in enumerate(Images):
        Image = DecodeDataUrl(DataUrl)
        if Image is None:
            continue
        if Mirrored:
            Image = cv2.flip(Image, 1)
        Detection = DetectRedDetailed(Image, Hint=Hint, Track=Hint is None)
        Point = _BlobCameraPoint(_UsableDepth(DepthFrames, Index, Image), Detection, DepthIntrinsics)
        if Point is not None:
            TapePoints.append(Point)
        Results.append(Detection)
        RawImages.append(Image)
    if not Results:
        return {"ok": False, "error": "No decodable frames."}
    Hits = [R for R in Results if R["centroid"] is not None]
    # Majority agreement around the median: one blurred frame must not veto.
    Agrees = False
    MedianHit = None
    if len(Hits) >= len(Images) // 2 + 1:
        Stack = np.stack([R["centroid"] for R in Hits])
        MedianHit = np.median(Stack, axis=0)
        Spread = 0.0 if len(Hits) == 1 else float(np.max(np.linalg.norm(Stack - MedianHit, axis=1)))
        Agrees = bool(Spread <= RedAgreePx)
    Best = Hits[0] if Hits else Results[0]
    if Initialize:
        with _Lock:
            _RedTracker.clear()
            if Agrees:
                _RedTracker.update(center=MedianHit.tolist(), area=float(np.median([R["area"] for R in Hits])),
                                   seed_area=float(np.median([R["area"] for R in Hits])),
                                   background=[C["centroid"].tolist() for C in Best["options"]
                                               if np.linalg.norm(C["centroid"] - MedianHit) > 20])
    Canvas = RawImages[0].copy()
    for Candidate in Best["options"]:
        cv2.drawContours(Canvas, [Candidate["contour"]], -1, (0, 200, 255), 1)
    if Best["centroid"] is not None:
        cv2.circle(Canvas, tuple(np.round(Best["centroid"]).astype(int)), 12, (50, 240, 50), 2)
    # Draw the user's click in blue so a misplaced click is visible, not silent.
    HintPoint = _HintPoint(Hint, Canvas.shape[1], Canvas.shape[0])
    if HintPoint is not None:
        cv2.circle(Canvas, (int(HintPoint[0]), int(HintPoint[1])), 6, (255, 0, 0), -1, cv2.LINE_AA)
    Nearest = None
    if HintPoint is not None and Best["options"]:
        Nearest = float(min(abs(cv2.pointPolygonTest(C["contour"], HintPoint, True)) for C in Best["options"]))
    Ok, Encoded = cv2.imencode(".jpg", Canvas)
    if not Ok:
        return {"ok": False, "error": "Could not encode the check preview."}
    # Leave evidence: checks otherwise save nothing, so repeat failures are undiagnosable.
    try:
        Folder = CapturesDir / time.strftime("%Y-%m-%d") / "selfcal"
        Folder.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(Folder / f"{time.strftime('%H%M%S')}_{int(time.time() * 1000) % 1000:03d}_check_red_{'hit' if Agrees else 'miss'}.jpg"),
                    Canvas, [cv2.IMWRITE_JPEG_QUALITY, 88])
    except (OSError, cv2.error):
        pass
    return {"ok": True, "detected": Agrees, "method": "red", "frames": len(Results),
            "frames_detected": len(Hits), "area_px": Best["area"],
            "reason": None if Agrees else (Best["reason"] or "Tip detections disagree between frames."),
            "frame_size": [Canvas.shape[1], Canvas.shape[0]],
            "centroid": Best["centroid"].tolist() if Best["centroid"] is not None else None,
            "tape_distance_cm": round(float(np.median([np.linalg.norm(P) for P in TapePoints])), 1) if TapePoints else None,
            "candidates": len(Best["options"]), "hint_dist_px": Nearest,
            "preview": "data:image/jpeg;base64," + base64.b64encode(Encoded).decode("ascii")}


def GripperFrame(Joints: list[dict[str, float]] | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Tip position and an orthonormal frame rigidly attached to the gripper.

    Joints are [base, shoulder, elbow, wrist, roll, tip]. The frame's x axis is
    the distal link direction (roll -> tip); its y axis is horizontal and
    perpendicular to the arm's vertical plane (from the pan direction).
    """
    Points = np.array(
        [[P["x"], P["y"], P["z"]] if isinstance(P, dict) else list(P) for P in Joints],
        dtype=np.float64,
    )
    if Points.shape[0] < 3:
        raise ValueError("Need at least base, roll and tip joint positions.")
    Base, Roll, Tip = Points[0], Points[-2], Points[-1]
    XAxis = Tip - Roll
    if np.linalg.norm(XAxis) < 1e-6:
        raise ValueError("Degenerate distal link.")
    XAxis /= np.linalg.norm(XAxis)
    Radial = Tip - Base
    Radial[2] = 0.0
    if np.linalg.norm(Radial) < 1e-6:
        Radial = XAxis.copy()
        Radial[2] = 0.0
    if np.linalg.norm(Radial) < 1e-6:
        raise ValueError("Tip directly above the base with a vertical distal link.")
    YAxis = np.cross([0.0, 0.0, 1.0], Radial)
    YAxis /= np.linalg.norm(YAxis)
    ZAxis = np.cross(XAxis, YAxis)
    ZAxis /= np.linalg.norm(ZAxis)
    YAxis = np.cross(ZAxis, XAxis)
    return Tip, np.column_stack([XAxis, YAxis, ZAxis])


def _StaleBridgeReason(BridgeState: Any) -> str | None:
    if BridgeState is None:
        return None
    if not isinstance(BridgeState, dict):
        return None
    if BridgeState.get("motion_fault"):
        return f"Bridge reports motion fault: {BridgeState.get('motion_fault')}"
    if BridgeState.get("faults"):
        return f"Bridge reports servo faults: {BridgeState.get('faults')}"
    if BridgeState.get("torque") is False:
        return "Bridge torque off; joint positions may be stale."
    Age = BridgeState.get("control_age_ms")
    if isinstance(Age, (int, float)) and Age > 750:
        return f"Stale bridge feedback ({Age} ms old)."
    Servos = BridgeState.get("servos")
    if isinstance(Servos, dict):
        for Name, Servo in Servos.items():
            if isinstance(Servo, dict) and Servo.get("feedback_ok") is False:
                return f"Stale joints: {Name} lacks fresh feedback."
            if isinstance(Servo, dict) and Servo.get("moving"):
                return "Arm was still moving at capture."
    return None


def _RecordDeskPlanes(DepthFrames, DepthIntrinsics, PoseIndex, Metadata, Archive) -> None:
    """Keep each depth frame's desk plane (camera frame) and the pose's roll and jaw.

    The solve turns them into the camera's drift baseline and checks that the camera
    held still during the run. Best effort: the observation itself never fails here.
    """
    try:
        Servos = ((Metadata or {}).get("bridge_state_at_receive") or {}).get("servos") or {}
        Raw = {Name: Servos[Name]["present_raw"] for Name in ("wrist_roll", "gripper")
               if isinstance(Servos.get(Name), dict) and isinstance(Servos[Name].get("present_raw"), (int, float))}
        Planes = []
        for Index, Depth in enumerate(DepthFrames or []):
            Plane = CameraDrift.DeskPlane(Depth, DepthIntrinsics) if Depth is not None and DepthIntrinsics else None
            if Plane is not None:
                Planes.append({**Plane, "pose_index": PoseIndex, "frame": Index, "received_at": time.time(),
                               "archive": str(Archive), **{f"{Name}_raw": int(Value) for Name, Value in Raw.items()}})
        if Planes:
            with _Lock:
                _Session.setdefault("desk_planes", []).extend(Planes)
    except Exception as Exc:  # the baseline is checked at solve time; never lose the pose over it
        print(f"[selfcal] desk plane not recorded for pose {PoseIndex}: {Exc}")


def _RequestedDisplayTarget(Metadata: dict | None) -> dict[str, float] | None:
    """The requested pose's visual_target in the display frame (X negated), to compare with the tip; None without one."""
    Requested = Metadata.get("requested_pose") if isinstance(Metadata, dict) else None
    Visual = Requested.get("visual_target") if isinstance(Requested, dict) else None
    if not isinstance(Visual, dict) or not all(isinstance(Visual.get(Axis), (int, float)) for Axis in ("x", "y", "z")):
        return None
    return {"x": -float(Visual["x"]), "y": float(Visual["y"]), "z": float(Visual["z"])}


def AddObservation(
    Images: list[str],
    Joints: list[dict[str, float]],
    Mirrored: bool,
    PoseIndex: int | None = None,
    Method: str = "aruco",
    Metadata: dict | None = None,
    DepthFrames: list[np.ndarray | None] | None = None,
    DepthIntrinsics: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Detect the marker in one pose's frames and store it with the gripper pose.

    DepthFrames (native, mm, aligned to each image) and DepthIntrinsics come
    from the depth camera; red-tape poses then record each blob's 3D point.
    """
    if Method == "red":
        # Preserve the original frames and their matching measured pose before
        # detection: rejected detections must remain recoverable offline too.
        try:
            Folder = (Path(_Session["archive_directory"]) / "observations" if _Session.get("archive_directory")
                      else CapturesDir / time.strftime("%Y-%m-%d") / "selfcal" / "observations")
            Folder.mkdir(parents=True, exist_ok=True)
            Archive = Folder / f"{time.time_ns()}_pose{PoseIndex if PoseIndex is not None else 'x'}.json"
            DepthArchives = []
            for Index, Depth in enumerate(DepthFrames or []):
                if Depth is None:
                    DepthArchives.append(None)
                    continue
                # 16-bit PNG keeps every millimetre for offline re-solves.
                DepthPath = Archive.with_name(f"{Archive.stem}_depth{Index}.png")
                if not cv2.imwrite(str(DepthPath), Depth.astype(np.uint16)):
                    raise OSError(f"could not write {DepthPath.name}")
                DepthArchives.append(DepthPath.name)
            CalibrationArchive.WriteJson(Archive, {"format_version": 2, "received_at": time.time(), "pose_index": PoseIndex,
                           "images": Images, "joints": Joints, "mirrored": bool(Mirrored), "metadata": Metadata or {},
                           "tracker_before": copy.deepcopy(_RedTracker), "run_id": _Session.get("run_id"),
                           "marker_method": Method, "depth_frames_png": DepthArchives, "depth_intrinsics": DepthIntrinsics,
                           "frames": ArchiveFrames, "requested_display_target": _RequestedDisplayTarget(Metadata)})
        except (OSError, TypeError, ValueError, cv2.error) as Exc:
            return {"ok": False, "error": f"Could not archive calibration frames and joint data: {Exc}"}
        if (Metadata or {}).get("capture_valid") is False:
            Result = {"ok": False, "error": (Metadata or {}).get("capture_error", "Capture was not stationary or lacked fresh feedback.")}
        else:
            _RecordDeskPlanes(DepthFrames, DepthIntrinsics, PoseIndex, Metadata, Archive)
            Result = AddRedProbeObservation(Images, Joints, Mirrored, PoseIndex, None, None, None,
                                            DepthFrames=DepthFrames, DepthIntrinsics=DepthIntrinsics)
        Result["capture_archive"] = str(Archive)
        RecordEvent("Detection", {"capture_archive": str(Archive), "result": Result})
        return Result
    if Metadata is not None:
        if Metadata.get("capture_valid") is False:
            return {"ok": False, "error": Metadata.get("capture_error", "Capture was not stationary or lacked fresh feedback.")}
        Reject = _StaleBridgeReason(Metadata.get("bridge_state_at_receive"))
        if Reject is not None:
            return {"ok": False, "error": Reject}
    Detections: list[np.ndarray] = []
    ImageSize: list[int] | None = None
    FirstImage: np.ndarray | None = None
    FirstDetection: dict[str, Any] | None = None
    MostInformativeMiss: dict[str, Any] | None = None
    Stages: list[str] = []
    for DataUrl in Images:
        Image = DecodeDataUrl(DataUrl)
        if Image is None:
            continue
        if Mirrored:
            Image = cv2.flip(Image, 1)
        Height, Width = Image.shape[:2]
        if ImageSize is None:
            ImageSize = [Width, Height]
        elif ImageSize != [Width, Height]:
            return {"ok": False, "error": "Frames in one pose have different sizes."}
        Detection = DetectMarkerDetailed(Image)
        if FirstImage is None:
            FirstImage, FirstDetection = Image, Detection
        if Detection["corners"] is not None:
            Detections.append(Detection["corners"])
            Stages.append(Detection["stage"])
        elif MostInformativeMiss is None or len(Detection["rejected"]) > len(MostInformativeMiss["rejected"]):
            MostInformativeMiss = Detection
    Tag = f"pose{PoseIndex if PoseIndex is not None else 'x'}_{'hit' if Detections else 'miss'}"
    Archived = SaveDiagnosticFrame(FirstImage, FirstDetection, Tag) if FirstImage is not None else None

    if ImageSize is None:
        return {"ok": False, "error": "No decodable frames."}

    with _Lock:
        if _Session["image_size"] and _Session["image_size"] != ImageSize:
            return {"ok": False, "error": f"Frame size {ImageSize} differs from earlier frames {_Session['image_size']}."}
        _Session["image_size"] = ImageSize
        _Session["mirrored"] = bool(Mirrored)
        Count = len(_Session["observations"])

    if not Detections:
        return {"ok": True, "detected": False, "observations": Count, "reason": MissReason(MostInformativeMiss or FirstDetection)}

    Stack = np.stack(Detections)
    Median = np.median(Stack, axis=0)
    Spread = float(np.max(np.linalg.norm(Stack - Median, axis=-1)))
    if Spread > FrameAgreePx:
        return {
            "ok": True,
            "detected": False,
            "reason": f"frames disagree by {Spread:.1f} px (arm still moving)",
            "frames": len(Images),
            "observations": Count,
            "archived": Archived,
        }

    try:
        Tip, Frame = GripperFrame(Joints)
    except ValueError as Exc:
        return {"ok": False, "error": f"Bad joint positions: {Exc}"}

    Record = {
        "pose_index": PoseIndex,
        "corners": Median.tolist(),
        "tip": Tip.tolist(),
        "frame": Frame.tolist(),
        "frames_detected": len(Detections),
    }
    with _Lock:
        _Session["observations"].append(Record)
        Count = len(_Session["observations"])
        _PersistSession()
    EdgePx = float(np.mean(np.linalg.norm(Median - np.roll(Median, 1, axis=0), axis=1)))
    return {
        "ok": True,
        "detected": True,
        "method": "aruco",
        "frames": len(Images),
        "frames_detected": len(Detections),
        "marker_edge_px": round(EdgePx, 1),
        "stages": Stages,
        "observations": Count,
        "archived": Archived,
    }


def StableRedCandidates(Options: list[list[dict[str, Any]]]) -> list[dict[str, Any]]:
    """Red blobs present in most frames of one pose, as {"centroid": [u, v], "area": px}.

    Options holds DetectRedDetailed(...)["options"] for each frame. Unlike the
    tracker, nothing is chosen here: the solver decides which blob is the tape.
    """
    if not Options:
        return []
    Stable = []
    for Candidate in Options[0]:
        Matches = [Candidate]
        for Other in Options[1:]:
            Near = [C for C in Other if np.linalg.norm(C["centroid"] - Candidate["centroid"]) <= RedAgreePx]
            if Near:
                Matches.append(min(Near, key=lambda C: np.linalg.norm(C["centroid"] - Candidate["centroid"])))
        if len(Matches) >= len(Options) // 2 + 1:
            Entry = {"centroid": np.median([M["centroid"] for M in Matches], axis=0).tolist(),
                     "area": float(np.median([M["area"] for M in Matches]))}
            Points = [M["camera_cm"] for M in Matches if M.get("camera_cm") is not None]
            if Points:
                Entry["camera_cm"] = np.median(Points, axis=0).tolist()
            Stable.append(Entry)
    return Stable


def AddRedProbeObservation(Images, Joints, Mirrored, PoseIndex, FirstImage, ImageSize, ArchivedAruco,
                           DepthFrames=None, DepthIntrinsics=None):
    """Store the tracked red centroid and its measured 3D gripper frame.

    Every pose with frames and joints, tracked or not, also keeps all of its
    stable red blobs in the session's red_poses for RobustRedAssignment. With
    depth, every blob also keeps its camera-frame point ("camera_cm").
    """
    Centroids = []
    Areas = []
    TrackedPoints = []
    FirstRedImage = None
    FirstRedDetection = None
    BestMiss = None
    LastHitOptions = []
    AllOptions = []
    DepthUsed = 0
    Excluded = 0
    # The basket rim's strip, where the saved camera model puts it, unless the tape is predicted near it.
    Model = SavedCameraModel()
    TapeOffset = (Model or {}).get("marker_corners_gripper_cm") or []
    try:
        StripTip, StripFrame = GripperFrame(Joints)
        TapePoint = StripTip + StripFrame @ np.asarray(TapeOffset[0], dtype=np.float64) if len(TapeOffset) == 1 else None
    except (ValueError, TypeError):
        TapePoint = None
    Exclusions: dict[tuple, list] = {}
    for Index, DataUrl in enumerate(Images):
        Image = DecodeDataUrl(DataUrl)
        if Image is None:
            continue
        if Mirrored:
            Image = cv2.flip(Image, 1)
        if ImageSize is None:
            ImageSize = [Image.shape[1], Image.shape[0]]
        elif ImageSize != [Image.shape[1], Image.shape[0]]:
            return {"ok": False, "error": "Frame sizes changed during the observation."}
        # Poses jump farther in pixels than check frames (arm travels between
        # poses), so reacquire wide here; area/rivalry gates still disambiguate.
        # ponytail: fixed 0.30 window, derive from pose-to-pose pixel motion if it mislocks.
        Size = (Image.shape[1], Image.shape[0])
        if Size not in Exclusions:
            Exclusions[Size] = BasketStripExclusion(Model, ModelPixel(Model, TapePoint, DepthIntrinsics, Size, Mirrored),
                                                    DepthIntrinsics, Size, Mirrored)
        Detection = DetectRedDetailed(Image, Track=True, MaxJumpFrac=0.30, Exclude=Exclusions[Size])
        Excluded += Detection["excluded"]
        Depth = _UsableDepth(DepthFrames, Index, Image)
        if Depth is not None and DepthIntrinsics is not None:
            DepthUsed += 1
            for Option in Detection.get("options", []):
                Option["camera_cm"] = _BlobCameraPoint(Depth, Option, DepthIntrinsics)
        AllOptions.append(Detection.get("options", []))
        if FirstRedImage is None:
            FirstRedImage, FirstRedDetection = Image, Detection
        if Detection["centroid"] is not None:
            Centroids.append(Detection["centroid"])
            Areas.append(float(Detection["area"]))
            LastHitOptions = Detection.get("options", [])
            Tracked = next((Option for Option in LastHitOptions
                            if np.allclose(Option["centroid"], Detection["centroid"])), None)
            if Tracked is not None and Tracked.get("camera_cm") is not None:
                TrackedPoints.append(Tracked["camera_cm"])
        elif BestMiss is None:
            BestMiss = Detection
    Tag = f"pose{PoseIndex if PoseIndex is not None else 'x'}_red_{'hit' if Centroids else 'miss'}"
    Archived = SaveRedDiagnosticFrame(FirstRedImage, FirstRedDetection, Tag) if FirstRedImage is not None else ArchivedAruco
    if ImageSize is None:
        return {"ok": False, "error": "No decodable frames."}
    try:
        PoseTip, PoseFrame = GripperFrame(Joints)
    except ValueError:
        PoseTip = PoseFrame = None
    if DepthUsed:
        with _Lock:
            Known = _Session.get("depth_intrinsics")
            if Known is not None and Known != DepthIntrinsics:
                return {"ok": False, "error": "The depth camera's intrinsics changed during calibration; restart it."}
            _Session["depth_intrinsics"] = DepthIntrinsics
    if PoseTip is not None:
        with _Lock:
            if _Session["image_size"] in (None, ImageSize) and _Session["mirrored"] in (None, Mirrored):
                _Session["image_size"], _Session["mirrored"] = ImageSize, Mirrored
                _Session.setdefault("red_poses", []).append({
                    "pose_index": PoseIndex, "tip": PoseTip.tolist(), "frame": PoseFrame.tolist(),
                    "candidates": StableRedCandidates(AllOptions), "archived": Archived,
                    "basket_strip_px": next((Pixels[0] for Pixels in Exclusions.values() if Pixels), None),
                    "basket_strip_excluded": Excluded})
                _PersistSession()
    if len(Centroids) < len(Images) // 2 + 1:
        Reason = str((BestMiss or {}).get("reason", "red tape not found"))
        return {"ok": True, "detected": False, "method": "red",
                "frames": len(Images), "observations": len(_Session["observations"]),
                "reason": Reason, "archived": Archived}
    Stack = np.stack(Centroids)
    Median = np.median(Stack, axis=0)
    Spread = float(np.max(np.linalg.norm(Stack - Median, axis=-1))) if len(Stack) > 1 else 0.0
    if Spread > RedAgreePx:
        return {"ok": True, "detected": False, "method": "red",
                "reason": f"red frames disagree by {Spread:.1f} px (arm still moving)",
                "frames": len(Images), "observations": len(_Session["observations"]),
                "archived": Archived}
    try:
        Tip, Frame = GripperFrame(Joints)
    except ValueError as Exc:
        return {"ok": False, "error": f"Bad joint positions: {Exc}"}
    with _Lock:
        if _Session["image_size"] not in (None, ImageSize) or (_Session["mirrored"] is not None and _Session["mirrored"] != Mirrored):
            return {"ok": False, "error": "Camera frame changed; restart calibration."}
        if any(len(O["corners"]) != 1 for O in _Session["observations"]):
            return {"ok": False, "error": "Cannot mix red tape and ArUco observations."}
        _Session["image_size"], _Session["mirrored"] = ImageSize, Mirrored
        _Session["observations"].append({"pose_index": PoseIndex, "tip": Tip.tolist(),
                                        "frame": Frame.tolist(), "corners": [Median.tolist()], "method": "red",
                                        "area_px": float(np.median(Areas)), "spread_px": Spread,
                                        "archived": Archived,
                                        "camera_cm": np.median(TrackedPoints, axis=0).tolist() if TrackedPoints else None})
        _RedTracker["center"] = Median.tolist()
        MedianArea = float(np.median(np.array(Areas))) if Areas else float(_RedTracker.get("area", 0.0))
        _RedTracker["area"] = float(0.6 * float(_RedTracker.get("area", MedianArea)) + 0.4 * MedianArea)
        _RedTracker["background"] = [C["centroid"].tolist() for C in LastHitOptions
                                     if float(np.linalg.norm(np.array(C["centroid"]) - Median)) > 20]
        _PersistSession()
        Count = len(_Session["observations"])
    Result = {"ok": True, "detected": True, "method": "red", "observations": Count,
              "frames_detected": len(Centroids), "archived": Archived}
    if TrackedPoints:
        Result["tape_distance_cm"] = round(float(np.linalg.norm(np.median(TrackedPoints, axis=0))), 1)
    return Result


# --------------------------------------------------------------------------
# Camera model
# --------------------------------------------------------------------------

ParamCount = 23  # f, cx, cy, k1, k2, rvec(3), tvec(3), Q(4x3)
MirrorVector = np.array([-1.0, 1.0, 1.0])


def _Unpack(Params: np.ndarray):
    return (
        Params[0], Params[1], Params[2], Params[3], Params[4],
        Params[5:8], Params[8:11], Params[11:].reshape(-1, 3),
    )


def _ProjectCorners(Params, Tips, Frames, Mirror):
    F, Cx, Cy, K1, K2, Rvec, Tvec, Q = _Unpack(Params)
    Rotation, _ = cv2.Rodrigues(np.asarray(Rvec, dtype=np.float64))
    World = Tips[:, None, :] + np.einsum("nij,kj->nki", Frames, Q)
    if Mirror:
        World = World * MirrorVector
    Camera = World @ Rotation.T + Tvec
    Depth = Camera[..., 2]
    SafeDepth = np.where(Depth > 1e-3, Depth, 1e-3)
    X = Camera[..., 0] / SafeDepth
    Y = Camera[..., 1] / SafeDepth
    R2 = X * X + Y * Y
    Radial = 1.0 + K1 * R2 + K2 * R2 * R2
    Pixels = np.stack([F * X * Radial + Cx, F * Y * Radial + Cy], axis=-1)
    return Pixels, Depth


def _Residuals(Params, Tips, Frames, Observed, Mirror, FreeDistortion, ImageSize, PriorScale=1.0):
    if not FreeDistortion:
        Params = Params.copy()
        Params[3] = 0.0
        Params[4] = 0.0
    Pixels, Depth = _ProjectCorners(Params, Tips, Frames, Mirror)
    Residual = Pixels - Observed
    Behind = Depth <= 1e-3
    if Behind.any():
        Residual = Residual + (Behind * (1.0 - Depth) * 100.0)[..., None]
    Width, Height = ImageSize
    Centre = (np.array([Params[1], Params[2]]) - [(Width - 1) / 2.0, (Height - 1) / 2.0]) / (PrincipalPointPriorFraction * Width)
    Prior = np.concatenate([np.array([Params[3], Params[4]]) * DistortionPriorWeight, Centre * PrincipalPointPriorPx])
    return np.concatenate([Residual.ravel(), Prior * PriorScale])


def _Bounds(ImageSize, Count=ParamCount):
    Width, Height = ImageSize
    Lower = np.full(Count, -np.inf)
    Upper = np.full(Count, np.inf)
    Lower[0], Upper[0] = 0.2 * Width, 5.0 * Width
    Lower[1], Upper[1] = 0.0, float(Width)
    Lower[2], Upper[2] = 0.0, float(Height)
    Lower[3:5], Upper[3:5] = -1.0, 1.0
    Lower[11:], Upper[11:] = -40.0, 40.0  # marker corners within 40 cm of the tip
    return Lower, Upper


def _Refine(P0, Tips, Frames, Observed, Mirror, ImageSize, FreeDistortion=True, MaxEvaluations=400, PriorScale=1.0):
    Lower, Upper = _Bounds(ImageSize, len(P0))
    Start = np.clip(P0, Lower + 1e-6, Upper - 1e-6)
    Result = least_squares(
        _Residuals, Start,
        args=(Tips, Frames, Observed, Mirror, FreeDistortion, ImageSize, PriorScale),
        bounds=(Lower, Upper), method="trf", loss="soft_l1", f_scale=2.0,
        x_scale="jac", max_nfev=MaxEvaluations,
    )
    Params = Result.x.copy()
    if not FreeDistortion:
        Params[3] = 0.0
        Params[4] = 0.0
    return Params


def _CornerErrors(Params, Tips, Frames, Observed, Mirror):
    Pixels, Depth = _ProjectCorners(Params, Tips, Frames, Mirror)
    return np.linalg.norm(Pixels - Observed, axis=-1), Depth


def _InitialGuess(Tips, Observed, ImageSize, Mirror, FocalScale):
    Width, Height = ImageSize
    F0 = FocalScale * Width
    Cx0, Cy0 = (Width - 1) / 2.0, (Height - 1) / 2.0
    CameraMatrix = np.array([[F0, 0, Cx0], [0, F0, Cy0], [0, 0, 1]], dtype=np.float64)
    ObjectPoints = (Tips * MirrorVector if Mirror else Tips).astype(np.float64)
    Centers = Observed.mean(axis=1).astype(np.float64)
    try:
        Ok, Rvec, Tvec = cv2.solvePnP(ObjectPoints, Centers, CameraMatrix, None, flags=cv2.SOLVEPNP_SQPNP)
        if not Ok:
            return None
        Ok, Rvec, Tvec = cv2.solvePnP(
            ObjectPoints, Centers, CameraMatrix, None, Rvec, Tvec,
            useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE,
        )
    except cv2.error:
        return None
    if not Ok:
        return None
    return np.concatenate([[F0, Cx0, Cy0, 0.0, 0.0], Rvec.ravel(), Tvec.ravel(), np.zeros(3 * Observed.shape[1])])


def _FitHypothesis(Tips, Frames, Observed, ImageSize, Mirror):
    """Best fit for one handedness over several focal-length starting guesses.

    The priors on distortion and the principal point are scaled to the data's
    own error level (a first fit measures it), so noisy red-tape data held back
    by joint accuracy cannot trade a plausible camera for a little lower error,
    while precise marker data is not biased by them.
    """
    Best = None
    for FocalScale in (0.6, 0.9, 1.3, 2.0):
        P0 = _InitialGuess(Tips, Observed, ImageSize, Mirror, FocalScale)
        if P0 is None:
            continue
        # Stage 1 without distortion so the geometry settles; stage 2 frees it.
        Probe = _Refine(P0, Tips, Frames, Observed, Mirror, ImageSize, FreeDistortion=False)
        ProbeErrors, _ = _CornerErrors(Probe, Tips, Frames, Observed, Mirror)
        PriorScale = max(1.0, float(np.sqrt(np.mean(ProbeErrors ** 2))) / PriorNoiseReferencePx)
        P1 = _Refine(P0, Tips, Frames, Observed, Mirror, ImageSize, FreeDistortion=False, PriorScale=PriorScale)
        P2 = _Refine(P1, Tips, Frames, Observed, Mirror, ImageSize, FreeDistortion=True, PriorScale=PriorScale)
        Errors, Depth = _CornerErrors(P2, Tips, Frames, Observed, Mirror)
        if (Depth <= 0).any():
            continue
        Rms = float(np.sqrt(np.mean(Errors ** 2)))
        if Best is None or Rms < Best["rms"]:
            Best = {"params": P2, "rms": Rms, "mirror": Mirror, "prior_scale": PriorScale}
    return Best


def _RayToPlane(Model: dict[str, Any], UNative: np.ndarray, VNative: np.ndarray, TableZ: float):
    F, Cx, Cy = Model["f"], Model["cx"], Model["cy"]
    CameraMatrix = np.array([[F, 0, Cx], [0, F, Cy], [0, 0, 1]], dtype=np.float64)
    Dist = np.array(Model["dist"], dtype=np.float64)
    Pixels = np.stack([UNative, VNative], axis=-1).reshape(-1, 1, 2).astype(np.float64)
    Normalized = cv2.undistortPoints(Pixels, CameraMatrix, Dist).reshape(-1, 2)
    Rotation, _ = cv2.Rodrigues(np.array(Model["rvec"], dtype=np.float64))
    Tvec = np.array(Model["tvec"], dtype=np.float64)
    Center = -Rotation.T @ Tvec
    Directions = np.column_stack([Normalized, np.ones(len(Normalized))]) @ Rotation  # R^T d, row-wise
    Dz = Directions[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        Scale = (TableZ - Center[2]) / Dz
    Points = Center + Scale[:, None] * Directions
    Valid = np.isfinite(Scale) & (Scale > 0)
    if Model["mirror_x"]:
        Points = Points * MirrorVector
    return Points, Valid


def _ModelFromParams(Params, Mirror, ImageSize, InputMirrored, TableZ) -> dict[str, Any]:
    F, Cx, Cy, K1, K2, Rvec, Tvec, Q = _Unpack(Params)
    return {
        "type": "pinhole_selfcal_v1",
        "f": float(F), "cx": float(Cx), "cy": float(Cy),
        "dist": [float(K1), float(K2), 0.0, 0.0],
        "rvec": [float(v) for v in Rvec], "tvec": [float(v) for v in Tvec],
        "mirror_x": bool(Mirror),
        "image_size": [int(ImageSize[0]), int(ImageSize[1])],
        "input_mirrored": bool(InputMirrored),
        "table_z": float(TableZ),
        "marker_corners_gripper_cm": Q.tolist(),
    }


def _PlaneAccuracy(Model, Tips, PixelSigmaPx, TableZ, Rng) -> float | None:
    """Monte-Carlo: table points in the arm's working area, pixel noise, ray back."""
    Width, Height = Model["image_size"]
    XMin, YMin = Tips[:, 0].min() - 5.0, Tips[:, 1].min() - 5.0
    XMax, YMax = Tips[:, 0].max() + 5.0, Tips[:, 1].max() + 5.0
    Xs, Ys = np.meshgrid(np.linspace(XMin, XMax, 9), np.linspace(YMin, YMax, 9))
    Truth = np.column_stack([Xs.ravel(), Ys.ravel(), np.full(Xs.size, TableZ)])
    Rotation, _ = cv2.Rodrigues(np.array(Model["rvec"], dtype=np.float64))
    CameraMatrix = np.array([[Model["f"], 0, Model["cx"]], [0, Model["f"], Model["cy"]], [0, 0, 1]])
    World = Truth * MirrorVector if Model["mirror_x"] else Truth
    Projected, _ = cv2.projectPoints(
        World, np.array(Model["rvec"]), np.array(Model["tvec"]), CameraMatrix, np.array(Model["dist"])
    )
    Projected = Projected.reshape(-1, 2)
    InFront = (World @ Rotation.T + np.array(Model["tvec"]))[:, 2] > 0
    InImage = (
        InFront
        & (Projected[:, 0] >= 0) & (Projected[:, 0] <= Width - 1)
        & (Projected[:, 1] >= 0) & (Projected[:, 1] <= Height - 1)
    )
    if not InImage.any():
        return None
    Truth, Projected = Truth[InImage], Projected[InImage]
    Samples = 20
    AxisSigma = PixelSigmaPx / math.sqrt(2.0)
    Noisy = np.repeat(Projected, Samples, axis=0) + Rng.normal(0, AxisSigma, (len(Projected) * Samples, 2))
    Recovered, Valid = _RayToPlane(Model, Noisy[:, 0], Noisy[:, 1], TableZ)
    Errors = np.linalg.norm(Recovered[:, :2] - np.repeat(Truth, Samples, axis=0)[:, :2], axis=1)[Valid]
    return float(np.sqrt(np.mean(Errors ** 2))) if Errors.size else None


def RobustRedAssignment(RedPoses: list[dict[str, Any]], ImageSize, Seed: int = 0,
                        Intrinsics: dict[str, Any] | None = None):
    """Pick the tip tape among each pose's red blobs by camera consistency.

    The live tracker follows the nearest red blob, so one occluded or edge-on pose
    can hand it to the servo wiring for the rest of the run (on the hardware: 10
    of 19 tracked poses). Here every stable red blob stays a candidate. Blobs that
    recur at the same pixel across many poses (a notebook, a rim strip) are
    background. A RANSAC over poses, sampling each pose's candidates by area,
    finds the camera (tape assumed at the tip) that the most poses agree with;
    the full model is then refined while every pose re-picks its nearest
    candidate under a shrinking gate.

    Returns (observations like _Session["observations"], diagnostics); the
    observations are None when too few poses agree to compute a camera
    (MinSolvableObservations); fewer than MinObservations agreeing is Solve's
    quality warning. Known Intrinsics (a depth camera's factory values) replace
    the focal-length scan in the RANSAC.
    """
    Width, Height = ImageSize
    PixelScale, AreaScale = Width / 1280.0, Width * Height / (1280.0 * 720.0)
    Poses = [P for P in RedPoses if P.get("candidates")]
    Centroids = [np.array([C["centroid"] for C in P["candidates"]], dtype=np.float64) for P in Poses]
    Areas = [np.array([C["area"] for C in P["candidates"]], dtype=np.float64) for P in Poses]
    Cameras = [np.array([C.get("camera_cm") if C.get("camera_cm") is not None else [np.nan] * 3
                         for C in P["candidates"]], dtype=np.float64) for P in Poses]
    Diagnostics = {"red_poses": len(RedPoses), "poses_with_red": len(Poses)}
    Radius, StaticRemoved = RedStaticRadiusPx * PixelScale, 0
    # Judge recurrence against every pose's full blob list, then filter: filtering in place
    # emptied poses whose blobs were all small, and the next pose's check crashed on them.
    Keeps = []
    for Index in range(len(Poses)):
        Keep = Areas[Index] >= RedCandidateMinAreaPx * AreaScale
        for Candidate, Centroid in enumerate(Centroids[Index]):
            Recurring = sum(1 for Other in range(len(Poses)) if Other != Index
                            and np.min(np.linalg.norm(Centroids[Other] - Centroid, axis=1)) <= Radius)
            if Recurring >= RedStaticRecurrence * max(1, len(Poses) - 1):
                Keep[Candidate] = False
                StaticRemoved += 1
        Keeps.append(Keep)
    for Index, Keep in enumerate(Keeps):
        Centroids[Index], Areas[Index], Cameras[Index] = Centroids[Index][Keep], Areas[Index][Keep], Cameras[Index][Keep]
    Usable = [Index for Index in range(len(Poses)) if len(Centroids[Index])]
    Diagnostics["background_blobs_removed"] = StaticRemoved
    if len(Usable) < MinSolvableObservations:
        Diagnostics["assigned"] = 0
        return None, Diagnostics
    Poses = [Poses[Index] for Index in Usable]
    Centroids, Areas = [Centroids[Index] for Index in Usable], [Areas[Index] for Index in Usable]
    Cameras = [Cameras[Index] for Index in Usable]
    Tips = np.array([P["tip"] for P in Poses], dtype=np.float64)
    Frames = np.array([P["frame"] for P in Poses], dtype=np.float64)
    Most = max(len(C) for C in Centroids)
    Padded = np.full((len(Poses), Most, 2), np.nan)
    for Index, Centroid in enumerate(Centroids):
        Padded[Index, :len(Centroid)] = Centroid

    def Nearest(Pixels, Depth):
        Distances = np.linalg.norm(Padded - Pixels[:, None, :], axis=2)
        Distances = np.where(np.isnan(Distances), np.inf, Distances)
        Choice = np.argmin(Distances, axis=1)
        Distance = Distances[np.arange(len(Poses)), Choice]
        return Choice, np.where(Depth > 0, Distance, np.inf)

    Rng = np.random.default_rng(Seed)
    Gate = RedRansacGateFraction * Width
    Best = None
    if Intrinsics is not None:
        Known = DepthGeometry.CameraMatrix(Intrinsics)
        Matrices = [Known]
    else:
        Matrices = [np.array([[Scale * Width, 0, (Width - 1) / 2.0], [0, Scale * Width, (Height - 1) / 2.0], [0, 0, 1]])
                    for Scale in (0.5, 0.7, 0.9, 1.2, 1.6, 2.2)]
    for Mirror in (False, True):
        Object = Tips * MirrorVector if Mirror else Tips
        for CameraMatrix in Matrices:
            Focal = CameraMatrix[0, 0]
            for _ in range(RedRansacSamples):
                Sample = Rng.choice(len(Poses), 5, replace=False)
                Picks = np.array([Centroids[S][Rng.choice(len(Areas[S]), p=Areas[S] / Areas[S].sum())] for S in Sample])
                try:
                    Ok, Rvec, Tvec = cv2.solvePnP(Object[Sample], Picks, CameraMatrix, None, flags=cv2.SOLVEPNP_SQPNP)
                except cv2.error:
                    continue
                if not Ok:
                    continue
                Rotation, _ = cv2.Rodrigues(Rvec)
                Camera = Object @ Rotation.T + Tvec.ravel()
                Depth = Camera[:, 2]
                Pixels = Camera[:, :2] / np.where(Depth > 1e-6, Depth, 1e-6)[:, None] * Focal + CameraMatrix[:2, 2]
                Choice, Distance = Nearest(Pixels, Depth)
                Inliers = Distance <= Gate
                Score = (int(Inliers.sum()), -float(Distance[Inliers].sum()))
                if Best is None or Score > Best["score"]:
                    Best = {"score": Score, "mirror": Mirror, "choice": Choice, "inliers": Inliers}
    if Best is None or Best["score"][0] < MinSolvableObservations:
        Diagnostics["assigned"] = 0 if Best is None else Best["score"][0]
        return None, Diagnostics
    Diagnostics["consensus"] = Best["score"][0]
    Mirror, Choice, Inliers = Best["mirror"], Best["choice"], Best["inliers"]
    # A refinement round may not shrink the poses below MinObservations, or below a smaller consensus.
    Floor = min(MinObservations, Best["score"][0])
    for Round in range(4):
        Observed = Padded[np.arange(len(Poses)), Choice][:, None, :]
        Fit = _FitHypothesis(Tips[Inliers], Frames[Inliers], Observed[Inliers], ImageSize, Mirror)
        if Fit is None:
            break
        Pixels, Depth = _ProjectCorners(Fit["params"], Tips, Frames, Mirror)
        NewChoice, Distance = Nearest(Pixels[:, 0], Depth[:, 0])
        RoundGate = (0.05 * Width, 0.03 * Width, max(8.0 * PixelScale, 3.0 * Fit["rms"]), max(8.0 * PixelScale, 3.0 * Fit["rms"]))[Round]
        NewInliers = Distance <= RoundGate
        if NewInliers.sum() < Floor:
            break
        Stable = np.array_equal(NewInliers, Inliers) and np.array_equal(NewChoice[NewInliers], Choice[NewInliers])
        Choice, Inliers = NewChoice, NewInliers
        if Stable:
            break
    Observations = [{"pose_index": Poses[Index]["pose_index"], "tip": Poses[Index]["tip"], "frame": Poses[Index]["frame"],
                     "corners": [Centroids[Index][Choice[Index]].tolist()], "method": "red",
                     "area_px": float(Areas[Index][Choice[Index]]), "archived": Poses[Index].get("archived"),
                     "camera_cm": (Cameras[Index][Choice[Index]].tolist()
                                   if np.isfinite(Cameras[Index][Choice[Index]]).all() else None)}
                    for Index in np.flatnonzero(Inliers)]
    Diagnostics["assigned"] = len(Observations)
    return Observations, Diagnostics


def _NotComputed(Reason: str, **Fields) -> dict[str, Any]:
    """A run from which no camera could be computed: nothing is saved and the current calibration stays.

    The only outcome that keeps the old calibration without the user rejecting the new one: every
    run that computes a camera is saved and applied, its quality problems shown as warnings (_Conclude).
    """
    return {"ok": False, "computed": False, "saved": False,
            "error": f"Could not compute a calibration: {Reason}. The current calibration is still in use.", **Fields}


def _Conclude(Report: dict[str, Any], Model: dict[str, Any], Problems: list[str], Count: int, Stamp, Single: bool,
              Save: bool, Session: dict[str, Any]) -> dict[str, Any]:
    """Finish a computed camera's report: it is kept whatever its quality.

    Problems (what used to reject the fit: fit or hold-out error, depth disagreement, camera motion
    during the run, no drift baseline, few poses) become warnings with their numbers, in the report
    and in the model (quality_problems, saved with it). With Save it is saved and applied at once;
    only the user rejects a calibration (calibration.reject_camera_model).
    """
    Model["quality_problems"] = list(Problems)
    Report.update(ok=True, computed=True, quality_ok=not Problems, problems=list(Problems), fitted_at=Model.get("fitted_at"),
                  drift_baseline="drift_reference" in Model)
    if Problems:
        FitMisses = any(Problem.startswith(("fit error", "hold-out error")) for Problem in Problems)
        Advice = (f"{Count} clicked poses used: re-click or skip the flagged poses and fit again" if Stamp else
                  f"{Count} tape poses used" + (" (check that the green outline follows only the tip tape, not the orange wiring)"
                                                if FitMisses and Single else "") + ": fix what the warnings name and calibrate again")
        Report["quality_warning"] = f"{Advice}, or press Reject calibration to restore the previous calibration."
    if Save:
        import calibration  # local import: calibration imports this module for queries

        Saved = calibration.save_camera_model(Model, Session.get("archive_directory"))
        Report.update(saved=True, can_reject=Saved["can_reject"], replaced_fitted_at=Saved["replaced_fitted_at"])
    return Report


def Solve(TableZ: float | None = None, Save: bool = True, Seed: int = 0, Source: str = "auto") -> dict[str, Any]:
    """Fit the camera from the stored observations and report its quality; with Save, save it.

    Every camera that can be computed is reported ok, and saved with Save, whatever its quality: the
    checks that used to reject a fit are "problems", warnings for the user, who alone rejects a
    calibration. Only a run that computes no camera at all (too few poses, degenerate poses, no
    converging fit) is ok False, computed False, and keeps the current calibration (_NotComputed).

    Both a rigid red centroid and four ArUco corners use the same validated
    perspective camera fit. Sessions cannot mix the two marker types. A red-tape
    session with every pose's red blobs on record picks the tape by camera
    consistency (RobustRedAssignment) rather than trusting the live tracker.
    Source "manual" fits the user's clicks on the frozen frames instead
    (SelfCalClicks.py: semi-automatic calibration), with the same fit, outlier
    drop, hold-out, validation and drift baseline.
    """
    with _Lock:
        Session = copy.deepcopy(_Session)
    Observations = Session["observations"]
    Assignment = None
    Stamp = None
    # Quality problems found before the fit (few poses, the tape's identity); the fit adds its own.
    Early: list[str] = []
    DepthIntrinsics = Session.get("depth_intrinsics")
    if DepthIntrinsics is not None and Session.get("image_size") != [DepthIntrinsics["width"], DepthIntrinsics["height"]]:
        DepthIntrinsics = None  # frames were resized; the factory intrinsics no longer apply
    if Source == "manual":
        import SelfCalClicks  # local import: SelfCalClicks imports this module
        Observations = SelfCalClicks.ClickObservations(Session)
        Assignment = SelfCalClicks.ClickSummary(Session)
        Stamp = {"observation_source": "manual", "click_target": SelfCalClicks.ClickTarget}
        if len(Observations) < MinSolvableObservations:
            return _NotComputed(f"only {len(Observations)} poses have a click; at least {MinSolvableObservations} are needed to "
                                f"compute a camera (click {SelfCalClicks.ClickTargetText} on more poses)",
                                observations=len(Observations), tape_assignment=Assignment, observation_source="manual")
    elif Session.get("red_poses") and all(len(O["corners"]) == 1 for O in Observations) and Session.get("image_size"):
        Assigned, Assignment = RobustRedAssignment(Session["red_poses"], Session["image_size"], Seed, DepthIntrinsics)
        if Assigned is not None:
            Tracked = {O["pose_index"]: O["corners"][0] for O in Observations}
            Assignment["changed_from_tracker"] = sum(
                1 for O in Assigned if O["pose_index"] not in Tracked
                or np.linalg.norm(np.array(Tracked[O["pose_index"]]) - O["corners"][0]) > RedAgreePx)
            Observations = Assigned
            if len(Assigned) < MinObservations:
                Early.append(f"only {len(Assigned)} poses had the tip tape in view ({MinObservations} or more recommended)"
                             if len(Assigned) == Assignment["poses_with_red"] else
                             f"only {len(Assigned)} of {Assignment['poses_with_red']} poses with red in view agree on one camera "
                             f"({MinObservations} or more recommended): check that the tape, not other red, was fitted")
        elif len(Observations) >= MinSolvableObservations:
            Early.append(f"the tip tape could not be picked by camera consistency (only {Assignment['assigned']} of "
                         f"{Assignment['poses_with_red']} poses with red in view agree on one camera), so the live tracker's "
                         "blobs were fitted: check that they followed the tape")
    if len(Observations) < MinSolvableObservations:
        Reason = f"only {len(Observations)} valid tip poses; at least {MinSolvableObservations} are needed to compute a camera"
        if Assignment is not None and Assignment.get("poses_with_red", 0) < MinSolvableObservations:
            Reason = (f"only {Assignment['poses_with_red']} poses had red in view; at least {MinSolvableObservations} are needed "
                      "to compute a camera (keep the tip tape in view and matte)")
        elif Assignment is not None:
            Reason = (f"only {Assignment['assigned']} of {Assignment['poses_with_red']} poses with red in view agree on one "
                      f"camera; at least {MinSolvableObservations} are needed to compute one (keep the tip tape in view and matte, "
                      "and cover other red objects)")
        return _NotComputed(Reason, observations=len(Observations), tape_assignment=Assignment)
    if len(Observations) < MinObservations and not any("recommended)" in Problem for Problem in Early):
        Early.append(f"only {len(Observations)} {'clicked ' if Stamp else ''}poses could be used ({MinObservations} or more recommended)")
    if len({len(O["corners"]) for O in Observations}) != 1:
        return _NotComputed("the session mixes marker types (reset the calibration and run it again)")
    ImageSize = Session["image_size"]
    TableZ = DefaultTableZCm if TableZ is None else float(TableZ)
    Tips = np.array([O["tip"] for O in Observations], dtype=np.float64)
    Frames = np.array([O["frame"] for O in Observations], dtype=np.float64)
    Observed = np.array([O["corners"] for O in Observations], dtype=np.float64)
    if not np.isfinite(Tips).all() or not np.isfinite(Observed).all():
        return _NotComputed("the observations hold non-finite numbers")
    Distinct = len(np.unique(np.round(Observed.mean(axis=1), 0), axis=0))
    if Distinct < MinSolvableObservations:
        return _NotComputed(f"the detected blob stayed stationary (at {Distinct} pixel position(s) over {len(Observations)} poses), "
                            "so it was not the moving tip")
    if Distinct < min(MinObservations, len(Observations)):
        Early.append(f"the detected blob was at only {Distinct} distinct pixel positions over {len(Observations)} poses: "
                     "it may not be the moving tip")
    if np.linalg.matrix_rank(Tips - Tips.mean(axis=0), tol=0.1) < 3:
        return _NotComputed("the tip poses do not span three dimensions (a 3D camera fit needs poses at different heights "
                            "and pan angles)")
    Rng = np.random.default_rng(Seed)
    PoseIndices = [O.get("pose_index") for O in Observations]
    Warnings: list[str] = []
    if DepthIntrinsics is not None and Observed.shape[1] == 1:
        CameraPoints = np.array([O.get("camera_cm") if O.get("camera_cm") is not None else [np.nan] * 3
                                 for O in Observations], dtype=np.float64)
        WithDepth = int(np.isfinite(CameraPoints).all(axis=1).sum())
        What = "clicks" if Stamp else "tape poses"
        if WithDepth >= MinSolvableObservations:
            if WithDepth < min(MinObservations, len(Observations)):
                Early.append(f"only {WithDepth} of {len(Observations)} {What} have depth under them ({MinObservations} or more "
                             "recommended)" + (": click the middle of the red tape face, not its edge" if Stamp else ""))
            Areas = np.array([O.get("area_px") if O.get("area_px") is not None else np.nan for O in Observations], dtype=np.float64)
            return _SolveDepth(Session, Tips, Frames, Observed[:, 0, :], CameraPoints, DepthIntrinsics,
                               Assignment, TableZ, Save, Rng, Areas, PoseIndices=PoseIndices, Stamp=Stamp, EarlyProblems=Early)
        # Too few points with depth for the depth fit: the camera is fitted from colour alone (no factory
        # intrinsics) and has no drift baseline, which the depth fit makes.
        Early.append(f"only {WithDepth} of {len(Observations)} {What} have depth under them (at least {MinSolvableObservations} "
                     f"needed for the depth camera's fit), so the camera was fitted from colour alone and {NoBaselineEffect}"
                     + (". Click the middle of the red tape face, not its edge" if Stamp else ""))
    else:
        Warnings.append("a colour-only calibration has no camera-check baseline (that needs the depth camera's tape fit), so "
                        "tasks that act on an object stop (camera not verified)")
    AllTips, AllFrames, AllObserved = Tips, Frames, Observed

    # 1. Both handedness hypotheses; keep the better one.
    Hypotheses = [H for H in (_FitHypothesis(Tips, Frames, Observed, ImageSize, M) for M in (False, True)) if H]
    if not Hypotheses:
        return _NotComputed("no camera hypothesis converged (are the poses too similar?)")
    Hypotheses.sort(key=lambda H: H["rms"])
    Best = Hypotheses[0]
    OtherRms = Hypotheses[1]["rms"] if len(Hypotheses) > 1 else None
    Mirror = Best["mirror"]
    # A mirrored world only fits a real camera sitting below the table or with
    # a focal length pinned at its bound; such a runner-up leaves no doubt.
    OtherPlausible = False
    if OtherRms is not None:
        Other = Hypotheses[1]["params"]
        OtherRotation, _ = cv2.Rodrigues(np.asarray(Other[5:8], dtype=np.float64))
        OtherCentre = -OtherRotation.T @ Other[8:11]
        TableZForCheck = DefaultTableZCm if TableZ is None else float(TableZ)
        OtherPlausible = bool(OtherCentre[2] >= TableZForCheck + MinCameraHeightAboveTableCm
                              and Other[0] < 0.98 * _Bounds(ImageSize)[1][0])

    # 2. Drop bad detections and refit.
    Errors, _ = _CornerErrors(Best["params"], Tips, Frames, Observed, Mirror)
    WorstPerObservation = Errors.max(axis=1)
    Threshold = max(OutlierFloorPx, OutlierFactor * float(np.median(WorstPerObservation)))
    Keep = WorstPerObservation <= Threshold
    Dropped = [int(i) for i in np.where(~Keep)[0]]
    if Keep.sum() >= MinObservations and Dropped:
        Tips, Frames, Observed = Tips[Keep], Frames[Keep], Observed[Keep]
        Params = _Refine(Best["params"], Tips, Frames, Observed, Mirror, ImageSize, PriorScale=Best["prior_scale"])
    else:
        Dropped = []
        Params = Best["params"]
    Errors, Depth = _CornerErrors(Params, Tips, Frames, Observed, Mirror)
    FitRms = float(np.sqrt(np.mean(Errors ** 2)))
    FitRmsCm = float(np.sqrt(np.mean((Errors * Depth / Params[0]) ** 2)))

    # 3. Hold-out: five folds, each refined from the full solution without it.
    Count = len(Tips)
    Folds = np.arange(Count) % 5
    HoldoutErrors, HoldoutErrorsCm = [], []
    for Fold in range(5):
        Train, Test = Folds != Fold, Folds == Fold
        if Test.sum() == 0 or Train.sum() < MinSolvableObservations:
            continue
        FoldParams = _Refine(Params, Tips[Train], Frames[Train], Observed[Train], Mirror, ImageSize, MaxEvaluations=150,
                             PriorScale=Best["prior_scale"])
        FoldErrors, FoldDepth = _CornerErrors(FoldParams, Tips[Test], Frames[Test], Observed[Test], Mirror)
        HoldoutErrors.append(FoldErrors.ravel())
        HoldoutErrorsCm.append((FoldErrors * np.abs(FoldDepth) / FoldParams[0]).ravel())
    HoldoutRms = float(np.sqrt(np.mean(np.concatenate(HoldoutErrors) ** 2))) if HoldoutErrors else None
    HoldoutRmsCm = float(np.sqrt(np.mean(np.concatenate(HoldoutErrorsCm) ** 2))) if HoldoutErrorsCm else None

    Model = _ModelFromParams(Params, Mirror, ImageSize, Session["mirrored"], TableZ)
    Rotation, _ = cv2.Rodrigues(np.array(Model["rvec"]))
    CameraCenter = -Rotation.T @ np.array(Model["tvec"])
    if Mirror:
        CameraCenter = CameraCenter * MirrorVector
    PlaneRms = _PlaneAccuracy(Model, Tips, FitRms, TableZ, Rng)

    Q = np.array(Model["marker_corners_gripper_cm"])
    Edges = np.linalg.norm(Q - np.roll(Q, 1, axis=0), axis=1)
    EdgeMeanMm = float(np.mean(Edges) * 10.0)
    EdgeSpread = float(np.std(Edges) / max(np.mean(Edges), 1e-9))

    ObservedPx = Observed.reshape(-1, 2)
    Model.update({
        "observed_px_bounds": [float(ObservedPx[:, 0].min()), float(ObservedPx[:, 1].min()),
                               float(ObservedPx[:, 0].max()), float(ObservedPx[:, 1].max())],
        "fit_rms_px": FitRms,
        "holdout_rms_px": HoldoutRms,
        "plane_rms_cm": PlaneRms,
        "observations_used": int(Count),
        "fitted_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        **(Stamp or {}),
    })
    AllPixels, AllDepth = _ProjectCorners(Params, AllTips, AllFrames, Mirror)
    AllErrors = np.linalg.norm(AllPixels - AllObserved, axis=-1).max(axis=1)
    PerPose = _PerPoseReport(PoseIndices, AllObserved.mean(axis=1), AllPixels.mean(axis=1), AllErrors,
                             AllErrors * np.abs(AllDepth.mean(axis=1)) / Params[0], np.full(len(AllErrors), np.nan), set(Dropped))

    # Quality problems: warnings on a calibration that is kept (_Conclude), never a rejection.
    Problems: list[str] = list(Early)
    if FitRmsCm > MaxFitRmsCm:
        Problems.append(f"fit error {FitRmsCm:.2f} cm at the marker ({FitRms:.1f} px) exceeds {MaxFitRmsCm} cm")
    if HoldoutRmsCm is None:
        Problems.append(f"no hold-out check: {Count} poses are too few to hold any out")
    elif HoldoutRmsCm > MaxHoldoutRmsCm:
        Problems.append(f"hold-out error {HoldoutRmsCm:.2f} cm at the marker ({HoldoutRms:.1f} px) exceeds {MaxHoldoutRmsCm} cm")
    if PlaneRms is None:
        Problems.append("the table area is not visible in the fitted camera")
    elif PlaneRms > MaxPlaneRmsCm:
        Problems.append(f"expected table accuracy {PlaneRms:.2f} cm exceeds {MaxPlaneRmsCm} cm")
    if CameraCenter[2] < TableZ + MinCameraHeightAboveTableCm:
        Problems.append(f"camera solved {CameraCenter[2] - TableZ:.1f} cm above the table, which is implausible")
    if len(Q) > 1 and (not (0.5 <= EdgeMeanMm / 10.0 <= 12.0) or EdgeSpread > 0.25):
        Warnings.append(f"marker corners solved as a {EdgeMeanMm:.0f} mm, {EdgeSpread * 100:.0f}% irregular shape")
    if OtherPlausible and OtherRms < 2.0 * FitRms:
        Warnings.append("handedness was not clearly determined; add pan variety")
    if Dropped:
        Warnings.append(f"dropped {len(Dropped)} inconsistent detection(s)")

    Report = {
        "ok": True,
        "saved": False,
        "problems": Problems,
        "warnings": Warnings,
        "fit_rms_px": round(FitRms, 3),
        "holdout_rms_px": None if HoldoutRms is None else round(HoldoutRms, 3),
        "fit_rms_cm": round(FitRmsCm, 3),
        "holdout_rms_cm": None if HoldoutRmsCm is None else round(HoldoutRmsCm, 3),
        "plane_rms_cm": None if PlaneRms is None else round(PlaneRms, 3),
        "observations_used": int(Count),
        "observations_dropped": len(Dropped),
        "tape_assignment": Assignment,
        "mirror_x": bool(Mirror),
        "other_handedness_rms_px": None if OtherRms is None else round(OtherRms, 2),
        "focal_px": round(Model["f"], 1),
        "principal_point_px": [round(Model["cx"], 1), round(Model["cy"], 1)],
        "distortion": [round(v, 4) for v in Model["dist"][:2]],
        "camera_position_cm": [round(float(v), 1) for v in CameraCenter],
        "camera_height_above_table_cm": round(float(CameraCenter[2] - TableZ), 1),
        "marker_edge_mm": round(EdgeMeanMm, 1),
        "table_z": TableZ,
        "per_pose": PerPose,
        "observation_source": (Stamp or {}).get("observation_source", "auto"),
        "model": Model,
    }
    return _Conclude(Report, Model, Problems, Count, Stamp, Observed.shape[1] == 1, Save, Session)


def _RmsOf(Chunks: list[np.ndarray]) -> float | None:
    if not Chunks:
        return None
    Values = np.concatenate(Chunks)
    Values = Values[np.isfinite(Values)]
    return float(np.sqrt(np.mean(Values ** 2))) if Values.size else None


def _DriftBaseline(Session: dict[str, Any], Model: dict[str, Any], TapeAreasCm2=None) -> list[str]:
    """Record the fit-time desk planes in Model and save its drift baseline; returns quality problems.

    The 15:04 fit on 2026-09-28 ran while the camera rotated 0.3 deg and nothing
    noticed; a camera that moves during the fit is a problem here, and so is a fit with
    no baseline, because /locate refuses (camera_not_verified) any calibration without
    one, and a fit whose stillness could not be checked (the desk found at too few poses).
    Problems are warnings on a calibration that is still saved (_Conclude): only the
    user rejects one. TapeAreasCm2, the tape's area at each fitted pose, lets /locate
    tell the tape from other red.

    The planes recorded at each pose were found before any camera was fitted; one far from the
    fitted camera's own desk is searched for again near it (CameraDrift.DeskPlanesNearFit: on
    2026-10-01 every frame had locked onto a bookshelf's side). Stillness is then measured on the
    desk's own cells against the first third's map (CameraDrift.MotionFromDeskMap), falling back
    to per-frame planes (MotionDuringFit) when the raw depth is not on disk.
    """
    Intrinsics = Session.get("depth_intrinsics")
    Recorded = Session.get("desk_planes") or []
    Planes, Redetected = CameraDrift.DeskPlanesNearFit(Recorded, Model, Intrinsics)
    Model["fit_desk_planes_redetected"] = {"recorded": len(Recorded), "kept": len(Planes) - Redetected,
                                           "found_again": Redetected, "dropped": len(Recorded) - len(Planes)}
    Servos = {Name: [P[f"{Name}_raw"] for P in Planes if f"{Name}_raw" in P] for Name in ("wrist_roll", "gripper")}
    TapePose = {f"{Name}_raw": int(np.median(Values)) for Name, Values in Servos.items() if Values}
    Reference, Problem = CameraDrift.BuildReference(Planes, Model, TapePose or None,
                                                    Keyframe=Planes[-1].get("archive") if Planes else None,
                                                    Intrinsics=Intrinsics, TapeAreasCm2=TapeAreasCm2)
    Problems = []
    Motion = (CameraDrift.MotionFromDeskMap(Planes, Reference["desk_n"], Reference["desk_d"], Intrinsics)
              if Reference is not None else None) or CameraDrift.MotionDuringFit(Planes)
    Model["fit_camera_motion"] = Motion
    if Motion is None:
        Poses = len(CameraDrift.PoseAverages(Planes))
        Problems.append(f"camera stillness could not be verified: the desk was found in depth at only {Poses} poses "
                        f"(need {CameraDrift.MinFitMotionPoses}), so a camera that moved during the run would go unseen; "
                        "keep the desk in view and recalibrate")
    elif Motion["moved"]:
        Problems.append(f"the camera moved {Motion['angle_deg']:.2f} deg / {Motion['offset_cm']:+.2f} cm during calibration "
                        f"(desk in depth, first against last third of the poses; limits {CameraDrift.MaxFitMotionDeg} deg / "
                        f"{CameraDrift.MaxFitMotionCm} cm): lock the camera head and recalibrate")
    if Problem:
        Problems.append(f"{Problem}: {NoBaselineEffect}")
    else:
        Model["drift_reference"] = Reference
        Model["fit_desk_planes"] = [{"pose_index": Pose["pose_index"], "frames": Pose["frames"], "d": round(Pose["d"], 3),
                                     "angle_to_baseline_deg": round(CameraDrift.AngleDeg(Pose["n"], Reference["desk_n"]), 3)}
                                    for Pose in CameraDrift.PoseAverages(Planes)]
    return Problems


def _PerPoseReport(PoseIndices, Observed, Predicted, ErrorsPx, ErrorsCm, DepthErrorsCm, Dropped) -> list[dict[str, Any]]:
    """Every observation's residual under the final camera, in input order, for the app to point at.

    flagged: dropped by the solver, or off by more than the solver's own outlier limits
    (OutlierFactor x the kept poses' median, floored) under the final fit.
    """
    ErrorsPx, DepthErrorsCm = np.asarray(ErrorsPx, dtype=np.float64), np.asarray(DepthErrorsCm, dtype=np.float64)
    Kept = np.array([Index not in Dropped for Index in range(len(ErrorsPx))], dtype=bool)
    PixelLimit = max(2 * OutlierFloorPx, OutlierFactor * float(np.median(ErrorsPx[Kept] if Kept.any() else ErrorsPx)))
    Finite = DepthErrorsCm[Kept & np.isfinite(DepthErrorsCm)]
    DepthLimit = max(DepthOutlierFloorCm, OutlierFactor * float(np.median(Finite))) if Finite.size else np.inf
    Rows = []
    for Index, PoseIndex in enumerate(PoseIndices):
        Depth = float(DepthErrorsCm[Index]) if np.isfinite(DepthErrorsCm[Index]) else None
        Reasons = (["dropped by the fit"] if Index in Dropped else []) + (
            [f"{ErrorsPx[Index]:.1f} px off"] if ErrorsPx[Index] > PixelLimit else []) + (
            [f"depth {Depth:.1f} cm off"] if Depth is not None and Depth > DepthLimit else [])
        Rows.append({"pose_index": None if PoseIndex is None else int(PoseIndex),
                     "observed_px": [round(float(Value), 2) for Value in Observed[Index]],
                     "predicted_px": [round(float(Value), 2) for Value in Predicted[Index]],
                     "residual_px": round(float(ErrorsPx[Index]), 2), "residual_cm": round(float(ErrorsCm[Index]), 3),
                     "depth_error_cm": None if Depth is None else round(Depth, 3),
                     "dropped": Index in Dropped, "flagged": bool(Reasons), "reason": "; ".join(Reasons) or None})
    return Rows


def _SolveDepth(Session, Tips, Frames, Pixels, CameraPoints, Intrinsics, Assignment, TableZ, Save, Rng,
                Areas=None, PoseIndices=None, Stamp=None, EarlyProblems=None) -> dict[str, Any]:
    """Solve with the depth camera: factory intrinsics fixed, pose and tape offset fitted.

    Pixels are native tape centroids and CameraPoints their depth-measured
    camera points (NaN rows where depth was missing); Areas the tape blobs' pixel
    areas. The quality checks mirror the colour-only fit, plus a limit on the 3D
    disagreement with depth and the drift baseline; like the colour fit, every
    computed camera is kept with its problems as warnings (_Conclude). PoseIndices
    name the rows in the per-pose report; Stamp is merged into the model (the
    semi-automatic run's source); EarlyProblems are Solve's, found before the fit.
    """
    Areas = np.full(len(Tips), np.nan) if Areas is None else np.asarray(Areas, dtype=np.float64)
    PoseIndices = list(range(len(Tips))) if PoseIndices is None else list(PoseIndices)
    AllTips, AllFrames, AllPixels, AllCameraPoints = Tips, Frames, Pixels, CameraPoints
    Hypotheses = [H for H in (DepthGeometry.FitDepthCamera(Tips, Frames, Pixels, CameraPoints, Intrinsics, Mirror)
                              for Mirror in (False, True)) if H]
    if not Hypotheses:
        return _NotComputed("no camera pose converged (are the poses too similar?)",
                            observation_source=(Stamp or {}).get("observation_source", "auto"))
    Hypotheses.sort(key=lambda H: H["cost"])
    Best, Other = Hypotheses[0], (Hypotheses[1] if len(Hypotheses) > 1 else None)
    Mirror = Best["mirror"]

    # Drop inconsistent detections (a wrong blob, or depth read off the tape) and refit.
    PixelLimit = max(2 * OutlierFloorPx, OutlierFactor * float(np.median(Best["pixel_errors"])))
    DepthErrors = Best["depth_errors_cm"]
    DepthLimit = (max(DepthOutlierFloorCm, OutlierFactor * float(np.nanmedian(DepthErrors)))
                  if np.isfinite(DepthErrors).any() else np.inf)
    Keep = (Best["pixel_errors"] <= PixelLimit) & ~(np.nan_to_num(DepthErrors, nan=0.0) > DepthLimit)
    Dropped = [int(Index) for Index in np.flatnonzero(~Keep)]
    Refit = None
    if Dropped and Keep.sum() >= MinObservations:
        Refit = DepthGeometry.FitDepthCamera(Tips[Keep], Frames[Keep], Pixels[Keep], CameraPoints[Keep], Intrinsics,
                                             Mirror, Initial=Best["params"])
    if Refit is not None:
        Tips, Frames, Pixels, CameraPoints, Best = Tips[Keep], Frames[Keep], Pixels[Keep], CameraPoints[Keep], Refit
        Areas = Areas[Keep]
    else:
        Dropped = []
    Params = Best["params"]

    # Hold-out: five folds, each refined from the full solution without it.
    Count = len(Tips)
    Folds = np.arange(Count) % 5
    HoldPx, HoldCm, HoldDepth = [], [], []
    for Fold in range(5):
        Train, Test = Folds != Fold, Folds == Fold
        if Test.sum() == 0 or Train.sum() < MinSolvableObservations:
            continue
        Fit = DepthGeometry.FitDepthCamera(Tips[Train], Frames[Train], Pixels[Train], CameraPoints[Train], Intrinsics,
                                           Mirror, Initial=Params, MaxEvaluations=150)
        if Fit is None:
            continue
        Px, Cm, Depth, _ = DepthGeometry.PoseErrors(Fit["params"], Tips[Test], Frames[Test], Pixels[Test],
                                                    CameraPoints[Test], Intrinsics, Mirror)
        HoldPx.append(Px)
        HoldCm.append(Cm)
        HoldDepth.append(Depth)
    HoldoutRms, HoldoutRmsCm, HoldoutDepthCm = _RmsOf(HoldPx), _RmsOf(HoldCm), _RmsOf(HoldDepth)

    Rotation, _ = cv2.Rodrigues(np.asarray(Params[:3], dtype=np.float64))
    CameraCenter = -Rotation.T @ Params[3:6]
    if Mirror:
        CameraCenter = CameraCenter * MirrorVector
    Width, Height = int(Intrinsics["width"]), int(Intrinsics["height"])
    Model = {
        "type": "pinhole_depth_v1",
        "f": 0.5 * (float(Intrinsics["fx"]) + float(Intrinsics["fy"])),
        "fx": float(Intrinsics["fx"]), "fy": float(Intrinsics["fy"]),
        "cx": float(Intrinsics["cx"]), "cy": float(Intrinsics["cy"]),
        "dist": [float(Value) for Value in DepthGeometry.Distortion(Intrinsics)],
        "rvec": [float(Value) for Value in Params[:3]], "tvec": [float(Value) for Value in Params[3:6]],
        "mirror_x": bool(Mirror),
        "image_size": [Width, Height],
        "input_mirrored": bool(Session["mirrored"]),
        "table_z": float(TableZ),
        "marker_corners_gripper_cm": [[float(Value) for Value in Params[6:9]]],
        "intrinsics_source": Intrinsics.get("source"),
        "observed_px_bounds": [float(Pixels[:, 0].min()), float(Pixels[:, 1].min()),
                               float(Pixels[:, 0].max()), float(Pixels[:, 1].max())],
        # Where the tips were (display frame): /locate trusts depth-measured objects near it.
        **TipRegion(Tips),
        "fit_rms_px": Best["rms_px"],
        "holdout_rms_px": HoldoutRms,
        "depth_rms_cm": Best["depth_rms_cm"],
        "observations_used": int(Count),
        "fitted_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        **(Stamp or {}),
    }
    AllPx, AllCm, AllDepth, AllPredicted = DepthGeometry.PoseErrors(Params, AllTips, AllFrames, AllPixels, AllCameraPoints,
                                                                   Intrinsics, Mirror)
    PerPose = _PerPoseReport(PoseIndices, AllPixels, DepthGeometry.Project(AllPredicted, Intrinsics), AllPx, AllCm, AllDepth,
                             set(Dropped))
    PlaneRms = _PlaneAccuracy(Model, Tips, Best["rms_px"], TableZ, Rng)
    Model["plane_rms_cm"] = PlaneRms
    # Elevation of the distal link (the gripper frame's x axis) per used pose.
    Tilts = np.degrees(np.arcsin(np.clip(Frames[:, 2, 0], -1.0, 1.0)))
    TiltSpread = float(np.std(Tilts))
    Model["gripper_tilt_spread_deg"] = round(TiltSpread, 1)

    # Quality problems: warnings on a calibration that is kept (_Conclude), never a rejection.
    Problems: list[str] = list(EarlyProblems or [])
    Warnings: list[str] = []
    if TiltSpread < MinGripperTiltSpreadDeg:
        Problems.append(f"the gripper tilt barely varied across poses ({TiltSpread:.1f} deg spread), so the camera height "
                        "cannot be separated from the tape offset; calibrate with more than one wrist pitch")
    if Best["rms_cm"] > MaxFitRmsCm:
        Problems.append(f"fit error {Best['rms_cm']:.2f} cm at the marker ({Best['rms_px']:.1f} px) exceeds {MaxFitRmsCm} cm")
    if HoldoutRmsCm is None:
        Problems.append(f"no hold-out check: {Count} poses are too few to hold any out, or no held-out fit converged")
    elif HoldoutRmsCm > MaxHoldoutRmsCm:
        Problems.append(f"hold-out error {HoldoutRmsCm:.2f} cm at the marker ({HoldoutRms:.1f} px) exceeds {MaxHoldoutRmsCm} cm")
    if Best["depth_rms_cm"] is not None and Best["depth_rms_cm"] > MaxDepthRmsCm:
        Problems.append(f"depth disagreement {Best['depth_rms_cm']:.2f} cm exceeds {MaxDepthRmsCm} cm")
    if PlaneRms is None:
        Problems.append("the table area is not visible in the fitted camera")
    elif PlaneRms > MaxPlaneRmsCm:
        Problems.append(f"expected table accuracy {PlaneRms:.2f} cm exceeds {MaxPlaneRmsCm} cm")
    if CameraCenter[2] < TableZ + MinCameraHeightAboveTableCm:
        Problems.append(f"camera solved {CameraCenter[2] - TableZ:.1f} cm above the table, which is implausible")
    if Other is not None and Other["cost"] < 2.0 * Best["cost"]:
        Warnings.append("handedness was not clearly determined; add pan variety")
    if Dropped:
        Warnings.append(f"dropped {len(Dropped)} inconsistent detection(s)")
    # The tape's physical area at each kept pose (pixel area at its measured depth).
    Problems.extend(_DriftBaseline(Session, Model, Areas * (CameraPoints[:, 2] / float(Intrinsics["fx"])) ** 2))
    Missing = int((~np.isfinite(CameraPoints).all(axis=1)).sum())
    if Missing:
        Warnings.append(f"{Missing} pose(s) had no depth on the tape")

    Report = {
        "ok": True,
        "saved": False,
        "method": "depth",
        "problems": Problems,
        "warnings": Warnings,
        "fit_rms_px": round(Best["rms_px"], 3),
        "holdout_rms_px": None if HoldoutRms is None else round(HoldoutRms, 3),
        "fit_rms_cm": round(Best["rms_cm"], 3),
        "holdout_rms_cm": None if HoldoutRmsCm is None else round(HoldoutRmsCm, 3),
        "depth_rms_cm": None if Best["depth_rms_cm"] is None else round(Best["depth_rms_cm"], 3),
        "holdout_depth_rms_cm": None if HoldoutDepthCm is None else round(HoldoutDepthCm, 3),
        "plane_rms_cm": None if PlaneRms is None else round(PlaneRms, 3),
        "observations_used": int(Count),
        "observations_dropped": len(Dropped),
        "tape_assignment": Assignment,
        "mirror_x": bool(Mirror),
        "focal_px": round(Model["f"], 1),
        "principal_point_px": [round(Model["cx"], 1), round(Model["cy"], 1)],
        "camera_position_cm": [round(float(Value), 1) for Value in CameraCenter],
        "camera_height_above_table_cm": round(float(CameraCenter[2] - TableZ), 1),
        "tape_offset_cm": [round(float(Value), 2) for Value in Params[6:9]],
        "table_z": TableZ,
        "fit_camera_motion": Model.get("fit_camera_motion"),
        "drift_baseline": "drift_reference" in Model,
        "per_pose": PerPose,
        "observation_source": (Stamp or {}).get("observation_source", "auto"),
        "model": Model,
    }
    return _Conclude(Report, Model, Problems, Count, Stamp, True, Save, Session)


def PixelToPlane(Model: dict[str, Any], U: float, V: float, Mirrored: bool | None = None) -> dict[str, Any]:
    """Map a capture-space pixel (continuous) to the display-frame table plane.

    Mirrored says whether this capture was flipped; by default the capture
    orientation the camera was calibrated with.
    """
    Width, Height = Model["image_size"]
    Flipped = Model["input_mirrored"] if Mirrored is None else bool(Mirrored)
    UContinuous = (Width - float(U)) if Flipped else float(U)
    UNative = UContinuous - 0.5
    VNative = float(V) - 0.5
    Points, Valid = _RayToPlane(Model, np.array([UNative]), np.array([VNative]), Model["table_z"])
    if not Valid[0]:
        return {"ok": False, "error": "That pixel's ray does not reach the table."}
    Bounds = Model.get("observed_px_bounds")
    Extrapolated = False
    if Bounds:
        UMin, VMin, UMax, VMax = Bounds
        MarginU = max(40.0, (UMax - UMin) * 0.25)
        MarginV = max(40.0, (VMax - VMin) * 0.25)
        Extrapolated = not (UMin - MarginU <= UNative <= UMax + MarginU and VMin - MarginV <= VNative <= VMax + MarginV)
    return {
        "ok": True,
        "x": float(Points[0][0]),
        "y": float(Points[0][1]),
        "z": float(Model["table_z"]),
        "extrapolated": Extrapolated,
    }


def _HullXY(Points: np.ndarray) -> list[tuple[float, float]]:
    """Convex hull of 2D points, counterclockwise (monotone chain); 1-2 points when they are degenerate."""
    Unique = sorted({(float(X), float(Y)) for X, Y in Points})
    if len(Unique) < 3:
        return Unique

    def Turn(O, A, B):
        return (A[0] - O[0]) * (B[1] - O[1]) - (A[1] - O[1]) * (B[0] - O[0])

    Lower: list[tuple[float, float]] = []
    Upper: list[tuple[float, float]] = []
    for Point in Unique:
        while len(Lower) >= 2 and Turn(Lower[-2], Lower[-1], Point) <= 0:
            Lower.pop()
        Lower.append(Point)
    for Point in reversed(Unique):
        while len(Upper) >= 2 and Turn(Upper[-2], Upper[-1], Point) <= 0:
            Upper.pop()
        Upper.append(Point)
    Hull = Lower[:-1] + Upper[:-1]
    return Hull if len(Hull) >= 3 else [Unique[0], Unique[-1]]


def TipRegion(Tips: np.ndarray) -> dict[str, Any]:
    """The calibrated tips' extent in the display frame, stored with the fit.

    observed_tip_bounds_cm is {"x": [min, max], "y": ..., "z": ...}; observed_tip_hull_cm the
    XY convex hull, counterclockwise. Rounded to 0.01 mm.
    """
    Tips = np.asarray(Tips, dtype=np.float64).reshape(-1, 3)
    return {
        "observed_tip_bounds_cm": {Axis: [round(float(Tips[:, Index].min()), 3), round(float(Tips[:, Index].max()), 3)]
                                   for Index, Axis in enumerate("xyz")},
        "observed_tip_hull_cm": [[round(X, 3), round(Y, 3)] for X, Y in _HullXY(Tips[:, :2])],
    }


def TipRegionDistance(Model: dict[str, Any] | None, X: float, Y: float) -> float | None:
    """How far (cm, XY, display frame) a point lies outside the calibrated tips' hull; 0 inside.

    None when the model stores no tip region (fits before 2026-09-29 that were not backfilled).
    """
    Hull = (Model or {}).get("observed_tip_hull_cm")
    if not Hull:
        return None
    Point = np.array([float(X), float(Y)])
    if not np.isfinite(Point).all():
        return math.inf
    Starts = np.asarray(Hull, dtype=np.float64).reshape(-1, 2)
    Ends = np.roll(Starts, -1, axis=0)
    Edges = Ends - Starts
    if len(Starts) >= 3:
        Left = Edges[:, 0] * (Point[1] - Starts[:, 1]) - Edges[:, 1] * (Point[0] - Starts[:, 0])
        if (Left >= -1e-9).all():
            return 0.0
    Lengths = np.maximum((Edges ** 2).sum(axis=1), 1e-12)
    Along = np.clip(((Point - Starts) * Edges).sum(axis=1) / Lengths, 0.0, 1.0)
    return float(np.min(np.linalg.norm(Starts + Along[:, None] * Edges - Point, axis=1)))
