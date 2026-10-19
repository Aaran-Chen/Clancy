"""Semi-automatic camera calibration: the user clicks the tape on each pose's frozen frame.

Why
---
The automatic run (SelfCalibration.py) finds the red tape on the fixed jaw by itself, and on
the hardware it misses it or picks another red blob when the open jaw hides the tape, under
shadow, or next to other red things; its fit then absorbs 1-2 cm. Here the arm still visits
every planned pose on its own (same plan, motion and stillness checks as the automatic run),
and /selfcal/observe archives each pose's frames and joints exactly as before. The app then
shows the frozen frame and the user clicks the point; that click replaces the blob search.

What to click: the CENTRE OF THE RED TAPE on the FIXED jaw (ClickTarget). The fit's tape
offset (marker_corners_gripper_cm) keeps the meaning it has for automatic runs, so the drift
check's tape test (CameraDrift.TapeCheck), the tape-area baseline and the grasp planner's
tape correction keep working, and depth on the flat tape face is reliable where a fingertip
would sit on a depth edge.

Each click is stored against the archived capture it was made on (the observation JSON that
/selfcal/observe wrote in this run's directory): the frame's own joint pose (frame_metadata)
gives the gripper frame, the frame's archived 16-bit depth PNG gives depth under the clicked
pixel, and the red blob containing the click, if any, gives the tape's area for the drift
baseline. Clicks arrive in continuous capture pixels (mirrored or not, like /calibration/probe)
and are stored in native OpenCV pixels. Nothing here moves the arm.

Session keys (SelfCalibration._Session, persisted to the run's Session.json):
  manual_clicks       {str(pose_index): click record}, one per pose; a re-click replaces it
  manual_click_order  pose indices, oldest click first (Undo removes the newest)
  manual_skips        pose indices the user skipped
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

import DepthGeometry
import SelfCalibration as SC

ClickTarget = "tape_centre"
ClickTargetText = "the centre of the red tape on the fixed jaw"
# A click on or within this many native pixels of a red blob names that blob (its area feeds the
# drift baseline's tape-area range, as the fitted blobs' areas do in an automatic run).
BlobNearPx = 4.0
# Depth under the click: the near cluster of a (2r+1)^2 window, as /depthcam/point reads it.
DepthWindowRadius = 3


def _Number(Value: Any, Name: str) -> float:
    if isinstance(Value, bool) or not isinstance(Value, (int, float)) or not math.isfinite(float(Value)):
        raise ValueError(f"{Name} must be a finite number")
    return float(Value)


def _Index(Value: Any, Name: str) -> int:
    if isinstance(Value, bool) or not isinstance(Value, (int, float)) or float(Value) != int(Value) or int(Value) < 0:
        raise ValueError(f"{Name} must be a whole number of at least 0")
    return int(Value)


def _Plain(Value: Any) -> Any:
    """JSON-safe copy: numpy scalars and arrays to Python, non-finite floats to None."""
    if isinstance(Value, dict):
        return {str(Key): _Plain(Item) for Key, Item in Value.items()}
    if isinstance(Value, (list, tuple, np.ndarray)):
        return [_Plain(Item) for Item in (Value.tolist() if isinstance(Value, np.ndarray) else Value)]
    if isinstance(Value, (np.floating, float)):
        return float(Value) if math.isfinite(float(Value)) else None
    if isinstance(Value, np.integer):
        return int(Value)
    return Value


def _CaptureFile(CaptureArchive: Any, RunDirectory: str) -> Path:
    """The archived observation a click refers to; only this run's own observation files."""
    if not isinstance(CaptureArchive, str) or not CaptureArchive:
        raise ValueError("capture_archive must name the pose's archived capture")
    Folder = (Path(RunDirectory) / "observations").resolve()
    File = Path(CaptureArchive).resolve()
    if File.parent != Folder or File.suffix.lower() != ".json" or not File.is_file():
        raise ValueError("capture_archive is not an observation of the current calibration run")
    return File


def _NearestBlob(Image: np.ndarray, U: float, V: float) -> dict[str, Any] | None:
    """The red blob containing the native point (U, V), or within BlobNearPx of it."""
    Best = None
    for Option in SC.DetectRedDetailed(Image).get("options", []):
        Distance = float(cv2.pointPolygonTest(Option["contour"], (float(U), float(V)), True))  # > 0 inside
        if Distance >= -BlobNearPx and (Best is None or Distance > Best[0]):
            Best = (Distance, Option)
    if Best is None:
        return None
    Distance, Option = Best
    Centroid = [float(Value) for Value in Option["centroid"]]
    return {"centroid": Centroid, "area": float(Option["area"]), "inside": Distance >= 0,
            "edge_distance_px": round(Distance, 2),  # positive inside the blob
            "click_to_centroid_px": round(math.hypot(Centroid[0] - U, Centroid[1] - V), 2)}


def AddClick(Payload: dict[str, Any]) -> dict[str, Any]:
    """Record the user's click on one pose's frozen frame; a second click on a pose replaces the first.

    Payload: pose_index, capture_archive (from /selfcal/observe), frame_index (which of that
    pose's frames was shown), u, v (continuous capture pixels, 0..width), mirrored (that
    capture's mirroring), image_size [width, height] (optional check), clicked_at (optional).
    """
    try:
        PoseIndex = _Index(Payload.get("pose_index"), "pose_index")
        FrameIndex = _Index(Payload.get("frame_index", 0), "frame_index")
        U, V = _Number(Payload.get("u"), "u"), _Number(Payload.get("v"), "v")
        Mirrored = Payload.get("mirrored")
        if not isinstance(Mirrored, bool):
            raise ValueError("mirrored must be true or false")
        with SC._Lock:
            RunDirectory = SC._Session.get("archive_directory")
        if not RunDirectory:
            return {"ok": False, "error": "No calibration run is recording; start the semi-automatic calibration first."}
        File = _CaptureFile(Payload.get("capture_archive"), RunDirectory)
        Capture = json.loads(File.read_text(encoding="utf-8"))
    except (ValueError, TypeError, OSError, json.JSONDecodeError) as Exc:
        return {"ok": False, "error": f"Bad click: {Exc}"}
    if Capture.get("pose_index") != PoseIndex:
        return {"ok": False, "error": f"That capture belongs to pose {Capture.get('pose_index')}, not pose {PoseIndex}."}
    Metadata = Capture.get("metadata") or {}
    if Metadata.get("capture_valid") is False:
        return {"ok": False, "error": "The arm moved or lost feedback during that capture; retake the pose."}
    if bool(Capture.get("mirrored")) != Mirrored:
        return {"ok": False, "error": "The click's mirroring differs from its capture's."}
    Images = Capture.get("images") or []
    if FrameIndex >= len(Images):
        return {"ok": False, "error": f"That capture has {len(Images)} frames; frame {FrameIndex} does not exist."}
    Image = SC.DecodeDataUrl(Images[FrameIndex])
    if Image is None:
        return {"ok": False, "error": "The archived frame could not be decoded."}
    Height, Width = Image.shape[:2]
    Size = Payload.get("image_size")
    if Size is not None and (not isinstance(Size, (list, tuple)) or [int(Value) for Value in Size[:2]] != [Width, Height]):
        return {"ok": False, "error": f"The clicked image is {Size}, but the archived frame is {Width}x{Height}."}
    if not (0.0 <= U <= Width and 0.0 <= V <= Height):
        return {"ok": False, "error": f"The click ({U:.1f}, {V:.1f}) lies outside the {Width}x{Height} frame."}
    UNative, VNative = (float(Value) for Value in DepthGeometry.NativeFromCapture(U, V, Width, Mirrored))

    # The gripper as it stood for this very frame; the pose's joints as posted otherwise.
    Frames = Metadata.get("frame_metadata") if isinstance(Metadata.get("frame_metadata"), list) else []
    FrameData = Frames[FrameIndex] if FrameIndex < len(Frames) and isinstance(Frames[FrameIndex], dict) else {}
    Joints, JointSource = FrameData.get("joints"), "frame_metadata"
    if not (isinstance(Joints, list) and len(Joints) >= 3):
        Joints, JointSource = Capture.get("joints"), "pose"
    try:
        Tip, GripFrame = SC.GripperFrame(Joints or [])
    except (ValueError, TypeError, KeyError) as Exc:
        return {"ok": False, "error": f"No usable joint positions for that capture: {Exc}"}

    Native = cv2.flip(Image, 1) if Mirrored else Image
    Blob = _NearestBlob(Native, UNative, VNative)

    DepthCm = CameraCm = None
    DepthNote = "no depth recorded with this capture"
    DepthNames = Capture.get("depth_frames_png") or []
    Intrinsics = Capture.get("depth_intrinsics")
    if FrameIndex < len(DepthNames) and DepthNames[FrameIndex] and isinstance(Intrinsics, dict):
        DepthMm = cv2.imread(str(File.with_name(str(DepthNames[FrameIndex]))), cv2.IMREAD_UNCHANGED)
        if DepthMm is None or DepthMm.shape[:2] != (Height, Width):
            DepthNote = "the archived depth frame is missing or not aligned to the image"
        else:
            Point = DepthGeometry.PointAtPixel(DepthMm, U, V, Intrinsics, Mirrored, Radius=DepthWindowRadius)
            if Point.get("ok"):
                DepthCm, CameraCm, DepthNote = float(Point["depth_cm"]), [float(Value) for Value in Point["camera_cm"]], None
            else:
                DepthNote = str(Point.get("error"))

    Record = _Plain({
        "pose_index": PoseIndex, "frame_index": FrameIndex, "capture_archive": str(File),
        "clicked_at": Payload.get("clicked_at") if isinstance(Payload.get("clicked_at"), (int, float)) else time.time(),
        "received_at": time.time(), "target": ClickTarget, "method": "manual",
        "click_capture_px": [U, V], "mirrored": Mirrored, "image_size": [Width, Height],
        "corners": [[UNative, VNative]], "tip": Tip, "frame": GripFrame, "joint_source": JointSource,
        "captured_at_epoch_ms": FrameData.get("captured_at_epoch_ms"), "servos": FrameData.get("servos"),
        "camera_cm": CameraCm, "depth_cm": DepthCm, "depth_note": DepthNote,
        "area_px": Blob["area"] if Blob else None, "red_blob": Blob,
    })
    with SC._Lock:
        if SC._Session.get("archive_directory") != RunDirectory:
            return {"ok": False, "error": "A new calibration run started while the click was read; click again."}
        Known = SC._Session.get("image_size")
        if Known not in (None, [Width, Height]):
            return {"ok": False, "error": f"The frame is {Width}x{Height}, but this run's frames are {Known[0]}x{Known[1]}."}
        if SC._Session.get("mirrored") not in (None, Mirrored):
            return {"ok": False, "error": "The camera's mirroring changed during the run; start again."}
        Clicks = SC._Session.setdefault("manual_clicks", {})
        Replaced = str(PoseIndex) in Clicks
        Clicks[str(PoseIndex)] = Record
        Order = [Index for Index in SC._Session.setdefault("manual_click_order", []) if Index != PoseIndex]
        SC._Session["manual_click_order"] = Order + [PoseIndex]
        SC._Session["manual_skips"] = [Index for Index in SC._Session.get("manual_skips", []) if Index != PoseIndex]
        SC._PersistSession()
        Counts = _Counts()
    SC.RecordEvent("ManualClick", {"request": _Plain({Key: Value for Key, Value in Payload.items()}), "record": Record,
                                   "replaced": Replaced})
    return {"ok": True, "pose_index": PoseIndex, "replaced": Replaced, "native_px": [UNative, VNative],
            "depth_cm": DepthCm, "camera_cm": CameraCm, "depth_note": DepthNote, "red_blob": Record["red_blob"],
            "target": ClickTarget, **Counts}


def _Counts() -> dict[str, int]:
    return {"clicks": len(SC._Session.get("manual_clicks", {})), "skipped": len(SC._Session.get("manual_skips", []))}


def RemoveClick(Payload: dict[str, Any] | None = None) -> dict[str, Any]:
    """Undo: remove the click on pose_index, or the newest click when none is named."""
    Payload = Payload or {}
    try:
        Named = None if Payload.get("pose_index") is None else _Index(Payload.get("pose_index"), "pose_index")
    except ValueError as Exc:
        return {"ok": False, "error": f"Bad undo: {Exc}"}
    with SC._Lock:
        Clicks = SC._Session.setdefault("manual_clicks", {})
        Order = SC._Session.setdefault("manual_click_order", [])
        PoseIndex = Named if Named is not None else (Order[-1] if Order else None)
        Removed = Clicks.pop(str(PoseIndex), None) if PoseIndex is not None else None
        SC._Session["manual_click_order"] = [Index for Index in Order if Index != PoseIndex]
        if Removed is not None:
            SC._PersistSession()
        Counts = _Counts()
    if Removed is not None:
        SC.RecordEvent("ManualUndo", {"pose_index": PoseIndex, "removed": Removed})
    return {"ok": True, "pose_index": PoseIndex if Removed is not None else None, "removed": Removed is not None, **Counts}


def SkipPose(Payload: dict[str, Any]) -> dict[str, Any]:
    """The user skipped a pose (tape hidden, blurred, out of view): no click, recorded as skipped."""
    try:
        PoseIndex = _Index(Payload.get("pose_index"), "pose_index")
    except ValueError as Exc:
        return {"ok": False, "error": f"Bad skip: {Exc}"}
    Reason = str(Payload.get("reason") or "skipped by the user")[:200]
    with SC._Lock:
        Removed = SC._Session.setdefault("manual_clicks", {}).pop(str(PoseIndex), None)
        SC._Session["manual_click_order"] = [Index for Index in SC._Session.get("manual_click_order", []) if Index != PoseIndex]
        Skips = [Index for Index in SC._Session.get("manual_skips", []) if Index != PoseIndex]
        SC._Session["manual_skips"] = Skips + [PoseIndex]
        SC._PersistSession()
        Counts = _Counts()
    SC.RecordEvent("ManualSkip", {"pose_index": PoseIndex, "reason": Reason, "removed_click": Removed})
    return {"ok": True, "pose_index": PoseIndex, **Counts}


def Status() -> dict[str, Any]:
    """The run's clicks (summary) and skips, for the app and for checking the bridge supports clicks."""
    with SC._Lock:
        Clicks = sorted(SC._Session.get("manual_clicks", {}).values(), key=lambda Click: Click["pose_index"])
        Skips = sorted(SC._Session.get("manual_skips", []))
        RunId = SC._Session.get("run_id")
    return {"ok": True, "version": 1, "target": ClickTarget, "target_text": ClickTargetText, "run_id": RunId,
            "clicks": [{Key: Click.get(Key) for Key in ("pose_index", "frame_index", "click_capture_px", "corners", "depth_cm",
                                                         "red_blob", "capture_archive")} for Click in Clicks],
            "skipped": Skips}


def ClickObservations(Session: dict[str, Any]) -> list[dict[str, Any]]:
    """The session's clicks as solver observations (the shape of SelfCalibration observations), by pose."""
    Clicks = sorted((Session.get("manual_clicks") or {}).values(), key=lambda Click: Click["pose_index"])
    return [{"pose_index": Click["pose_index"], "tip": Click["tip"], "frame": Click["frame"], "corners": Click["corners"],
             "method": "manual", "area_px": Click.get("area_px"), "camera_cm": Click.get("camera_cm"),
             "archived": Click.get("capture_archive")} for Click in Clicks]


def ClickSummary(Session: dict[str, Any]) -> dict[str, Any]:
    """Shown where an automatic run reports its tape assignment."""
    Clicks = list((Session.get("manual_clicks") or {}).values())
    return {"source": "manual", "target": ClickTarget, "clicked": len(Clicks), "skipped": len(Session.get("manual_skips") or []),
            "with_depth": sum(1 for Click in Clicks if Click.get("camera_cm") is not None),
            "on_red_blob": sum(1 for Click in Clicks if Click.get("red_blob"))}
