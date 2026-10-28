"""Is the camera still where it was calibrated, where the calibrated desk cannot say? Checked against the robot.

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
The basket rim (RimCheck: the depth profile of its top edge against RimBaseline) is the second witness: it
sees the one turn the tape cannot, about the tape's own line of sight, and the pan no desk sees. Since
2026-10-25 a rim positively off past RimMaxYawDeg / RimMaxShiftCm blocks as camera_moved here too (it was
advisory); it is not required with no desk in view (a tape within its limit verifies alone, robot_witnesses
["tape"], as Aaran decided). RobotCameraReference's RGB keyframe stays advisory.

A desk in view that disagrees (2026-10-25 22:15:25, the shelf; Check DeskDiffers)
-------------------------------------------------------------------------------
"grab the pille bottle of the shelf" was refused and told to recalibrate: the plane the desk search found at the
bookshelf cabinet (201 calibrated desk cells) was 4.17 deg / 3.77 cm off the calibrated desk, more than a cart
move, while the roll-aware tape sat 0.58 cm from where the arm puts it. Where the desk path cannot explain a desk
that disagrees (CameraDrift._DeskDiffers), the robot decides with both witnesses and assumes nothing: tape and rim
within their limits (the rim's closer there, RimDiffersMaxYawDeg / RimDiffersMaxShiftCm) verify; either off blocks;
the rim not judgeable, or not yet known, blocks with what lets it judge, never with "recalibrate".

What the two witnesses resolve (the 22:15 shelf frame and the 2026-10-06 15:01 cabinet frame re-rendered as the camera
moved on the robot, TestShelfCapture): 2 deg pans, 2-4 deg tilts, rolls and turns about the camera's y (the 4.16 deg
hand on the lens of 2026-10-06 15:02 among them) and 2 cm slides all block, on either frame; 1.5 deg pans on the shelf
frame too. A 1-1.5 deg pan, a 1 deg tilt or a 1 cm slide toward the tape's standing offset (0.5-0.6 cm on both frames,
mostly along its line of sight) can pass: 5 of 52 probes of 1-4 deg and 1-2 cm (26 on each frame), the tape reading
0.36-0.69 cm, the rim 0.4-1.0 deg and up to 0.69 cm, about the design's resolution (.claude/NoDeskDesign.md 11.1: pans
from 1.25 deg, tilts from 0.85, slides from 0.75 cm). That leaves an object up to about 1 cm off, as the cart-move
path's tape alone already allows. The rim's per-bin scatter (0.2-0.3 cm) and the tape's standing offset limit it.

Where the rim baseline comes from (RimBaseline)
-----------------------------------------------
The calibration's own frames (CalibrationRim: its first poses with 24 bins clear; where none has, as on 2026-10-14
09:33 with 19-22, every pose of the run bin by bin), and the rims recorded from desk passes that verified the camera
in full since (RecordRim, by /locate; the notes before RimRecordTapeMaxCm), bin by bin the median of those that saw
it. Stored per calibration identity (RimIdentity): a new calibration starts fresh, a desk re-measure does not. The
camera model is an input throughout, so a head motor can supply each capture's pose later.

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

import hashlib
import json
import math
import os
import threading
import time
import uuid
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
RimGrossCm = 1.0
RimMaxYawDeg = 1.25
RimMaxShiftCm = 0.75
# Where a desk in view disagrees with the calibration's (Check, DeskDiffers), the rim must agree closer: that desk is
# itself a sign something moved. Rest readings on 2026-10-25 22:15 (the camera unmoved: the tape 0.58 cm, and 0.08 deg
# from the desk-verified 22:08 capture) are 0.2-0.4 deg and up to 0.22 cm against the calibration's frames; the 22:15 frame
# re-rendered as the camera slid 1 cm toward the scene reads 0.74 cm, which the general limit lets through.
RimDiffersMaxYawDeg = 1.0
RimDiffersMaxShiftCm = 0.6
# The calibration's profile: its first poses (the arm sweeps over the rim later in the run), each with at
# least this many clear bins (2026-10-02: poses 0, 3 and 4 of the first 8).
RimBaselinePoses = 8
RimBaselineMinBins = 24
# With no early pose that clear, every pose of the run, a bin kept where this many saw it (_RimFromEveryPose).
RimBaselineMinPoses = 3
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
    "agreeing"}, or None with fewer than RimMinBins bins seen in both. Bins RimGrossCm or more off the
    least-median line (through one pair of bins at least 3 cm apart) in dY or dZ are not the rim (a cup or
    the cloth on it, a gap in its depth: 2026-10-25 22:08, 5 bins in a row 2.5 cm off); dY and dZ along X are
    fitted on the rest by Theil-Sen (the median slope over bin pairs at least 3 cm apart, the median intercept
    under it), and the bins within RimAgreeCm of that line agree. (Until 2026-10-25 the least-median line itself,
    refitted on the bins within RimAgreeCm of it: with the rim's 0.2-0.3 cm per-bin scatter it follows two bins,
    and the 2026-10-06 15:01 frame re-rendered as the camera panned 1 deg read 0.0 deg one way and 1.7 the other;
    this reads 0.7-1.1 deg a degree of pan either way.)"""
    X = _RimBins() + RimBinCm / 2.0
    Change = np.asarray(Profile, dtype=np.float64) - np.asarray(Baseline, dtype=np.float64)
    Seen = np.isfinite(Change).all(axis=1)
    if int(Seen.sum()) < RimMinBins:
        return None
    Along = X[Seen]
    First, Second = _BinPairs(Along)
    Rim = _OnOneLine(Along, Change[Seen])
    Kept = Rim[First] & Rim[Second]
    if not Kept.any():  # nothing near one line: the agreeing count says so
        Rim, Kept = np.ones(len(Along), dtype=bool), np.ones(len(First), dtype=bool)
    Fits = {}
    for Column, Name in ((0, "y"), (1, "z")):
        Values = Change[Seen, Column]
        Slope = float(np.median((Values[Second] - Values[First])[Kept] / (Along[Second] - Along[First])[Kept]))
        Intercept = float(np.median(Values[Rim] - Slope * Along[Rim]))
        Agree = np.abs(Values - Intercept - Slope * Along) <= RimAgreeCm
        Fits[Name] = (Intercept, math.degrees(math.atan(Slope)), int(Agree.sum()))
    return {"yaw_deg": round(Fits["y"][1], 3), "shift_y_cm": round(Fits["y"][0], 3), "shift_z_cm": round(Fits["z"][0], 3),
            "roll_deg": round(Fits["z"][1], 3), "bins": int(Seen.sum()), "agreeing": min(Fits["y"][2], Fits["z"][2])}


def _BinPairs(Along: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    First, Second = np.triu_indices(len(Along), 1)
    Apart = np.abs(Along[Second] - Along[First]) >= 3.0
    return First[Apart], Second[Apart]


def _OnOneLine(Along: np.ndarray, Values: np.ndarray) -> np.ndarray:
    """The bins (Values: n x 2, dY and dZ, or Y and Z) within RimGrossCm of the least-median line through one pair
    of bins at least 3 cm apart, in both columns: the rim, not what stands on it or a gap in its depth."""
    First, Second = _BinPairs(Along)
    Rim = np.ones(len(Along), dtype=bool)
    if not len(First):
        return Rim
    for Column in (0, 1):
        Value = Values[:, Column]
        Slopes = (Value[Second] - Value[First]) / (Along[Second] - Along[First])
        Intercepts = Value[First] - Slopes * Along[First]
        Spread = np.median(np.abs(Value[None, :] - Intercepts[:, None] - Slopes[:, None] * Along[None, :]), axis=1)
        Best = int(np.argmin(Spread))
        Rim &= np.abs(Value - Intercepts[Best] - Slopes[Best] * Along) < RimGrossCm
    return Rim


def RimOnly(Profile: np.ndarray) -> np.ndarray:
    """A measured profile with the bins that are not the rim (_OnOneLine on its own Y and Z) set to NaN: what a rim
    record keeps when nothing is known of the rim to compare with (2026-10-25 22:08: a cup on the rim's right half and
    a gap in its depth, 6 bins 1.3-2.8 cm off the rest)."""
    Profile = np.array(Profile, dtype=np.float64)
    Seen = np.isfinite(Profile).all(axis=1)
    if int(Seen.sum()) >= 2:
        Keep = np.zeros(len(Profile), dtype=bool)
        Keep[Seen] = _OnOneLine((_RimBins() + RimBinCm / 2.0)[Seen], Profile[Seen])
        Profile[~Keep] = np.nan
    return Profile


_RimBaselines: dict[tuple, dict[str, Any] | None] = {}


def _RimKey(Model: dict[str, Any], Reference: dict[str, Any]) -> tuple:
    # The calibration's run and camera pose; a desk re-measured after a cart move changes neither.
    return (Model.get("fitted_at"), str(Reference.get("keyframe")), tuple(Model.get("rvec") or ()), tuple(Model.get("tvec") or ()),
            bool(Model.get("mirror_x")))


def CalibrationRim(Model: dict[str, Any] | None) -> dict[str, Any] | None:
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


# --------------------------------------------------------------------------
# The rim recorded from verified captures (the calibration's own frames may not show enough of it)
# --------------------------------------------------------------------------
# A capture verifies the camera in full when the calibrated desk matches it on its own terms (tilt and height: 3 degrees
# of freedom) and the roll-aware gripper tape sits within RimRecordTapeMaxCm of where the arm puts it (5 more: all but a
# turn about its own line of sight, which the desk sees). Its rim profile is then recorded (RecordRim, by /locate) for
# the calibration it was checked against, under that calibration's identity (RimIdentity: a new calibration starts
# fresh); a profile that disagrees with what is already known of the rim (RimCheck "moved", or not judgeable against
# it) is never recorded. The tape limit is tighter than the check's (TapeVerifiedCm) so a pan of about 1 deg at most
# (0.485 cm/deg at Home, .claude/NoDeskDesign.md 3) can hide in a recorded rim.
RimRecordTapeMaxCm = 0.5
RimRecordKeep = 5
RimRecordVersion = 1
RimRecordKey = "rim_record"
RimProfileKey = "_rim_profile"  # RimCheck's measured profile, for the record; never in a camera_check


def RimRecordPath() -> Path:
    """Where the recorded rim profiles live: SO101_RIM_BASELINE, else rim_baseline.json in the captures root (beside
    the day folders, never pruned with a day's recordings; tests point SO101_CAPTURES_ROOT elsewhere)."""
    import SessionLog  # local import: the capture root's one definition

    Named = os.environ.get("SO101_RIM_BASELINE")
    return Path(Named) if Named else SessionLog.CapturesRoot() / "rim_baseline.json"


def RimIdentity(Model: dict[str, Any] | None) -> str | None:
    """The calibration a recorded rim belongs to: its fit and camera pose (a desk re-measure changes neither), or None
    without a verified calibration."""
    Reference, _ = CameraDrift.VerifiedReference(Model)
    if Reference is None:
        return None
    Fields = {Key: Model.get(Key) for Key in ("type", "fitted_at", "rvec", "tvec", "mirror_x", "image_size", "table_z")}
    Fields["keyframe"] = Reference.get("keyframe")
    return hashlib.sha256(json.dumps(Fields, sort_keys=True).encode("utf-8")).hexdigest()


_RecordedCache: dict[str, Any] = {"key": None, "value": None}


def RecordedRims(Model: dict[str, Any] | None) -> list[dict[str, Any]]:
    """The rim profiles recorded from verified captures of this calibration, oldest first ({"profile": 28 x 2 array,
    "recorded_at", "seq", "evidence"}); [] with none, another calibration's, or an unreadable file."""
    Identity = RimIdentity(Model)
    Path_ = RimRecordPath()
    try:
        Stat = Path_.stat()
    except OSError:
        return []
    Key = (str(Path_), Stat.st_mtime_ns, Stat.st_size)
    if _RecordedCache["key"] != Key:
        try:
            Stored = json.loads(Path_.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            Stored = None
        _RecordedCache.update(key=Key, value=Stored)
    Stored = _RecordedCache["value"]
    if not isinstance(Stored, dict) or Identity is None or Stored.get("identity") != Identity or Stored.get("version") != RimRecordVersion:
        return []
    Rows = []
    for Row in Stored.get("profiles") or []:
        try:
            Profile = np.asarray([[np.nan if Value is None else float(Value) for Value in Bin] for Bin in Row["profile"]], dtype=np.float64)
        except (KeyError, TypeError, ValueError):
            continue
        if Profile.shape == (len(_RimBins()), 2):
            Rows.append({**Row, "profile": Profile})
    return Rows


def RimBaseline(Model: dict[str, Any] | None) -> dict[str, Any] | None:
    """What the rim is checked against: the calibration's own frames (CalibrationRim) and the profiles recorded from
    verified captures since (RecordedRims), bin by bin the median of those that saw it: {"profile", "edge", "poses",
    "source", "sources": {"calibration": bool, "recorded": n}}, or None with neither (or under RimMinBins bins)."""
    Calibrated = CalibrationRim(Model)
    Recorded = RecordedRims(Model)[-RimRecordKeep:]
    Profiles = ([Calibrated["profile"]] if Calibrated else []) + [Row["profile"] for Row in Recorded]
    if not Profiles:
        return None
    if not Recorded:
        return {**Calibrated, "sources": {"calibration": True, "recorded": 0}}
    Profile = _NanMedian(Profiles)
    Seen = np.isfinite(Profile).all(axis=1)
    if int(Seen.sum()) < RimMinBins:
        return Calibrated and {**Calibrated, "sources": {"calibration": True, "recorded": 0}}
    Edge = Calibrated["edge"] if Calibrated else (float(np.median(Profile[Seen, 0])), float(np.median(Profile[Seen, 1])))
    return {"profile": Profile, "edge": Edge, "poses": (Calibrated or {}).get("poses", []),
            "source": "calibration frames and verified captures" if Calibrated else "verified captures",
            "sources": {"calibration": bool(Calibrated), "recorded": len(Recorded)}}


def RimRecordCandidate(Result: dict[str, Any], Shadow: dict[str, Any], Profile: np.ndarray | None, Seq=None) -> dict[str, Any] | None:
    """The rim record a desk-path pass may make (the notes above), or None. Result is the desk path's camera_check,
    Shadow the robot's reading beside it, Profile the rim it measured."""
    if not Result.get("ok") or Result.get("alignment_source") == "robot" or Result.get("desk_occluded") or Result.get("desk_remeasured"):
        return None
    if Shadow.get("tape_check") != "ok" or Shadow.get("tape_error_cm") is None or float(Shadow["tape_error_cm"]) > RimRecordTapeMaxCm:
        return None
    Profile = None if Profile is None else RimOnly(Profile)
    if Profile is None or int(np.isfinite(Profile).all(axis=1).sum()) < RimMinBins:
        return None
    # Against what is known of the rim already, it must agree, as closely as where a desk disagrees (DeskDiffers); with
    # nothing known, the desk and the tape verify it.
    if Shadow.get("rim_check") == "unavailable":
        if Shadow.get("rim_reason") != NoRimReason:
            return None
    elif Shadow.get("rim_check") != "ok" or abs(float(Shadow["rim_yaw_deg"])) > RimDiffersMaxYawDeg             or max(abs(float(Value)) for Value in Shadow["rim_shift_cm"]) > RimDiffersMaxShiftCm:
        return None
    return {"profile": [[None if not np.isfinite(Value) else round(float(Value), 3) for Value in Bin] for Bin in Profile],
            "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"), "seq": Seq,
            "evidence": {"desk_angle_deg": Result.get("angle_deg"), "desk_offset_cm": Result.get("offset_cm"),
                         "desk_agreeing_share": Result.get("desk_agreeing_share"), "tape_error_cm": Shadow["tape_error_cm"],
                         "rim_check": Shadow.get("rim_check"), "rim_yaw_deg": Shadow.get("rim_yaw_deg"),
                         "rim_shift_cm": Shadow.get("rim_shift_cm")}}


def TakeRimRecord(Result: dict[str, Any]) -> dict[str, Any] | None:
    """Remove and return the rim record a CheckCapture result carries, if any: /locate saves it (RecordRim)."""
    return Result.pop(RimRecordKey, None)


_RecordLock = threading.Lock()


def RecordRim(Record: dict[str, Any], Model: dict[str, Any] | None) -> tuple[bool, str]:
    """Save a verified capture's rim (RimRecordCandidate) for Model's calibration, written whole (a temporary file
    renamed over it); a file of another calibration is replaced, so a new calibration starts fresh. (saved, why not)."""
    Identity = RimIdentity(Model)
    if Identity is None:
        return False, "the camera is not calibrated with a verified baseline"
    Path_ = RimRecordPath()
    with _RecordLock:
        try:
            Stored = json.loads(Path_.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            Stored = None
        if not isinstance(Stored, dict) or Stored.get("identity") != Identity or Stored.get("version") != RimRecordVersion:
            Stored = {"version": RimRecordVersion, "identity": Identity, "fitted_at": (Model or {}).get("fitted_at"), "profiles": []}
        Stored["profiles"] = [*(Stored.get("profiles") or []), Record][-RimRecordKeep:]
        Temporary = Path_.with_name(f"{Path_.name}.{uuid.uuid4().hex[:12]}.tmp")
        try:
            Path_.parent.mkdir(parents=True, exist_ok=True)
            with Temporary.open("x", encoding="utf-8") as Output:
                Output.write(json.dumps(Stored, indent=1))
                Output.flush()
                os.fsync(Output.fileno())
            os.replace(Temporary, Path_)
        except (OSError, TypeError, ValueError) as Exc:
            return False, f"the rim record could not be written: {Exc}"
        finally:
            Temporary.unlink(missing_ok=True)
    return True, ""


def Status(Model: dict[str, Any] | None) -> dict[str, Any]:
    """What a capture with no calibrated desk in view is checked against (GET /calibration/status):
    robot_reference "present" when the gripper tape can check the camera (a verified calibration with a
    tape offset and the wrist roll it was fitted at), else "none"; and the advisory rim's profile once a
    capture has built it from the calibration's run, and how many verified captures recorded it since."""
    Reference, Why = CameraDrift.VerifiedReference(Model)
    Tape = (Model or {}).get("marker_corners_gripper_cm") or []
    Roll = ReferenceRaw(Model)
    Present = Reference is not None and len(Tape) == 1 and Roll is not None
    Rim = _RimBaselines.get(_RimKey(Model, Reference), "not built yet") if Reference is not None and Reference.get("keyframe") else None
    Recorded = len(RecordedRims(Model)) if Reference is not None else 0
    return {"robot_reference": "present" if Present else "none",
            "robot_reference_detail": {
                "tape": ({"offset_cm": [round(float(Value), 3) for Value in Tape[0]], "tape_pose": Reference.get("tape_pose"),
                          "tape_area_cm2": Reference.get("tape_area_cm2"), "limit_cm": TapeVerifiedCm} if Present
                         else Why or "the calibration has no tape offset, or no wrist roll recorded with it"),
                "rim": Rim if isinstance(Rim, str) or Rim is None else {"poses": Rim["poses"], "edge_cm": [round(Value, 2) for Value in Rim["edge"]],
                                                                        "every_pose": bool(Rim.get("every_pose"))},
                "rim_recorded_captures": Recorded}}


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
    Profiles, Used, Every = [], [], []
    for Position, (PoseIndex, Path_) in enumerate(sorted(Records)):
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
        Every.append((PoseIndex, Pose))
        if Position < RimBaselinePoses and int(np.isfinite(Pose).all(axis=1).sum()) >= RimBaselineMinBins:
            Profiles.append(Pose)
            Used.append(PoseIndex)
    if Profiles:
        Profile = _NanMedian(Profiles)
        Seen = np.isfinite(Profile).all(axis=1)
        return {"profile": Profile, "edge": (float(np.median(Profile[Seen, 0])), float(np.median(Profile[Seen, 1]))),
                "poses": Used, "source": str(Folder)}
    return _RimFromEveryPose(Every, Folder)


def _RimFromEveryPose(Every: list[tuple[int, np.ndarray]], Folder: Path) -> dict[str, Any] | None:
    """The calibration's rim when no early pose shows RimBaselineMinBins of it clear (2026-10-14 09:33: 19-22 bins each):
    bin by bin over every pose of the run, the camera still throughout (the fit checks that), each bin's median over
    the RimBaselineMinPoses or more poses that saw it clear of the arm, and only where those poses agree within
    RimAgreeCm of it (the arm's jaw, not left out by its joints, reads high on the rim it passes over). None under
    RimMinBins such bins."""
    if not Every:
        return None
    Stack = np.stack([Pose for _, Pose in Every])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # bins no pose saw stay NaN
        Profile = np.nanmedian(Stack, axis=0)
        Spread = np.nanmedian(np.abs(Stack - Profile[None]), axis=0)
    Count = np.isfinite(Stack).all(axis=2).sum(axis=0)
    Profile[(Count < RimBaselineMinPoses) | ~(np.nanmax(Spread, axis=1) <= RimAgreeCm)] = np.nan
    Seen = np.isfinite(Profile).all(axis=1)
    if int(Seen.sum()) < RimMinBins:
        return None
    Used = [Index for Index, Pose in Every if (np.isfinite(Pose).all(axis=1) & Seen).any()]
    return {"profile": Profile, "edge": (float(np.median(Profile[Seen, 0])), float(np.median(Profile[Seen, 1]))),
            "poses": Used, "source": str(Folder), "every_pose": True}


NoRimReason = "no rim reference for this calibration yet: neither its own frames nor a verified capture show enough of the rim"


def RimCheck(ImageBgr, DepthMm, Intrinsics, Model, Arm, Limits: tuple[float, float] = (RimMaxYawDeg, RimMaxShiftCm)) -> dict[str, Any]:
    """The basket rim against the calibration's (RimBaseline). rim_check "ok" (within Limits: RimMaxYawDeg and
    RimMaxShiftCm unless given), "moved", "not_judgeable" (too little of the rim clear), or "unavailable" (no depth, or
    NoRimReason); rim_baseline names what it was compared with; rim_keyframe: RobotCameraReference's RGB/depth
    keyframe match (None when it found too few features; advisory). Under RimProfileKey, the rim as measured
    here (for a rim record; WithShadow and Check never pass it on)."""
    import RobotCameraReference  # the fixed-rim keyframe registration (HEAD 0f58fdc), advisory here

    Started = time.perf_counter()
    Result: dict[str, Any] = {"rim_check": "unavailable"}
    Joints = np.asarray(Arm["joints"], dtype=np.float64) if Arm and Arm.get("ok") and Arm.get("joints") else None
    try:
        Baseline = RimBaseline(Model) if DepthMm is not None and Intrinsics else None
        if Baseline is None:
            Result["rim_reason"] = "no depth frame" if DepthMm is None else NoRimReason
            if DepthMm is not None and Intrinsics and Joints is not None:
                # Measured as a check would measure it: in the window about the edge this frame itself shows.
                Profile = RimOnly(RimProfile(DepthMm, Intrinsics, Model, Joints))
                Seen = np.isfinite(Profile).all(axis=1)
                if int(Seen.sum()) >= RimMinBins:
                    Edge = (float(np.median(Profile[Seen, 0])), float(np.median(Profile[Seen, 1])))
                    Result[RimProfileKey] = RimProfile(DepthMm, Intrinsics, Model, Joints, Edge)
        else:
            Result["rim_baseline"] = Baseline.get("sources")
            Change, Short = None, None
            Known = np.isfinite(Baseline["profile"]).all(axis=1)
            for Widen in (0.0, RimWidenCm):
                Profile = RimProfile(DepthMm, Intrinsics, Model, Joints, Baseline["edge"], Widen)
                if not Widen and Joints is not None:
                    Result[RimProfileKey] = Profile
                Change = RimChange(Profile, Baseline["profile"])
                Here = np.isfinite(Profile).all(axis=1)
                if Change is None and not Widen and int(Here.sum()) >= RimMinBins:
                    # The rim is clear here where it was looked for; what is known of it covers too little of the same part.
                    Short = (int((Here & Known).sum()), int(Here.sum()), int(Known.sum()))
                    break
                if Change is not None:
                    break
            if Short is not None:
                Result.update(rim_check="not_judgeable", rim_bins=Short[0], rim_reference_short=True,
                              rim_reason=f"only {Short[0]} of {len(_RimBins())} cm of the rim are seen both here ({Short[1]} cm clear) and in "
                                         f"its reference ({Short[2]} cm; need {RimMinBins})")
            elif Change is None or Change["agreeing"] < RimMinAgreeing:
                Result.update(rim_check="not_judgeable", rim_bins=Change["bins"] if Change else None,
                              rim_reason=f"only {Change['bins'] if Change else 'under ' + str(RimMinBins)} of {len(_RimBins())} cm of the rim are clear"
                              + (f", {Change['agreeing']} agreeing" if Change else ""))
            else:
                Moved = abs(Change["yaw_deg"]) > Limits[0] or max(abs(Change["shift_y_cm"]), abs(Change["shift_z_cm"])) > Limits[1]
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

DeskAbsent = "desk_absent"
DeskDiffers = "desk_differs"


def _RimText(Rim: dict[str, Any]) -> str:
    Shift = Rim.get("rim_shift_cm") or [0.0, 0.0]
    return f"{abs(Rim['rim_yaw_deg']):.2f} deg / {max(abs(Shift[0]), abs(Shift[1])):.2f} cm"


def _DeskDiffersClause(DeskCheck: dict[str, Any] | None) -> str:
    Change = (DeskCheck or {}).get("desk_change") or {}
    if Change.get("plane_angle_deg") is not None:
        return (f"the desk in view does not match the calibrated desk (it moved {Change['plane_angle_deg']:.2f} deg / "
                f"{Change.get('plane_offset_cm') or 0.0:+.2f} cm against the camera: another surface, or the cart moved)")
    return "the desk in view does not match the calibrated desk"


def Check(ImageBgr, DepthMm, Intrinsics, Model, Arm, Seq=None, DeskCheck: dict[str, Any] | None = None,
          Mode: str = DeskAbsent, KeepProfile: bool = False) -> dict[str, Any]:
    """The camera_check of a capture the calibrated desk cannot check (CameraDrift.CheckCapture), against the robot's two
    cart-fixed witnesses: the gripper tape through every joint (TapeCheck) and the basket's front rim (RimCheck), which
    sees the one turn the tape cannot, about its own line of sight. Either one positively off past its limits blocks:
    robot_check "moved", camera_moved. Both within them: "verified". "failed" (the tape check itself raised):
    camera_check_failed. Otherwise it depends on why the desk could not check the camera (Mode, robot_mode):
    - DeskAbsent, no calibrated desk in view (Aaran, 2026-10-06: assume the camera at the same place): a tape within its
      limit verifies alone where the rim cannot be judged ("verified", robot_witnesses ["tape"]); a tape that cannot
      judge passes as "assumed" with a one-sentence "warning";
    - DeskDiffers, a desk in view that disagrees with the calibration's and that the cart-move path could not explain
      (beyond a cart's tilt or shift, or too little of it to re-measure): nothing is assumed and the tape never verifies
      alone. A tape that cannot judge is "unverified" (CheckCapture keeps the desk path's own block); a tape within its
      limit with the rim not judgeable or not yet known is "unverified" too, blocked as camera_check_failed with what
      lets the rim judge (no recalibration: RimRecordCandidate).
    DeskCheck, the desk path's own result, rides along under "desk_check"; the rim's reading under rim_*, and with
    KeepProfile the rim as measured under RimProfileKey (WithShadow's rim record)."""
    Started = time.perf_counter()
    Reference, _ = CameraDrift.VerifiedReference(Model)
    FittedAt = (Reference or {}).get("fitted_at") or (Model or {}).get("fitted_at")
    Clock = _Clock(FittedAt)
    Result: dict[str, Any] = {"ok": True, "angle_deg": None, "offset_cm": None, "reason": "", "reference_fitted_at": FittedAt,
                              "seq": Seq, "alignment_source": "robot", "robot_mode": Mode}
    Strict = Mode == DeskDiffers
    Context = _DeskDiffersClause(DeskCheck) if Strict else "no calibrated desk in view"
    try:
        Tape = TapeCheck(ImageBgr, DepthMm, Intrinsics, Model, Arm)
    except Exception as Exc:  # a fault in the check itself is no verdict on the camera: fail closed
        Tape = {"tape_check": "failed", "tape_error_cm": None, "tape_reason": f"{type(Exc).__name__}: {Exc}"}
    Limits = (RimDiffersMaxYawDeg, RimDiffersMaxShiftCm) if Strict else (RimMaxYawDeg, RimMaxShiftCm)
    Rim = RimCheck(ImageBgr, DepthMm, Intrinsics, Model, Arm, Limits)
    Profile = Rim.pop(RimProfileKey, None)
    State, RimState = Tape["tape_check"], Rim["rim_check"]
    TapeText = (f"the gripper tape is {Tape['tape_error_cm']:.2f} cm from where the arm puts it (limit {TapeVerifiedCm} cm, through every joint)"
                if Tape.get("tape_error_cm") is not None else "")
    RimLimits = f"limits {Limits[0]:g} deg / {Limits[1]:g} cm"
    if State == "failed":
        Result.update(ok=False, robot_check="failed", error_code="camera_check_failed",
                      reason=f"{Context}, and the gripper tape check failed ({Tape['tape_reason']})",
                      error="The camera could not be checked against the gripper tip: the check itself failed (the reason is "
                            "in the bridge's camera check). Send the task again.")
    elif State == "moved":
        Result.update(ok=False, robot_check="moved", error_code="camera_moved", reason=f"{Context}; {TapeText}",
                      error=f"The gripper tip is {Tape['tape_error_cm']:.1f} cm from where the calibration at {Clock} puts it, so the camera "
                            "moved on the robot: recalibrate (Automatic calibration).")
    elif RimState == "moved":
        Result.update(ok=False, robot_check="moved", error_code="camera_moved",
                      reason=f"{Context}; the basket's front rim is {_RimText(Rim)} from the calibration's ({RimLimits})"
                             + (f", while {TapeText}" if TapeText else ""),
                      error=f"The camera turned on the robot since calibration at {Clock}: the basket's front rim looks {_RimText(Rim)} "
                            f"off ({RimLimits}). Lock the camera head, then run Automatic calibration.")
    elif State == "ok" and RimState == "ok":
        Result.update(robot_check="verified", robot_witnesses=["tape", "rim"],
                      reason=f"{Context}; {TapeText}, and the basket's front rim {_RimText(Rim)} from the calibration's ({RimLimits}): "
                             "the camera is where it was calibrated on the robot")
    elif State == "ok" and not Strict:
        Result.update(robot_check="verified", robot_witnesses=["tape"],
                      reason=f"{Context}; {TapeText}: the camera is where it was calibrated (the basket rim did not check it: "
                             f"{Rim.get('rim_reason') or RimState})")
    elif State == "ok":
        Unknown = RimState == "unavailable" and Rim.get("rim_reason") == NoRimReason
        Short = bool(Rim.get("rim_reference_short"))
        Record = ("Send one task with the cart back at the calibrated desk (the desk clear, the basket's front edge clear, the arm at "
                  "Home with the red tape in view): that capture records the rim. No recalibration is needed")
        Fix = Record if Unknown or Short else \
            "Take things off the basket's front edge, keep the arm at Home, then send again. No recalibration is needed"
        Why = ("the basket's front rim has not been recorded for this calibration yet, and it checks the one turn the tape cannot see. "
               if Unknown else
               f"too little of the basket's front rim has been recorded yet to check the one turn the tape cannot see ({Rim.get('rim_reason')}). "
               if Short else
               f"too little of the basket's front rim is clear to check the one turn the tape cannot see ({Rim.get('rim_reason')}). ")
        Result.update(ok=False, robot_check="unverified", error_code="camera_check_failed",
                      reason=f"{Context}; {TapeText}, but the basket rim could not check the one turn the tape cannot see "
                             f"({Rim.get('rim_reason') or RimState})",
                      error=f"The desk in view does not match the calibration at {Clock} (was the cart moved?), so the camera is checked "
                            f"against the robot instead: the gripper tape agrees ({Tape['tape_error_cm']:.2f} cm), but {Why}{Fix}.")
    elif Strict:
        Result.update(ok=False, robot_check="unverified", error_code="camera_check_failed",
                      reason=f"{Context}, and the gripper tape could not check the camera ({Tape.get('tape_reason') or State})")
    else:
        Why = TapeAdvice(Tape)
        Result.update(robot_check="assumed",
                      reason=f"{Context}; the camera is assumed where it was calibrated: the gripper tape could not "
                             f"check it ({Tape.get('tape_reason') or State})",
                      warning=f"Camera assumed where it was calibrated at {Clock}: the gripper tip could not be checked ({Why}).")
    Result.update(Tape)
    Result.update(Rim)
    if KeepProfile and Profile is not None:
        Result[RimProfileKey] = Profile
    if DeskCheck is not None:
        Result["desk_check"] = DeskCheck
    # The desk path's time and this one's: what the capture waited for.
    Result["check_ms"] = round((time.perf_counter() - Started) * 1000 + float((DeskCheck or {}).get("check_ms") or 0.0), 1)
    return Result


ShadowKey = "robot_shadow"


def WithShadow(Result: dict[str, Any], ImageBgr, DepthMm, Intrinsics, Model, Arm, Seq=None,
               Reading: dict[str, Any] | None = None) -> dict[str, Any]:
    """The desk path's camera_check as it stands, with the robot's reading beside it under ShadowKey (the
    roll-aware tape and the rim; logged for comparison, it never changes ok, error_code, error or reason).
    Reading, when given, is that reading already taken (CheckCapture's DeskDiffers attempt). A desk pass that
    verifies the camera in full carries its rim under RimRecordKey (RimRecordCandidate) for /locate to save."""
    if DepthMm is None or Intrinsics is None or CameraDrift.VerifiedReference(Model)[0] is None:
        return Result
    try:
        Shadow = dict(Reading) if Reading is not None else Check(ImageBgr, DepthMm, Intrinsics, Model, Arm, KeepProfile=True)
        Profile = Shadow.pop(RimProfileKey, None)
        Result[ShadowKey] = {Key: Value for Key, Value in Shadow.items()
                             if Key not in ("ok", "angle_deg", "offset_cm", "reason", "reference_fitted_at", "seq", "alignment_source", "error",
                                            "robot_mode")}
        Record = RimRecordCandidate(Result, Shadow, Profile, Seq) if Reading is None else None
        if Record is not None:
            Result[RimRecordKey] = Record
    except Exception as Exc:  # a shadow must never stop a capture the desk passed
        Result[ShadowKey] = {"robot_check": "unavailable", "error": f"{type(Exc).__name__}: {Exc}"}
    return Result
