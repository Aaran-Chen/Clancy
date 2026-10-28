"""Orbbec Gemini 335L service: color plus depth aligned to it, owned by the bridge.

Only one process can stream the camera, so the bridge opens it and serves:
  * JPEG color frames to the browser (the live view and every AI capture),
  * the depth frame matching a browser capture, by frame sequence number.

Captures reach the bridge seconds after they are taken (the VLM runs first),
so the first request naming a sequence pins that depth frame. A pin holds the
per-pixel median of the frames just before it, which removes most stereo
speckle while the arm and scene are still. Without a camera or the SDK the
service just reports itself unavailable; nothing else in the bridge changes.
"""

from __future__ import annotations

import collections
import os
import threading
import time
import warnings
from pathlib import Path
from typing import Any, Callable

import cv2
import numpy as np

import LogRotation

HistoryFrames = 45        # about 3 s at 15 fps
PinnedFrames = 24
PinMedianFrames = 3
PinMaxSpreadS = 0.4
JpegQuality = 88
ReconnectDelayS = 2.0
# Opening the camera (the SDK's Context and device query) holds Python's interpreter lock for 1-2 s, freezing every
# other thread, the 30 Hz servo loop too: with the camera unplugged, a retry every 2 s froze the loop for 1-2 s of
# every 4 (2026-10-10). Failed opens back off to this, and none is tried while Quiet() says the arm needs the loop.
ReconnectMaxDelayS = 30.0
QuietPollS = 0.5
ColorSize = (1280, 720)
DepthSizes = ((848, 480), (640, 480), (640, 400))
# The SDK's own log (Log/OrbbecSDK.log.txt), errors only since 2026-10-03: cut to 10 MB, the last 3 kept, once per
# bridge before the SDK opens it (2026-10-07 audit 2 M4: 102 MB were left from before; the SDK's own cap is 3 x 100 MB).
SdkLogDirectory = Path(__file__).resolve().parent / "Log"
SdkLogCapBytes = 10_000_000
SdkLogKeep = 3
_SdkLogRotated = False


def RotateSdkLog(Directory: Path = SdkLogDirectory) -> str:
    """Rotate the SDK's log the first time a camera is opened in this process (LogRotation.RotateLog's answer)."""
    global _SdkLogRotated
    if _SdkLogRotated:
        return "done before"
    _SdkLogRotated = True
    return LogRotation.RotateLog(Directory / "OrbbecSDK.log.txt", SdkLogCapBytes, SdkLogKeep)


class Frame:
    __slots__ = ("Seq", "Time", "Jpeg", "DepthMm")

    def __init__(Self, Seq: int, Time: float, Jpeg: bytes, DepthMm: np.ndarray):
        Self.Seq, Self.Time, Self.Jpeg, Self.DepthMm = Seq, Time, Jpeg, DepthMm


class OrbbecSource:
    """pyorbbecsdk pipeline: color at 1280x720, depth aligned to color in software."""

    def __init__(Self):
        import pyorbbecsdk as Ob

        Self.Ob = Ob
        # Without Orbbec's UVC metadata registry keys, Windows makes the SDK
        # log a warning for every frame (about 1 GB a day); keep errors only.
        RotateSdkLog()
        try:
            Ob.Context.set_logger_to_console(Ob.OBLogLevel.ERROR)
            Ob.Context.set_logger_to_file(Ob.OBLogLevel.ERROR, str(SdkLogDirectory))
        except (AttributeError, RuntimeError):
            pass
        Context = Ob.Context()
        Devices = Context.query_devices()
        if Devices.get_count() == 0:
            raise RuntimeError("no Orbbec camera connected")
        Self.Device = Devices.get_device_by_index(0)
        Info = Self.Device.get_device_info()
        Self.Info = {"name": Info.get_name(), "serial": Info.get_serial_number(),
                     "firmware": Info.get_firmware_version(), "connection": Info.get_connection_type()}
        Self.Pipeline = Ob.Pipeline(Self.Device)
        ColorProfile, DepthProfile = Self._PickProfiles()
        Config = Ob.Config()
        Config.enable_stream(ColorProfile)
        Config.enable_stream(DepthProfile)
        Config.set_frame_aggregate_output_mode(Ob.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
        Self.Pipeline.start(Config)
        Self.Align = Ob.AlignFilter(align_to_stream=Ob.OBStreamType.COLOR_STREAM)
        Param = Self.Pipeline.get_camera_param()
        Rgb, Dist = Param.rgb_intrinsic, Param.rgb_distortion
        Self.Intrinsics = {"fx": float(Rgb.fx), "fy": float(Rgb.fy), "cx": float(Rgb.cx), "cy": float(Rgb.cy),
                           "width": int(Rgb.width), "height": int(Rgb.height),
                           # OpenCV order: k1, k2, p1, p2, k3.
                           "dist": [float(Dist.k1), float(Dist.k2), float(Dist.p1), float(Dist.p2), float(Dist.k3)],
                           "source": "orbbec-factory"}
        Self.Info.update(color=f"{ColorProfile.get_width()}x{ColorProfile.get_height()}@{ColorProfile.get_fps()} "
                               f"{str(ColorProfile.get_format()).split('.')[-1]}",
                         depth=f"{DepthProfile.get_width()}x{DepthProfile.get_height()}@{DepthProfile.get_fps()}")
        Self.ColorFormat = ColorProfile.get_format()

    def _PickProfiles(Self):
        """Highest shared frame rate with 1280x720 color; USB 2 caps it at 15 fps."""
        Ob = Self.Ob
        Colors = Self.Pipeline.get_stream_profile_list(Ob.OBSensorType.COLOR_SENSOR)
        Depths = Self.Pipeline.get_stream_profile_list(Ob.OBSensorType.DEPTH_SENSOR)

        def Find(Profiles, Width, Height, Format, Fps):
            try:
                return Profiles.get_video_stream_profile(Width, Height, Format, Fps)
            except Exception:
                return None

        for Fps in (30, 15, 10, 5):
            Color = next((P for Format in (Ob.OBFormat.BGR, Ob.OBFormat.RGB, Ob.OBFormat.MJPG)
                          if (P := Find(Colors, *ColorSize, Format, Fps)) is not None), None)
            Depth = next((P for Size in DepthSizes if (P := Find(Depths, *Size, Ob.OBFormat.Y16, Fps)) is not None), None)
            if Color is not None and Depth is not None:
                return Color, Depth
        raise RuntimeError("the camera offers no 1280x720 color stream with matching depth")

    def Read(Self, TimeoutMs: int = 500) -> tuple[np.ndarray, np.ndarray] | None:
        Ob = Self.Ob
        Frames = Self.Pipeline.wait_for_frames(TimeoutMs)
        if Frames is None:
            return None
        Aligned = Self.Align.process(Frames)
        if Aligned is None:
            return None
        Aligned = Aligned.as_frame_set()
        Color, Depth = Aligned.get_color_frame(), Aligned.get_depth_frame()
        if Color is None or Depth is None:
            return None
        Raw = np.frombuffer(Color.get_data(), dtype=np.uint8)
        Width, Height = Color.get_width(), Color.get_height()
        Format = Color.get_format()
        if Format == Ob.OBFormat.MJPG:
            Bgr = cv2.imdecode(Raw, cv2.IMREAD_COLOR)
        elif Format == Ob.OBFormat.RGB:
            Bgr = cv2.cvtColor(Raw.reshape(Height, Width, 3), cv2.COLOR_RGB2BGR)
        else:
            Bgr = Raw.reshape(Height, Width, 3).copy()
        Scale = float(Depth.get_depth_scale())
        DepthRaw = np.frombuffer(Depth.get_data(), dtype=np.uint16).reshape(Depth.get_height(), Depth.get_width())
        DepthMm = DepthRaw.copy() if Scale == 1.0 else np.round(DepthRaw * Scale).astype(np.uint16)
        return Bgr, DepthMm

    def Close(Self):
        try:
            Self.Pipeline.stop()
        except Exception:
            pass


class DepthCamera:
    def __init__(Self, Opener: Callable[[], Any] = OrbbecSource, History: int = HistoryFrames):
        Self.Opener = Opener
        Self.History: collections.deque[Frame] = collections.deque(maxlen=History)
        Self.Pinned: collections.OrderedDict[int, dict[str, Any]] = collections.OrderedDict()
        Self.Condition = threading.Condition()
        Self.Thread: threading.Thread | None = None
        Self.Running = False
        Self.Source = None
        Self.Seq = 0
        Self.Info: dict[str, Any] = {}
        Self.IntrinsicsValue: dict[str, Any] | None = None
        Self.Error: str | None = None
        Self.Times: collections.deque[float] = collections.deque(maxlen=30)
        # True while an open must wait (the bridge: the arm is moving, calibrating or hand guiding, or a motion is being
        # checked and planned). It must not wait for anything: it is called holding OpenLock.
        Self.Quiet: Callable[[], bool] = lambda: False
        Self.FailedOpens = 0
        # The open interlock: an open and an arm motion never overlap. Quiet() is read and Opening set under OpenLock,
        # and the bridge holds it from a motion's last check until the motion counts as moving (Quiet() true) and
        # refuses while Opening: a motion cannot start between the open's check and the open (ad5a7a8 left that gap:
        # an open begun just before a route froze the loop under it). Opening covers the whole open, the SDK's
        # Context, device query and pipeline start, which hold the interpreter lock for 1-7 s (2026-10-25 stalls).
        Self.OpenLock = threading.Lock()
        Self.Opening = False
        # Set once the first open ended, however it ended, or at once when the camera is off: the bridge's startup
        # warm-up waits for it (no motion before), since that first open is the longest freeze (6-7 s, 2026-10-25).
        Self.FirstOpenDone = threading.Event()

    # -- lifecycle ---------------------------------------------------------

    def Start(Self) -> None:
        if os.environ.get("SO101_DEPTH_CAMERA", "1") == "0":
            Self.FirstOpenDone.set()
            return
        if Self.Running:
            return
        Self.Running = True
        Self.Thread = threading.Thread(target=Self._Run, name="DepthCamera", daemon=True)
        Self.Thread.start()

    def Stop(Self) -> None:
        Self.Running = False
        with Self.Condition:
            Self.Condition.notify_all()
        if Self.Thread is not None:
            Self.Thread.join(timeout=3.0)
        Self.Thread = None
        Self.FirstOpenDone.set()

    def _IsQuiet(Self) -> bool:
        try:
            return bool(Self.Quiet())
        except Exception:
            return False

    def _BeginOpen(Self) -> bool:
        """Whether an open may start now; if so it counts as Opening from here (checked and set under OpenLock)."""
        with Self.OpenLock:
            if Self._IsQuiet():
                return False
            Self.Opening = True
            return True

    def _Run(Self) -> None:
        while Self.Running:
            while Self.Running and not Self._BeginOpen():
                Self._Sleep(QuietPollS)
            if not Self.Running:
                Self.Opening = False
                break
            try:
                Source = Self.Opener()
            except Exception as Exc:
                Source = None
                Self.Error = f"{type(Exc).__name__}: {Exc}"
            finally:
                Self.Opening = False
                Self.FirstOpenDone.set()
            if Source is None:
                Self.FailedOpens += 1
                Self._Sleep(min(ReconnectMaxDelayS, ReconnectDelayS * 2 ** (Self.FailedOpens - 1)))
                continue
            Self.FailedOpens = 0
            Self.Source, Self.Info, Self.IntrinsicsValue, Self.Error = Source, dict(Source.Info), dict(Source.Intrinsics), None
            Misses = 0
            try:
                while Self.Running:
                    Pair = Source.Read(500)
                    if Pair is None:
                        Misses += 1
                        if Misses >= 10:
                            raise RuntimeError("the camera stopped sending frames")
                        continue
                    Misses = 0
                    Self.Push(*Pair)
            except Exception as Exc:
                Self.Error = f"{type(Exc).__name__}: {Exc}"
            finally:
                Source.Close()
                Self.Source = None
            Self._Sleep(ReconnectDelayS)

    def _Sleep(Self, Seconds: float) -> None:
        with Self.Condition:
            Self.Condition.wait_for(lambda: not Self.Running, timeout=Seconds)

    def Push(Self, Bgr: np.ndarray, DepthMm: np.ndarray) -> int:
        Ok, Encoded = cv2.imencode(".jpg", Bgr, [cv2.IMWRITE_JPEG_QUALITY, JpegQuality])
        if not Ok:
            return Self.Seq
        Now = time.time()
        with Self.Condition:
            Self.Seq += 1
            Self.History.append(Frame(Self.Seq, Now, Encoded.tobytes(), DepthMm))
            Self.Times.append(Now)
            Self.Condition.notify_all()
            return Self.Seq

    # -- queries -----------------------------------------------------------

    @property
    def Connected(Self) -> bool:
        return bool(Self.History) and time.time() - Self.History[-1].Time < 2.0

    def Intrinsics(Self) -> dict[str, Any] | None:
        return dict(Self.IntrinsicsValue) if Self.IntrinsicsValue else None

    def Status(Self) -> dict[str, Any]:
        with Self.Condition:
            Times = list(Self.Times)
            Last = Self.History[-1] if Self.History else None
        Fps = (len(Times) - 1) / (Times[-1] - Times[0]) if len(Times) >= 2 and Times[-1] > Times[0] else 0.0
        Connection = str(Self.Info.get("connection", ""))
        Warnings = []
        if Connection.upper().startswith("USB2"):
            Warnings.append("Camera is on a USB 2 link: move it to a USB 3 port (blue or SuperSpeed) for 30 fps depth.")
        return {"ok": True, "available": Self.Connected, "running": Self.Running, "seq": Last.Seq if Last else 0,
                "age_s": round(time.time() - Last.Time, 3) if Last else None, "fps": round(Fps, 1),
                "info": dict(Self.Info), "intrinsics": Self.Intrinsics(), "error": Self.Error, "warnings": Warnings}

    def WaitJpeg(Self, After: int = 0, TimeoutS: float = 1.5) -> tuple[int, bytes] | None:
        """Newest JPEG with a sequence above After, waiting briefly for one. An After beyond the
        newest sequence came from an earlier bridge process (which counted from 1 too): no wait."""
        with Self.Condition:
            Self.Condition.wait_for(lambda: not Self.Running or (Self.History and Self.History[-1].Seq != After),
                                    timeout=TimeoutS)
            if not Self.History:
                return None
            Last = Self.History[-1]
            return Last.Seq, Last.Jpeg

    def WaitDepth(Self, After: int = 0, TimeoutS: float = 1.5) -> tuple[int, np.ndarray] | None:
        """Newest raw depth frame after After, waiting briefly. Never pinned: for
        live views, which must not evict the frames an AI capture is waiting on."""
        with Self.Condition:
            Self.Condition.wait_for(lambda: not Self.Running or (Self.History and Self.History[-1].Seq != After),
                                    timeout=TimeoutS)
            if not Self.History:
                return None
            Last = Self.History[-1]
            return Last.Seq, Last.DepthMm

    def Depth(Self, Seq: int | None = None) -> dict[str, Any] | None:
        """Pinned depth for a frame sequence (default: newest), as {seq, time, depth_mm}.

        The pin is the per-pixel median of up to PinMedianFrames frames ending at
        Seq and spanning at most PinMaxSpreadS; zero (no depth) is ignored.
        """
        with Self.Condition:
            if Seq is not None and Seq in Self.Pinned:
                Self.Pinned.move_to_end(Seq)
                return Self.Pinned[Seq]
            Frames = list(Self.History)
        if not Frames:
            return None
        Target = Frames[-1] if Seq is None else next((F for F in Frames if F.Seq == Seq), None)
        if Target is None:
            return None
        Recent = [F for F in Frames if F.Seq <= Target.Seq and Target.Time - F.Time <= PinMaxSpreadS][-PinMedianFrames:]
        Pin = {"seq": Target.Seq, "time": Target.Time, "depth_mm": MedianDepth([F.DepthMm for F in Recent]),
               "frames": len(Recent)}
        with Self.Condition:
            Self.Pinned[Target.Seq] = Pin
            while len(Self.Pinned) > PinnedFrames:
                Self.Pinned.popitem(last=False)
        return Pin

    def Jpeg(Self, Seq: int) -> bytes | None:
        with Self.Condition:
            return next((F.Jpeg for F in Self.History if F.Seq == Seq), None)


def MedianDepth(Frames: list[np.ndarray]) -> np.ndarray:
    """Per-pixel median of depth frames (mm) that ignores missing (zero) samples."""
    if len(Frames) == 1:
        return Frames[0]
    Stack = np.stack(Frames).astype(np.float32)
    Stack[Stack <= 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN pixels stay NaN, then 0
        Median = np.nanmedian(Stack, axis=0)
    return np.nan_to_num(Median, nan=0.0).astype(np.uint16)


def OverlayPng(DepthMm: np.ndarray, NearCm: float = 25.0, FarCm: float = 150.0, Scale: float = 0.5) -> bytes:
    """Turbo depth over a fixed range, transparent where there is no reading, to
    draw over the live view. A fixed range keeps colours steady from frame to
    frame; half resolution keeps it light (the view scales it back up)."""
    if Scale != 1.0:
        DepthMm = cv2.resize(DepthMm, None, fx=Scale, fy=Scale, interpolation=cv2.INTER_NEAREST)
    Valid = DepthMm > 0
    Scaled = np.clip((DepthMm.astype(np.float32) / 10.0 - NearCm) / max(FarCm - NearCm, 1.0), 0.0, 1.0)
    Colours = cv2.applyColorMap((Scaled * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    Ok, Encoded = cv2.imencode(".png", np.dstack([Colours, np.where(Valid, 255, 0).astype(np.uint8)]))
    if not Ok:
        raise RuntimeError("could not encode the depth overlay")
    return Encoded.tobytes()


def ColorizeDepth(DepthMm: np.ndarray, Mirrored: bool) -> tuple[bytes, dict[str, Any]]:
    """Turbo-colorized depth (blue near, red far; black = no reading) as PNG bytes plus stats in cm."""
    Valid = DepthMm > 0
    Stats: dict[str, Any] = {"valid_fraction": round(float(Valid.mean()), 3)}
    if Valid.sum() < 10:
        Near, Far = 10.0, 200.0
    else:
        Values = DepthMm[Valid].astype(np.float32) / 10.0
        Near, Far = float(np.percentile(Values, 2)), float(np.percentile(Values, 98))
        Stats.update(min_cm=round(float(Values.min()), 1), median_cm=round(float(np.median(Values)), 1),
                     max_cm=round(float(Values.max()), 1))
        if Far - Near < 5.0:
            Far = Near + 5.0
    Scaled = np.clip((DepthMm.astype(np.float32) / 10.0 - Near) / (Far - Near), 0.0, 1.0)
    Image = cv2.applyColorMap((Scaled * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    Image[~Valid] = 0
    if Mirrored:
        Image = cv2.flip(Image, 1)
    Ok, Encoded = cv2.imencode(".png", Image)
    if not Ok:
        raise RuntimeError("could not encode the depth image")
    return Encoded.tobytes(), {"near_cm": round(Near, 1), "far_cm": round(Far, 1), **Stats}
