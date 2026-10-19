import { memo, useEffect, useRef, useState, type PointerEvent as ReactPointerEvent, type RefObject } from "react";
import { Bot, Layers } from "lucide-react";
import FloatingCameraWindow from "./FloatingCameraWindow";
import HoverLoupe from "./HoverLoupe";
import { DepthReadingText, VideoPixelAt, type DepthReading } from "../lib/LiveViewDepth";

export type CameraSource = "depth" | "webcam";
// Source is null until the app knows whether the bridge streams the depth camera.
type LiveCameraProps = { VideoRef: RefObject<HTMLVideoElement | null>; Profile: "Smooth" | "Detail"; Source: CameraSource | null;
  DepthFrameUrl: string; OnStatus: (Value: string) => void; OnActive: (Value: boolean) => void; Countdown: number | null;
  ReconnectKey: number; DepthOverlay: boolean; DepthOverlayUrl: string; OnDepthOverlayChange: (Value: boolean) => void };
export function CameraConstraints(Profile: "Smooth" | "Detail"): MediaStreamConstraints {
  return { audio: false, video: { width: { ideal: Profile === "Detail" ? 1920 : 1280 },
    height: { ideal: Profile === "Detail" ? 1080 : 720 }, frameRate: { ideal: 60, max: 60 } } };
}
// The bridge streams the Orbbec depth camera itself, so a browser webcam must
// never open it: Windows lets only one program stream a camera.
export function IsDepthCameraLabel(Label: string) { return /orbbec|gemini/i.test(Label); }

// In depth mode the <video> plays a canvas stream fed by bridge frames. The
// canvas holds exactly the frame whose depth the bridge can look up, so
// captures read pixels and the frame number from it rather than the video.
const DepthCanvases = new WeakMap<HTMLVideoElement, HTMLCanvasElement>();
export function DepthCanvasFor(Video: HTMLVideoElement | null) { return Video ? DepthCanvases.get(Video) ?? null : null; }
export function DepthSequenceFor(Video: HTMLVideoElement | null): number | null {
  const Value = Number(DepthCanvasFor(Video)?.dataset.seq);
  return Number.isFinite(Value) && Value > 0 ? Value : null;
}

async function OpenWebcam(Profile: "Smooth" | "Detail"): Promise<MediaStream> {
  if (!navigator.mediaDevices?.getUserMedia) throw new Error("Camera unavailable");
  const Open = async (DeviceId?: string) => {
    const Constraints = CameraConstraints(Profile);
    const Video = { ...(Constraints.video as MediaTrackConstraints), ...(DeviceId ? { deviceId: { exact: DeviceId } } : {}) };
    try { return await navigator.mediaDevices.getUserMedia({ audio: false, video: Video }); }
    catch (Error) {
      if (!(Error instanceof DOMException) || Error.name !== "OverconstrainedError") throw Error;
      return navigator.mediaDevices.getUserMedia({ audio: false, video: DeviceId ? { deviceId: { exact: DeviceId } } : true });
    }
  };
  const Webcams = async () => (await navigator.mediaDevices.enumerateDevices?.() ?? [])
    .filter(Device => Device.kind === "videoinput" && Device.label && !IsDepthCameraLabel(Device.label));
  // Labels are only visible once camera permission exists; pick a webcam up
  // front when they are, otherwise open the default and swap if it is the Orbbec.
  const Known = await Webcams();
  const Stream = await Open(Known[0]?.deviceId);
  if (!IsDepthCameraLabel(Stream.getVideoTracks()[0]?.label ?? "")) return Stream;
  Stream.getTracks().forEach(Track => Track.stop());
  const Other = (await Webcams())[0];
  if (!Other) throw new Error("Only the depth camera is connected");
  return Open(Other.deviceId);
}

export default memo(function LiveCamera({ VideoRef, Profile, Source, DepthFrameUrl, OnStatus, OnActive, Countdown, ReconnectKey,
  DepthOverlay, DepthOverlayUrl, OnDepthOverlayChange }: LiveCameraProps) {
  const [State, SetState] = useState("Connecting"), [Active, SetActive] = useState(false);
  const [Expanded, SetExpanded] = useState(false);
  const LastFrame = useRef(0);
  const OverlayRef = useRef<HTMLCanvasElement>(null);
  const [OverlayNote, SetOverlayNote] = useState<string | null>(null), [OverlayRange, SetOverlayRange] = useState<string | null>(null);
  const ShowOverlay = Source === "depth" && DepthOverlay;
  // The overlay draws on its own canvas above the video, so captures for the
  // AI and calibration (taken from the depth canvas) never include it.
  useEffect(() => {
    if (!ShowOverlay) { SetOverlayNote(null); return; }
    let Cancelled = false, Seq = 0;
    const Abort = new AbortController();
    const Pause = (Milliseconds: number) => new Promise(Resolve => window.setTimeout(Resolve, Milliseconds));
    const Run = async () => {
      while (!Cancelled) {
        const Started = performance.now();
        try {
          const Response = await fetch(`${DepthOverlayUrl}?after=${Seq}`, { cache: "no-store", signal: Abort.signal });
          if (Response.status === 404) { SetOverlayNote("Restart the bridge to turn on the depth overlay"); return; }
          if (Response.status === 204) { await Pause(100); continue; }
          if (!Response.ok) throw new Error(`HTTP ${Response.status}`);
          const Next = Number(Response.headers.get("X-Frame-Seq"));
          Seq = Number.isFinite(Next) && Next > 0 ? Next : Seq + 1;
          SetOverlayRange(Response.headers.get("X-Depth-Range-Cm"));
          const Bitmap = await createImageBitmap(await Response.blob());
          const Canvas = OverlayRef.current, Context = Canvas?.getContext("2d");
          if (Cancelled || !Canvas || !Context) { Bitmap.close(); return; }
          if (Canvas.width !== Bitmap.width || Canvas.height !== Bitmap.height) { Canvas.width = Bitmap.width; Canvas.height = Bitmap.height; }
          Context.clearRect(0, 0, Canvas.width, Canvas.height);
          Context.drawImage(Bitmap, 0, 0);
          Bitmap.close();
          SetOverlayNote(null);
        } catch {
          if (Cancelled) return;
          Seq = 0;  // a restarted bridge counts frames from 1 again
          SetOverlayNote("Depth overlay unavailable · retrying");
          await Pause(1000);
        }
        // At most ten overlay frames a second: enough to read depth, light on the machine.
        const Wait = 100 - (performance.now() - Started);
        if (Wait > 0) await Pause(Wait);
      }
    };
    void Run();
    return () => { Cancelled = true; Abort.abort(); };
  }, [ShowOverlay, DepthOverlayUrl, ReconnectKey]);
  const [Near, Far] = (OverlayRange ?? "").split(",");
  // Hover readout: while the pointer is over the picture, read the depth under it (the bridge
  // serves depth-at beside the overlay). Readings repeat while it rests, so moving objects update.
  const [Hovering, SetHovering] = useState(false), [HoverText, SetHoverText] = useState<string | null>(null);
  const HoverPixel = useRef<{ U: number; V: number } | null>(null);
  const DepthAtUrl = DepthOverlayUrl.replace(/\/overlay\.png$/, "/depth-at");
  useEffect(() => {
    if (!ShowOverlay || !Hovering) { SetHoverText(null); return; }
    let Cancelled = false;
    const Abort = new AbortController();
    const Run = async () => {
      while (!Cancelled) {
        const Started = performance.now(), Pixel = HoverPixel.current;
        if (!Pixel) SetHoverText(null);
        else {
          try {
            const Response = await fetch(`${DepthAtUrl}?u=${Pixel.U.toFixed(1)}&v=${Pixel.V.toFixed(1)}`, { cache: "no-store", signal: Abort.signal });
            if (Response.status === 404) { SetHoverText("Restart the bridge to read depth on hover"); return; }
            if (!Response.ok) throw new Error(`HTTP ${Response.status}`);
            const Reading = await Response.json() as DepthReading;
            if (!Cancelled) SetHoverText(DepthReadingText(Reading));
          } catch {
            if (Cancelled) return;
            SetHoverText("Depth reading unavailable");
          }
        }
        // At most ten readings a second, like the overlay itself.
        const Wait = 100 - (performance.now() - Started);
        await new Promise(Resolve => window.setTimeout(Resolve, Math.max(0, Wait)));
      }
    };
    void Run();
    return () => { Cancelled = true; Abort.abort(); };
  }, [ShowOverlay, Hovering, DepthAtUrl]);
  const TrackHover = (Event: ReactPointerEvent<HTMLDivElement>) => {
    const Video = VideoRef.current;
    if (!Video) return;
    HoverPixel.current = VideoPixelAt(Event.clientX, Event.clientY, Video.getBoundingClientRect(), Video.videoWidth, Video.videoHeight,
      getComputedStyle(Video).objectFit);
    SetHovering(true);
  };
  const EndHover = () => { HoverPixel.current = null; SetHovering(false); };
  const FrameRef = useRef<HTMLDivElement>(null);
  useEffect(() => {
    let Cancelled = false, Stream: MediaStream | null = null, FrameId = 0, LastReady = false;
    let Count = 0, PreviousCount = 0, PreviousTime = performance.now(), LastPresented = 0, PreviousMediaTime = 0;
    const Abort = new AbortController();
    const VideoElement = VideoRef.current;
    const LiveText = Source === "depth" ? "Live depth camera" : "Live feed";
    const Report = (Text: string, Ready: boolean) => { LastReady = Ready; if (VideoElement) VideoElement.dataset.captureReady = String(Ready); SetState(Text); SetActive(Ready); OnStatus(Text); OnActive(Ready); };
    const Frame = (Now: number, Metadata: VideoFrameCallbackMetadata) => {
      if (Cancelled || !VideoElement) return;
      Count += LastPresented ? Math.max(1, Metadata.presentedFrames - LastPresented) : 1;
      LastPresented = Metadata.presentedFrames; LastFrame.current = Now;
      FrameId = VideoElement.requestVideoFrameCallback(Frame);
    };
    const Play = async (Media: MediaStream) => {
      if (!VideoElement) return false;
      VideoElement.srcObject = Media;
      await VideoElement.play();
      if (Cancelled) return false;
      LastFrame.current = performance.now();
      if (typeof VideoElement.requestVideoFrameCallback === "function") FrameId = VideoElement.requestVideoFrameCallback(Frame);
      return true;
    };
    // Pull bridge frames into a canvas: each request long-polls for a frame
    // newer than the last one, so the loop runs at the camera's own rate.
    const StartDepth = async () => {
      Report("Connecting depth camera", false);
      if (!VideoElement) return;
      const Canvas = document.createElement("canvas");
      Canvas.width = 1280; Canvas.height = 720;
      const Context = Canvas.getContext("2d");
      if (!Context || typeof Canvas.captureStream !== "function") { Report("Depth camera view unsupported in this browser", false); return; }
      DepthCanvases.set(VideoElement, Canvas);
      Stream = Canvas.captureStream(30);
      let Seq = 0, Playing = false;
      while (!Cancelled) {
        try {
          const Response = await fetch(`${DepthFrameUrl}?after=${Seq}`, { cache: "no-store", signal: Abort.signal });
          // No newer frame: normally after a 1.5 s long-poll, but never spin.
          if (Response.status === 204) { await new Promise(Resolve => window.setTimeout(Resolve, 100)); continue; }
          if (!Response.ok) throw new Error(`HTTP ${Response.status}`);
          const Next = Number(Response.headers.get("X-Frame-Seq"));
          const Bitmap = await createImageBitmap(await Response.blob());
          if (Cancelled) { Bitmap.close(); return; }
          if (Canvas.width !== Bitmap.width || Canvas.height !== Bitmap.height) { Canvas.width = Bitmap.width; Canvas.height = Bitmap.height; }
          Context.drawImage(Bitmap, 0, 0);
          Bitmap.close();
          Seq = Number.isFinite(Next) ? Next : Seq + 1;
          Canvas.dataset.seq = String(Seq);
          if (!Playing) { Playing = await Play(Stream); if (Playing) Report(LiveText, true); }
          else if (!LastReady) Report(LiveText, true);  // frames are back after an outage
        } catch {
          if (Cancelled) return;
          Seq = 0;  // a restarted bridge counts frames from 1 again
          delete Canvas.dataset.seq;
          Report("Depth camera unavailable · retrying", false);
          await new Promise(Resolve => window.setTimeout(Resolve, 1000));
        }
      }
    };
    const StartWebcam = async () => {
      Report("Connecting camera", false);
      try {
        Stream = await OpenWebcam(Profile);
        if (Cancelled) { Stream.getTracks().forEach(Track => Track.stop()); return; }
        if (!(await Play(Stream))) return;
        Stream.getVideoTracks().forEach(Track => Track.addEventListener("ended", () => { if (!Cancelled) Report("Camera disconnected", false); }));
        Report(LiveText, true);
      } catch (Failure) {
        if (!Cancelled) Report(Failure instanceof DOMException && Failure.name === "NotAllowedError" ? "Camera access unavailable"
          : Failure instanceof Error && Failure.message.startsWith("Only the depth") ? "Only the depth camera is connected · select it in settings"
          : "Camera unavailable · retry connection", false);
      }
    };
    if (Source === "depth") void StartDepth();
    else if (Source === "webcam") void StartWebcam();
    else Report("Looking for the depth camera", false);
    const Timer = window.setInterval(() => {
      if (!VideoElement) return;
      const Now = performance.now(), Quality = VideoElement.getVideoPlaybackQuality?.();
      const Delivered = typeof VideoElement.requestVideoFrameCallback === "function" ? Count : Math.max(0, (Quality?.totalVideoFrames ?? 0) - (Quality?.droppedVideoFrames ?? 0));
      const Fps = (Delivered - PreviousCount) * 1000 / Math.max(1, Now - PreviousTime);
      // A collapsed video can stop presentation callbacks while capture keeps
      // delivering frames. Advancing media time still proves source freshness.
      if (VideoElement.currentTime !== PreviousMediaTime) LastFrame.current = Now;
      PreviousMediaTime = VideoElement.currentTime;
      if (typeof VideoElement.requestVideoFrameCallback !== "function" && Delivered > PreviousCount) LastFrame.current = Now;
      const Stalled = !!VideoElement.srcObject && Now - LastFrame.current > 2000;
      if (Stream?.active && VideoElement.srcObject === Stream && Stalled === LastReady) Report(Stalled ? "Camera feed paused · reconnect to recover" : LiveText, !Stalled);
      VideoElement.dataset.fps = String(Math.round(Fps));
      VideoElement.dataset.dropped = String(Quality?.droppedVideoFrames ?? 0);
      PreviousTime = Now; PreviousCount = Delivered;
    }, 1000);
    return () => { Cancelled = true; Abort.abort(); window.clearInterval(Timer); if (FrameId) VideoElement?.cancelVideoFrameCallback(FrameId);
      if (VideoElement) DepthCanvases.delete(VideoElement);
      Stream?.getTracks().forEach(Track => Track.stop()); if (VideoElement?.srcObject === Stream) { VideoElement.srcObject = null; VideoElement.dataset.captureReady = "false"; } };
  }, [Profile, Source, DepthFrameUrl, ReconnectKey, OnActive, OnStatus, VideoRef]);
  useEffect(() => { if (!Expanded) return; const Close = (Event: KeyboardEvent) => { if (Event.key === "Escape") SetExpanded(false); }; window.addEventListener("keydown", Close); return () => window.removeEventListener("keydown", Close); }, [Expanded]);
  const OverlayToggle = Source === "depth" ? <button type="button" aria-label="Depth overlay" aria-pressed={DepthOverlay}
    title={DepthOverlay ? "Hide the depth overlay" : "Show depth over the live view (blue near, red far)"}
    onClick={() => OnDepthOverlayChange(!DepthOverlay)}><Layers size={16} /></button> : null;
  return <FloatingCameraWindow Expanded={Expanded} OnExpandedChange={SetExpanded} Actions={OverlayToggle}>
    <div ref={FrameRef} className="video-frame" data-depth-hover={ShowOverlay} onPointerMove={ShowOverlay ? TrackHover : undefined} onPointerLeave={EndHover}>
      <video ref={VideoRef} autoPlay playsInline muted />
      {/* With the depth readout on, a magnifier follows the pointer too (the exact frame, from the depth canvas). */}
      <HoverLoupe ElementRef={VideoRef} ListenRef={FrameRef} Enabled={ShowOverlay && Active} Live HideWhilePressed Name="Live view magnifier"
        GetSource={() => { const Canvas = DepthCanvasFor(VideoRef.current); return Canvas && Canvas.width > 0 ? { Source: Canvas, Width: Canvas.width, Height: Canvas.height } : null; }} />
      {ShowOverlay && <canvas ref={OverlayRef} className="depth-overlay" aria-hidden="true" />}
      {ShowOverlay && <div className="depth-legend" role="status">
        {HoverText ?? OverlayNote ?? (Near && Far ? `Depth: blue ${Near} cm to red ${Far} cm` : "Depth overlay")}</div>}
      {!Active && <div className="video-placeholder robot-avatar"><Bot size={50} strokeWidth={1.2} /><span>{State}</span></div>}
      {Countdown !== null && <div className="countdown-overlay"><span>{Countdown}</span></div>}
    </div>
  </FloatingCameraWindow>;
});
