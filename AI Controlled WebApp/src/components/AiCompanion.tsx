import { useLayoutEffect, useRef } from "react";

type AiCompanionProps = {
  Model: string | null;
  Busy: boolean;
  Moving: boolean;
  Waiting: boolean;
  HasProposal: boolean;
  Fault: string | null;
  Status: string;
};

/** Decorative vector pixels stay sharp without an image download or canvas loop.
    This 24-grid drawing is the 24px mode-rail icon, where a 48-grid one would land on half pixels. */
export function PixelBot() {
  return (
    <svg className="pixel-bot" viewBox="0 0 24 24" fill="none" shapeRendering="crispEdges" aria-hidden="true" focusable="false">
      <path className="pixel-bot-antenna" d="M11 1h2v2h-2zM12 3h1v2h-1z" fill="currentColor" />
      <path d="M6 5h12v2h2v10h-2v2H6v-2H4V7h2z" fill="currentColor" />
      <path d="M7 6h10v1H7zM5 8h1v7H5z" fill="white" opacity=".4" />
      <path d="M7 8h10v8H7z" className="pixel-bot-screen" />
      <g className="pixel-bot-eyes" fill="currentColor">
        <path d="M8 10h2v3H8zM14 10h2v3h-2z" />
      </g>
      <path className="pixel-bot-mouth" d="M10 14h4v1h-4z" fill="currentColor" />
      <path d="M1 10h2v5H1zM21 10h2v5h-2zM8 20h3v2H8zM14 20h3v2h-3z" fill="currentColor" opacity=".65" />
    </svg>
  );
}

/** Clancy at 48x48. Every pose and face lives here on the integer grid;
    ArcadeHud.css picks the pose for each data-state and steps whole art pixels or
    swaps whole frames, never a smooth tween. useCrispPortrait sizes and places it. */
function PixelBotPortrait() {
  return (
    <svg className="pixel-bot pixel-bot-portrait" viewBox="0 0 48 48" fill="none" shapeRendering="crispEdges" aria-hidden="true" focusable="false">
      {/* One rig, so a shake moves every part by whole art pixels, never CSS pixels. */}
      <g className="pixel-bot-rig">
        <g className="pixel-bot-feet" fill="currentColor" opacity=".65">
          <path className="pixel-bot-foot-left" d="M17 39h2v2h-2zM16 41h4v1h-4zM15 42h6v1h-6zM16 43h4v1h-4z" />
          <path className="pixel-bot-foot-right" d="M29 39h2v2h-2zM28 41h4v1h-4zM27 42h6v1h-6zM28 43h4v1h-4z" />
        </g>
        <g className="pixel-bot-body">
          <g fill="currentColor" opacity=".65">
            <path className="pixel-bot-arm-left" d="M3 21h2v1h-2zM2 22h4v2h-4zM2 24h6v4h-6zM2 28h4v2h-4zM3 30h2v1h-2z" />
            <path className="pixel-bot-arm-right" d="M43 21h2v1h-2zM42 22h4v2h-4zM40 24h6v4h-6zM42 28h4v2h-4zM43 30h2v1h-2z" />
          </g>
          <g className="pixel-bot-antenna">
            <path d="M23 7h2v4h-2z" fill="currentColor" />
            <path className="pixel-bot-glow" d="M23 0h2v2h-2zM21 2h1v1h-1zM26 2h1v1h-1zM19 4h2v2h-2zM27 4h2v2h-2z" />
            <path className="pixel-bot-bulb" d="M23 3h2v1h-2zM22 4h4v2h-4zM23 6h2v1h-2z" />
            <path className="pixel-bot-bulb-shine" d="M23 4h1v1h-1z" fill="white" opacity=".6" />
          </g>
          {/* Head, its light and shade, and the inset screen bezel. */}
          <path d="M11 11h26v1h-26zM9 12h30v2h-30zM8 14h32v22h-32zM9 36h30v2h-30zM11 38h26v1h-26z" fill="currentColor" />
          <path d="M12 12h24v1h-24zM10 13h1v2h-1zM9 15h1v17h-1z" fill="white" opacity=".4" />
          <path d="M38 17h1v18h-1zM37 35h1v2h-1zM12 37h24v1h-24z" fill="black" opacity=".22" />
          <path d="M13 15h22v1h-22z" fill="black" opacity=".3" />
          <path d="M13 34h22v1h-22z" fill="white" opacity=".18" />
          <path className="pixel-bot-screen" d="M13 16h22v1h-22zM12 17h24v16h-24zM13 33h22v1h-22z" />
          <path d="M31 17h3v1h-3zM34 18h1v2h-1z" fill="white" opacity=".1" />
          <g fill="currentColor">
            {/* The gaze moves every pair of eyes together, so a blink keeps the look. */}
            <g className="pixel-bot-gaze">
              <g className="pixel-bot-eyes">
                <path d="M17 19h2v1h-2zM29 19h2v1h-2zM16 20h4v4h-4zM28 20h4v4h-4zM17 24h2v1h-2zM29 24h2v1h-2z" />
                <path d="M17 20h1v1h-1zM29 20h1v1h-1z" fill="white" opacity=".8" />
              </g>
              <path className="pixel-bot-eyes-half" d="M16 22h4v3h-4zM28 22h4v3h-4z" />
              <path className="pixel-bot-eyes-shut" d="M16 23h4v2h-4zM28 23h4v2h-4z" />
              <path className="pixel-bot-eyes-x" d="M15 19h2v1h-2zM19 19h2v1h-2zM27 19h2v1h-2zM31 19h2v1h-2zM16 20h4v1h-4zM28 20h4v1h-4zM17 21h2v2h-2zM29 21h2v2h-2zM16 23h4v1h-4zM28 23h4v1h-4zM15 24h2v1h-2zM19 24h2v1h-2zM27 24h2v1h-2zM31 24h2v1h-2z" />
            </g>
            <path className="pixel-bot-blush" d="M13 26h3v1h-3zM32 26h3v1h-3z" />
            <path className="pixel-bot-mouth" d="M20 28h8v2h-8z" />
            <path className="pixel-bot-smile" d="M18 27h2v2h-2zM28 27h2v2h-2zM20 29h8v2h-8z" />
            <path className="pixel-bot-mouth-o" d="M23 27h2v1h-2zM22 28h1v2h-1zM25 28h1v2h-1zM23 30h2v1h-2z" />
            <path className="pixel-bot-mouth-zigzag" d="M19 28h2v1h-2zM23 28h2v1h-2zM27 28h2v1h-2zM21 29h2v1h-2zM25 29h2v1h-2z" />
            <g className="pixel-bot-loader">
              <path d="M19 28h2v2h-2z" />
              <path d="M23 28h2v2h-2z" />
              <path d="M27 28h2v2h-2z" />
            </g>
          </g>
        </g>
        {/* State props around the head: a question and a sparkle, or the zzz of sleep. */}
        <path className="pixel-bot-question" d="M39 1h4v1h-4zM38 2h2v1h-2zM42 2h2v2h-2zM41 4h2v1h-2zM40 5h2v1h-2zM40 7h2v1h-2z" />
        <path className="pixel-bot-sparkle" d="M6 4h1v2h-1zM4 6h2v1h-2zM7 6h2v1h-2zM6 7h1v2h-1z" fill="white" />
        <path className="pixel-bot-sparkle-small" d="M6 5h1v1h-1zM5 6h1v1h-1zM7 6h1v1h-1zM6 7h1v1h-1z" fill="white" />
        <g className="pixel-bot-snore" fill="currentColor">
          <path d="M30 6h4v1h-4zM32 7h1v1h-1zM31 8h1v1h-1zM30 9h4v1h-4z" />
          <path d="M35 2h5v1h-5zM38 3h1v1h-1zM37 4h1v1h-1zM36 5h1v1h-1zM35 6h5v1h-5z" />
          <path d="M41 0h6v1h-6zM45 1h1v1h-1zM44 2h1v1h-1zM43 3h1v1h-1zM42 4h1v1h-1zM41 5h6v1h-6z" />
        </g>
      </g>
    </svg>
  );
}

/** Pixel art is only crisp when one art pixel is a whole number of device pixels:
    at 1 CSS pixel each, a 2.25 display draws them alternately 2 and 3 pixels wide.
    This picks that whole number (the one closest to the 48 CSS px Clancy has always
    been, with the full 48 grid inside the portrait) and puts the sprite's corner on a
    device pixel. No loop: it runs again only when something could have moved it,
    a resize, a scroll, a new pixel ratio (browser zoom, another monitor) or a CSS
    zoom on an ancestor (the dashboard's column fit scales a crowded column's panels). */
function useCrispPortrait() {
  const PortraitRef = useRef<HTMLDivElement>(null);
  useLayoutEffect(() => {
    const Portrait = PortraitRef.current, Sprite = Portrait?.querySelector<SVGSVGElement>(".pixel-bot-portrait");
    if (!Portrait || !Sprite) return;
    let Frame = 0, Ratio: MediaQueryList | null = null;
    const Place = () => {
      Frame = 0;
      // Under a CSS zoom, sizes and offsets are set in the portrait's own (zoomed) units,
      // client sizes come back in them, and the bounding box comes back in viewport pixels.
      const Zoom = Portrait.currentCSSZoom || 1, Dpr = window.devicePixelRatio || 1, Device = Dpr * Zoom;
      const Room = Math.min(Portrait.clientWidth, Portrait.clientHeight);
      const Scale = Math.max(1, Math.min(Math.round(Device), Math.floor(Room * Device / 48)));
      const Size = 48 * Scale / Dpr, Bounds = Portrait.getBoundingClientRect();
      // Centre in the padding box, then round that corner to the nearest device pixel.
      const Snap = (Edge: number, Span: number) => (Math.round((Edge + (Span - Size) / 2) * Dpr) / Dpr - Edge) / Zoom;
      const Left = Snap(Bounds.left + Portrait.clientLeft * Zoom, Portrait.clientWidth * Zoom);
      const Top = Snap(Bounds.top + Portrait.clientTop * Zoom, Portrait.clientHeight * Zoom);
      Sprite.style.width = Sprite.style.height = `${Size / Zoom}px`;
      Sprite.style.left = `${Left}px`;
      Sprite.style.top = `${Top}px`;
      // A zoomed border is drawn in whole device pixels, so the padding box can sit a fraction
      // away from where its zoomed width says: measure where the corner landed and nudge it once.
      if (Zoom !== 1) {
        const Placed = Sprite.getBoundingClientRect();
        const Nudge = (Edge: number) => (Math.round(Edge * Dpr) / Dpr - Edge) / Zoom;
        Sprite.style.left = `${Left + Nudge(Placed.left)}px`;
        Sprite.style.top = `${Top + Nudge(Placed.top)}px`;
      }
      Sprite.dataset.scale = String(Scale);
    };
    const Schedule = () => { if (!Frame) Frame = requestAnimationFrame(Place); };
    // A resolution query stops matching once the ratio changes, so watch the new one each time.
    const WatchRatio = () => {
      Ratio?.removeEventListener("change", OnRatio);
      Ratio = window.matchMedia(`(resolution: ${window.devicePixelRatio}dppx)`);
      Ratio.addEventListener("change", OnRatio);
    };
    function OnRatio() { WatchRatio(); Schedule(); }
    // The portrait shifts by fractions of a pixel when it, an ancestor or anything laid out before them resizes.
    const Observer = new ResizeObserver(Place);
    // A zoom set on an ancestor's style scales the portrait without resizing it in its own units.
    const Restyled = new MutationObserver(Schedule);
    for (let Node: Element | null = Portrait; Node && Node !== document.body; Node = Node.parentElement) {
      Observer.observe(Node);
      if (Node !== Portrait) Restyled.observe(Node, { attributes: true, attributeFilter: ["style"] });
      for (let Before = Node.previousElementSibling; Before; Before = Before.previousElementSibling) Observer.observe(Before);
    }
    Place();
    WatchRatio();
    window.addEventListener("resize", Schedule);
    window.addEventListener("scroll", Schedule, { capture: true, passive: true });
    return () => {
      cancelAnimationFrame(Frame);
      Observer.disconnect();
      Restyled.disconnect();
      Ratio?.removeEventListener("change", OnRatio);
      window.removeEventListener("resize", Schedule);
      window.removeEventListener("scroll", Schedule, { capture: true });
    };
  }, []);
  return PortraitRef;
}

export default function AiCompanion({ Model, Busy, Moving, Waiting, HasProposal, Fault, Status }: AiCompanionProps) {
  const PortraitRef = useCrispPortrait();
  const State = Fault ? "fault" : !Model ? "offline" : Busy ? "thinking" : Waiting ? "settling" : Moving ? "moving" : HasProposal ? "review" : "ready";
  const Copy = {
    fault: ["Arm needs attention", Fault || "Check the motion panel before continuing."],
    // No model yet: Status says why (Ollama starting, or nothing installed).
    offline: ["Connecting to the AI model", Status],
    thinking: ["Thinking it through", Status],
    settling: ["Waiting for the arm", "The next observation starts after motion settles."],
    moving: ["Motion in progress", "You can stop the arm at any time."],
    review: ["A move to review", "Check the proposed target before approving."],
    ready: ["Ready when you are", "Describe a task. You approve every move."],
  }[State];

  return (
    <div className="ai-companion" data-state={State}>
      <div className="ai-companion-portrait" ref={PortraitRef}><PixelBotPortrait /></div>
      <div className="ai-companion-copy" role="status" aria-live="polite" aria-atomic="true">
        <span className="ai-companion-name">Clancy <span>AI assistant</span></span>
        <strong>{Copy[0]}</strong>
        <p>{Copy[1]}</p>
      </div>
      <span className="ai-companion-light" title={Copy[0]} aria-hidden="true" />
    </div>
  );
}
