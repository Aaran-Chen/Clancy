import { useId, useLayoutEffect, useRef, useState } from "react";
import { Maximize2, X } from "lucide-react";

// The AI's last reply. It takes the height the AI panel has left (Dashboard.css);
// when that is too short for all of it, it shows the start, faded, and a button
// opens the whole reply above the workspace, so the panel never scrolls.
export default function AiReply({ Text }: { Text: string }) {
  const TextRef = useRef<HTMLSpanElement>(null);
  const [Truncated, SetTruncated] = useState(false);
  const FullId = useId();
  useLayoutEffect(() => {
    const Element = TextRef.current;
    if (!Element) return;
    const Check = () => SetTruncated(Element.scrollHeight > Element.clientHeight + 1);
    Check();
    const Observer = new ResizeObserver(Check);
    Observer.observe(Element);
    return () => Observer.disconnect();
  }, [Text]);
  return (
    <div className="ai-reply" data-truncated={Truncated}>
      <span className="ai-reply-label">AI</span>
      <span className="ai-reply-text" ref={TextRef}>{Text}</span>
      {Truncated ? (
        <button type="button" className="ai-reply-more" popoverTarget={FullId} aria-label="Show the whole AI reply" title="Show the whole reply">
          <Maximize2 size={13} />
        </button>
      ) : null}
      <div className="ai-reply-full" id={FullId} popover="auto" role="dialog" aria-label="AI reply">
        <header><span>AI reply</span>
          <button type="button" popoverTarget={FullId} popoverTargetAction="hide" aria-label="Close the AI reply"><X size={15} /></button>
        </header>
        {Text}
      </div>
    </div>
  );
}
