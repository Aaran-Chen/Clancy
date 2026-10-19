import { useEffect, useRef, useState } from "react";
import { CheckCircle2, Info, AlertTriangle, X } from "lucide-react";
export type HudMessage = { id: number; level: "info" | "ok" | "warn"; text: string };
export const NoticeDurationMs = 10_000;
function Notice({ Message, OnDismiss }: { Message: HudMessage; OnDismiss: (Id: number) => void }) {
  const Dismiss = useRef(OnDismiss); Dismiss.current = OnDismiss;
  useEffect(() => { const Timer = window.setTimeout(() => Dismiss.current(Message.id), NoticeDurationMs); return () => window.clearTimeout(Timer); }, [Message.id]);
  const Icon = Message.level === "ok" ? CheckCircle2 : Message.level === "warn" ? AlertTriangle : Info;
  return <div className={`hud-notice ${Message.level}`} role="status" data-notice-id={Message.id}>
    <Icon size={19} /><div><span>SYSTEM UPDATE</span><p>{Message.text}</p></div>
    <button type="button" aria-label="Dismiss notification" onClick={() => OnDismiss(Message.id)}><X size={15} /></button>
    <div className="notice-timer" />
  </div>;
}
export default function HudNotice({ Messages }: { Messages: HudMessage[] }) {
  const [Visible, SetVisible] = useState<HudMessage[]>([]);
  const Seen = useRef(new Set<number>());
  useEffect(() => {
    const New = Messages.filter(Message => !Seen.current.has(Message.id));
    if (!New.length) return;
    New.forEach(Message => Seen.current.add(Message.id));
    if (Seen.current.size > 100) Seen.current = new Set(Messages.map(Message => Message.id));
    SetVisible(Previous => {
      const Combined = [...New, ...Previous];
      return Combined.filter((Message, Index) => Combined.findIndex(Item => Item.level === Message.level && Item.text === Message.text) === Index).slice(0, 3);
    });
  }, [Messages]);
  return <div className="hud-notices" aria-label="System notifications">{Visible.map(Message => <Notice key={Message.id} Message={Message} OnDismiss={Id => SetVisible(Previous => Previous.filter(Item => Item.id !== Id))} />)}</div>;
}
