// The session video (back end/SessionRecorder.py) records the depth camera and the demo-area webcam only while an
// AI task runs (Aaran, 2026-10-08), from 5 s before it starts to 10 s after it ends, and keeps it 7 days. This page
// says when its task starts, that it still runs (every TaskAliveMs) and when it ends, through the dev server
// (POST /session-log/recording, scripts/SessionVideo.ts), which passes it on to the recorder on the robot's PC. A
// page that closes says so on its way out; one that crashes goes quiet, and the recorder stops 3 minutes later.

export const TaskAliveMs = 15_000;
export type TaskVideoState = "start" | "alive" | "end";
export type TaskVideoDetails = { Task?: string; Reason?: string };
type Sender = (State: TaskVideoState, Key: string, Details?: TaskVideoDetails) => void;

// One task's recording key: distinct across this page's tasks and other pages' (crypto.randomUUID needs a secure
// context, which another device's http:// page is not).
export function NewTaskVideoKey(Random: () => number = Math.random, Now: () => number = Date.now): string {
  return `${Now().toString(36)}-${Math.floor(Random() * 36 ** 8).toString(36).padStart(8, "0")}`;
}

export function TaskVideoBody(State: TaskVideoState, Key: string, Details: TaskVideoDetails = {}): Record<string, string> {
  return { state: State, task_id: Key, ...(Details.Task ? { task: Details.Task.slice(0, 500) } : {}),
    ...(Details.Reason ? { reason: Details.Reason.slice(0, 300) } : {}) };
}

// Fire and forget, like the session record: the recording never delays or breaks a task. keepalive lets the last
// word go out while the page closes. Automated test browsers stay out, as they do of the record.
export function SignalTaskVideo(State: TaskVideoState, Key: string, Details: TaskVideoDetails = {}): void {
  if (typeof navigator !== "undefined" && navigator.webdriver) return;
  try {
    void fetch("/session-log/recording", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(TaskVideoBody(State, Key, Details)), keepalive: true,
    }).catch(() => {});
  } catch {
    // Recording is best effort.
  }
}

// This page's side of it. Update with the running task's id (App.tsx TaskIdRef, new for every task) on every
// change of the task, or null once none runs: it starts, ends or switches the recording, and says "alive" while one
// runs. A step of the same task (the same id) changes nothing.
export class TaskVideoSignals {
  private Current: { Id: number; Key: string } | null = null;
  private Timer: ReturnType<typeof setInterval> | null = null;

  constructor(private readonly Send: Sender = SignalTaskVideo, private readonly NewKey: () => string = NewTaskVideoKey,
              private readonly EveryMs: number = TaskAliveMs) {}

  Update(Id: number | null, Task: string, EndReason: string | null): void {
    if (Id != null && this.Current?.Id === Id) return;
    if (this.Current) this.End(EndReason ?? (Id != null ? "a new task replaced it" : "the task ended"));
    if (Id == null) return;
    const Started = { Id, Key: this.NewKey() };
    this.Current = Started;
    this.Send("start", Started.Key, { Task });
    this.Timer = setInterval(() => { if (this.Current === Started) this.Send("alive", Started.Key); }, this.EveryMs);
  }

  End(Reason: string): void {
    if (this.Timer) clearInterval(this.Timer);
    this.Timer = null;
    if (this.Current) this.Send("end", this.Current.Key, { Reason });
    this.Current = null;
  }

  get Running(): string | null {
    return this.Current?.Key ?? null;
  }
}
