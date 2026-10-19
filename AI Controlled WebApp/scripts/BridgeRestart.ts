import { spawn } from "node:child_process";

// The PowerShell helper verifies a fresh bridge state before it can terminate
// the old process. Starting a new process does not connect or enable the arm.
export function RunBridgeRestart(ScriptPath: string, Launch = spawn): Promise<void> {
  return new Promise((Resolve, Reject) => {
    const Child = Launch("powershell.exe", ["-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", ScriptPath], { windowsHide: true });
    let Output = "", Settled = false;
    Child.stdin?.end();
    const Capture = (Chunk: Buffer) => { Output = (Output + Chunk.toString()).slice(-8000); };
    Child.stdout?.on("data", Capture); Child.stderr?.on("data", Capture);
    const Finish = (Code: number | null) => {
      if (Settled) return;
      Settled = true;
      clearTimeout(Timer);
      if (Code === 0) { Resolve(); return; }
      Reject(new Error(/Disconnect the arm|arm is not confirmed disconnected/.test(Output)
        ? "The arm is still connected. Support it and click Disconnect before loading the bridge update."
        : "Bridge restart failed. Check api-error.log and retry after confirming the arm is disconnected."));
    };
    const Timer = setTimeout(() => {
      if (Settled) return;
      Settled = true;
      Child.kill();
      Reject(new Error("Bridge restart timed out. Check the bridge log before retrying."));
    }, 25_000);
    Child.once("error", Failure => { if (Settled) return; Settled = true; clearTimeout(Timer); Reject(Failure); });
    // Settle on exit, not close: the new bridge inherits this script's output
    // pipes, so 'close' only fires once that bridge stops. The short delay lets
    // the script's last output arrive before it is read.
    Child.once("exit", Code => { setTimeout(() => Finish(Code), 50); });
    Child.once("close", Code => Finish(Code));
  });
}
