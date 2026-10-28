"""How long the bridge holds the motor lock, and what disk work it does meanwhile. Offline only.

Never opens a serial port or a camera: a FakeBus (test_bridge_home) stands in for the servos, as in
TestSafety. The control loop waits for the motor lock, so a hold here is a control frame that waits
(the 2026-10-13..25 control_stall records: "the motor lock, held by asyncio_N", whose stack dumps
show a WebSocket client's state snapshot reading WorkspaceFrame.json from disk under it).

    python MeasureLockHolds.py            # normal disk, then every file access made 200 ms slow
    python MeasureLockHolds.py --slow 0.5 # the slow disk's delay per access, in seconds

Rows (each on its own fake arm, so one row's trip does not carry into the next)
  state snapshot     one WebSocket client's state message (state_snapshot), 30 a second per client
  control frame      one 30 Hz control frame (refresh) of a holding arm
  route dispatch     a gripper move from the command to its route start (move_servo), both lock sections
  route frame        one control frame of a running route (CheckMotionRoute, BeginRouteStage)
Columns: the longest and the median motor-lock hold in ms, and the disk accesses (stat, open,
read) made while the thread held the lock, with the bridge function that made them. A protection
trip the row ended with is printed under it.
"""
from __future__ import annotations

import argparse
import builtins
import collections
import io
import os
import statistics
import sys
import threading
import time
import traceback
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("SO101_DEPTH_CAMERA", "0")
os.environ.setdefault("SO101_RECORDER", "0")

import bridge_server as Bridge  # noqa: E402
from TestSafety import MakeController  # noqa: E402

Holds: list[float] = []
DiskUnderLock: collections.Counter = collections.Counter()
Held = threading.local()  # .Since: when this thread took a motor lock (perf_counter), .Busy: inside a disk wrapper


class Refused(Exception):
    """The route would not start."""


def Caller() -> str:
    """The innermost bridge_server function on this stack (who did the disk access)."""
    for Frame in reversed(traceback.extract_stack(limit=25)[:-2]):
        if Frame.filename.endswith("bridge_server.py"):
            return f"{Frame.name}:{Frame.lineno}"
    return "?"


def Watch(Slow: float):
    """Patches: every motor-lock hold is timed (perf_counter: the lock's own time.monotonic ticks every 15.6 ms on
    Windows); every disk access sleeps Slow, and is counted when its thread holds a motor lock."""
    Lock = Bridge.InstrumentedLock
    Acquire, Release = Lock.acquire, Lock.release

    def Taken(Self, *Args, **Options):
        Got = Acquire(Self, *Args, **Options)
        if Got and Self._Depth == 1:
            Held.Since = time.perf_counter()
        return Got

    def Timed(Self):
        if Self._Depth == 1 and getattr(Held, "Since", None) is not None:
            Holds.append(time.perf_counter() - Held.Since)
            Held.Since = None
        Release(Self)

    def Disk(Original, Kind):
        def Wrapped(*Args, **Options):
            if not getattr(Held, "Busy", False):
                Held.Busy = True
                try:
                    if getattr(Held, "Since", None) is not None:
                        DiskUnderLock[f"{Kind} in {Caller()}"] += 1
                    if Slow:
                        time.sleep(Slow)
                finally:
                    Held.Busy = False
            return Original(*Args, **Options)
        return Wrapped
    return [patch.object(Lock, "acquire", Taken), patch.object(Lock, "__enter__", Taken),
            patch.object(Lock, "release", Timed),
            patch.object(Path, "stat", Disk(Path.stat, "stat")),
            patch.object(Path, "read_text", Disk(Path.read_text, "read")),
            patch.object(Path, "read_bytes", Disk(Path.read_bytes, "read")),
            patch.object(builtins, "open", Disk(builtins.open, "open")),
            patch.object(io, "open", Disk(io.open, "open"))]


def Measure(Name, Action, Count, Slow, Controller):
    Holds.clear()
    DiskUnderLock.clear()
    Held.Since = None
    Note = ""
    Patches = Watch(Slow)
    for Each in Patches:
        Each.start()
    try:
        for _ in range(Count):
            Action()
    except Refused as Text:
        Note = f"; stopped: {Text}"
    finally:
        for Each in reversed(Patches):
            Each.stop()
    Longest = max(Holds) * 1000 if Holds else 0.0
    Median = statistics.median(Holds) * 1000 if Holds else 0.0
    Disk = ", ".join(f"{Where} x{N}" for Where, N in DiskUnderLock.most_common()) or "none"
    print(f"  {Name:<16} longest {Longest:7.1f} ms   median {Median:6.2f} ms   disk under the lock: {Disk}")
    if Controller.ProtectionTrip or Note:
        print(f"  {'':<16} protection trip: {(Controller.ProtectionTrip or {}).get('reason')}{Note}")
    return Longest


def GripperRoute(Controller):
    """A gripper-only route opening the jaws 300 steps (MakeController's arm is holding, nothing else moves)."""
    Gripper = Controller.servos["gripper"]
    Ok, Text = Controller.move_servo("gripper", Controller._clamp_raw(Gripper, Gripper.present_raw + 300))
    if not Ok:
        raise Refused(Text)


def Run(Slow: float) -> dict[str, float]:
    print(f"File accesses {'%g ms slow each' % (Slow * 1000) if Slow else 'at disk speed'}:")
    Count = 10 if Slow else 200
    Found = {}
    Controller, Bus = MakeController()
    Found["state snapshot"] = Measure("state snapshot", Controller.state_snapshot, Count, Slow, Controller)
    Controller, Bus = MakeController()
    Found["control frame"] = Measure("control frame", Controller.refresh, Count, Slow, Controller)
    Arms = [MakeController()[0] for _ in range(3)]
    Found["route dispatch"] = Measure("route dispatch", lambda: GripperRoute(Arms.pop()), 3, Slow, Arms[0])
    Controller, Bus = MakeController()
    GripperRoute(Controller)

    def RouteFrame():
        for Name in Bridge.MOTOR_NAMES:
            Bus.step(Name, steps_per_tick=2)  # about 150 frames for the route: it runs through every measured one
        Controller.refresh()
    Found["route frame"] = Measure("route frame", RouteFrame, 10 if Slow else 60, Slow, Controller)
    return Found


def main() -> int:
    Parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    Parser.add_argument("--slow", type=float, default=0.2, help="seconds each file access takes on the slow disk")
    Options = Parser.parse_args()
    Run(0.0)
    Run(Options.slow)
    return 0


if __name__ == "__main__":
    sys.exit(main())
