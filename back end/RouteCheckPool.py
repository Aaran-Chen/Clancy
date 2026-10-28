"""Route collision checks in worker processes, so a route's checks run side by side instead of one after another.

Planning spends most of its time in one call, BasketCollision's EdgeReason: the waypoint reach of
the 27.3 cm move of 2026-10-04 15:31 (bridge_server.AssignRouteReach) cost 16 of them, 1.4 s of its
1.6 s plan offline, and the live bridge took about twice as long as offline, because the camera,
the web server and the planner share one interpreter lock. Under that load the reach clock
(ROUTE_REACH_ASSIGN_SECONDS) also ran out before the wider reaches were tried, which slowed the
route itself. Each worker here is its own Python process (its own lock, its own core) running the
very same check on the same data: EdgeReason of a CollisionWorld built from the geometry and frame
the bridge sends. It answers only what the bridge asks; the bridge decides what to do with it.

    Pool = RouteCheckPool.Start()        # spawns the workers in the background; Ready() once they answer
    Results = Pool.Map(Requests, 2.0)    # ("ok", reason or None) each, None where no worker answered in time

A request is (Geometry, Frame, Start, Goal, Budget, Relief, Reach), or (WorldRef, kind, *arguments): a WorldRef names
the world, which each worker builds once and keeps (a measured scene took 34 ms to build, per request before), and the
kind is the CollisionWorld method it runs (Kinds: "edge" EdgeReason, "path" PathFailure, the route search's candidate
paths, BasketCollision.PathScout).

A worker is used only when its collision code and data are byte for byte the bridge's own (the
Fingerprint of the files below, taken when each side imported them): one started after a file
changed is turned away at its first answer, and the bridge does the checks itself. Workers never
touch a serial port, a camera or the network; they exit when the bridge does (their input pipe
closes).
"""
from __future__ import annotations

import hashlib
import os
import pickle
import queue
import struct
import subprocess
import sys
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path

import BasketCollision as Collision

Root = Path(__file__).resolve().parent
# Everything EdgeReason reads: the collision modules and the data they load at import.
FingerprintFiles = ("BasketCollision.py", "BasketSurface.py", "ArmMotionEnvelope.py", "CadKinematics.py",
                    "CollisionModel.json", "BasketSurface.json", "RobotBodySurface.json", "CadModel.json", "SceneSafety.py")
# One worker per four logical cores, at most this many: a long route's reach is about 16 checks, so 8
# finish it in two rounds, and the rest of the machine (the bridge's control loop above all) keeps its cores.
MaximumWorkers = 8
# A worker that has not imported and answered by then is given up on (it normally takes about 0.3 s).
StartSeconds = 20.0
_Header = struct.Struct("<I")


def Fingerprint() -> str:
    Hash = hashlib.sha256()
    for Name in FingerprintFiles:
        Hash.update(Name.encode())
        Hash.update((Root / Name).read_bytes())
    return Hash.hexdigest()[:16]


# The collision code this process loaded (it imported BasketCollision above).
LoadedFingerprint = Fingerprint()


def _Write(Stream, Message) -> None:
    Data = pickle.dumps(Message, protocol=pickle.HIGHEST_PROTOCOL)
    Stream.write(_Header.pack(len(Data)) + Data)
    Stream.flush()


def _Read(Stream):
    Head = Stream.read(_Header.size)
    if len(Head) < _Header.size:
        raise EOFError("check worker pipe closed")
    (Length,) = _Header.unpack(Head)
    Data = Stream.read(Length)
    if len(Data) < Length:
        raise EOFError("check worker pipe closed")
    return pickle.loads(Data)


def Check(Geometry, Frame, Start, Goal, Budget, Relief, Reach):
    """The check itself, as bridge_server.ValidateMotionPath runs it: why the edge fails, or None."""
    return Collision.CollisionWorld(Geometry, Frame).EdgeReason(Start, Goal, Budget=Budget, Relief=Relief, Reach=Reach)


class WorldRef:
    """A collision world (CollisionWorld(Geometry, Frame) under this process's collision model) by key.

    A request naming one carries the world itself (Payload) only to a worker that has not built it yet; each
    worker keeps the few it built last (WorldsKept). A measured scene of ~1400 boxes took 34 ms to build, once
    per request before. The payload holds the bridge's Model too: a worker whose own differs (a test patched the
    bridge's) refuses, and the bridge checks itself."""

    __slots__ = ("Key", "Payload")

    def __init__(Self, Geometry, Frame):
        Self.Payload = pickle.dumps((Geometry, Frame, Collision.Model), protocol=pickle.HIGHEST_PROTOCOL)
        Self.Key = hashlib.sha256(Self.Payload).hexdigest()[:24]


# Worlds a worker keeps built (WorldRef), the most recently used.
WorldsKept = 4
# Request kinds a WorldRef request may name: the CollisionWorld method each runs, with the request's arguments.
Kinds = {"edge": "EdgeReason", "path": "PathFailure"}


class Worker:
    """One worker process and the thread that reads its answers."""

    def __init__(Self):
        # WorldRef keys this worker was sent (it may have dropped one since: it then answers "miss").
        Self.Known: set[str] = set()
        Flags = 0
        if sys.platform == "win32":
            Flags = subprocess.CREATE_NO_WINDOW | subprocess.NORMAL_PRIORITY_CLASS
        Self.Process = subprocess.Popen([sys.executable, "-u", str(Path(__file__).resolve()), "--worker"], cwd=str(Root),
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                        creationflags=Flags)
        Self.Answers: queue.Queue = queue.Queue()
        Self.Alive = True
        threading.Thread(target=Self._ReadAnswers, name="RouteCheckReader", daemon=True).start()

    def _ReadAnswers(Self):
        try:
            while True:
                Self.Answers.put(_Read(Self.Process.stdout))
        except Exception:
            Self.Alive = False
            Self.Answers.put(None)

    def Answer(Self, Seconds):
        try:
            Answer = Self.Answers.get(timeout=max(0.0, Seconds))
        except queue.Empty:
            Answer = None
        if Answer is None:
            Self.Stop()
        return Answer

    def Call(Self, Request, Seconds):
        Deadline = time.monotonic() + Seconds
        Ref = Request[0] if Request and isinstance(Request[0], WorldRef) else None
        for Attempt in range(2):
            if Ref is None:
                Message = ("check", *Request)
            else:
                Message = ("call", Ref.Key, None if Ref.Key in Self.Known else Ref.Payload, Request[1], tuple(Request[2:]))
            try:
                _Write(Self.Process.stdin, Message)
            except Exception:
                Self.Stop()
                return None
            Answer = Self.Answer(Deadline - time.monotonic())
            if Ref is None or Answer is None:
                return Answer
            if Answer == ("miss",):
                Self.Known.discard(Ref.Key)  # It dropped that world: send it again, once.
                continue
            Self.Known.add(Ref.Key)
            return Answer
        return ("error", "worker kept missing the world")

    def Stop(Self):
        # A worker that missed its deadline may still answer later, out of turn: it is never reused.
        Self.Alive = False
        try:
            Self.Process.kill()
        except Exception:
            pass


class Pool:
    def __init__(Self, Count: int):
        Self.Count = Count
        Self.Idle: queue.Queue = queue.Queue()
        Self.Workers: list[Worker] = []
        Self.Threads = ThreadPoolExecutor(max_workers=Count, thread_name_prefix="RouteCheck")
        Self.Problem: str | None = None
        Self.Started = threading.Event()

    def _Start(Self):
        Deadline = time.monotonic() + StartSeconds
        try:
            Pending = [Worker() for _ in range(Self.Count)]
        except Exception as Error:
            Self.Problem = f"check workers could not start: {Error}"
            Self.Started.set()
            return
        for Each in Pending:
            Hello = Each.Answer(Deadline - time.monotonic())
            if Hello == ("ready", LoadedFingerprint):
                Self.Workers.append(Each)
                Self.Idle.put(Each)
            else:
                Each.Stop()
                if Hello is not None and Hello[0] == "ready":
                    Self.Problem = "check workers loaded different collision code or data than the bridge"
        if not Self.Workers and Self.Problem is None:
            Self.Problem = "no check worker answered"
        Self.Started.set()

    def Ready(Self) -> bool:
        return Self.Started.is_set() and any(Each.Alive for Each in Self.Workers)

    def _Run(Self, Request, Deadline, Patience=0.0, Wanted=None):
        """One request on the next idle worker: ("ok", reason or None), or None when no worker answered by Deadline.
        Patience: seconds a request that started is given to answer, however close Deadline is (a worker that
        misses its time is stopped for good). Wanted: a callable; once it answers False the request gives up
        before it takes a worker."""
        while True:
            Remaining = Deadline - time.monotonic()
            if Remaining <= 0 or not any(Each.Alive for Each in Self.Workers) or (Wanted is not None and not Wanted()):
                return None
            try:
                Each = Self.Idle.get(timeout=min(Remaining, 0.05))
            except queue.Empty:
                continue
            if not Each.Alive:
                continue
            Answer = Each.Call(Request, max(Deadline - time.monotonic(), Patience))
            if Each.Alive:
                Self.Idle.put(Each)
            if Answer is not None and Answer[0] == "ok":
                return Answer
            if Answer is not None:
                return None  # The worker failed on this request; the caller checks it itself.

    def Submit(Self, Request, Deadline) -> Future:
        """A future of ("ok", reason or None) for one request, or None when no worker answered in time.
        Requests start in the order submitted; cancel() drops one that has not started."""
        return Self.Threads.submit(lambda: Self._Run(Request, Deadline))

    def SubmitPatient(Self, Request, Deadline, Patience, Wanted=None) -> Future:
        """Submit, for a request that may start until Deadline and is then given Patience seconds to answer, and
        is dropped before it starts once Wanted() answers False (BasketCollision.PathScout)."""
        return Self.Threads.submit(lambda: Self._Run(Request, Deadline, Patience, Wanted))

    @staticmethod
    def WorldRef(Geometry, Frame) -> WorldRef:
        """The WorldRef of CollisionWorld(Geometry, Frame), for requests (WorldRef, kind, *arguments)."""
        return WorldRef(Geometry, Frame)

    def Map(Self, Requests, Seconds):
        Deadline = time.monotonic() + Seconds
        Futures = [Self.Submit(Request, Deadline) for Request in Requests]
        return [Each.result() for Each in Futures]

    def Close(Self):
        for Each in Self.Workers:
            Each.Stop()
        Self.Threads.shutdown(wait=False, cancel_futures=True)


_Pool: Pool | None = None
_PoolLock = threading.Lock()


def Start(Count: int | None = None) -> Pool:
    """The process-wide pool, started once in the background (the server calls this at startup)."""
    global _Pool
    with _PoolLock:
        if _Pool is None:
            _Pool = Pool(Count or max(1, min(MaximumWorkers, (os.cpu_count() or 1) // 4)))
            threading.Thread(target=_Pool._Start, name="RouteCheckStart", daemon=True).start()
        return _Pool


def Running() -> Pool | None:
    """The pool when its workers are up, else None: offline imports and tests check in process, as before."""
    return _Pool if _Pool is not None and _Pool.Ready() else None


def WorkerCount() -> int:
    """Workers answering now (the bridge's /health reports it; 0 while they start, or when they could not)."""
    return 0 if _Pool is None or not _Pool.Started.is_set() else sum(Each.Alive for Each in _Pool.Workers)


def Respond(Worlds, Message):
    """A worker's answer to one message: ("check", ...) as Check, ("call", key, payload or None, kind, arguments)
    on the WorldRef world it names (("miss",) when it no longer has it and was not sent it)."""
    if Message[0] == "check":
        return ("ok", Check(*Message[1:]))
    _, Key, Payload, Kind, Arguments = Message
    World = Worlds.pop(Key, None)
    if World is None:
        if Payload is None:
            return ("miss",)
        Geometry, Frame, BridgeModel = pickle.loads(Payload)
        if BridgeModel != Collision.Model:
            return ("error", "the bridge's collision model differs from this worker's")
        World = Collision.CollisionWorld(Geometry, Frame)
    Worlds[Key] = World  # Most recent last.
    while len(Worlds) > WorldsKept:
        Worlds.pop(next(iter(Worlds)))
    return ("ok", getattr(World, Kinds[Kind])(*Arguments))


def WorkerMain() -> int:
    Input, Output = sys.stdin.buffer, sys.stdout.buffer
    # Nothing else may write to the answer pipe.
    sys.stdout = sys.stderr
    _Write(Output, ("ready", LoadedFingerprint))
    Worlds: dict = {}
    while True:
        try:
            Message = _Read(Input)
        except EOFError:
            return 0
        try:
            _Write(Output, Respond(Worlds, Message))
        except Exception as Error:
            _Write(Output, ("error", f"{type(Error).__name__}: {Error}"))


if __name__ == "__main__" and "--worker" in sys.argv:
    sys.exit(WorkerMain())
