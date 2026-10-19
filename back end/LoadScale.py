"""How busy this PC is right now, for the few tests that bound a wall-clock time (audit 1 L5, audit 2's test table).

A time bound set on this PC idle fails on a loaded one although nothing changed: TestDeskCheckIsCheap read 0.35 s
against its 0.25 s while other suites ran (0.11 s idle). Such a test takes the fastest of a few runs (Best: a run that
other work interrupted is not the code's cost) and scales its bound by Factor(): the cost now of a fixed piece of
pure-Python work over its cost on this PC idle, never below 1. Idle the bound means what it did; under load it does
not fail. (The app's node suites do the same with the cost of one IK solve: TestAiTargetSnap.mjs, TestGraspPlan.mjs.)
"""
from __future__ import annotations

import time
from typing import Callable

IdleSeconds = 0.0053  # Reference() on this PC idle, 2026-10-08: the median of 15 runs, 5.2-5.4 ms


def Reference() -> float:
    """Seconds for a fixed piece of pure-Python work."""
    Started = time.perf_counter()
    Total = 0
    for Index in range(200_000):
        Total += Index * Index
    return time.perf_counter() - Started


def Factor(Repeats: int = 5) -> float:
    """The reference work's cost now over its idle cost (the median of Repeats runs), at least 1."""
    Times = sorted(Reference() for _ in range(Repeats))
    return max(1.0, Times[len(Times) // 2] / IdleSeconds)


def Best(Work: Callable[[], object], Repeats: int = 5) -> float:
    """The fastest of Repeats runs of Work(), in seconds."""
    Fastest = float("inf")
    for _ in range(Repeats):
        Started = time.perf_counter()
        Work()
        Fastest = min(Fastest, time.perf_counter() - Started)
    return Fastest
