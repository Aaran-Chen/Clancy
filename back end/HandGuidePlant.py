"""The offline plant of HandGuideSim, driving the bridge's own law (HandGuide.py) instead of the reference copy.

HandGuideSim/arm_plant_sim.py (the arm, gears, motors, servo firmware, bus and host timing, faults) and
HandGuideSim/guide_scenarios.py (the 323 scenarios and their metrics) look the law up as the module attribute L at the
moment they use it, so pointing L at HandGuide runs every scenario on the code the bridge runs. Offline only: no serial
port, no bridge, no camera. TestHandGuideLaw.py asserts the scenario results; RunNamed is its worker (one process each,
ProcessPoolExecutor), and Reference=True runs the same scenario on the reference law, to show the port is faithful.

    python HandGuidePlant.py "free release wrist_roll from 15 deg/s mid assist"
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

Here = Path(__file__).resolve().parent
sys.path.insert(0, str(Here / "HandGuideSim"))
sys.path.insert(0, str(Here))

import HandGuide  # noqa: E402
import arm_plant_sim as Plant  # noqa: E402
import guide_scenarios as Scenarios  # noqa: E402
import hand_guide_law as ReferenceLaw  # noqa: E402

Plant.L = HandGuide
Scenarios.L = HandGuide


def AllScenarios() -> dict:
    """Every scenario of guide_scenarios.build() by its name, built afresh (a scenario's hand keeps what it measured)."""
    return {Scenario.name: (Group, Scenario) for Group, Items in Scenarios.build().items() for Scenario in Items}


def RunNamed(Name: str, Reference: bool = False) -> dict:
    """Run one scenario by name on the plant and return its summary with guide_scenarios' metrics (and "group").
    Reference: on HandGuideSim/hand_guide_law.py instead of HandGuide.py."""
    Group, Scenario = AllScenarios()[Name]
    Law = ReferenceLaw if Reference else HandGuide
    Plant.L, Scenarios.L = Law, Law
    try:
        Started = time.perf_counter()
        Sim = Plant.ArmSim(Scenario)
        Result = Sim.run()
        Result.update(Scenarios.metrics(Scenario)(Sim))
        Result["wall_s"] = round(time.perf_counter() - Started, 2)
    finally:
        Plant.L, Scenarios.L = HandGuide, HandGuide
    Result["group"] = Group
    return Result


def CalibrateOnPlant(Joints: tuple, Spin: tuple = (), MassScale: float = 1.0, Friction: str = "mid", FreeFriction: str = "mid",
                     CoastBrakes: bool = False, HalfSpan: dict | None = None, StartRaw: dict | None = None) -> dict:
    """Run HandGuideCalibration's sweeps of Joints (and the spin-down of Spin) on the plant from the feel tests' pose,
    as the bridge's guide loop runs them (100 Hz, goals with the run's own Torque_Limit, Present_Load = the applied duty).
    Returns {"samples", "spin_downs", "volts", "true_p", "error"}: the fit is the caller's (samples of several runs merge)."""
    import HandGuideCalibration as C
    from gravity_model import RawMap
    Scenario = Plant.Scenario("calibration", seconds=1e9, start_raw=StartRaw or Scenarios.WORK, mass_scale=MassScale,
                              gear=Scenarios.gears(Friction, free=FreeFriction), coast_brakes=CoastBrakes)
    Sim = Plant.ArmSim(Scenario)
    Map = RawMap()
    Limits = {Name: Map.urdf_limits_rad(Name) for Name in HandGuide.ALL_JOINTS}
    Present = {Name: Sim.servos[Name].present() for Name in HandGuide.ALL_JOINTS}
    Rad = {Name: HandGuide.urdf_rad(Name, Raw) for Name, Raw in Present.items()}
    Segments, Problems = C.PlanSweeps(Rad, Limits, Joints=Joints, Spin=Spin, HalfSpan=HalfSpan)
    if Problems:
        return {"error": "; ".join(Problems)}
    Run = C.CalibrationRun(Segments, {Name: Sim.servos[Name].r.goal for Name in HandGuide.ALL_JOINTS}, Present)
    Next = 0.0
    while not (Run.Done or Run.Error):
        for Name, Servo in Sim.servos.items():
            if Sim.t >= Servo.t_next:
                Servo.tick(Sim.t, Sim.link_theta(Name), Sim.gear_omega(Name))
                Servo.t_next += Plant.Servo.TICK
        Sim.physics_step({})
        if Sim.t < Next:
            continue
        Next = Sim.t + 0.01
        Readings = {Name: {"pos": S.present(), "load": S.duty, "volts": S.volts(Sim.t), "temp_c": 30.0, "status": 0}
                    for Name, S in Sim.servos.items()}
        Command = Run.Step(Sim.t, Readings)
        for Name, (Goal, Limit) in Command["writes"].items():
            Sim.servos[Name].write_goal(Goal, Limit)
        for Name in Command["release"]:
            Sim.servos[Name].write_torque(False)
    return {"samples": Run.Samples, "spin_downs": Run.SpinDowns, "volts": Run.Volts, "error": Run.Error,
            "true_p": list(Sim.true_p), "seconds": round(Sim.t, 1)}


def GuideOnPlant(Data: dict, Seconds: float = 8.0, Pose: str = "extended", MassScale: float = 1.0, Friction: str = "mid",
                 Hand=None) -> dict:
    """Hand guiding on the plant with the law the bridge would build from calibration Data (HandGuideCalibration.BuildLaw,
    the plant's servo settings and slider arcs): the summary with the metrics; hands off unless Hand."""
    import HandGuideCalibration as C
    from gravity_model import RawMap
    Map = RawMap()
    Limits = {Name: Map.urdf_limits_rad(Name) for Name in HandGuide.ALL_JOINTS}
    Settings = {Name: {"p": 32, "punch": 16, "dead_zone": [1, 1]} for Name in HandGuide.ALL_JOINTS}
    Scenario = Plant.Scenario("guided with the plant's own calibration", seconds=Seconds, start_raw=Scenarios.POSES[Pose],
                              mass_scale=MassScale, gear=Scenarios.gears(Friction), record=True, hand=Hand)
    Sim = Plant.ArmSim(Scenario)
    Sim.make_law = lambda: C.BuildLaw(Data, Settings, HandGuide.REFERENCE_RAW, Limits)
    Result = Sim.run()
    Result.update(Scenarios.metrics(Scenario)(Sim))
    Result["problems"] = Sim.law.problems if Sim.law is not None else None
    return Result


if __name__ == "__main__":
    for Argument in sys.argv[1:]:
        print(json.dumps(RunNamed(Argument), indent=1, default=str))
