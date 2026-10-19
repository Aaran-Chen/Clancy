"""Four checks on the offline plant (arm_plant_sim.py), for docs/HandGuide.md:

1. Torque gap: how far does each joint sag if its torque is off for a few ms (an Operating_Mode switch clears
   Torque_Enable until the next goal write), at the worst pose, before a preloaded goal re-energises it?
2. Identification: do slow up/down position-mode sweeps (host-ramped goal, Present_Load recorded) recover the
   gravity holding duty (band centre) and the friction half-band, despite backlash, quantisation and load-dependent
   friction? And does a faster pass give the velocity slope k_v (the back-EMF + viscous duty per rad/s)?
3. The same sweeps on pan, roll and gripper: the kinetic friction and k_v their assist is built from.
4. Spin-down: does torque off let the motor float or brake it (which decides the assist band)?
Offline only. `python gap_and_ident_sim.py --free` runs only the free-joint part (3, 4).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_plant_sim as S  # noqa: E402
import guide_scenarios as G  # noqa: E402
import hand_guide_law as L  # noqa: E402

DEG = math.pi / 180


def step_servos(sim):
    for j, s in sim.servos.items():
        if sim.t >= s.t_next:
            s.tick(sim.t, sim.link_theta(j), sim.gear_omega(j))
            s.t_next += S.Servo.TICK


def torque_gap(gap_s: float, fric: str, pose="extended"):
    sc = S.Scenario("gap", seconds=1.0, start_raw=G.POSES[pose], gear=G.gears(fric))
    sim = S.ArmSim(sc)
    while sim.t < 0.3:
        step_servos(sim); sim.physics_step({})
    th0 = sim.theta_l.copy()
    goals = {j: sim.servos[j].r.goal for j in S.COMP}
    for j in S.COMP:
        sim.servos[j].write_torque(False)
    t_off = sim.t
    peak = np.zeros(3)
    while sim.t < t_off + gap_s:
        step_servos(sim); sim.physics_step({})
        peak = np.maximum(peak, np.abs(sim.theta_l - th0))
    for j in S.COMP:
        sim.servos[j].write_goal(goals[j], 1000.0)      # the goal write that re-energises (preloaded: same goal)
    while sim.t < t_off + gap_s + 0.5:
        step_servos(sim); sim.physics_step({})
        peak = np.maximum(peak, np.abs(sim.theta_l - th0))
    return [round(math.degrees(x), 3) for x in peak], [round(math.degrees(x), 3) for x in (sim.theta_l - th0)]


def sweep(fric: str, elbow_deg: float, wrist_deg: float, speed_dps: float, lo=-20.0, hi=70.0):
    """Ramp the lift goal from lo to hi and back at speed_dps (100 Hz host updates), others held where they are.
    Returns samples (lift URDF deg, direction, Present_Load, true holding duty)."""
    start = G.raw(lo, elbow_deg, wrist_deg)
    sc = S.Scenario("sweep", seconds=99, start_raw=start, gear=G.gears(fric))
    sim = S.ArmSim(sc)
    lift = sim.servos["shoulder_lift"]
    out = []
    t_next = 0.0
    q_cmd = lo
    phase = "settle"
    t_phase = 0.0
    kd = L.kd_nm_per_unit(8.9)
    while True:
        step_servos(sim); sim.physics_step({})
        if sim.t < t_next:
            continue
        t_next = sim.t + 0.01
        if phase == "settle" and sim.t > 0.5:
            phase, t_phase = "up", sim.t
        if phase == "up":
            q_cmd = lo + speed_dps * (sim.t - t_phase)
            if q_cmd >= hi:
                phase, t_phase = "pause", sim.t
        elif phase == "pause" and sim.t - t_phase > 0.5:
            phase, t_phase = "down", sim.t
        elif phase == "down":
            q_cmd = hi - speed_dps * (sim.t - t_phase)
            if q_cmd <= lo:
                break
        # the goal: the commanded angle plus the hold offset the current load needs (the host knows neither g nor F;
        # it just ramps the goal; the P-loop error supplies the effort, Present_Load reads what was applied)
        goal = L.REFERENCE_RAW["shoulder_lift"] + q_cmd * 4096 / 360 + lift.turn
        lift.write_goal(goal, 1000.0)
        if phase in ("up", "down"):
            th = sim.theta_l
            g = S.hold_torques(th, sim.true_p)[0] / kd
            out.append((math.degrees(th[0]), phase, lift.duty, g))
    return out


def fit_lift(samples_by_config):
    """Fit the lift's holding duty basis [cos T2, sin T2, cos T3, sin T3, cos T4, sin T4] to (up+down)/2 binned by angle."""
    rows, ys, true = [], [], []
    halfband = []
    for (elbow, wrist), samples in samples_by_config.items():
        bins = {}
        for q, ph, load, g in samples:
            b = round(q / 2.0) * 2.0
            bins.setdefault(b, {"up": [], "down": [], "g": []})
            bins[b][ph].append(load)
            bins[b]["g"].append(g)
        for b, d in bins.items():
            if len(d["up"]) >= 2 and len(d["down"]) >= 2:
                up, dn = np.median(d["up"]), np.median(d["down"])
                q2, q3, q4 = b * DEG, elbow * DEG, wrist * DEG
                T2, T3, T4 = q2, q2 + q3, q2 + q3 + q4
                rows.append([math.cos(T2), math.sin(T2), math.cos(T3), math.sin(T3), math.cos(T4), math.sin(T4)])
                ys.append((up + dn) / 2)
                true.append(np.mean(d["g"]))
                halfband.append((up - dn) / 2)
    X, y, gt = np.array(rows), np.array(ys), np.array(true)
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ coef
    return {"points": len(y), "rms_fit_vs_truth_units": round(float(np.sqrt(np.mean((pred - gt) ** 2))), 1),
            "rms_bandcentre_vs_truth_units": round(float(np.sqrt(np.mean((y - gt) ** 2))), 1),
            "max_true_units": round(float(np.max(np.abs(gt))), 0),
            "halfband_units_median": round(float(np.median(halfband)), 1),
            "halfband_units_range": [round(float(np.min(halfband)), 1), round(float(np.max(halfband)), 1)]}


def free_sweep(joint: str, fric: str, speed_dps: float, span: float = 40.0, coast_brakes: bool = False):
    """Ramp a free joint's goal up `span` deg and back at speed_dps (100 Hz host updates, Torque_Limit 1000), the
    other joints held. Returns the median Present_Load moving up and moving down over the middle of each pass."""
    sc = S.Scenario("free sweep", seconds=99, start_raw=G.WORK, gear=G.gears("mid", free=fric), coast_brakes=coast_brakes)
    sim = S.ArmSim(sc)
    sv = sim.servos[joint]
    g0 = sv.r.goal
    up, down = [], []
    t_next, phase, t_phase, q_cmd = 0.0, "settle", 0.0, 0.0
    while True:
        step_servos(sim); sim.physics_step({})
        if sim.t < t_next:
            continue
        t_next = sim.t + 0.01
        if phase == "settle" and sim.t > 0.3:
            phase, t_phase = "up", sim.t
        if phase == "up":
            q_cmd = speed_dps * (sim.t - t_phase)
            if q_cmd >= span:
                phase, t_phase = "pause", sim.t
        elif phase == "pause" and sim.t - t_phase > 0.3:
            phase, t_phase = "down", sim.t
        elif phase == "down":
            q_cmd = span - speed_dps * (sim.t - t_phase)
            if q_cmd <= 0:
                break
        sv.write_goal(g0 + q_cmd * 4096 / 360, 1000.0)
        if phase in ("up", "down") and 0.3 * span <= q_cmd <= 0.8 * span:
            (up if phase == "up" else down).append(sv.duty)
    return float(np.median(up)), float(np.median(down))


def spin_down(joint: str, fric: str, speed_dps: float = 60.0, coast_brakes: bool = False):
    """Drive a free joint at speed_dps with a goal ramp, then write Torque_Enable 0 and read its position at 100 Hz
    for 0.4 s, as the calibration would. Returns how far it ran on (deg) and the deceleration it showed (rad/s^2) over
    its first 40 ms."""
    sc = S.Scenario("spin down", seconds=99, start_raw=G.WORK, gear=G.gears("mid", free=fric), coast_brakes=coast_brakes)
    sim = S.ArmSim(sc)
    sv = sim.servos[joint]
    g0 = sv.r.goal
    t_next = 0.0
    while sim.t < 0.3 + 30.0 / speed_dps:
        step_servos(sim); sim.physics_step({})
        if sim.t >= t_next and sim.t > 0.3:
            t_next = sim.t + 0.01
            sv.write_goal(g0 + speed_dps * (sim.t - 0.3) * 4096 / 360, 1000.0)
    sv.write_torque(False)
    t_off, reads, t_next = sim.t, [], sim.t
    while sim.t < t_off + 0.4:
        step_servos(sim); sim.physics_step({})
        if sim.t >= t_next:
            t_next = sim.t + 0.01
            reads.append((sim.t - t_off, sv.present()))
    q0 = reads[0][1]
    ran = abs(reads[-1][1] - q0) * 360 / 4096
    # speed over the first 20 ms and the next 20 ms (steps -> rad/s)
    v1 = (reads[2][1] - reads[0][1]) / (reads[2][0] - reads[0][0]) * L.STEP_RAD
    v2 = (reads[4][1] - reads[2][1]) / (reads[4][0] - reads[2][0]) * L.STEP_RAD
    decel = (abs(v1) - abs(v2)) / 0.02
    return ran, decel


def free_joints():
    print("\n3. free joints (pan, roll, gripper): what the calibration measures for the assist")
    kd = L.kd_nm_per_unit(8.9)
    kv_true = L.KE / 8.9 * 1000.0 + 0.02 / kd
    for joint in L.FREE_JOINTS:
        for fric in ("low", "mid", "high"):
            f0 = G.FREE_FRICTION[fric]["f0"]
            hb = {}
            for speed in (5.0, 20.0):
                lu, ld = free_sweep(joint, fric, speed)
                hb[speed] = (lu - ld) / 2
            kv_est = (hb[20.0] - hb[5.0]) / (15.0 * DEG)
            fk_est = hb[5.0] - kv_est * 5.0 * DEG
            fk_true = 0.8 * f0 / kd
            print(f"   {joint:12s} {fric:4s}: kinetic friction {fk_est:5.1f} units (gear alone {fk_true:5.1f}, plus the "
                  f"output bearing), velocity slope {kv_est:5.0f} (back-EMF + viscous {kv_true:.0f})")
    print("\n4. spin-down: driven at 45 and then 90 deg/s, then torque off: does the driver float or brake? Coulomb "
          "friction alone (floating) makes the run-on grow with v^2 (ratio ~4); the back-EMF brake with v (ratio ~2). "
          "Verdict: floats if the ratio is at least 2.2 and the 90 deg/s run-on at least 3 deg")
    for joint in ("wrist_roll", "gripper"):
        for fric in ("low", "mid", "high"):
            for brakes in (False, True):
                r30, _ = spin_down(joint, fric, 45.0, coast_brakes=brakes)
                r60, decel = spin_down(joint, fric, 90.0, coast_brakes=brakes)
                ratio = r60 / max(r30, 0.05)
                verdict = "floats" if ratio >= 2.2 and r60 >= 3.0 else "brakes"
                print(f"   {joint:12s} {fric:4s} {'braking' if brakes else 'floating'} driver: ran on {r30:4.2f} / {r60:4.2f} "
                      f"deg (ratio {ratio:3.1f}) -> {verdict}{'' if (verdict == 'brakes') == brakes else '  WRONG'}")


def main():
    if "--free" in sys.argv:
        free_joints()
        return
    print("1. torque gap at the extended pose (worst lift load): peak and final sag (deg, lift/elbow/wrist)")
    for fric in ("low", "mid"):
        for gap in (0.001, 0.003, 0.010, 0.030, 0.100):
            peak, final = torque_gap(gap, fric)
            print(f"   {fric:4s} gap {gap * 1000:5.0f} ms: peak {peak}  final {final}")
    print("\n2. identification sweeps of the lift (position-mode goal ramp, Present_Load), 3 elbow configurations")
    kd = L.kd_nm_per_unit(8.9)
    for fric in ("low", "mid"):
        res = {}
        for speed in (5.0, 20.0):
            data = {}
            for elbow, wrist in ((-40.0, -20.0), (0.0, 0.0), (-80.0, 10.0)):
                data[(elbow, wrist)] = sweep(fric, elbow, wrist, speed)
            res[speed] = fit_lift(data)
            print(f"   {fric} friction, {speed:4.0f} deg/s: {res[speed]}")
        # velocity slope: half-band growth between the two speeds = k_v * (w2 - w1) (back-EMF + viscous, duty units)
        dw = (20.0 - 5.0) * DEG
        kv_est = (res[20.0]["halfband_units_median"] - res[5.0]["halfband_units_median"]) / dw
        kv_true = (L.KE / 8.9 * 1000.0) + 0.02 / kd
        print(f"   {fric}: velocity slope from the two speeds {kv_est:.0f} duty units per rad/s "
              f"(back-EMF + viscous expected {kv_true:.0f})")
    free_joints()


if __name__ == "__main__":
    main()
