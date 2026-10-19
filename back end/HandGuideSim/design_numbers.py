"""Numbers quoted in docs/HandGuide.md, computed (offline). Run: python design_numbers.py"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hand_guide_law as L  # noqa: E402
from gravity_model import RawMap, default_model  # noqa: E402

rm = RawMap()
STEP = L.STEP_RAD
COMP = L.COMP_JOINTS
lims = {j: rm.urdf_limits_rad(j) for j in COMP}
print("soft limits (URDF deg):", {j: [round(math.degrees(a), 1) for a in v] for j, v in lims.items()})

# 1. worst-case model effort over the soft box, in duty units, per supply voltage; caps
g = L.GravityModel()
grid = [np.linspace(lims[j][0], lims[j][1], 61) for j in COMP]
worst = {j: 0.0 for j in COMP}
for a in grid[0]:
    for b in grid[1]:
        for c in grid[2]:
            t = g.torques(a, b, c)
            for j in COMP:
                worst[j] = max(worst[j], abs(t[j]))
print("\n1. worst-case holding torque over the soft box (N m, bare arm):", {j: round(v, 3) for j, v in worst.items()})
caps = {"shoulder_lift": 650, "elbow_flex": 450, "wrist_flex": 250}
for V in (7.5, 8.9, 12.0):
    kd = L.kd_nm_per_unit(V)
    row = {}
    for j in COMP:
        need = worst[j] / kd
        need20 = 1.2 * need
        row[j] = f"model {need:.0f}, +20 % {need20:.0f}, cap {caps[j]} -> headroom {caps[j] - need20:.0f} units"
    print(f"   {V:4.1f} V (kd {kd * 1000:.2f} mN m/unit): {row}")
for j in COMP:
    kd = L.kd_nm_per_unit(8.9)
    print(f"   cap {j}: {caps[j]} units = {caps[j] / 10:.0f} % of full duty = {caps[j] * kd:.2f} N m at 8.9 V "
          f"({caps[j] * kd * 100 / 9.80665:.1f} kg cm) vs stall {1000 * kd:.2f} N m; overload trip 800 units")

# 2. tip speed at the velocity caps, at full reach (levers from the TCP to each axis, stretched pose)
m = default_model()
q = {"shoulder_pan": 0.0, "shoulder_lift": math.radians(77.5), "elbow_flex": math.radians(-75), "wrist_flex": math.radians(-1),
     "wrist_roll": 0.0, "gripper": 0.0}
joint_T, link_T = m.fk(q)
tcp = link_T["gripper_frame_link"][:3, 3]
vcaps = {"shoulder_lift": 45, "elbow_flex": 50, "wrist_flex": 80}
print("\n2. velocity caps as tip speeds at full stretch:")
for j in COMP:
    T = joint_T[j]
    a = T[:3, :3] @ m.joints[j].axis
    r = np.linalg.norm(np.cross(a, tcp - T[:3, 3]))
    print(f"   {j}: lever {r * 100:.1f} cm, cap {vcaps[j]} deg/s -> {r * math.radians(vcaps[j]):.2f} m/s")

# 3. the guide loop's bus budget (1 Mbaud, 10 us per byte; packet layouts of scservo_sdk / Feetech protocol 0)
B = 10e-6
def sync_read(n, L_): return (8 + n) * B + n * (6 + L_) * B + n * 30e-6
def sync_write(n, L_): return (8 + n * (1 + L_)) * B
r15 = sync_read(6, 15)
w8 = sync_write(3, 8)
w8_5 = sync_write(5, 8)
w8_6 = sync_write(6, 8)
off3 = sync_write(3, 1)
r16 = sync_read(6, 16)
exitw = sync_write(5, 9)
exit6 = sync_write(6, 9)
print("\n3. bus wire time per guide cycle:")
print(f"   SYNC_READ regs 56..70 (15 B) x6: {r15 * 1e3:.2f} ms; SYNC_WRITE regs 42..49 (8 B) x3: {w8 * 1e3:.2f} ms "
      f"(x5 with pan and roll caught: {w8_5 * 1e3:.2f} ms)")
print(f"   every 4th cycle SYNC_READ regs 40..55 (16 B: torque, goal/limit read-back, Lock) x6: {r16 * 1e3:.2f} ms")
print(f"   exit packet SYNC_WRITE regs 41..49 (9 B) x5: {exitw * 1e3:.2f} ms (x6 with the gripper: {exit6 * 1e3:.2f} ms)")
print(f"   with pan, roll and gripper assisted: SYNC_WRITE regs 42..49 x6 {w8_6 * 1e3:.2f} ms; a float (Torque_Enable 0) "
      f"SYNC_WRITE reg 40 for up to 3 joints {off3 * 1e3:.2f} ms, only on a cycle where one stops being assisted")
avg_wire = r15 + w8 + r16 / 4
worst_wire = r15 + w8_6 + off3 + r16
print(f"   worst cycle (6 written, 3 floated, the 4th-cycle read): wire {worst_wire * 1e3:.2f} ms")
for over in (0.8, 1.5):
    cyc = avg_wire + over / 1000.0 * (1 + 0.4 + 0.25)     # ~0.8 ms per awaited transaction (measured), less for TX-only
    print(f"   mean wire {avg_wire * 1e3:.2f} ms + adapter/OS {over} ms per transaction -> {cyc * 1e3:.1f} ms per cycle "
          f"= {cyc / 0.01 * 100:.0f} % of a 100 Hz period")

# 4. excursion bounds as tip distances
print("\n4. a start abort at 25 steps of the lift, at full reach:", f"{25 * STEP * 0.441 * 100:.1f} cm at the tip")
print("   a 1 deg glide of the roll after release, at the gripper's side (3 cm):", f"{math.radians(1.0) * 0.03 * 1000:.1f} mm")

# 5. holding current and copper heat at the worst pose (no payload / 100 g)
for tau, label in ((worst["shoulder_lift"], "bare"), (1.268, "100 g")):
    I = tau / L.KT
    print(f"\n5. lift worst pose {label}: {tau:.3f} N m -> {I:.2f} A, {I * I * L.R_OHM:.1f} W copper heat")

# 6. back-driven current under the cap at the speed cap (firmware overcurrent at 2.0 A for 2 s)
for j in COMP:
    V = 8.9
    I = (caps[j] / 1000 * V + L.KE * math.radians(vcaps[j])) / L.R_OHM
    print(f"6. {j}: cap {caps[j]} at 8.9 V while back-driven at its speed cap -> {I:.2f} A (overcurrent 2.0 A)")

# 7. the start's preload + ramp time, and how long the user keeps hands off
p = L.GuideParams()
probe = 2 * p.probe_s + p.probe_settle_s
print(f"\n7. start: preload {p.preload_s} s + ramp {p.ramp_s} s + probe {probe:.1f} s = "
      f"{p.preload_s + p.ramp_s + probe:.1f} s hands off")
# 8. back-EMF duty slope at 8.9 V and the lead at the default share
kv = L.KE / 8.9 * 1000
print(f"8. back-EMF duty per rad/s at 8.9 V: {kv:.0f} units; lead c=0.6 at 30 deg/s: {0.6 * kv * math.radians(30):.0f} units")

# 9. the free joints (pan, roll, gripper): assist caps, back-driven current, feed-forward sizes and band tops
print("\n9. free joints")
kd = L.kd_nm_per_unit(8.9)
kv_true = L.kv_prior(8.9) + 0.02 / kd
FREE_FRICTION = {"low": 0.04, "mid": 0.08, "high": 0.15}       # plant F0 (N m at the gear), guide_scenarios.py
LEVER = {"shoulder_pan": 0.30, "wrist_roll": 0.03, "gripper": 0.05}
for j in L.FREE_JOINTS:
    f = L.FreeConfig(j, **L.FREE_DEFAULTS[j])
    I = (f.cap_max / 1000 * 8.9 + L.KE * f.v_hi) / L.R_OHM
    print(f"   {j}: assist ceiling {f.cap_max:.0f} units = {f.cap_max * kd:.2f} N m at 8.9 V; caught at a wall "
          f"{f.catch_limit:.0f}; back-driven at the ceiling at {math.degrees(f.v_hi):.0f} deg/s -> {I:.2f} A (overcurrent 2.0 A)")
for lvl, f0 in FREE_FRICTION.items():
    fk = 0.8 * f0 / kd
    for k_f in (0.5, 0.7):
        for c in (0.85, 0.9):
            cfg = L.FreeConfig("wrist_roll", f_kin=fk, k_f=k_f, c_lead=c, k_v=kv_true, v_hi=90 * L.DEG)
            law = L.HandGuideLaw({}, L.GravityModel(), L.GuideParams(), free_configs={"wrist_roll": cfg})
            top = law.band_top("wrist_roll", 8.9)
            cross = k_f * fk / ((1 - c) * kv_true + cfg.k_loop)
            print(f"   {lvl:4s} friction (kinetic {fk:4.1f} units = {fk * kd:.3f} N m = {fk * kd / LEVER['wrist_roll']:.1f} N "
                  f"at the roll's 3 cm): k_f {k_f}, c {c}: feed-forward {k_f * fk:4.1f} units, crossover "
                  f"{math.degrees(cross):4.1f} deg/s, band top {math.degrees(top):4.1f} deg/s"
                  f"{' (no band: the joint just floats)' if not top else ''}")

# 10. the entry supply threshold and the guiding floor: the lift's worst model effort + 20 % against its 650 ceiling
for V in (8.5, 7.0):
    need = 1.2 * worst["shoulder_lift"] / L.kd_nm_per_unit(V)
    print(f"\n10. at {V} V the lift's worst model effort + 20 % is {need:.0f} units (ceiling 650)", end="")
print()
