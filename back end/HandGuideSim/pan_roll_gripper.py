import math, numpy as np
from gravity_model import *
rm = RawMap()
rng = np.random.default_rng(3)
lim = {n: rm.urdf_limits_rad(n) for n in JOINTS}
for label, payload, extra in (("no payload", 0, 0), ("100 g at TCP", 0.1, 0.0), ("100 g held 2 cm off the roll axis (modelled by 2 cm beyond TCP; on-axis here)", 0.1, 0.02)):
    m = default_model(payload, extra)
    worst = {"shoulder_pan": 0.0, "wrist_roll": 0.0, "gripper": 0.0}
    for _ in range(4000):
        q = {n: rng.uniform(*lim[n]) for n in JOINTS}
        q["gripper"] = rng.uniform(math.radians(-5), math.radians(100))
        t = m.hold_torques(q)
        for k in worst: worst[k] = max(worst[k], abs(t[k]))
    print(label, {k: f"{v:.5f} N*m = {v*KGCM_PER_NM:.3f} kg*cm" for k, v in worst.items()})
# off-axis payload: put a 100 g mass 2 cm sideways off the roll axis: roll torque = m g d
print("roll torque of 100 g at 2 cm off-axis: %.4f N*m = %.3f kg*cm" % (0.1*G*0.02, 0.1*G*0.02*KGCM_PER_NM))
# pan torque from cart tilt: total moment of arm about base (max horizontal moment) * sin(tilt)
m = default_model(0.1)
worst = 0
for _ in range(3000):
    q = {n: rng.uniform(*lim[n]) for n in JOINTS}
    joint_T, link_T, items = m.masses_and_coms(q)
    p = joint_T["shoulder_pan"][:3,3]
    mom = sum(mm*G*math.hypot(c[0]-p[0], c[1]-p[1]) for _, mm, c in items)
    worst = max(worst, mom)
print("max horizontal gravity moment of whole arm about pan axis (with 100 g): %.3f N*m (%.2f kg*cm); on a 2 deg tilted cart pan sees at most %.4f N*m" % (worst, worst*KGCM_PER_NM, worst*math.sin(math.radians(2))))
