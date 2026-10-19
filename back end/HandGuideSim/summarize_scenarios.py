"""Compact table of out_guide_scenarios.json, grouped (offline; reads the file guide_scenarios.py wrote).

    python summarize_scenarios.py [file.json] > out_guide_final_table.txt
"""
import json
import sys
from pathlib import Path

F = Path(__file__).resolve().parent / (sys.argv[1] if len(sys.argv) > 1 else "out_guide_scenarios.json")
rows = json.loads(F.read_text(encoding="utf-8"))
group = None
print(f"{len(rows)} scenarios")
for r in rows:
    if r.get("group") != group:
        group = r.get("group")
        print(f"\n== {group}")
    if "error" in r and "trace" in r:
        print(f"  {r['name']}: ERROR {r['error']}")
        continue
    bits = []
    if r.get("exit"):
        bits.append(f"exit {r['exit']} @{r['exit_t']}s ({(r.get('reason') or '')[:80]})")
    else:
        bits.append("no exit")
    for k in ("hand_force_N", "breakaway_N", "mean_speed_dps", "after_release_deg", "stop_s", "creep_last_2s_deg",
              "end_speed_dps", "flick_peak_N", "flick_rms_Nm", "hand_max_torque_Nm", "past_limit_max_deg",
              "past_limit_end_deg", "band_depth_max_cm", "band_depth_end_cm"):
        if r.get(k) is not None:
            bits.append(f"{k}={r[k]}")
    for k in ("moved_after_deg", "drop_0.5s_deg", "drop_1.0s_deg", "hover_drift_deg"):
        if r.get(k):
            bits.append(f"{k}=" + "/".join(f"{v}" for v in r[k].values()))
    off = [j for j, v in (r.get("ff_on") or {}).items() if not v]
    if off:
        bits.append("assist off: " + ",".join(off))
    ac = r.get("assist_counts") or {}
    on = {j: c for j, c in ac.items() if any(c.values())}
    if on:
        bits.append("free: " + "; ".join(f"{j} " + ",".join(f"{k}{v}" for k, v in c.items() if v) for j, c in on.items()))
    ms = r.get("max_speed_dps") or {}
    bits.append("vmax=" + "/".join(f"{v}" for v in ms.values()))
    print(f"  {r['name'][:100]:100s} | " + " | ".join(bits))
