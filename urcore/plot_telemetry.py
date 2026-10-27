"""Plot a urcore.telemetry CSV: TCP actual vs target (x, y, z), force vs limit, phase.

  python3 plot_telemetry.py telemetry_HHMMSS.csv [out.png]
Needs matplotlib (run it on the laptop; the Jetson venv has none).
"""
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PHASES = {0: "idle", 1: "approach", 2: "contact", 3: "search", 4: "engage", 5: "wiggle",
          6: "seat", 7: "back-off", 8: "extract", 9: "touch"}

src = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else src.rsplit(".", 1)[0] + ".png"
rows = list(csv.DictReader(open(src)))
t0 = float(rows[0]["t_ctrl"])
t = [float(r["t_ctrl"]) - t0 for r in rows]
col = lambda k, s=1.0: [float(r[k]) * s for r in rows]  # noqa: E731

fig, ax = plt.subplots(5, 1, figsize=(13, 13), sharex=True)
for i, a in enumerate("xyz"):
    ax[i].plot(t, col(f"tgt_{a}", 1000), lw=1, ls="--", label="target")
    ax[i].plot(t, col(f"act_{a}", 1000), lw=1, label="actual")
    ax[i].set_ylabel(f"TCP {a} [mm]")
    ax[i].legend(loc="upper right", fontsize=8)
fx, fy, fz = col("F_x"), col("F_y"), col("F_z")
ax[3].plot(t, fx, lw=0.8, label="Fx")
ax[3].plot(t, fy, lw=0.8, label="Fy")
ax[3].plot(t, fz, lw=0.8, label="Fz")
ax[3].plot(t, [(a * a + b * b + c * c) ** 0.5 for a, b, c in zip(fx, fy, fz)], lw=1.2, color="k", label="|F|")
ax[3].plot(t, col("force_limit"), lw=1.5, ls="--", color="r", label="force limit")
ax[3].set_ylabel("force [N]")
ax[3].legend(loc="upper right", fontsize=8, ncol=5)
ph = [int(float(r["phase"])) for r in rows]
ax[4].step(t, ph, where="post", lw=1.2)
ax[4].set_yticks(list(PHASES), [PHASES[k] for k in PHASES], fontsize=8)
ax[4].set_ylabel("phase")
ax[4].set_xlabel("controller time [s]")
stops = [t[i] for i in range(1, len(rows)) if rows[i]["safety_mode"] != rows[i - 1]["safety_mode"]]
for a in ax:
    a.grid(alpha=0.3)
    for s in stops:
        a.axvline(s, color="r", alpha=0.5)
fig.suptitle(f"{src.rsplit('/', 1)[-1]}  ({len(rows)} samples, {t[-1]:.1f} s; red lines = safety mode change)")
fig.tight_layout()
fig.savefig(out, dpi=110)
print(out)
