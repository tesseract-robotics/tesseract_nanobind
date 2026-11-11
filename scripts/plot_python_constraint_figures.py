"""Figures from the Python-constraint experiments.

    pixi run python scripts/plot_python_constraint_effect.py effect.json
    pixi run python scripts/sim_world_hold.py hold_50.json --rate 50
    pixi run python scripts/sim_world_hold.py hold_200.json --rate 200
    pixi exec --spec matplotlib-base python scripts/plot_python_constraint_figures.py \
        effect.json hold_50.json hold_200.json OUT_DIR

Writes carry_level.png (tool-axis deviation per replan, with and without
the constraint) and world_hold.png (tool error in the world while the base moves).
matplotlib is not a dependency of the repo's environments, hence `pixi exec`: the script reads
the JSON files only and needs no tesseract.
"""

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

EFFECT, HOLD_50, HOLD_200, OUT = sys.argv[1], sys.argv[2], sys.argv[3], Path(sys.argv[4])
OUT.mkdir(parents=True, exist_ok=True)

INK, MUTED, GRID, AXIS = "#1C1A17", "#4E4A43", "#E9E3D6", "#C9C1B3"
GRAY, LIGHT, ACCENT = "#9C9485", "#1F78B4", "#A84A26"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 18})


def style(ax):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(AXIS)
    ax.tick_params(colors="#6B655B", labelsize=18)
    ax.grid(axis="y", color=GRID, lw=1)
    ax.set_axisbelow(True)


# 1 · tool-axis deviation per replan
d = json.load(open(EFFECT))
fig, ax = plt.subplots(figsize=(10.4, 4.4), dpi=200)
x = range(len(d["without"]))
ax.plot(x, d["without"], color=GRAY, lw=3.5, ls=(0, (6, 3)))
ax.plot(x, d["with"], color=ACCENT, lw=3.5)
ax.text(30.6, d["without"][-1], "without", va="center", color=MUTED, fontsize=18)
ax.text(
    30.6,
    1.45,
    "with the\nPython constraint",
    va="center",
    color=INK,
    fontsize=18,
    fontweight="bold",
)
ax.text(13.0, 1.3, "bound: 0.5°", va="center", color="#6B655B", fontsize=18)
ax.set_xlim(-0.6, 39.5)
ax.set_ylim(0, 10)
ax.set_xticks(range(0, 31, 5))
ax.set_xlabel("replan (one SQP step each; the obstacle keeps moving)", color=MUTED, fontsize=18)
ax.set_ylabel("tool-axis deviation, °", color=MUTED, fontsize=18)
style(ax)
fig.tight_layout()
fig.savefig(OUT / "carry_level.png")

# 2 · tool error in the world while the base moves
r50 = json.load(open(HOLD_50))["rows"]
r200 = json.load(open(HOLD_200))["rows"]
fig, ax = plt.subplots(figsize=(10.4, 3.5), dpi=200)
ax.plot([r["t"] for r in r50], [r["frozen_mm"] for r in r50], color=GRAY, lw=3, ls=(0, (6, 4)))
ax.plot([r["t"] for r in r50], [r["held_mm"] for r in r50], color=LIGHT, lw=2.5)
ax.plot([r["t"] for r in r200], [r["held_mm"] for r in r200], color=ACCENT, lw=2.5)
ax.set_yscale("log")
ax.set_ylim(0.1, 1000)
ax.set_xlim(0, 10.9)
ax.set_xticks(range(0, 9, 2))
ax.set_yticks([0.1, 1, 10, 100, 1000])
ax.set_yticklabels(["0.1", "1", "10", "100", "1000"])
ax.text(8.15, r50[-1]["frozen_mm"], "arm not compensating", va="center", color=MUTED, fontsize=18)
ax.text(8.15, 1.9, "held, 50 Hz loop", va="center", color=INK, fontsize=18, fontweight="bold")
ax.text(8.15, 0.36, "held, 200 Hz loop", va="center", color=INK, fontsize=18, fontweight="bold")
ax.set_xlabel("time, s (base driving at 50 mm/s and rocking)", color=MUTED, fontsize=18)
ax.set_ylabel("tool error, mm", color=MUTED, fontsize=18)
style(ax)
fig.tight_layout()
fig.savefig(OUT / "world_hold.png")
