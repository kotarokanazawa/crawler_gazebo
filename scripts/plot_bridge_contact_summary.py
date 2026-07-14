#!/usr/bin/env python3
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


root = Path(__file__).resolve().parents[3] / "docs" / "crawler_track_contact_comparison" / "results"
with (root / "summary.csv").open(encoding="utf-8") as stream:
    rows = list(csv.DictReader(stream))

labels = ["rectangle", "semicircle", "spike"]
by_shape = {row["shape"]: row for row in rows}
colors = ["#d95f59", "#42a77b", "#d69b35"]

fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
metrics = [
    ("mean_points_per_frame", "mean contact points / frame", False),
    ("mean_span_x_m", "mean longitudinal contact span [m]", False),
    ("max_abs_pitch_deg", "maximum absolute pitch [deg]", True),
    ("max_abs_estimated_force_n", "max |estimated net Fx| [N]", False),
]
for axis, (key, title, logarithmic) in zip(axes.flat, metrics):
    values = [float(by_shape[label][key]) for label in labels]
    bars = axis.bar(labels, values, color=colors)
    axis.set_title(title)
    axis.grid(axis="y", alpha=0.25)
    if logarithmic:
        axis.set_yscale("log")
    for bar, value in zip(bars, values):
        axis.text(bar.get_x() + bar.get_width() / 2, value,
                  f"{value:.3g}", ha="center", va="bottom")
fig.suptitle("Bridge contact comparison: x=-3 to -1 m (or until failure)", fontsize=15)
fig.savefig(root / "shape_comparison.png", dpi=180)
