#!/usr/bin/env python3
"""Convert Ignition bridge contact logs into per-frame CSV, plots and videos."""

import argparse
import csv
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import cv2


FLOAT = r"[-+0-9.eE]+"


def parse_contacts(path):
    frames = []
    frame = None
    contact_name = ""
    position = None
    in_top_stamp = False
    sec = nsec = 0

    def finish():
        nonlocal frame
        if frame is not None:
            frame["time"] = sec + nsec * 1e-9
            frames.append(frame)

    with path.open(encoding="utf-8", errors="replace") as stream:
        for raw in stream:
            line = raw.rstrip()
            if line == "header {":
                finish()
                frame = {"points": []}
                contact_name = ""
                in_top_stamp = True
                sec = nsec = 0
                continue
            if frame is None:
                continue
            stripped = line.strip()
            if in_top_stamp and stripped.startswith("sec:"):
                sec = int(stripped.split(":", 1)[1])
            elif in_top_stamp and stripped.startswith("nsec:"):
                nsec = int(stripped.split(":", 1)[1])
                in_top_stamp = False
            elif stripped.startswith("name:"):
                contact_name = stripped.split(":", 1)[1].strip().strip('"')
            elif stripped == "position {":
                position = {}
            elif position is not None:
                match = re.match(rf"\s*([xyz]):\s*({FLOAT})", line)
                if match:
                    position[match.group(1)] = float(match.group(2))
                elif stripped == "}":
                    # The bridge sensor also sees pallets and other arena
                    # objects. Keep crawler contacts only.
                    if (all(axis in position for axis in "xyz") and
                            "crawler::" in contact_name):
                        frame["points"].append((
                            position["x"], position["y"], position["z"],
                            contact_name,
                        ))
                    position = None
    finish()
    return [frame for frame in frames if frame["points"]]


def parse_tf(path):
    rows = []
    stamp = None
    xyz = None
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if line.startswith("At time "):
                stamp = float(line.split()[2])
            elif line.startswith("- Translation:"):
                xyz = [float(value) for value in
                       re.search(r"\[([^]]+)\]", line).group(1).split(",")]
            elif "RPY (degree)" in line and stamp is not None and xyz is not None:
                rpy = [float(value) for value in
                       re.search(r"\[([^]]+)\]", line).group(1).split(",")]
                rows.append((stamp, *xyz, *rpy))
    return np.asarray(rows, dtype=float)


def urdf_mass(path):
    root = ET.parse(path).getroot()
    return sum(float(node.get("value", node.text or 0.0))
               for node in root.findall(".//mass"))


def first_run_slice(values, low=-3.1, high=-1.0):
    start_candidates = np.flatnonzero(values >= low)
    if not len(start_candidates):
        return slice(0, len(values))
    start = int(start_candidates[0])
    end_candidates = np.flatnonzero(values[start:] >= high)
    end = start + int(end_candidates[0]) + 1 if len(end_candidates) else len(values)
    return slice(start, end)


def smooth_force(tf, mass):
    if len(tf) < 5:
        return np.zeros(len(tf))
    t = tf[:, 0] - tf[0, 0]
    x = tf[:, 1]
    # A short symmetric moving average limits the 1 Hz TF quantization noise.
    window = min(5, len(x) if len(x) % 2 else len(x) - 1)
    kernel = np.ones(window) / window
    xs = np.convolve(np.pad(x, window // 2, mode="edge"), kernel, mode="valid")
    velocity = np.gradient(xs, t)
    acceleration = np.gradient(velocity, t)
    return mass * acceleration


def analyze(shape, raw_dir, output_dir, fps=30):
    contacts = parse_contacts(raw_dir / f"{shape}_contacts.txt")
    tf = parse_tf(raw_dir / f"{shape}_tf.txt")
    mass = urdf_mass(raw_dir / f"crawler_body_{shape}.urdf")

    centers = np.asarray([
        (np.mean([p[0] for p in f["points"]]),
         np.mean([p[1] for p in f["points"]])) for f in contacts
    ])
    contact_slice = first_run_slice(centers[:, 0])
    contacts = contacts[contact_slice]
    t0 = contacts[0]["time"] if contacts else 0.0

    tf_slice = first_run_slice(tf[:, 1]) if len(tf) else slice(0, 0)
    tf = tf[tf_slice]
    force_x = smooth_force(tf, mass)

    point_csv = output_dir / f"{shape}_contact_points.csv"
    frame_csv = output_dir / f"{shape}_frames.csv"
    with point_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["frame", "sim_time_s", "world_x_m", "world_y_m",
                         "world_z_m", "collision"])
        for index, frame in enumerate(contacts):
            for point in frame["points"]:
                writer.writerow([index, frame["time"] - t0, *point])
    with frame_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["frame", "sim_time_s", "point_count", "center_x_m",
                         "center_y_m", "span_x_m", "span_y_m"])
        for index, frame in enumerate(contacts):
            points = np.asarray([p[:3] for p in frame["points"]])
            writer.writerow([
                index, frame["time"] - t0, len(points),
                points[:, 0].mean(), points[:, 1].mean(),
                np.ptp(points[:, 0]), np.ptp(points[:, 1]),
            ])

    # One video image is emitted for every retained contact publication. OpenCV
    # keeps this practical even when the contact system publishes hundreds of
    # frames per second.
    video = output_dir / f"{shape}_contacts_2d.mp4"
    width_px, height_px = 960, 540
    video_writer = cv2.VideoWriter(
        str(video), cv2.VideoWriter_fourcc(*"mp4v"), fps,
        (width_px, height_px))
    for index, frame in enumerate(contacts):
        points = np.asarray([p[:3] for p in frame["points"]])
        cx = points[:, 0].mean()
        image = np.full((height_px, width_px, 3), 248, dtype=np.uint8)
        cv2.rectangle(image, (70, 55), (910, 475), (80, 80, 80), 1)
        cv2.line(image, (490, 55), (490, 475), (205, 205, 205), 1)
        cv2.line(image, (70, 265), (910, 265), (205, 205, 205), 1)
        for px, py, pz in points:
            ix = int(70 + ((px - cx) + 0.45) / 0.9 * 840)
            iy = int(475 - (py + 0.35) / 0.7 * 420)
            color_value = int(np.clip((pz - 0.145) / 0.015 * 255, 0, 255))
            cv2.circle(image, (ix, iy), 3,
                       (255 - color_value, 120, color_value), -1)
        cv2.putText(image,
                    f"{shape} frame {index}/{len(contacts)-1}  x={cx:.3f} m  points={len(points)}",
                    (70, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (30, 30, 30), 2)
        cv2.putText(image, "relative longitudinal x [m]", (370, 515),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 40), 1)
        cv2.putText(image, "world y [m]", (8, 270),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 40, 40), 1)
        video_writer.write(image)
    video_writer.release()

    summary = []
    frame_times = np.asarray([f["time"] - t0 for f in contacts])
    counts = np.asarray([len(f["points"]) for f in contacts])
    spans = np.asarray([np.ptp([p[0] for p in f["points"]]) for f in contacts])
    fig, axes = plt.subplots(4, 1, figsize=(10, 11), constrained_layout=True)
    axes[0].plot(frame_times, counts, lw=0.8)
    axes[0].set(ylabel="contact points / frame", title=f"{shape}: bridge x=-3 to -1 m")
    axes[1].plot(frame_times, spans, lw=0.8)
    axes[1].set(ylabel="contact span x [m]")
    if len(tf):
        tt = tf[:, 0] - tf[0, 0]
        axes[2].plot(tt, tf[:, 5], label="pitch")
        axes[2].plot(tt, tf[:, 4], label="roll", alpha=0.8)
        axes[2].legend()
        axes[3].plot(tt, force_x, label="estimated net Fx = m ax")
        friction_limit = 0.8 * mass * 9.80665
        axes[3].axhline(friction_limit, color="tab:red", ls="--", label="mu m g")
        axes[3].axhline(-friction_limit, color="tab:red", ls="--")
        axes[3].legend()
    axes[2].set(ylabel="angle [deg]")
    axes[3].set(ylabel="force [N]", xlabel="elapsed time [s]")
    for axis in axes:
        axis.grid(True, alpha=0.25)
    fig.savefig(output_dir / f"{shape}_metrics.png", dpi=160)
    plt.close(fig)

    reached = bool(len(tf) and np.max(tf[:, 1]) >= -1.0)
    summary = {
        "shape": shape,
        "mass_kg": mass,
        "contact_frames": len(contacts),
        "contact_points": int(counts.sum()) if len(counts) else 0,
        "mean_points_per_frame": float(counts.mean()) if len(counts) else 0.0,
        "max_points_per_frame": int(counts.max()) if len(counts) else 0,
        "mean_span_x_m": float(spans.mean()) if len(spans) else 0.0,
        "reached_x_minus_1": reached,
        "max_abs_pitch_deg": float(np.max(np.abs(tf[:, 5]))) if len(tf) else math.nan,
        "max_abs_roll_deg": float(np.max(np.abs(tf[:, 4]))) if len(tf) else math.nan,
        "max_abs_estimated_force_n": float(np.max(np.abs(force_x))) if len(force_x) else math.nan,
        "friction_limit_n": 0.8 * mass * 9.80665,
    }
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--shapes", nargs="+", default=["rectangle", "semicircle", "spike"])
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summaries = [analyze(shape, args.raw_dir, args.output_dir) for shape in args.shapes]
    keys = list(summaries[0])
    with (args.output_dir / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(summaries)
    print(*(f"{row['shape']}: {row}" for row in summaries), sep="\n")


if __name__ == "__main__":
    main()
