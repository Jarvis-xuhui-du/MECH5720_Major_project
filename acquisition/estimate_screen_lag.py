#!/usr/bin/env python3
"""Estimate coarse screen optical lag relative to UDP arrival on the Pi.

Use a preview-mode run with many short DNG exposures. This is an optical
cross-check, not a hardware-synchronisation measurement.
"""

import argparse
import csv
from pathlib import Path

import numpy as np
import rawpy

def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    cameras = read_csv(run / "camera_frames.csv")
    events = read_csv(run / "illumination_events.csv")
    changes = [(int(e["pi_receive_monotonic_ns"]) / 1e9,
                int(e["state_white_1_black_0"])) for e in events]
    changes = [changes[i] for i in range(len(changes))
               if i == 0 or changes[i][1] != changes[i - 1][1]]
    starts, ends, brightness = [], [], []
    for row in cameras:
        duration = int(row["exposure_time_us"]) / 1e6
        if duration > 0.1 or not row["sensor_timestamp_ns"]:
            continue
        end = int(row["sensor_timestamp_ns"]) / 1e9
        with rawpy.imread(str(run / "camera_dng" / row["file"])) as raw:
            level = np.percentile(raw.raw_image_visible[::4, ::4], 85)
        starts.append(end - duration)
        ends.append(end)
        brightness.append(level)
    starts = np.asarray(starts)
    ends = np.asarray(ends)
    y = np.asarray(brightness, dtype=float)
    if len(y) < 20 or len(changes) < 8:
        parser.error("Need a preview-mode run with at least 20 DNGs and 8 optical changes")

    def score(lag):
        shifted = np.asarray([t + lag for t, _ in changes])
        states = np.asarray([state for _, state in changes])
        mask = (starts >= shifted[0]) & (ends <= shifted[-1])
        x = np.zeros(mask.sum())
        for i, (a, b) in enumerate(zip(starts[mask], ends[mask])):
            points = np.concatenate(([a], shifted[(shifted > a) & (shifted < b)], [b]))
            mid = (points[:-1] + points[1:]) / 2
            idx = np.searchsorted(shifted, mid, side="right") - 1
            x[i] = np.sum(np.diff(points) * states[idx]) / (b - a)
        if len(x) < 20 or np.std(x) < 0.1:
            return float("nan"), len(x)
        yy = y[mask]
        corr = np.corrcoef(x, yy)[0, 1]
        return corr * corr if corr > 0 else -corr * corr, len(x)

    lags = np.arange(-0.3, 0.3001, 0.005)
    scores = [score(float(lag))[0] for lag in lags]
    best_idx = int(np.nanargmax(scores))
    best_lag = float(lags[best_idx])
    best_score, n = score(best_lag)
    near = lags[np.asarray(scores) >= best_score - 0.01]
    print(f"Optical lag relative to Pi UDP arrival: {best_lag * 1000:+.0f} ms")
    print(f"Fit plateau within 0.01 of peak R^2 (not timing uncertainty): {near.min() * 1000:+.0f} to {near.max() * 1000:+.0f} ms")
    print(f"Screen-state/RAW-brightness R^2: {best_score:.3f}; overlapping preview frames: {n}")
    print("Use this as a coarse offset only; display latency, camera timing, and UDP receipt remain unverified.")

if __name__ == "__main__":
    main()
