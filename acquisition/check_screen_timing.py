#!/usr/bin/env python3
"""Plot measured target brightness against screen-step UDP announcements.

Run on Windows after copying a short-exposure timing run from the Pi:
    python check_screen_timing.py motion_YYYYMMDD_HHMMSS

This is a coarse optical check. UDP arrival is not the physical light-switch
time, and camera sensor timestamps have no independently verified trigger.
"""

import argparse
import csv
from pathlib import Path
import statistics

import matplotlib.pyplot as plt
import numpy as np
import rawpy

def read_rows(path):
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="copied motion_... folder")
    args = parser.parse_args()
    run = args.run.resolve()
    cameras = read_rows(run / "camera_frames.csv")
    events = read_rows(run / "illumination_events.csv")
    if len(cameras) < 15 or len(events) < 8:
        parser.error("Need at least 15 short-exposure DNGs and 8 screen events")

    exposures = [int(r["exposure_time_us"]) for r in cameras]
    median_exposure = statistics.median(exposures)
    if median_exposure > 100_000:
        parser.error(f"Actual exposure is {median_exposure / 1000:.1f} ms; use preview mode (about 33 ms) for this timing check")

    times, levels = [], []
    for row in cameras:
        stamp = row["sensor_timestamp_ns"]
        if not stamp:
            continue
        path = run / "camera_dng" / row["file"]
        with rawpy.imread(str(path)) as raw:

            pixels = raw.raw_image_visible[::4, ::4]
            level = float(np.percentile(pixels, 85))
        times.append(int(stamp) / 1e9)
        levels.append(level)
    if len(times) < 15:
        parser.error("Too few camera frames have sensor timestamps")

    event_times = np.asarray([int(r["pi_receive_monotonic_ns"]) / 1e9 for r in events])
    event_states = np.asarray([int(r["state_white_1_black_0"]) for r in events])
    camera_times = np.asarray(times)
    brightness = np.asarray(levels)
    in_range = (camera_times >= event_times[0]) & (camera_times <= event_times[-1])
    if in_range.sum() < 10:
        parser.error("Camera frames do not overlap the screen events; start player during capture")
    white = np.searchsorted(event_times, camera_times[in_range], side="right") - 1
    white = event_states[white] == 1
    bright_white = brightness[in_range][white]
    bright_black = brightness[in_range][~white]
    if len(bright_white) < 3 or len(bright_black) < 3:
        parser.error("Need at least three camera frames during both white and black states")

    origin = min(camera_times[0], event_times[0])
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(camera_times - origin, brightness, ".-", label="DNG bright-pixel level")
    ymin, ymax = ax.get_ylim()
    for i, start in enumerate(event_times[:-1]):
        if event_states[i]:
            ax.axvspan(start - origin, event_times[i + 1] - origin,
                       color="orange", alpha=0.13)
    ax.set_ylim(ymin, ymax)
    ax.set_xlabel("Seconds on Pi monotonic clock")
    ax.set_ylabel("85th percentile raw pixel value")
    ax.set_title("Optical response (orange = announced white, before timing correction)")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    output = run / "screen_timing_check.png"
    fig.savefig(output, dpi=170)
    plt.close(fig)

    print(f"Camera frames: {len(camera_times)}; screen events: {len(event_times)}")
    print(f"Actual median exposure: {median_exposure / 1000:.1f} ms")
    print(f"Median camera timestamp interval: {statistics.median(np.diff(camera_times)) * 1000:.1f} ms")
    print(f"Median brightness under announced white: {statistics.median(bright_white):.0f}")
    print(f"Median brightness under announced black: {statistics.median(bright_black):.0f}")
    print(f"Plot: {output}")
    print("The coloured bands are UDP announcements, not verified optical switching times.")
    print("Inspect transition edges in the plot before making any timing or deblurring claim.")

if __name__ == "__main__":
    main()
