#!/usr/bin/env python3

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import statistics
import subprocess
import sys

FRAME_RE = re.compile(r"^Frame #(\d+) \(res=8x8\):\s*$")
ERROR_RE = re.compile(
    r"(?:^|\s)(?:#Err|Failed to|Download Error|CPU is not ready|"
    r"Read device .* error|Could not|Unexpected application ID)", re.IGNORECASE
)

def parse_frames(raw):

    lines = raw.splitlines()
    frames = []
    for index, line in enumerate(lines):
        match = FRAME_RE.match(line)
        if not match:
            continue
        rows = []
        for row_text in lines[index + 1:index + 9]:
            fields = row_text.split()
            if len(fields) != 8 or not all(field.isdecimal() for field in fields):
                raise ValueError(f"Frame {match.group(1)} has an incomplete 8x8 table")
            rows.append([int(field) for field in fields])
        if len(rows) != 8:
            raise ValueError(f"Frame {match.group(1)} has an incomplete 8x8 table")
        frames.append((int(match.group(1)), rows))
    return frames

def summarise(frames, rows, cols):

    stable = frames[1:]
    readings = []
    for number, grid in stable:
        valid = [grid[row - 1][col - 1] for row in rows for col in cols
                 if grid[row - 1][col - 1] > 0]
        if valid:
            readings.append((number, statistics.median(valid), len(valid)))
    if not readings:
        raise ValueError("No valid distances remain after skipping the first frame")
    medians = [item[1] for item in readings]
    return readings, statistics.median(medians), min(medians), max(medians)

def main():
    parser = argparse.ArgumentParser(
        description="Measure a stationary target; save full LiDAR data and print a short summary."
    )
    parser.add_argument("target_cm", type=float, help="Ruler distance from sensor to target, in cm")
    parser.add_argument("--seconds", type=int, default=3, help="Capture time (default: 3 s)")
    parser.add_argument("--rows", type=int, nargs="+", default=[4, 5],
                        help="1-based LiDAR row numbers seeing the target (default: 4 5)")
    parser.add_argument("--cols", type=int, nargs="+", default=[4, 5],
                        help="1-based LiDAR column numbers seeing the target (default: 4 5)")
    parser.add_argument("--out", type=Path, default=Path(__file__).resolve().parent / "runs",
                        help="Output folder (default: runs next to this script)")
    args = parser.parse_args()
    if args.target_cm <= 0 or args.seconds < 1:
        parser.error("target_cm and --seconds must be positive")
    if any(not 1 <= value <= 8 for value in args.rows + args.cols):
        parser.error("--rows and --cols must contain only numbers from 1 to 8")
    if len(set(args.rows)) != len(args.rows) or len(set(args.cols)) != len(args.cols):
        parser.error("--rows and --cols cannot contain duplicates")

    driver = Path(__file__).resolve().parent / "tmf8829"
    if not driver.is_file():
        parser.error(f"Driver not found: {driver}. Run make -B on the Pi first.")

    args.out.mkdir(parents=True, exist_ok=True)
    label = f"{args.target_cm:g}".replace(".", "p")
    run_id = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
    stem = args.out / f"lidar_{label}cm_{run_id}"
    command = [str(driver), "-m", "-b", "0", "-d", "0", "-t", str(args.seconds), "-u"]
    started = datetime.now(timezone.utc).isoformat()
    print(f"Measuring {args.target_cm:g} cm for {args.seconds} s; please keep the target still...")
    try:
        result = subprocess.run(command, cwd=driver.parent, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=args.seconds + 20, check=False)
    except subprocess.TimeoutExpired as exc:
        partial = exc.stdout or b""
        if isinstance(partial, bytes):
            partial = partial.decode(errors="replace")
        stem.with_suffix(".txt").write_text(partial, encoding="utf-8")
        print(f"Driver timed out. Partial log: {stem}.txt", file=sys.stderr)
        return 1
    ended = datetime.now(timezone.utc).isoformat()
    raw = result.stdout
    stem.with_suffix(".txt").write_text(raw, encoding="utf-8")

    try:
        frames = parse_frames(raw)
        errors = [line for line in raw.splitlines() if ERROR_RE.search(line)]
        if result.returncode != 0 or errors or len(frames) < 2:
            details = errors[-1] if errors else f"exit={result.returncode}, frames={len(frames)}"
            raise ValueError(f"Measurement failed or produced too few frames: {details}")
        readings, median_mm, minimum_mm, maximum_mm = summarise(
            frames, args.rows, args.cols
        )
    except ValueError as exc:
        print(f"{exc}\nFull driver log: {stem}.txt", file=sys.stderr)
        return 1

    with stem.with_suffix(".csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame", "row", "col", "distance_mm"])
        for number, grid in frames:
            for row, values in enumerate(grid, 1):
                for col, distance in enumerate(values, 1):
                    writer.writerow([number, row, col, distance])

    summary = {
        "target_cm": args.target_cm,
        "measured_from": "LiDAR sensor face; user-supplied ruler position",
        "started_utc": started,
        "ended_utc": ended,
        "mode": "8x8",
        "roi_rows_1based": args.rows,
        "roi_cols_1based": args.cols,
        "frames_total": len(frames),
        "frames_used": len(readings),
        "frames_skipped_initial": 1,
        "invalid_zero_values_excluded": True,
        "distance_mm": median_mm,
        "per_frame_roi_median_min_mm": minimum_mm,
        "per_frame_roi_median_max_mm": maximum_mm,
        "difference_from_ruler_mm": median_mm - args.target_cm * 10,
        "method": "median of positive ROI values in each frame, then median across frames",
        "raw_log": str(stem.with_suffix(".txt")),
        "zones_csv": str(stem.with_suffix(".csv")),
    }
    stem.with_suffix(".json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"LiDAR: {median_mm:g} mm ({median_mm / 10:g} cm); "
          f"ruler: {args.target_cm:g} cm; difference: {summary['difference_from_ruler_mm']:+g} mm")
    print(f"ROI rows {args.rows}, columns {args.cols}; "
          f"{len(readings)} stable frames; per-frame range {minimum_mm:g}–{maximum_mm:g} mm")
    if abs(summary["difference_from_ruler_mm"]) > 50:
        print("Check: result differs from the ruler by more than 50 mm. "
              "Confirm the target fills the selected zones and recheck the ruler reference.")
    if maximum_mm - minimum_mm > 50:
        print("Check: the selected zones changed by more than 50 mm during capture. "
              "Keep the target still or select zones fully on the target.")
    print(f"Saved: {stem}.txt, .csv, .json")
    return 0

if __name__ == "__main__":
    sys.exit(main())
