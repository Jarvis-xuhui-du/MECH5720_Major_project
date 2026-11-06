#!/usr/bin/env python3

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import socket
import statistics
import subprocess
import sys
import threading
import time
from urllib.request import urlopen

HEADER = re.compile(r"^Result Frame #(\d+):.*\bres=8x8, systick=(\d+)")
GRID = re.compile(r"^Frame #(\d+) \(res=8x8\):\s*$")
ERROR = re.compile(r"#Err|Failed to|Download Error|CPU is not ready|Read device .* error", re.I)

def stamp():
    return time.monotonic_ns()

def camera_worker(url, folder, stop, rows, failures, ready):

    count = 0
    try:
        while not stop.is_set():
            request_ns = stamp()
            with urlopen(url, timeout=20) as response:
                if response.status != 200:
                    raise RuntimeError(f"camera returned HTTP {response.status}")
                dng = response.read()
                received_ns = stamp()
                if len(dng) < 1024 or dng[:2] not in (b"II", b"MM"):
                    raise RuntimeError("camera returned an invalid DNG")
                sensor_ns = response.headers.get("X-Sensor-Timestamp-Ns", "")
                exposure_us = response.headers.get("X-Exposure-Time-Us", "")
                frame_duration_us = response.headers.get("X-Frame-Duration-Us", "")
            count += 1
            name = f"frame_{count:06d}.dng"
            (folder / name).write_bytes(dng)
            rows.append((count, request_ns, received_ns, sensor_ns, exposure_us,
                         frame_duration_us, name, len(dng)))
            ready.set()
    except Exception as exc:
        failures.append(f"camera: {exc}")
        ready.set()

def lidar_worker(driver, seconds, rows, raw_lines, failures, ready):

    command = ["stdbuf", "-oL", "-eL", str(driver), "-m", "-b", "0", "-d", "0",
               "-t", str(seconds), "-u"]
    current_header = None
    grid_number = None
    grid_rows = []
    process = None
    try:
        process = subprocess.Popen(command, cwd=driver.parent, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, bufsize=1)
        ready.set()
        for line in process.stdout:
            received_ns = stamp()
            line = line.rstrip("\r\n")
            raw_lines.append(f"{received_ns}\t{line}")
            if ERROR.search(line):
                failures.append(f"LiDAR driver: {line}")
            found = HEADER.match(line)
            if found:
                current_header = (int(found.group(1)), int(found.group(2)))
            found = GRID.match(line)
            if found:
                grid_number = int(found.group(1))
                grid_rows = []
                continue
            if grid_number is not None:
                values = line.split()
                if len(values) == 8 and all(x.isdecimal() for x in values):
                    grid_rows.append([int(x) for x in values])
                    if len(grid_rows) == 8:
                        if current_header and current_header[0] == grid_number:
                            rows.append((grid_number, current_header[1], received_ns, grid_rows))
                        else:
                            failures.append(f"LiDAR frame {grid_number}: missing matching systick")
                        grid_number = None
                        grid_rows = []
                else:
                    failures.append(f"LiDAR frame {grid_number}: incomplete grid")
                    grid_number = None
                    grid_rows = []
        code = process.wait()
        if code != 0:
            failures.append(f"LiDAR driver exited with code {code}")
    except Exception as exc:
        failures.append(f"LiDAR: {exc}")
        ready.set()
    finally:
        if process and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

def illumination_worker(port, stop, rows, failures, ready):

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", port))
        sock.settimeout(0.2)
        ready.set()
        while not stop.is_set():
            try:
                packet, sender = sock.recvfrom(4096)
            except socket.timeout:
                continue
            received_ns = stamp()
            try:
                event = json.loads(packet)
                if event.get("kind") != "illumination_step" or event.get("state") not in (0, 1):
                    continue
                rows.append((event.get("run_id", ""), event.get("sequence", ""),
                             event["state"], event.get("windows_perf_counter_ns", ""),
                             received_ns, sender[0]))
            except (ValueError, TypeError, KeyError):
                failures.append("Ignored malformed illumination event")
    except Exception as exc:
        failures.append(f"Illumination receiver: {exc}")
        ready.set()
    finally:
        sock.close()

def main():
    parser = argparse.ArgumentParser(description="Save repeated camera DNGs and continuous LiDAR 8x8 data")
    parser.add_argument("--seconds", type=int, default=20, help="capture duration, default 20 s")
    parser.add_argument("--camera-url", default="http://127.0.0.1:8000/capture_dng")
    parser.add_argument("--out", type=Path, default=Path(__file__).resolve().parent / "motion_runs")
    parser.add_argument("--rows", type=int, nargs="+", default=[4, 5])
    parser.add_argument("--cols", type=int, nargs="+", default=[4, 5])
    parser.add_argument("--illumination-udp-port", type=int, default=0,
                        help="optional UDP port for coded-screen event timestamps (e.g. 5051)")
    args = parser.parse_args()
    if args.seconds < 3 or any(not 1 <= x <= 8 for x in args.rows + args.cols):
        parser.error("--seconds must be at least 3; rows and columns must be 1..8")
    if args.illumination_udp_port and not 1024 <= args.illumination_udp_port <= 65535:
        parser.error("--illumination-udp-port must be 1024..65535")
    driver = Path(__file__).resolve().parent / "tmf8829"
    if not driver.is_file():
        parser.error(f"Driver not found: {driver}. Run make -B first.")
    try:
        with urlopen("http://127.0.0.1:8000/config", timeout=3) as response:
            camera_config = json.load(response)
        with urlopen("http://127.0.0.1:8000/exposure_mode", timeout=3) as response:
            exposure_mode = json.load(response)
    except Exception as exc:
        parser.error(f"Camera server is not ready on Pi port 8000: {exc}")

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = args.out / f"motion_{run_id}"
    folder.mkdir(parents=True, exist_ok=False)
    images = folder / "camera_dng"
    images.mkdir()
    camera_rows, lidar_rows, illumination_rows, raw_lines, failures = [], [], [], [], []
    stop = threading.Event()
    illumination = None
    if args.illumination_udp_port:
        illumination_ready = threading.Event()
        illumination = threading.Thread(
            target=illumination_worker,
            args=(args.illumination_udp_port, stop, illumination_rows, failures, illumination_ready),
            daemon=True,
        )
        illumination.start()
        illumination_ready.wait(timeout=2)
        if not illumination_ready.is_set() or failures:
            print(f"Illumination receiver failed: {failures}", file=sys.stderr)
            stop.set()
            return 1
    camera_ready, lidar_ready = threading.Event(), threading.Event()
    camera = threading.Thread(target=camera_worker, args=(args.camera_url, images, stop,
                             camera_rows, failures, camera_ready), daemon=True)
    camera.start()
    camera_ready.wait(timeout=22)
    if not camera_ready.is_set() or failures:
        print(f"Camera stream failed: {failures}; output folder: {folder}", file=sys.stderr)
        stop.set()
        return 1
    lidar = threading.Thread(target=lidar_worker, args=(driver, args.seconds, lidar_rows,
                            raw_lines, failures, lidar_ready), daemon=True)
    started_monotonic_ns = stamp()
    started_utc = datetime.now(timezone.utc).isoformat()
    lidar.start()
    lidar_ready.wait(timeout=3)
    print(f"Recording {args.seconds} s to {folder}. Push target through the path once; keep both sensors fixed.")
    if exposure_mode.get("name") == "long":
        print("Note: long camera exposure may blur a moving target and yield few DNG frames. Increase steady lighting or move the target between still captures.")
    lidar.join(timeout=args.seconds + 20)
    stop.set()
    camera.join(timeout=5)
    if illumination:
        illumination.join(timeout=2)
    ended_monotonic_ns = stamp()
    if lidar.is_alive():
        failures.append("LiDAR worker did not stop in time")
    if args.illumination_udp_port and not illumination_rows:
        failures.append("No illumination events received; check that the player used the Pi IP and port")

    with (folder / "camera_frames.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame", "request_monotonic_ns", "receive_monotonic_ns",
                         "sensor_timestamp_ns", "exposure_time_us", "frame_duration_us",
                         "file", "dng_bytes"])
        writer.writerows(camera_rows)
    with (folder / "lidar_frames.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame", "sensor_systick_8us", "receive_monotonic_ns", "roi_median_mm",
                         "valid_roi_zones", "distances_8x8_json"])
        for number, systick, received_ns, grid in lidar_rows:
            values = [grid[r - 1][c - 1] for r in args.rows for c in args.cols
                      if grid[r - 1][c - 1] > 0]
            writer.writerow([number, systick, received_ns,
                             statistics.median(values) if values else "", len(values),
                             json.dumps(grid, separators=(",", ":"))])
    (folder / "lidar_driver_log.txt").write_text("\n".join(raw_lines) + "\n", encoding="utf-8")
    if args.illumination_udp_port:
        with (folder / "illumination_events.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["run_id", "sequence", "state_white_1_black_0",
                             "windows_perf_counter_ns", "pi_receive_monotonic_ns", "sender_ip"])
            writer.writerows(illumination_rows)
    manifest = {
        "started_utc_pi_clock": started_utc,
        "started_monotonic_ns": started_monotonic_ns,
        "ended_monotonic_ns": ended_monotonic_ns,
        "camera_config": camera_config,
        "exposure_mode": exposure_mode,
        "camera_url": args.camera_url,
        "duration_requested_s": args.seconds,
        "roi_rows_1based": args.rows,
        "roi_cols_1based": args.cols,
        "camera_frames": len(camera_rows),
        "lidar_frames": len(lidar_rows),
        "illumination_events": len(illumination_rows),
        "illumination_udp_port": args.illumination_udp_port,
        "failures": failures,
        "timing_note": "Camera DNG header contains a sensor timestamp and exposure duration. Host request/receive_monotonic_ns and LiDAR receive_monotonic_ns use the Pi clock. LiDAR sensor systick is preserved separately. These are not hardware-synchronised; verify timestamp clock domains before computing any per-frame delay or matching.",
        "pi_clock_note": "Verify the Pi wall clock before using started_utc_pi_clock; this setup has previously shown an incorrect August 2026 date.",
        "illumination_timing_note": "Pi receive_monotonic_ns records UDP packet arrival, not the actual light transition. Display refresh and network delay require calibration before precise reconstruction.",
    }
    (folder / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Saved {len(camera_rows)} camera DNGs and {len(lidar_rows)} LiDAR 8x8 frames")
    print(f"Camera index: {folder / 'camera_frames.csv'}")
    print(f"LiDAR index: {folder / 'lidar_frames.csv'}")
    if args.illumination_udp_port:
        print(f"Illumination events: {folder / 'illumination_events.csv'} ({len(illumination_rows)} received)")
    if failures:
        print("Warnings: " + "; ".join(failures), file=sys.stderr)
    return 0 if camera_rows and lidar_rows and not failures else 1

if __name__ == "__main__":
    sys.exit(main())
