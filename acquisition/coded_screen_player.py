#!/usr/bin/env python3

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import socket
import time
import tkinter as tk
import uuid

PATTERN = "1101001101"

def main():
    parser = argparse.ArgumentParser(description="Display a timed binary light pattern and announce steps to the Pi")
    parser.add_argument("--pi-ip", required=True, help="Pi Ethernet IP, usually 10.42.1.1")
    parser.add_argument("--port", type=int, default=5051, help="UDP event port (default 5051)")
    parser.add_argument("--bit-ms", type=int, default=200, help="bit period in ms (default 200)")
    parser.add_argument("--seconds", type=int, default=12, help="pattern duration in s (default 12)")
    parser.add_argument("--x", type=int, default=None, help="optional left edge of the display in virtual-screen pixels")
    parser.add_argument("--y", type=int, default=0, help="optional top edge of the display in virtual-screen pixels")
    args = parser.parse_args()
    if args.bit_ms < 100 or args.seconds < 2:
        parser.error("use --bit-ms >= 100 and --seconds >= 2")

    run_id = datetime.now().strftime("coded_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:6]
    out = Path(__file__).resolve().parent / f"{run_id}_screen_events.csv"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    root = tk.Tk()
    root.title("MECH5720 coded illumination")
    root.configure(bg="black")
    root.geometry(f"800x600+{args.x}+{args.y}" if args.x is not None else "800x600")
    root.focus_force()
    status = tk.Label(root, text="Move to the monitor. F = full screen, S = start, Esc = stop.",
                      fg="white", bg="black", font=("Arial", 24))
    status.place(relx=0.5, rely=0.5, anchor="center")
    rows = []
    running = False
    start_ns = None
    next_step = 0

    def save():
        with out.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["run_id", "sequence", "state_white_1_black_0",
                             "windows_perf_counter_ns", "elapsed_ms", "pi_ip", "pi_port",
                             "bit_ms", "pattern"])
            writer.writerows(rows)
        print(f"Saved {len(rows)} illumination steps: {out}")

    def stop(event=None):
        nonlocal running
        running = False
        save()
        sock.close()
        root.destroy()

    def tick():
        nonlocal next_step, running
        if not running:
            return
        scheduled_ms = next_step * args.bit_ms
        if scheduled_ms >= args.seconds * 1000:
            root.configure(bg="black")
            stop()
            return
        state = int(PATTERN[next_step % len(PATTERN)])
        root.configure(bg="white" if state else "black")
        local_ns = time.perf_counter_ns()
        event = {"kind": "illumination_step", "run_id": run_id,
                 "sequence": next_step, "state": state,
                 "windows_perf_counter_ns": local_ns}
        try:
            sock.sendto(json.dumps(event, separators=(",", ":")).encode(),
                        (args.pi_ip, args.port))
        except OSError as exc:
            print(f"UDP event send failed: {exc}")
        rows.append((run_id, next_step, state, local_ns,
                     round((local_ns - start_ns) / 1e6, 3), args.pi_ip, args.port,
                     args.bit_ms, PATTERN))
        next_step += 1
        delay_ms = max(1, round(next_step * args.bit_ms - (time.perf_counter_ns() - start_ns) / 1e6))
        root.after(delay_ms, tick)

    def start(event=None):
        nonlocal running, start_ns
        if running:
            return
        status.place_forget()
        root.configure(bg="black")
        root.update_idletasks()
        running = True
        start_ns = time.perf_counter_ns()
        tick()

    root.bind("<Key-s>", start)
    root.bind("<Key-S>", start)
    root.bind("<Key-f>", lambda event: root.attributes("-fullscreen", True))
    root.bind("<Key-F>", lambda event: root.attributes("-fullscreen", True))
    root.bind("<Escape>", stop)
    root.protocol("WM_DELETE_WINDOW", stop)
    print(f"Ready on display. Press S to start {args.seconds} s pattern {PATTERN} at {args.bit_ms} ms/bit.")
    print(f"Sending step announcements to {args.pi_ip}:{args.port}; run ID {run_id}")
    root.mainloop()

if __name__ == "__main__":
    main()
