#!/usr/bin/env python3
"""
Maglev controller — live plotter + data logger
------------------------------------------------
Shows real-time plots while running.
On Ctrl-C: saves maglev_log_YYYYMMDD_HHMMSS.csv + .xlsx with charts.

Install deps:
    pip3 install pyserial matplotlib openpyxl

Expected CSV columns from Teensy (no header, 9 values per line):
    B_target, B_meas, Error, P, I, D, iRef, iMeas, PWM%
"""

import sys
import glob
import time
import csv
import threading
import collections
from datetime import datetime

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from openpyxl import Workbook
from openpyxl.chart import LineChart, Reference

# ── Config ────────────────────────────────────────────────────────────────────
BAUD     = 115200
HISTORY  = 500      # samples visible on screen at once
INTERVAL = 50       # plot refresh ms (~20 fps)

COLS = ["B_target", "B_meas", "Error", "P", "I", "D", "iRef", "iMeas", "PWM%"]
NUM_COLS = len(COLS)

COLORS = {
    "B_target": "#888",
    "B_meas":   "#E24B4A",
    "Error":    "#EF9F27",
    "P":        "#378ADD",
    "I":        "#1D9E75",
    "D":        "#D4537E",
    "iRef":     "#7F77DD",
    "iMeas":    "#D85A30",
    "PWM%":     "#639922",
}

GROUPS = [
    ("Position (Gauss)", [0, 1, 2], ["B_target", "B_meas", "Error"]),
    ("PID terms",        [3, 4, 5], ["P", "I", "D"]),
    ("Current (A)",      [6, 7],    ["iRef", "iMeas"]),
    ("PWM duty (%)",     [8],       ["PWM%"]),
]

# ── Auto-detect serial port ───────────────────────────────────────────────────
def find_port():
    candidates = (
        glob.glob("/dev/ttyACM*") +
        glob.glob("/dev/ttyUSB*") +
        glob.glob("/dev/cu.usbmodem*") +
        glob.glob("/dev/cu.usbserial*") +
        glob.glob("COM[0-9]*")
    )
    if not candidates:
        print("ERROR: No serial ports found. Plug in the Teensy and retry.")
        sys.exit(1)
    if len(candidates) == 1:
        print(f"Using port: {candidates[0]}")
        return candidates[0]
    print("Multiple ports found:")
    for i, p in enumerate(candidates):
        print(f"  [{i}] {p}")
    choice = input("Select port number: ").strip()
    return candidates[int(choice)]

# ── Shared state ──────────────────────────────────────────────────────────────
buffers  = [collections.deque([0.0] * HISTORY, maxlen=HISTORY) for _ in COLS]
all_rows = []           # full history: [time_s, B_target, B_meas, ...]
lock     = threading.Lock()
running  = True
start_t  = time.time()

# ── Serial reader thread ──────────────────────────────────────────────────────
def reader(port):
    global running
    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"Cannot open {port}: {e}")
        running = False
        return

    print("Connected. Waiting for data...")
    while running:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) != NUM_COLS:
                print(f"  [Teensy] {line}")
                continue
            values = [float(p) for p in parts]
            elapsed = round(time.time() - start_t, 4)
            with lock:
                for buf, val in zip(buffers, values):
                    buf.append(val)
                all_rows.append([elapsed] + values)
        except (ValueError, serial.SerialException):
            continue
    ser.close()

# ── Save CSV ──────────────────────────────────────────────────────────────────
def save_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s"] + COLS)
        writer.writerows(rows)
    print(f"CSV saved   -> {path}")

# ── Save Excel ────────────────────────────────────────────────────────────────
def save_xlsx(rows, path):
    wb = Workbook()
    ws = wb.active
    ws.title = "Data"
    ws.append(["time_s"] + COLS)
    for row in rows:
        ws.append(row)

    n = len(rows)
    if n < 2:
        wb.save(path)
        print("Not enough data for charts.")
        return

    time_ref = Reference(ws, min_col=1, min_row=2, max_row=n + 1)

    def make_chart(title, y_title, col_indices, colors, anchor):
        chart = LineChart()
        chart.title        = title
        chart.style        = 10
        chart.y_axis.title = y_title
        chart.x_axis.title = "Time (s)"
        chart.width        = 22
        chart.height       = 12
        for col_idx, color in zip(col_indices, colors):
            # col_idx is 0-based into COLS; +2 because col 1 = time_s
            data_ref = Reference(ws, min_col=col_idx + 2, min_row=1, max_row=n + 1)
            chart.add_data(data_ref, titles_from_data=True)
            chart.series[-1].graphicalProperties.line.solidFill = color
        chart.set_categories(time_ref)
        ws.add_chart(chart, anchor)

    make_chart("Position (Gauss)", "B (Gauss)",
               [0, 1, 2], ["888888", "E24B4A", "EF9F27"], "K2")
    make_chart("PID terms", "Value",
               [3, 4, 5], ["378ADD", "1D9E75", "D4537E"], "K26")
    make_chart("Current (A)", "A",
               [6, 7], ["7F77DD", "D85A30"], "K50")
    make_chart("PWM duty (%)", "Duty (0-255 scaled %)",
               [8], ["639922"], "K74")

    wb.save(path)
    print(f"Excel saved -> {path}")

# ── Live plot setup ───────────────────────────────────────────────────────────
fig, axes = plt.subplots(len(GROUPS), 1, figsize=(12, 9), sharex=True)
fig.canvas.manager.set_window_title("Maglev controller telemetry")
fig.patch.set_facecolor("#1a1a1a")
x_data = list(range(HISTORY))

subplot_lines = []
for ax, (title, idxs, labels) in zip(axes, GROUPS):
    ax.set_facecolor("#111")
    ax.set_ylabel(title, color="#aaa", fontsize=9)
    ax.tick_params(colors="#666")
    for spine in ax.spines.values():
        spine.set_edgecolor("#333")
    lines = []
    for idx, lbl in zip(idxs, labels):
        ln, = ax.plot(x_data, list(buffers[idx]),
                      label=lbl, color=COLORS[lbl], linewidth=1.2)
        lines.append((ln, idx))
    ax.legend(loc="upper left", fontsize=8, framealpha=0.3,
              labelcolor="white", facecolor="#222")
    subplot_lines.append((ax, lines))

axes[-1].set_xlabel("samples", color="#aaa", fontsize=9)
plt.tight_layout(rect=[0, 0, 1, 0.97])
fig.suptitle("Maglev telemetry  |  Ctrl-C to stop and save", color="#ccc", fontsize=10)

def update(_frame):
    with lock:
        snapshot = [list(b) for b in buffers]
    for ax, lines in subplot_lines:
        for ln, idx in lines:
            ln.set_ydata(snapshot[idx])
        ax.relim()
        ax.autoscale_view(scalex=False)
    return [ln for _, lines in subplot_lines for ln, _ in lines]

ani = animation.FuncAnimation(fig, update, interval=INTERVAL, blit=True,
                               cache_frame_data=False)

# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    port = find_port()
    t = threading.Thread(target=reader, args=(port,), daemon=True)
    t.start()

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        running = False

    with lock:
        rows_snapshot = list(all_rows)

    print(f"\nStopped. Captured {len(rows_snapshot)} samples.")

    if rows_snapshot:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = f"maglev_log_{ts}"
        save_csv(rows_snapshot,  base + ".csv")
        save_xlsx(rows_snapshot, base + ".xlsx")
    else:
        print("No data captured — nothing saved.")