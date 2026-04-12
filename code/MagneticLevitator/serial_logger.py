"""
ESP32 Serial Logger + Excel Plotter
------------------------------------
Reads CSV serial output from the ESP32, prints live to terminal,
and saves to CSV + Excel with charts on Ctrl+C.

Expected CSV format from ESP32:
    duty,i_ref_A,i_meas_A,B_gauss
    128,0.400,0.398,123.4

Usage:
    pip install pyserial openpyxl
    python serial_logger.py
"""

import csv
import time
from datetime import datetime

import serial
import serial.tools.list_ports
from openpyxl import Workbook
from openpyxl.chart import LineChart, Reference

BAUD   = 115200
HEADER = ["duty", "i_ref_A", "i_meas_A", "B_gauss"]


# ─────────────────────────── port detection ───────────────────────────
def find_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Plug in your ESP32 and try again.")
    for p in ports:
        desc = (p.description or "").lower()
        if any(kw in desc for kw in ["cp210", "ch340", "uart", "usb serial"]):
            return p.device
    return ports[0].device


# ─────────────────────────── save functions ───────────────────────────
def save_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "duty", "i_ref_A", "i_meas_A", "B_gauss"])
        writer.writerows(rows)
    print(f"\nCSV saved  → {path}")


def save_xlsx(rows, path):
    wb = Workbook()
    ws = wb.active
    ws.title = "Data"

    ws.append(["time_s", "duty", "i_ref_A", "i_meas_A", "B_gauss"])
    for row in rows:
        ws.append(row)

    n = len(rows)
    if n < 2:
        print("Not enough data for charts.")
        wb.save(path)
        return

    time_ref = Reference(ws, min_col=1, min_row=2, max_row=n + 1)

    # Chart 1: Current
    chart_i = LineChart()
    chart_i.title        = "Current vs Time"
    chart_i.style        = 10
    chart_i.y_axis.title = "Current (A)"
    chart_i.x_axis.title = "Time (s)"
    chart_i.width        = 20
    chart_i.height       = 12
    chart_i.add_data(Reference(ws, min_col=3, min_row=1, max_row=n + 1), titles_from_data=True)
    chart_i.add_data(Reference(ws, min_col=4, min_row=1, max_row=n + 1), titles_from_data=True)
    chart_i.set_categories(time_ref)
    chart_i.series[0].graphicalProperties.line.solidFill = "FF0000"  # i_ref  = red
    chart_i.series[1].graphicalProperties.line.solidFill = "0070C0"  # i_meas = blue
    ws.add_chart(chart_i, "G2")

    # Chart 2: Magnetic field
    chart_b = LineChart()
    chart_b.title        = "Magnetic Field vs Time"
    chart_b.style        = 10
    chart_b.y_axis.title = "B (Gauss)"
    chart_b.x_axis.title = "Time (s)"
    chart_b.width        = 20
    chart_b.height       = 12
    chart_b.add_data(Reference(ws, min_col=5, min_row=1, max_row=n + 1), titles_from_data=True)
    chart_b.set_categories(time_ref)
    chart_b.series[0].graphicalProperties.line.solidFill = "00B050"  # green
    ws.add_chart(chart_b, "G26")

    # Chart 3: PWM duty
    chart_d = LineChart()
    chart_d.title        = "PWM Duty vs Time"
    chart_d.style        = 10
    chart_d.y_axis.title = "Duty (0-255)"
    chart_d.x_axis.title = "Time (s)"
    chart_d.width        = 20
    chart_d.height       = 12
    chart_d.add_data(Reference(ws, min_col=2, min_row=1, max_row=n + 1), titles_from_data=True)
    chart_d.set_categories(time_ref)
    chart_d.series[0].graphicalProperties.line.solidFill = "7030A0"  # purple
    ws.add_chart(chart_d, "G50")

    wb.save(path)
    print(f"Excel saved → {path}")


# ─────────────────────────── main ───────────────────────────
def main():
    port = find_port()
    timestamp   = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_base = f"esp32_log_{timestamp}"
    csv_path    = output_base + ".csv"
    xlsx_path   = output_base + ".xlsx"

    print(f"  Port  : {port}")
    print(f"  Baud  : {BAUD}")
    print(f"  Output: {output_base}.[csv/xlsx]")
    print("\nLogging... press Ctrl+C to stop and save.\n")

    rows         = []
    header_found = False
    start        = time.time()

    with serial.Serial(port, BAUD, timeout=1) as ser:
        time.sleep(2)             # wait for ESP32 boot + calibration
        ser.reset_input_buffer()

        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                parts = line.split(",")

                # confirm header once, then skip it
                if parts == HEADER:
                    if not header_found:
                        print(f"  Header detected: {line}\n")
                        header_found = True
                    continue

                # pass through any non-data lines (calibration output etc.)
                if len(parts) != len(HEADER):
                    print(f"  [ESP32] {line}")
                    continue

                try:
                    duty    = int(parts[0])
                    i_ref   = float(parts[1])
                    i_meas  = float(parts[2])
                    b_gauss = float(parts[3])
                except ValueError:
                    continue

                elapsed = round(time.time() - start, 3)
                rows.append([elapsed, duty, i_ref, i_meas, b_gauss])

                print(f"  t={elapsed:7.3f}s | duty={duty:3d} | "
                      f"I_ref={i_ref:.3f}A | I_meas={i_meas:.3f}A | B={b_gauss:.1f}G")

        except KeyboardInterrupt:
            pass   # fall through to save

    print(f"\nStopped. Captured {len(rows)} samples.")

    if rows:
        save_csv(rows, csv_path)
        save_xlsx(rows, xlsx_path)
    else:
        print("No data captured — nothing saved.")


if __name__ == "__main__":
    main()