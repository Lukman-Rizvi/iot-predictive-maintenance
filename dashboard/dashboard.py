"""
dashboard.py — Real-time vibration monitoring dashboard
========================================================
IoT-Based Predictive Maintenance System
Dept : Electrical & Electronic Engineering
Uni  : University of Peradeniya, Sri Lanka
Team : E22215, E22028, E22177, E22287, E22199

Usage:
    pip install -r requirements.txt
    python dashboard.py

Requires a running Mosquitto MQTT broker on the host configured
in config.yaml (default: localhost:1883).
"""

import json
import csv
import time
import threading
import datetime
import os
from collections import deque

import yaml
import paho.mqtt.client as mqtt
import matplotlib
matplotlib.use("TkAgg")          # change to "Qt5Agg" if TkAgg not available
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
import numpy as np

# ── Load config ───────────────────────────────────────────────────
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "config.yaml")
with open(CONFIG_FILE) as f:
    cfg = yaml.safe_load(f)

BROKER       = cfg["mqtt"]["broker"]
PORT         = cfg["mqtt"]["port"]
TOPIC        = cfg["mqtt"]["topic"]
HISTORY      = cfg["display"]["history_samples"]   # rolling window length
UPDATE_MS    = cfg["display"]["update_ms"]
LOG_DIR      = cfg["logging"]["directory"]

# ── Data buffers (thread-safe via deque) ─────────────────────────
timestamps = deque(maxlen=HISTORY)
ax_rms_buf = deque(maxlen=HISTORY)
ay_rms_buf = deque(maxlen=HISTORY)
az_rms_buf = deque(maxlen=HISTORY)
thr_buf    = deque(maxlen=HISTORY)

latest = {
    "ax_rms": 0.0, "ay_rms": 0.0, "az_rms": 0.0,
    "threshold": 0.20, "fault": False, "uptime_s": 0,
    "max_ax": 0.0, "max_ay": 0.0, "max_az": 0.0,
    "total_faults": 0, "connected": False,
}

data_lock = threading.Lock()

# ── CSV logger ────────────────────────────────────────────────────
os.makedirs(LOG_DIR, exist_ok=True)
log_filename = os.path.join(
    LOG_DIR,
    f"vibration_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
)
log_file   = open(log_filename, "w", newline="")
log_writer = csv.writer(log_file)
log_writer.writerow(["timestamp", "ax_rms", "ay_rms", "az_rms",
                     "threshold", "fault_status", "uptime_s"])

def log_row(ts, ax, ay, az, thr, fault, uptime):
    log_writer.writerow([ts, f"{ax:.4f}", f"{ay:.4f}", f"{az:.4f}",
                         f"{thr:.4f}", "FAULT" if fault else "OK", uptime])
    log_file.flush()

# ── MQTT callbacks ────────────────────────────────────────────────
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[MQTT] Connected to broker {BROKER}:{PORT}")
        client.subscribe(TOPIC)
        with data_lock:
            latest["connected"] = True
    else:
        print(f"[MQTT] Connection failed (rc={rc})")

def on_disconnect(client, userdata, rc):
    with data_lock:
        latest["connected"] = False
    print("[MQTT] Disconnected.")

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        ts  = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        ax  = float(payload.get("ax_rms", 0))
        ay  = float(payload.get("ay_rms", 0))
        az  = float(payload.get("az_rms", 0))
        thr = float(payload.get("threshold", 0.20))
        flt = bool(payload.get("fault", False))
        upt = int(payload.get("uptime_s", 0))

        with data_lock:
            timestamps.append(ts)
            ax_rms_buf.append(ax)
            ay_rms_buf.append(ay)
            az_rms_buf.append(az)
            thr_buf.append(thr)
            latest["ax_rms"]     = ax
            latest["ay_rms"]     = ay
            latest["az_rms"]     = az
            latest["threshold"]  = thr
            latest["fault"]      = flt
            latest["uptime_s"]   = upt
            latest["max_ax"]     = max(latest["max_ax"], ax)
            latest["max_ay"]     = max(latest["max_ay"], ay)
            latest["max_az"]     = max(latest["max_az"], az)
            if flt:
                latest["total_faults"] += 1

        log_row(ts, ax, ay, az, thr, flt, upt)

    except Exception as e:
        print(f"[MSG ERROR] {e}")

# ── Start MQTT client in background thread ────────────────────────
mq = mqtt.Client(client_id="pdm_dashboard")
mq.on_connect    = on_connect
mq.on_disconnect = on_disconnect
mq.on_message    = on_message
mq.connect_async(BROKER, PORT, keepalive=30)
mq.loop_start()

# ── Matplotlib figure setup ───────────────────────────────────────
COLORS = {
    "bg":      "#0d1117",
    "ax_line": "#58a6ff",
    "ay_line": "#3fb950",
    "az_line": "#f0883e",
    "thr":     "#ff7b72",
    "panel_ok":    "#1a6b3c",
    "panel_fault": "#8b0000",
    "text":    "#e6edf3",
    "grid":    "#21262d",
    "warning": "#d29922",
}

plt.rcParams.update({
    "figure.facecolor":  COLORS["bg"],
    "axes.facecolor":    COLORS["bg"],
    "axes.edgecolor":    COLORS["grid"],
    "axes.labelcolor":   COLORS["text"],
    "xtick.color":       COLORS["text"],
    "ytick.color":       COLORS["text"],
    "text.color":        COLORS["text"],
    "grid.color":        COLORS["grid"],
    "legend.facecolor":  "#161b22",
    "legend.edgecolor":  COLORS["grid"],
    "font.family":       "monospace",
})

fig = plt.figure(figsize=(14, 8))
fig.canvas.manager.set_window_title(
    "IoT Predictive Maintenance — University of Peradeniya (EEE)")

gs = gridspec.GridSpec(3, 2, width_ratios=[3, 1], hspace=0.45, wspace=0.3,
                       left=0.07, right=0.97, top=0.93, bottom=0.07)

ax_plot = fig.add_subplot(gs[0, 0])   # X-axis RMS
ay_plot = fig.add_subplot(gs[1, 0])   # Y-axis RMS
az_plot = fig.add_subplot(gs[2, 0])   # Z-axis RMS
status_ax = fig.add_subplot(gs[:, 1]) # Status panel

for ax_obj, label, color in [
    (ax_plot, "X-Axis RMS (g) — Radial",    COLORS["ax_line"]),
    (ay_plot, "Y-Axis RMS (g) — Tangential", COLORS["ay_line"]),
    (az_plot, "Z-Axis RMS (g) — Axial",      COLORS["az_line"]),
]:
    ax_obj.set_ylabel(label, fontsize=9, color=color)
    ax_obj.set_xlim(0, HISTORY)
    ax_obj.set_ylim(0, 0.8)
    ax_obj.grid(True, linestyle="--", alpha=0.4)
    ax_obj.tick_params(axis="x", labelbottom=False)

az_plot.tick_params(axis="x", labelbottom=True)
az_plot.set_xlabel("Samples (most recent →)", fontsize=8)

line_ax, = ax_plot.plot([], [], color=COLORS["ax_line"], lw=1.5, label="AX RMS")
thr_ax   = ax_plot.axhline(0.20, color=COLORS["thr"], lw=1.2, ls="--", label="Threshold")

line_ay, = ay_plot.plot([], [], color=COLORS["ay_line"], lw=1.5, label="AY RMS")
thr_ay   = ay_plot.axhline(0.20, color=COLORS["thr"], lw=1.2, ls="--", label="Threshold")

line_az, = az_plot.plot([], [], color=COLORS["az_line"], lw=1.5, label="AZ RMS")
thr_az   = az_plot.axhline(0.20, color=COLORS["thr"], lw=1.2, ls="--", label="Threshold")

for line, axobj in [(line_ax, ax_plot), (line_ay, ay_plot), (line_az, az_plot)]:
    axobj.legend(loc="upper left", fontsize=8)

fig.suptitle(
    "IoT Predictive Maintenance — Vibration Monitor\n"
    "Electrical & Electronic Engineering, University of Peradeniya",
    fontsize=11, color=COLORS["text"], y=0.98
)

status_ax.set_axis_off()

# ── Animation update function ─────────────────────────────────────
def update(frame):
    with data_lock:
        ax_data  = list(ax_rms_buf)
        ay_data  = list(ay_rms_buf)
        az_data  = list(az_rms_buf)
        thr      = latest["threshold"]
        fault    = latest["fault"]
        cur_ax   = latest["ax_rms"]
        cur_ay   = latest["ay_rms"]
        cur_az   = latest["az_rms"]
        max_ax   = latest["max_ax"]
        max_ay   = latest["max_ay"]
        max_az   = latest["max_az"]
        uptime   = latest["uptime_s"]
        flt_cnt  = latest["total_faults"]
        conn     = latest["connected"]

    x = list(range(len(ax_data)))

    line_ax.set_data(x, ax_data)
    line_ay.set_data(x, ay_data)
    line_az.set_data(x, az_data)

    for thr_line in [thr_ax, thr_ay, thr_az]:
        thr_line.set_ydata([thr, thr])

    peak = max(cur_ax, cur_ay, cur_az)
    ymax = max(0.5, peak * 1.4, thr * 1.3)
    for axobj in [ax_plot, ay_plot, az_plot]:
        axobj.set_ylim(0, ymax)
        axobj.set_xlim(0, max(HISTORY, len(ax_data)))

    # ── Status panel ──────────────────────────────────────────────
    status_ax.cla()
    status_ax.set_axis_off()

    panel_color = COLORS["panel_fault"] if fault else COLORS["panel_ok"]
    status_label = "⚠  FAULT DETECTED" if fault else "✔  NORMAL OPERATION"

    fig.patch.set_facecolor(COLORS["bg"])
    status_ax.set_facecolor(panel_color)

    rect = plt.Rectangle((0, 0.75), 1, 0.25, transform=status_ax.transAxes,
                          color=panel_color, zorder=0)
    status_ax.add_patch(rect)

    status_ax.text(0.5, 0.87, status_label, ha="center", va="center",
                   fontsize=13, fontweight="bold", color="white",
                   transform=status_ax.transAxes)

    rows = [
        ("── CURRENT RMS ──", "", COLORS["warning"]),
        ("X-axis (Radial)",  f"{cur_ax:.4f} g", COLORS["ax_line"]),
        ("Y-axis (Tangential)", f"{cur_ay:.4f} g", COLORS["ay_line"]),
        ("Z-axis (Axial)",   f"{cur_az:.4f} g", COLORS["az_line"]),
        ("", "", COLORS["text"]),
        ("── PEAK RMS ──", "", COLORS["warning"]),
        ("X max", f"{max_ax:.4f} g", COLORS["ax_line"]),
        ("Y max", f"{max_ay:.4f} g", COLORS["ay_line"]),
        ("Z max", f"{max_az:.4f} g", COLORS["az_line"]),
        ("", "", COLORS["text"]),
        ("── SETTINGS ──", "", COLORS["warning"]),
        ("Threshold", f"{thr:.4f} g", COLORS["thr"]),
        ("Window", "256 samples", COLORS["text"]),
        ("Sample rate", "1 kHz", COLORS["text"]),
        ("", "", COLORS["text"]),
        ("── SESSION ──", "", COLORS["warning"]),
        ("Uptime",  f"{uptime}s", COLORS["text"]),
        ("Fault events", str(flt_cnt), COLORS["thr"] if flt_cnt > 0 else COLORS["text"]),
        ("Log file", "CSV ✔", COLORS["ay_line"]),
        ("MQTT", "Online ✔" if conn else "Offline ✗",
         COLORS["ay_line"] if conn else COLORS["thr"]),
    ]

    y_pos = 0.70
    for label, value, color in rows:
        if value == "":
            status_ax.text(0.5, y_pos, label, ha="center", fontsize=8,
                           color=color, transform=status_ax.transAxes,
                           fontweight="bold")
        else:
            status_ax.text(0.05, y_pos, label, ha="left", fontsize=8,
                           color=COLORS["text"], transform=status_ax.transAxes)
            status_ax.text(0.95, y_pos, value, ha="right", fontsize=8,
                           color=color, transform=status_ax.transAxes,
                           fontweight="bold")
        y_pos -= 0.038

    ts_now = datetime.datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
    status_ax.text(0.5, 0.01, ts_now, ha="center", fontsize=7,
                   color="#6a737d", transform=status_ax.transAxes)

    return line_ax, line_ay, line_az

# ── Run ───────────────────────────────────────────────────────────
print("=" * 60)
print("  IoT Predictive Maintenance Dashboard")
print("  Dept : Electrical & Electronic Engineering")
print("  Uni  : University of Peradeniya, Sri Lanka")
print("  Team : E22215 E22028 E22177 E22287 E22199")
print(f"  Log  : {log_filename}")
print("=" * 60)
print(f"[INFO] Connecting to MQTT broker at {BROKER}:{PORT}")
print("[INFO] Close the plot window to exit.\n")

ani = FuncAnimation(fig, update, interval=UPDATE_MS, blit=False, cache_frame_data=False)

try:
    plt.show()
finally:
    mq.loop_stop()
    mq.disconnect()
    log_file.close()
    print("\n[INFO] Dashboard closed. Log saved to:", log_filename)
