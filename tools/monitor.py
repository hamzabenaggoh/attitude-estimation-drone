#!/usr/bin/env python3
import sys
import time
import threading
import argparse
from collections import deque

import numpy as np
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, RadioButtons

parser = argparse.ArgumentParser(description="QuadFS serial monitor")
parser.add_argument("port", help="Serial port e.g. /dev/tty.usbmodem1234 or COM3")
parser.add_argument("--baud", type=int, default=115200)
args = parser.parse_args()

MAXLEN = 5000
roll_buf  = deque(maxlen=MAXLEN)
pitch_buf = deque(maxlen=MAXLEN)
p1_buf    = deque(maxlen=MAXLEN)
p2_buf    = deque(maxlen=MAXLEN)
p3_buf    = deque(maxlen=MAXLEN)
p4_buf    = deque(maxlen=MAXLEN)
time_buf  = deque(maxlen=MAXLEN)
lock = threading.Lock()

window_val  = 500
window_mode = "samples"
SMOOTH = 10

def smooth(data):
    if len(data) < SMOOTH:
        return np.array(data)
    return np.convolve(data, np.ones(SMOOTH) / SMOOTH, mode="valid")

def serial_reader():
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Cannot open port: {e}")
        sys.exit(1)

    print(f"Connected to {args.port}")
    t0 = time.time()

    while True:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except Exception:
            continue

        parts = line.split(",")
        if len(parts) != 6:
            continue
        try:
            r, p, m1, m2, m3, m4 = [float(x) for x in parts]
        except ValueError:
            continue

        with lock:
            time_buf.append(time.time() - t0)
            roll_buf.append(r)
            pitch_buf.append(p)
            p1_buf.append(m1)
            p2_buf.append(m2)
            p3_buf.append(m3)
            p4_buf.append(m4)

threading.Thread(target=serial_reader, daemon=True).start()

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
fig.suptitle("QuadFS Monitor", fontsize=13)
plt.subplots_adjust(left=0.07, right=0.97, top=0.92, bottom=0.22, wspace=0.3)

ax1.set_title("Attitude (deg)")
ax1.set_ylabel("degrees")
ax1.grid(True, alpha=0.3)

ax2.set_title("Motor PWM")
ax2.set_ylabel("PWM")
ax2.grid(True, alpha=0.3)

line_roll,  = ax1.plot([], [], label="roll",  color="tab:red",    linewidth=1)
line_pitch, = ax1.plot([], [], label="pitch", color="tab:blue",   linewidth=1)
line_p1,    = ax2.plot([], [], label="p1",    color="tab:blue",   linewidth=1)
line_p2,    = ax2.plot([], [], label="p2",    color="tab:orange", linewidth=1)
line_p3,    = ax2.plot([], [], label="p3",    color="tab:green",  linewidth=1)
line_p4,    = ax2.plot([], [], label="p4",    color="tab:red",    linewidth=1)

ax1.legend(loc="upper left")
ax2.legend(loc="upper left")

ax_text  = fig.add_axes([0.38, 0.07, 0.08, 0.06])
ax_radio = fig.add_axes([0.50, 0.03, 0.14, 0.12])

text_box = TextBox(ax_text, "Window: ", initial=str(window_val))
radio    = RadioButtons(ax_radio, ("Samples", "Seconds"), active=0)

def on_window_submit(val):
    global window_val
    try:
        window_val = max(1, float(val))
    except ValueError:
        pass

def on_radio(label):
    global window_mode
    window_mode = label.lower()

text_box.on_submit(on_window_submit)
radio.on_clicked(on_radio)

def update(_):
    with lock:
        if not time_buf:
            return
        t  = list(time_buf)
        r  = list(roll_buf)
        p  = list(pitch_buf)
        m1 = list(p1_buf)
        m2 = list(p2_buf)
        m3 = list(p3_buf)
        m4 = list(p4_buf)

    if window_mode == "samples":
        n  = int(window_val)
        t, r, p, m1, m2, m3, m4 = t[-n:], r[-n:], p[-n:], m1[-n:], m2[-n:], m3[-n:], m4[-n:]
    else:
        cutoff = t[-1] - window_val
        i = next((i for i, v in enumerate(t) if v >= cutoff), 0)
        t, r, p, m1, m2, m3, m4 = t[i:], r[i:], p[i:], m1[i:], m2[i:], m3[i:], m4[i:]

    r_s  = smooth(r)
    p_s  = smooth(p)
    m1_s = smooth(m1)
    m2_s = smooth(m2)
    m3_s = smooth(m3)
    m4_s = smooth(m4)
    t_s  = t[len(t) - len(r_s):]

    line_roll.set_data(t_s, r_s)
    line_pitch.set_data(t_s, p_s)
    line_p1.set_data(t_s, m1_s)
    line_p2.set_data(t_s, m2_s)
    line_p3.set_data(t_s, m3_s)
    line_p4.set_data(t_s, m4_s)

    ax1.relim(); ax1.autoscale_view()
    ax2.relim(); ax2.autoscale_view()

ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.show()
