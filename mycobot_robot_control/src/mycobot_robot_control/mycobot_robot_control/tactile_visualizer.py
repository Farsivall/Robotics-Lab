#!/usr/bin/env python3
"""Live visualizer for Arduino tactile sensor CSV output.

Expected serial format at 115200 baud:
    v0,v1,v2,...,v11
one sample per line.
"""
import argparse
import os
import select
import termios
import time
from collections import deque

import matplotlib.pyplot as plt
import numpy as np


BAUDS = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
}


def configure_serial(fd, baud):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = BAUDS[baud]
    attrs[5] = BAUDS[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def parse_line(line, channels):
    try:
        vals = [float(x) for x in line.strip().split(",") if x.strip() != ""]
    except ValueError:
        return None
    if len(vals) != channels:
        return None
    return np.array(vals, dtype=float)


def read_sample(fd, buf, channels):
    while True:
        ready, _, _ = select.select([fd], [], [], 0.02)
        if not ready:
            return None, buf
        chunk = os.read(fd, 4096)
        if not chunk:
            return None, buf
        buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("ascii", "ignore")
            sample = parse_line(line, channels)
            if sample is not None:
                return sample, buf


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, choices=sorted(BAUDS))
    parser.add_argument("--channels", type=int, default=12)
    parser.add_argument("--baseline-samples", type=int, default=40)
    parser.add_argument("--raw", action="store_true", help="show raw values instead of baseline-subtracted force")
    args = parser.parse_args()

    fd = os.open(args.port, os.O_RDONLY | os.O_NONBLOCK)
    configure_serial(fd, args.baud)

    print(f"Reading {args.channels} channels from {args.port} at {args.baud} baud")
    print("Keep the tactile sensor untouched for baseline calibration...")

    buf = b""
    baseline_samples = []
    deadline = time.time() + 5.0
    while len(baseline_samples) < args.baseline_samples and time.time() < deadline:
        sample, buf = read_sample(fd, buf, args.channels)
        if sample is not None:
            baseline_samples.append(sample)

    baseline = np.mean(baseline_samples, axis=0) if baseline_samples else np.zeros(args.channels)
    print("Baseline:", ",".join(f"{v:.1f}" for v in baseline))

    labels = [f"T{i + 1}" for i in range(args.channels)]
    values = np.zeros(args.channels)
    history = deque(maxlen=120)

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 5))
    bars = ax.bar(labels, values)
    ax.set_title("Tactile Sensor Live Contact Force")
    ax.set_ylabel("Raw value" if args.raw else "Contact force = value - baseline")
    ax.set_ylim(0, 100)
    ax.grid(axis="y", alpha=0.3)
    text = ax.text(0.01, 0.95, "", transform=ax.transAxes, va="top")

    try:
        while plt.fignum_exists(fig.number):
            sample, buf = read_sample(fd, buf, args.channels)
            if sample is None:
                plt.pause(0.01)
                continue

            values = sample if args.raw else np.maximum(sample - baseline, 0)
            history.append(values.copy())
            peak = max(20.0, float(np.max(history)) * 1.25 if history else 100.0)
            ax.set_ylim(0, peak)

            for bar, value in zip(bars, values):
                bar.set_height(float(value))
                bar.set_color("#d43f3a" if value > peak * 0.65 else "#3274a1")

            text.set_text(
                "latest: " + ",".join(f"{v:.0f}" for v in values)
                + f"\nmax: {np.max(values):.0f}"
            )
            fig.canvas.draw_idle()
            plt.pause(0.02)
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
