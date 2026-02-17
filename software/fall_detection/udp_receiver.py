import socket
import time
import threading
import numpy as np
from collections import deque, defaultdict
from dataclasses import dataclass
from typing import Optional, Deque, Dict, Tuple

import matplotlib.pyplot as plt

# =======================
# Configuration
# =======================
HOST = "0.0.0.0"
PORT = 9000

WINDOW_SECONDS = 20          # rolling window for clock fit
FIT_INTERVAL_SEC = 1.0       # update (a, b) once per second

FUSE_HZ = 100
FUSE_PERIOD_US = int(1_000_000 / FUSE_HZ)

# Process slightly in the past so both devices have arrived
LATENCY_BUFFER_US = 200_000  # 200 ms

# Keep this many seconds of aligned raw samples per device (for interpolation)
SAMPLE_KEEP_SECONDS = 10
SAMPLE_KEEP_US = SAMPLE_KEEP_SECONDS * 1_000_000

# Plot window (seconds)
PLOT_WINDOW_S = 10.0
PLOT_WINDOW_US = int(PLOT_WINDOW_S * 1_000_000)

DEVICES = ("chest", "wrist")

# =======================
# Data structures
# =======================
@dataclass
class ImuSample:
    t_aligned_us: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    temp: float

# =======================
# State
# =======================
time_pairs: Dict[str, Deque[Tuple[int, int]]] = defaultdict(lambda: deque())
clock_model: Dict[str, Tuple[float, float]] = {}

samples: Dict[str, Deque[ImuSample]] = defaultdict(lambda: deque())

packet_counts = defaultdict(int)
last_rate_print = time.time()

# Fused feature history for plotting (rolling window)
# each: (t_us, value)
fused_hist = {
    "chest_a": deque(),
    "wrist_a": deque(),
    "chest_w": deque(),
    "wrist_w": deque(),
}

# =======================
# UDP Socket
# =======================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))
print(f"Listening on UDP {HOST}:{PORT} ...")

# =======================
# Helper functions
# =======================
def align_time(device_id: str, t_device_us: int) -> Optional[int]:
    if device_id not in clock_model:
        return None
    a, b = clock_model[device_id]
    return int(a * t_device_us + b)

def mag3(x, y, z) -> float:
    return float(np.sqrt(x*x + y*y + z*z))

def interp_sample(buf: Deque[ImuSample], t_us: int) -> Optional[ImuSample]:
    """Linear interpolation of IMU values at timestamp t_us."""
    if len(buf) < 2:
        return None

    # Drop samples until buf[0], buf[1] bracket t_us (or close)
    while len(buf) >= 2 and buf[1].t_aligned_us < t_us:
        buf.popleft()

    if len(buf) < 2:
        return None

    s0, s1 = buf[0], buf[1]
    t0, t1 = s0.t_aligned_us, s1.t_aligned_us

    if not (t0 <= t_us <= t1):
        return None
    if t1 == t0:
        return s0

    alpha = (t_us - t0) / (t1 - t0)

    def lerp(v0, v1):
        return v0 + alpha * (v1 - v0)

    return ImuSample(
        t_aligned_us=t_us,
        ax=lerp(s0.ax, s1.ax),
        ay=lerp(s0.ay, s1.ay),
        az=lerp(s0.az, s1.az),
        gx=lerp(s0.gx, s1.gx),
        gy=lerp(s0.gy, s1.gy),
        gz=lerp(s0.gz, s1.gz),
        temp=lerp(s0.temp, s1.temp),
    )

def trim_deque_time(dq: Deque[Tuple[int, float]], cutoff_us: int):
    while dq and dq[0][0] < cutoff_us:
        dq.popleft()

# =======================
# UDP Receiver Thread
# =======================
def udp_receiver():
    global last_rate_print

    while True:
        data, addr = sock.recvfrom(2048)
        t_server_us = time.time_ns() // 1000

        line = data.decode(errors="ignore").strip()
        parts = line.split(",")

        # Expected:
        # device_id,t_device_us,ax,ay,az,gx,gy,gz,temp
        if len(parts) < 9:
            continue

        device_id = parts[0].strip()
        if device_id not in DEVICES:
            continue

        try:
            t_device_us = int(parts[1])
            ax, ay, az = float(parts[2]), float(parts[3]), float(parts[4])
            gx, gy, gz = float(parts[5]), float(parts[6]), float(parts[7])
            temp = float(parts[8])
        except ValueError:
            continue

        # Store pair for sync fit
        buf = time_pairs[device_id]
        buf.append((t_device_us, t_server_us))

        cutoff_pairs = t_server_us - WINDOW_SECONDS * 1_000_000
        while buf and buf[0][1] < cutoff_pairs:
            buf.popleft()

        packet_counts[device_id] += 1

        # If model is ready, store aligned sample for interpolation
        t_aligned = align_time(device_id, t_device_us)
        if t_aligned is not None:
            sbuf = samples[device_id]
            sbuf.append(ImuSample(t_aligned, ax, ay, az, gx, gy, gz, temp))

            cutoff_samples = t_aligned - SAMPLE_KEEP_US
            while sbuf and sbuf[0].t_aligned_us < cutoff_samples:
                sbuf.popleft()

        # Print packet rates once per second
        now = time.time()
        if now - last_rate_print >= 1.0:
            last_rate_print = now
            for dev in DEVICES:
                cnt = packet_counts[dev]
                print(f"[RATE] {dev:5s}: {cnt:4d} Hz | model={'OK' if dev in clock_model else '...'}")
                packet_counts[dev] = 0

# =======================
# Clock Fitting Thread
# =======================
def clock_fitter():
    while True:
        time.sleep(FIT_INTERVAL_SEC)

        for device_id, buf in list(time_pairs.items()):
            if len(buf) < 80:
                continue

            td = np.array([x[0] for x in buf], dtype=np.float64)
            ts = np.array([x[1] for x in buf], dtype=np.float64)

            # Least squares fit: ts ≈ a * td + b
            A = np.vstack([td, np.ones_like(td)]).T
            a, b = np.linalg.lstsq(A, ts, rcond=None)[0]
            clock_model[device_id] = (a, b)

            residuals = ts - (a * td + b)
            rms_err = float(np.sqrt(np.mean(residuals ** 2)))
            print(f"[SYNC] {device_id:5s} a={a:.9f} b={b:.0f}us rms={rms_err:.0f}us")

# =======================
# Fusion Thread (creates “meaningful” aligned features)
# =======================
def fusion_loop():
    while True:
        now_us = time.time_ns() // 1000
        t_fuse_us = now_us - LATENCY_BUFFER_US

        if not all(dev in clock_model for dev in DEVICES):
            time.sleep(0.05)
            continue

        chest = interp_sample(samples["chest"], t_fuse_us)
        wrist = interp_sample(samples["wrist"], t_fuse_us)

        if chest is not None and wrist is not None:
            chest_a = mag3(chest.ax, chest.ay, chest.az)
            wrist_a = mag3(wrist.ax, wrist.ay, wrist.az)
            chest_w = mag3(chest.gx, chest.gy, chest.gz)
            wrist_w = mag3(wrist.gx, wrist.gy, wrist.gz)

            fused_hist["chest_a"].append((t_fuse_us, chest_a))
            fused_hist["wrist_a"].append((t_fuse_us, wrist_a))
            fused_hist["chest_w"].append((t_fuse_us, chest_w))
            fused_hist["wrist_w"].append((t_fuse_us, wrist_w))

            cutoff = t_fuse_us - PLOT_WINDOW_US
            for k in fused_hist:
                trim_deque_time(fused_hist[k], cutoff)

        time.sleep(FUSE_PERIOD_US / 1_000_000.0)

# =======================
# Plotting (runs in main thread)
# =======================
def run_plot():
    plt.ion()

    fig1, ax1 = plt.subplots()
    line_c_a, = ax1.plot([], [], label="chest |a| (m/s^2)")
    line_w_a, = ax1.plot([], [], label="wrist |a| (m/s^2)")
    ax1.set_title("Acceleration Magnitude (aligned)")
    ax1.set_xlabel("Time (s, relative)")
    ax1.set_ylabel("|a|")
    ax1.legend(loc="upper right")

    fig2, ax2 = plt.subplots()
    line_c_w, = ax2.plot([], [], label="chest |ω|")
    line_w_w, = ax2.plot([], [], label="wrist |ω|")
    ax2.set_title("Gyro Magnitude (aligned)")
    ax2.set_xlabel("Time (s, relative)")
    ax2.set_ylabel("|ω|")
    ax2.legend(loc="upper right")

    while True:
        # Use the newest timestamp we have (from any stream)
        newest = None
        for k in fused_hist:
            if fused_hist[k]:
                t_last = fused_hist[k][-1][0]
                newest = t_last if newest is None else max(newest, t_last)

        if newest is None:
            plt.pause(0.05)
            continue

        # Convert to "seconds relative to newest"
        def to_xy(dq):
            if not dq:
                return [], []
            t0 = newest
            xs = [ (t - t0) / 1_000_000.0 for (t, _) in dq ]  # negative values (past)
            ys = [ v for (_, v) in dq ]
            return xs, ys

        x_c_a, y_c_a = to_xy(fused_hist["chest_a"])
        x_w_a, y_w_a = to_xy(fused_hist["wrist_a"])
        x_c_w, y_c_w = to_xy(fused_hist["chest_w"])
        x_w_w, y_w_w = to_xy(fused_hist["wrist_w"])

        line_c_a.set_data(x_c_a, y_c_a)
        line_w_a.set_data(x_w_a, y_w_a)
        line_c_w.set_data(x_c_w, y_c_w)
        line_w_w.set_data(x_w_w, y_w_w)

        # Keep x-axis fixed to last PLOT_WINDOW_S seconds
        ax1.set_xlim(-PLOT_WINDOW_S, 0)
        ax2.set_xlim(-PLOT_WINDOW_S, 0)

        # Auto-scale y (simple)
        if y_c_a or y_w_a:
            y_all = y_c_a + y_w_a
            ax1.set_ylim(min(y_all) - 1.0, max(y_all) + 1.0)
        if y_c_w or y_w_w:
            y_all = y_c_w + y_w_w
            ax2.set_ylim(min(y_all) - 0.5, max(y_all) + 0.5)

        fig1.canvas.draw()
        fig2.canvas.draw()
        plt.pause(0.05)  # ~20 FPS UI update

# =======================
# Main
# =======================
if __name__ == "__main__":
    threading.Thread(target=udp_receiver, daemon=True).start()
    threading.Thread(target=clock_fitter, daemon=True).start()
    threading.Thread(target=fusion_loop, daemon=True).start()

    print("Backend running + plotting. Close plot windows or Ctrl+C to stop.")
    run_plot()
