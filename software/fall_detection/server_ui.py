import socket
import threading
import time
from collections import deque
from flask import Flask, jsonify
from flask_cors import CORS

# -----------------------------
# Config
# -----------------------------
UDP_IP = "0.0.0.0"
UDP_PORT = 9000

WEB_HOST = "0.0.0.0"
WEB_PORT = 5050   # use 5050 to avoid macOS port 5000 conflicts

MAX_EVENTS = 50

# -----------------------------
# App state
# -----------------------------
app = Flask(__name__)
CORS(app)

state_lock = threading.Lock()

latest_status = {
    "device_id": "chest",
    "timestamp_us": None,
    "fall_state": "UNKNOWN",
    "chest_batt_mv": None,
    "chest_curr_mA": None,
    "wrist_batt_mv": None,
    "wrist_curr_mA": None,
    "wrist_fresh": None,
    "chest_a_mag": None,
    "chest_w_mag": None,
    "wrist_a_mag": None,
    "wrist_w_mag": None,
    "last_udp_time": None,
    "online": False,
}

recent_events = deque(maxlen=MAX_EVENTS)


# -----------------------------
# Helpers
# -----------------------------
def safe_int(x):
    try:
        return int(x)
    except Exception:
        return None


def safe_float(x):
    try:
        return float(x)
    except Exception:
        return None


def update_online_flag():
    with state_lock:
        last_udp = latest_status.get("last_udp_time")
        latest_status["online"] = last_udp is not None and (time.time() - last_udp) < 3.0


def parse_status(parts, addr):
    """
    Expected format:
    status,device_id,timestamp_us,fall_state,chest_batt_mv,chest_curr_mA,wrist_batt_mv,wrist_curr_mA,wrist_fresh,chest_a_mag,chest_w_mag,wrist_a_mag,wrist_w_mag
    """
    if len(parts) < 13:
        print(f"[UDP] Bad status packet from {addr}: {parts}")
        return

    parsed = {
        "device_id": parts[1],
        "timestamp_us": safe_int(parts[2]),
        "fall_state": parts[3],
        "chest_batt_mv": safe_int(parts[4]),
        "chest_curr_mA": safe_int(parts[5]),
        "wrist_batt_mv": safe_int(parts[6]),
        "wrist_curr_mA": safe_int(parts[7]),
        "wrist_fresh": bool(safe_int(parts[8])),
        "chest_a_mag": safe_float(parts[9]),
        "chest_w_mag": safe_float(parts[10]),
        "wrist_a_mag": safe_float(parts[11]),
        "wrist_w_mag": safe_float(parts[12]),
        "last_udp_time": time.time(),
        "online": True,
        "source_ip": addr[0],
        "source_port": addr[1],
    }

    with state_lock:
        latest_status.update(parsed)

    print(
        f"[STATUS] state={parsed['fall_state']} "
        f"chestBatt={parsed['chest_batt_mv']}mV "
        f"wristBatt={parsed['wrist_batt_mv']}mV"
    )


def parse_event(parts, addr):
    """
    Supports either:
    event,fall_confirmed,device_id,timestamp_us,chest_a_mag,chest_w_mag

    or:
    event,fall_confirmed,device_id,timestamp_us,fall_state,chest_a_mag,chest_w_mag,chest_batt_mv,chest_curr_mA,wrist_batt_mv,wrist_curr_mA
    """
    if len(parts) < 6:
        print(f"[UDP] Bad event packet from {addr}: {parts}")
        return

    event_type = parts[1]

    event = {
        "event_type": event_type,
        "device_id": parts[2] if len(parts) > 2 else None,
        "timestamp_us": safe_int(parts[3]) if len(parts) > 3 else None,
        "received_time": time.time(),
        "source_ip": addr[0],
        "source_port": addr[1],
    }

    # Old format
    if len(parts) == 6:
        event.update({
            "fall_state": None,
            "chest_a_mag": safe_float(parts[4]),
            "chest_w_mag": safe_float(parts[5]),
            "chest_batt_mv": None,
            "chest_curr_mA": None,
            "wrist_batt_mv": None,
            "wrist_curr_mA": None,
        })
    else:
        # New format
        event.update({
            "fall_state": parts[4] if len(parts) > 4 else None,
            "chest_a_mag": safe_float(parts[5]) if len(parts) > 5 else None,
            "chest_w_mag": safe_float(parts[6]) if len(parts) > 6 else None,
            "chest_batt_mv": safe_int(parts[7]) if len(parts) > 7 else None,
            "chest_curr_mA": safe_int(parts[8]) if len(parts) > 8 else None,
            "wrist_batt_mv": safe_int(parts[9]) if len(parts) > 9 else None,
            "wrist_curr_mA": safe_int(parts[10]) if len(parts) > 10 else None,
        })

    with state_lock:
        recent_events.appendleft(event)

    print(f"[EVENT] {event_type} from {event['device_id']} @ {event['timestamp_us']}")


def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"[UDP] Listening on {UDP_IP}:{UDP_PORT}")

    while True:
        try:
            data, addr = sock.recvfrom(2048)
            msg = data.decode("utf-8", errors="ignore").strip()

            if not msg:
                continue

            print(f"[UDP RX] {addr} -> {msg}")
            parts = [p.strip() for p in msg.split(",")]

            if not parts:
                continue

            packet_type = parts[0].lower()

            if packet_type == "status":
                parse_status(parts, addr)
            elif packet_type == "event":
                parse_event(parts, addr)
            else:
                print(f"[UDP] Unknown packet type from {addr}: {msg}")

        except Exception as e:
            print(f"[UDP] Error: {e}")


def monitor_online():
    while True:
        update_online_flag()
        time.sleep(0.5)


# -----------------------------
# API routes for React
# -----------------------------
@app.route("/api/health", methods=["GET"])
def api_health():
    update_online_flag()
    return jsonify({"ok": True, "udp_port": UDP_PORT, "web_port": WEB_PORT})


@app.route("/api/status", methods=["GET"])
def api_status():
    update_online_flag()
    with state_lock:
        return jsonify(dict(latest_status))


@app.route("/api/events", methods=["GET"])
def api_events():
    with state_lock:
        return jsonify(list(recent_events))


@app.route("/api/dashboard", methods=["GET"])
def api_dashboard():
    update_online_flag()
    with state_lock:
        return jsonify({
            "status": dict(latest_status),
            "events": list(recent_events),
        })


# -----------------------------
# Main
# -----------------------------
if __name__ == "__main__":
    threading.Thread(target=udp_listener, daemon=True).start()
    threading.Thread(target=monitor_online, daemon=True).start()

    print(f"[WEB] Open http://localhost:{WEB_PORT}")
    app.run(host=WEB_HOST, port=WEB_PORT, debug=False)