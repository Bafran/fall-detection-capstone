import socket
import threading
import time
from flask import Flask, jsonify, Response

HOST = "0.0.0.0"
UDP_PORT = 9000
WEB_PORT = 5000

app = Flask(__name__)

state = {
    "last_event": "none",
    "last_device": None,
    "last_time_us": None,
    "a_mag": None,
    "w_mag": None,
    "updated_at": None,
}

def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, UDP_PORT))
    print(f"[UDP] Listening on {HOST}:{UDP_PORT}")

    while True:
        data, addr = sock.recvfrom(2048)
        msg = data.decode(errors="ignore").strip()
        parts = msg.split(",")

        # Expect: event,fall_confirmed,DEVICE_ID,t_us,a_mag,w_mag
        if len(parts) >= 6 and parts[0] == "event":
            evt = parts[1]
            dev = parts[2]
            t_us = parts[3]
            a_mag = parts[4]
            w_mag = parts[5]

            state["last_event"] = evt
            state["last_device"] = dev
            state["last_time_us"] = int(t_us)
            state["a_mag"] = float(a_mag)
            state["w_mag"] = float(w_mag)
            state["updated_at"] = time.time()

            print(f"[EVENT] {evt} from {dev} a={a_mag} w={w_mag}")

@app.get("/api/state")
def api_state():
    return jsonify(state)

@app.get("/")
def index():
    # Super minimal HTML (polls /api/state every 500ms)
    html = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>Fall Detection Demo</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 30px; }
    .card { border: 1px solid #ddd; border-radius: 12px; padding: 18px; max-width: 520px; }
    .status { font-size: 28px; font-weight: 700; margin-bottom: 10px; }
    .ok { color: #1b7f3a; }
    .bad { color: #b00020; }
    .muted { color: #666; }
    .row { margin: 6px 0; }
  </style>
</head>
<body>
  <div class="card">
    <div id="status" class="status muted">Waiting for data...</div>
    <div class="row"><b>Device:</b> <span id="dev">-</span></div>
    <div class="row"><b>a_mag:</b> <span id="a">-</span></div>
    <div class="row"><b>w_mag:</b> <span id="w">-</span></div>
    <div class="row muted"><b>Last update:</b> <span id="t">-</span></div>
  </div>

<script>
async function tick() {
  const res = await fetch('/api/state');
  const s = await res.json();

  const statusEl = document.getElementById('status');
  const devEl = document.getElementById('dev');
  const aEl = document.getElementById('a');
  const wEl = document.getElementById('w');
  const tEl = document.getElementById('t');

  devEl.textContent = s.last_device ?? '-';
  aEl.textContent = (s.a_mag ?? '-');
  wEl.textContent = (s.w_mag ?? '-');

  if (s.last_event === 'fall_confirmed') {
    statusEl.textContent = 'FALL DETECTED';
    statusEl.className = 'status bad';
  } else {
    statusEl.textContent = 'No fall detected';
    statusEl.className = 'status ok';
  }

  if (s.updated_at) {
    const dt = new Date(s.updated_at * 1000);
    tEl.textContent = dt.toLocaleString();
  } else {
    tEl.textContent = '-';
  }
}
setInterval(tick, 500);
tick();
</script>
</body>
</html>
"""
    return Response(html, mimetype="text/html")

if __name__ == "__main__":
    threading.Thread(target=udp_listener, daemon=True).start()
    print(f"[WEB] Open http://localhost:{WEB_PORT}")
    app.run(host="0.0.0.0", port=WEB_PORT, debug=False)