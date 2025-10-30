import os, threading, time, re
from datetime import datetime

# Adafruit CircuitPython hardware
import board, busio, digitalio, adafruit_rfm9x

# Flask for the simple UI
from flask import Flask, jsonify, render_template_string

_latest_lock = threading.Lock()
_latest = {}

# IDs
SERVER_ID = "0681.7478"
WHITELIST = {"7454.1974"}  # allowed node ids

# RFM95 configuration
LORA_FREQ_MHZ = 915.0
_nodeid_re = re.compile(r'^\d{4}\.\d{4}$')

def init_radio():
    # For FT232H, ensure BLINKA_FT232H=1 in the environment (see systemd unit below)
    spi = busio.SPI(board.D3, board.D1, board.D2)
    cs = digitalio.DigitalInOut(board.D0)
    reset = digitalio.DigitalInOut(board.D4)

    while not spi.try_lock():
        time.sleep(0.01)
    spi.unlock()

    rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, LORA_FREQ_MHZ)
    print("radio init ok")
    return rfm9x

def radio_thread(rfm9x):
    print("radio thread listening...")
    while True:
        try:
            packet = rfm9x.receive(timeout=5.0)
            if packet is None:
                continue
            try:
                text = str(packet, "utf-8").strip()
            except Exception:
                continue

            # expected: node_id,lat,lon,server_id
            parts = text.split(",")
            if len(parts) != 4:
                continue
            node_id, lat_s, lon_s, target_id = parts

            if node_id not in WHITELIST:
                continue
            if target_id != SERVER_ID:
                continue
            if not _nodeid_re.match(node_id):
                continue

            try:
                lat = float(lat_s)
                lon = float(lon_s)
            except ValueError:
                continue

            with _latest_lock:
                _latest[node_id] = {
                    "lat": lat,
                    "lon": lon,
                    "time": datetime.utcnow().isoformat() + "Z",
                    "rssi": getattr(rfm9x, "last_rssi", None),
                }
            print(f"{node_id}: {lat},{lon}")

            ack = f"ACK:{node_id}:{SERVER_ID}"
            try:
                rfm9x.send(bytes(ack, "utf-8"))
            except Exception:
                pass

        except Exception as e:
            print("radio err", e)
            time.sleep(1)

# Flask
app = Flask(__name__)

PAGE = """<!doctype html>
<html>
  <head>
    <meta charset="utf-8">
    <title>LoRa GPS Server</title>
    <style>
      body { font-family: Arial, sans-serif; margin: 2rem; }
      .card { padding:1rem; border-radius:6px; box-shadow:0 0 8px rgba(0,0,0,0.08); max-width:720px; }
      table { border-collapse: collapse; }
      td, th { border: 1px solid #ccc; padding: .4rem .8rem; }
    </style>
  </head>
  <body>
    <div class="card">
      <h1>Latest GPS Fixes</h1>
      {% if not data %}
        <p>No fix received yet.</p>
      {% else %}
        <table>
          <tr><th>Node ID</th><th>Latitude</th><th>Longitude</th><th>Received (UTC)</th><th>RSSI</th></tr>
          {% for k, v in data.items() %}
          <tr><td>{{ k }}</td><td>{{ v.lat }}</td><td>{{ v.lon }}</td><td>{{ v.time }}</td><td>{{ v.rssi }}</td></tr>
          {% endfor %}
        </table>
      {% endif %}
    </div>
  </body>
</html>"""

@app.route("/")
def index():
    with _latest_lock:
        data = _latest.copy()
    return render_template_string(PAGE, data=data)

@app.route("/api/latest")
def api_latest():
    with _latest_lock:
        return jsonify(_latest)

def main():
    if os.environ.get("BLINKA_FT232H") != "1":
        print("WARNING: BLINKA_FT232H not set")

    rfm9x = init_radio()
    t = threading.Thread(target=radio_thread, args=(rfm9x,), daemon=True)
    t.start()
    app.run(host="127.0.0.1", port=5000)

if __name__ == "__main__":
    main()
