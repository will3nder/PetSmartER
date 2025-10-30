import os
import threading
import time
from datetime import datetime

# Adafruit CircuitPython hardware
import board
import busio
import digitalio
import adafruit_rfm9x

# Flask for the simple UI
from flask import Flask, jsonify, render_template_string

# Shared latest position storage
_latest_lock = threading.Lock()
_latest = {"lat": None, "lon": None, "time": None, "raw": None}

# RFM95 configuration
LORA_FREQ_MHZ = 915.0  # Make sure this matches your hardware region

def init_radio():
    # For FT232H, ensure BLINKA_FT232H=1 in the environment (see systemd unit below)
    # SPI pins are exposed by Blinka as board.SCLK, board.MOSI, board.MISO
    spi = busio.SPI(board.D3, board.D1, board.D2)
    cs = digitalio.DigitalInOut(board.D0)

    # Wait for SPI to be ready
    while not spi.try_lock():
        time.sleep(0.01)
    spi.unlock()

    rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, LORA_FREQ_MHZ)
    # optional: rfm9x.tx_power = 23
    print("Radio initialized:", rfm9x)
    return rfm9x

def radio_thread(rfm9x):
    """
    Background thread that listens for LoRa packets, parses them and updates the latest position.
    When a packet is parsed successfully, send a short ACK.
    """
    print("Radio thread starting, listening for packets...")
    while True:
        try:
            # receive returns bytes or None. timeout in seconds (float) is supported by CircuitPython.
            packet = rfm9x.receive(timeout=5.0)  # wait up to 5s for a packet
            if packet is None:
                # no packet this cycle
                continue
            try:
                text = str(packet, "utf-8").strip()
            except Exception:
                # if decoding fails, ignore
                continue

            # Expect payload: "lat,lon" (e.g., "47.123456,-122.123456")
            parts = text.split(",")
            if len(parts) >= 2:
                try:
                    lat = float(parts[0])
                    lon = float(parts[1])
                    with _latest_lock:
                        _latest["lat"] = lat
                        _latest["lon"] = lon
                        _latest["time"] = datetime.utcnow().isoformat() + "Z"
                        _latest["raw"] = text
                    print(f"Got position: {lat},{lon} -- sending ACK")
                    # send an ACK back
                    rfm9x.send(bytes("ACK", "utf-8"))
                except ValueError:
                    print("Malformed numeric payload:", text)
                    # optionally send a NACK or ignore
                    continue
            else:
                print("Unexpected payload format:", text)
                continue
        except Exception as e:
            # Avoid crashing thread on transient errors
            print("Radio thread error:", e)
            time.sleep(1)

# Flask app
app = Flask(__name__)

PAGE = """<!doctype html>
<html>
  <head>
    <meta charset="utf-8">
    <title>LoRa GPS Server</title>
    <style>
      body { font-family: Arial, sans-serif; margin: 2rem; }
      .card { padding:1rem; border-radius:6px; box-shadow:0 0 8px rgba(0,0,0,0.08); max-width:720px; }
    </style>
  </head>
  <body>
    <div class="card">
      <h1>Latest GPS Fix</h1>
      {% if lat is none %}
        <p>No fix received yet.</p>
      {% else %}
        <p><strong>Latitude:</strong> {{ lat }}</p>
        <p><strong>Longitude:</strong> {{ lon }}</p>
        <p><strong>Received (UTC):</strong> {{ time }}</p>
        <p><strong>Raw payload:</strong> {{ raw }}</p>
        <p><a href="https://www.openstreetmap.org/?mlat={{ lat }}&mlon={{ lon }}&zoom=15" target="_blank">Open in OSM</a></p>
      {% endif %}
    </div>
  </body>
</html>"""

@app.route("/")
def index():
    with _latest_lock:
        lat = _latest["lat"]
        lon = _latest["lon"]
        time_s = _latest["time"]
        raw = _latest["raw"]
    return render_template_string(PAGE, lat=lat, lon=lon, time=time_s, raw=raw)

@app.route("/api/latest")
def api_latest():
    with _latest_lock:
        return jsonify(_latest)

def main():
    # require BLINKA_FT232H to be set (systemd unit sets it)
    if os.environ.get("BLINKA_FT232H") != "1":
        print("WARNING: BLINKA_FT232H is not set. Set BLINKA_FT232H=1 before running.")
        # we'll continue but radio init may fail

    rfm9x = init_radio()

    t = threading.Thread(target=radio_thread, args=(rfm9x,), daemon=True)
    t.start()

    # Run Flask (development server). For production you may want gunicorn/reverse proxy,
    # but for local testing this is fine.
    app.run(host="127.0.0.1", port=5000)

if __name__ == "__main__":
    main()
