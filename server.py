import time
import random
import board
import busio
import digitalio
import wifi
import socketpool
import adafruit_requests
import adafruit_rfm9x
from adafruit_httpserver import Server, Request, Response
import pwmio
from adafruit_motor import servo
import supervisor

# Config
LORA_FREQ_MHZ = 915.0
SERVER_ID = "0681.7478"
NODE_ID   = "7454.1974"

# LoRa pins
PIN_RESET = board.GP15
PIN_CS    = board.GP17
PIN_MOSI  = board.GP19
PIN_MISO  = board.GP16
PIN_SCK   = board.GP18
PIN_G0    = board.GP26
PIN_EN    = board.GP27

# Wi-Fi AP
AP_SSID     = "PetSmartER"
AP_PASSWORD = "pets1234"

# Test mode
TEST_MODE = False
INTERVAL = 180

# Test startup delay
TEST_DELAY_MIN = 10
TEST_DELAY_MAX = 120

# Test GPS area
TEST_LAT_MIN = 40.01610
TEST_LAT_MAX = 40.01730
TEST_LON_MIN = -105.28110
TEST_LON_MAX = -105.27920

# Alert geofence
ALERT_LAT_MIN = 40.01650
ALERT_LAT_MAX = 40.01700
ALERT_LON_MIN = -105.28050
ALERT_LON_MAX = -105.28000

# Servo
SERVO_PIN   = board.GP0
ANGLE_SAFE  = 0
ANGLE_ALERT = 90

# Latest fix
_latest = {"lat": None, "lon": None, "rssi": None, "test": False}

# Start AP
print("Starting AP...")
wifi.radio.start_ap(AP_SSID, AP_PASSWORD)
print(f"AP: {AP_SSID} | http://192.168.4.1")

# LoRa init
print("Initializing LoRa...")
spi = busio.SPI(clock=PIN_SCK, MOSI=PIN_MOSI, MISO=PIN_MISO)
cs = digitalio.DigitalInOut(PIN_CS)
reset = digitalio.DigitalInOut(PIN_RESET)
en = digitalio.DigitalInOut(PIN_EN)
en.direction = digitalio.Direction.OUTPUT
en.value = True

rfm9x = adafruit_rfm9x.RFM9x(spi, cs, reset, LORA_FREQ_MHZ)
rfm9x.tx_power = 20
print("LoRa ready")

# Servo init
servo_motor = None
try:
    pwm = pwmio.PWMOut(SERVO_PIN, duty_cycle=2 ** 15, frequency=50)
    servo_motor = servo.Servo(pwm)
    servo_motor.angle = ANGLE_SAFE
    print(f"Servo at {ANGLE_SAFE} degrees")
except Exception as e:
    print("Servo failed:", e)

# Web server - HTML with escaped {{ and }} in CSS
HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
<title>LoRa GPS</title>
<meta http-equiv="refresh" content="15"> 
<style>
body {{
    background-color: #1e1e1e;
    color: #dcdcdc;
    font-family: Arial;
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    height: 100vh;
    margin: 0;
}}
h1 {{
    color: #0078d7;
    margin-bottom: 1rem;
}}
table {{
    border-collapse: collapse;
    min-width: 300px;
}}
th, td {{
    border: 1px solid #333;
    padding: 0.5rem 1rem;
    text-align: center;
}}
th {{
    background-color: #0078d7;
    color: #fff;
}}
td {{
    background-color: #2d2d2d;
}}
</style>
</head>
<body>
<h1>Latest GPS Fix</h1>
{}
</body>
</html>"""

pool = socketpool.SocketPool(wifi.radio)
requests = adafruit_requests.Session(pool, None)
server = Server(pool, "/")

@server.route("/")
def index(request: Request):
    if _latest["lat"] is None:
        body = "<p>No fix yet.</p>"
    else:
        body = f"""
        <table>
        <tr><th>Lat</th><th>Lon</th><th>RSSI</th></tr>
        <tr><td>{_latest['lat']:.6f}</td><td>{_latest['lon']:.6f}</td><td>{_latest['rssi']}</td></tr>
        </table>
        """
    return Response(request, HTML_PAGE.format(body), content_type="text/html")

server.start("0.0.0.0", 80)
print("Web server: http://192.168.4.1")

# Helpers
def generate_test():
    lat = random.uniform(TEST_LAT_MIN, TEST_LAT_MAX)
    lon = random.uniform(TEST_LON_MIN, TEST_LON_MAX)
    rssi = random.randint(-110, -35)
    return lat, lon, rssi

def in_geofence(lat, lon):
    return (ALERT_LAT_MIN <= lat <= ALERT_LAT_MAX and
            ALERT_LON_MIN <= lon <= ALERT_LON_MAX)

# Test timer
start_time = time.monotonic()
next_test = start_time + random.uniform(TEST_DELAY_MIN, TEST_DELAY_MAX) if TEST_MODE else float("inf")
print("Listening...")

# ===================================
# Main loop with Error Handling
# ===================================
try:
    while True:
        server.poll()

        now = time.monotonic()

        # Test packet
        if TEST_MODE and now >= next_test:
            lat, lon, rssi = generate_test()
            packet = f"NODE_1,{lat:.6f},{lon:.6f},{SERVER_ID}"
            print("Test RX:", packet)

            parts = packet.split(",")
            if len(parts) == 4 and parts[3] == SERVER_ID:
                _latest["lat"] = lat
                _latest["lon"] = lon
                _latest["rssi"] = rssi
                _latest["test"] = True
                print(f"Test update: {lat:.6f}, {lon:.6f} | RSSI: {rssi}")

                if servo_motor:
                    if in_geofence(lat, lon):
                        servo_motor.angle = ANGLE_ALERT
                        print("TEST: IN ZONE")
                    else:
                        servo_motor.angle = ANGLE_SAFE
                        print("TEST: OUT")

            next_test = now + INTERVAL

        # Real LoRa receive
        try:
            packet = rfm9x.receive(timeout=0.1)
        except Exception as e:
            print("Radio error:", e)
            packet = None

        if packet:
            try:
                text = packet.decode("utf-8").strip()
                print("RX:", text)
                parts = text.split(",")

                if len(parts) == 4 and parts[3] == SERVER_ID:
                    node_id, lat_s, lon_s, _ = parts
                    lat = float(lat_s)
                    lon = float(lon_s)

                    _latest["lat"] = lat
                    _latest["lon"] = lon
                    _latest["rssi"] = rfm9x.last_rssi
                    _latest["test"] = False
                    print(f"From {node_id}: {lat:.6f}, {lon:.6f} | RSSI: {rfm9x.last_rssi}")

                    # Geofence
                    if servo_motor:
                        if in_geofence(lat, lon):
                            servo_motor.angle = ANGLE_ALERT
                            print("IN ALERT AREA!")
                        else:
                            servo_motor.angle = ANGLE_SAFE
                            print("Outside area.")

                    # Send ACK
                    ack = f"ACK:{NODE_ID}:{SERVER_ID}"
                    rfm9x.send(bytes(ack, "utf-8"))
                    print("ACK sent")

            except Exception as e:
                print("Parse error:", e)

except Exception as error:
    # This block executes if an unhandled error (like BrokenPipeError) occurs
    print("\n\n!!! CRITICAL ERROR !!!")
    print(error)
    print("Restarting code.py in 10 seconds...")
    time.sleep(10)
    supervisor.reload()
