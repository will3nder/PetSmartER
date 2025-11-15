#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// ===================================
// Configuration
// ===================================
#define LORA_FREQ_MHZ 915.0
#define SERVER_ID "0681.7478"
#define NODE_ID "7454.1974"

#define PIN_LORA_RESET 15
#define PIN_LORA_CS 17
#define PIN_LORA_MOSI 19
#define PIN_LORA_MISO 16
#define PIN_LORA_SCK 18
#define PIN_LORA_DIO0 26
#define PIN_LORA_EN 27

#define AP_SSID "PetSmartER"
#define AP_PASSWORD "pets1234"

#define TEST_MODE true // Set false for real use
#define INTERVAL 180 // seconds (only used in test mode)

#define TEST_LAT_MIN 40.01610
#define TEST_LAT_MAX 40.01730
#define TEST_LON_MIN -105.28110
#define TEST_LON_MAX -105.27920

#define ALERT_LAT_MIN 40.01650
#define ALERT_LAT_MAX 40.01700
#define ALERT_LON_MIN -105.28050
#define ALERT_LON_MAX -105.28000

#define SERVO_PIN 5
#define ANGLE_UNLOCKED -30
#define ANGLE_LOCKED 90

#define R_LED 13
#define G_LED 12
#define B_LED 11

#define PIN_TFLM_RX 9
#define PIN_TFLM_TX 8

// ===================================
// Global Objects
// ===================================
WebServer server(80);
Servo servoMotor;
SoftwareSerial TFLMSerial(9,8);

struct Fix {
  float lat = 0.0;
  float lon = 0.0;
  int rssi = 0;
  bool test = false;
  bool valid = false;
  unsigned long timestamp = 0;
} latest;

unsigned long nextTest = 0;
bool testMode = TEST_MODE;
uint8_t currentAngle = ANGLE_UNLOCKED;

// ===================================
// WEB UI
// ===================================
const char* HTML_PAGE = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PetSmartER</title>
<meta http-equiv="refresh" content="10">
<style>
  body {font-family:system-ui,sans-serif;background:#121212;color:#eee;margin:0;padding:20px;text-align:center;}
  h1 {color:#00bfff;margin-bottom:0.2em;}
  .card {background:#1e1e1e;border-radius:16px;padding:24px;margin:20px auto;max-width:420px;box-shadow:0 8px 32px rgba(0,0,0,0.8);}
  table {width:100%;border-collapse:collapse;margin:20px 0;}
  th, td {padding:12px;border:1px solid #333;text-align:center;}
  th {background:#00bfff;color:#000;font-weight:bold;}
  td {background:#2d2d2d;}
  .btn {padding:18px;margin:12px;font-size:1.5em;border:none;border-radius:12px;cursor:pointer;width:85%;font-weight:bold;}
  .unlock {background:#28a745;color:white;}
  .lock {background:#dc3545;color:white;}
  .status {font-size:1.3em;margin:20px 0;font-weight:bold;}
  .ok {color:#28a745;}
  .out {color:#dc3545;}
  .small {font-size:0.9em;color:#aaa;margin-top:10px;}
</style>
</head>
<body>
<h1>PetSmartER</h1>
<div class="card">
  %s
</div>

<form action="/unlock" method="post">
  <button class="btn unlock" type="submit">UNLOCK</button>
</form>
<form action="/lock" method="post">
  <button class="btn lock" type="submit">LOCK</button>
</form>

<div class="small">Server ID: %s • Updated: %s</div>
</body>
</html>
)=====";

// ===================================
// Helper Functions
// ===================================

// === MODIFIED: This function now reports the actual servo state (currentAngle) ===
// === instead of recalculating based on geofence.                 ===
String generateHTMLContent() {
  // Always get the current state from the global variable
  String state = (currentAngle == ANGLE_UNLOCKED) ? "UNLOCKED" : "LOCKED";
  String color = (currentAngle == ANGLE_UNLOCKED) ? "ok" : "out";
  
  if (!latest.valid) {
    // Show current lock state even if no GPS fix
    return String("<p>No position received yet</p>") + "<div class='status " + color + "'>" + state + "</div>";
  }

  // We have a valid fix, show all info
  String source = latest.test ? "TEST MODE" : "REAL DEVICE";

  char table[512];
  snprintf(table, sizeof(table),
           "<table>"
           "<tr><th>Lat</th><th>Lon</th><th>RSSI</th></tr>"
           "<tr><td>%.6f</td><td>%.6f</td><td>%d dBm</td></tr>"
           "</table>"
           "<div class='status %s'>%s</div>" // These now use the state/color from currentAngle
           "<div class='small'>Source: %s • %.1f seconds ago</div>",
           latest.lat, latest.lon, latest.rssi,
           color.c_str(), state.c_str(), source.c_str(),
           (millis() - latest.timestamp) / 1000.0);
  return String(table);
}

bool inGeofence(float lat, float lon) {
  return (ALERT_LAT_MIN <= lat && lat <= ALERT_LAT_MAX &&
          ALERT_LON_MIN <= lon && lon <= ALERT_LON_MAX);
}

void unlockDoor() {
  currentAngle = ANGLE_UNLOCKED; // Update the global state
  servoMotor.write(currentAngle);
  Serial.println("DOOR UNLOCKED");
  digitalWrite(R_LED, LOW);
  digitalWrite(G_LED, HIGH);
}

void lockDoor() {
  currentAngle = ANGLE_LOCKED; // Update the global state
  servoMotor.write(currentAngle);
  Serial.println("DOOR LOCKED");
  digitalWrite(R_LED, HIGH);
  digitalWrite(G_LED, LOW);
}

void handleRoot() {
  String content = generateHTMLContent(); // This will now reflect currentAngle
  char page[2048];
  snprintf(page, sizeof(page), HTML_PAGE, content.c_str(), SERVER_ID,
           testMode ? "TEST MODE" : "LIVE");
  server.send(200, "text/html", page);
}

void handleUnlock() {
  unlockDoor(); // This updates currentAngle
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleLock() {
  lockDoor(); // This updates currentAngle
  server.sendHeader("Location", "/");
  server.send(303);
}

// New WiFi node endpoint
void handleWiFiData() {
  String body = server.arg("plain");
  Serial.println("WiFi RX: " + body);

  int c1 = body.indexOf(',');
  int c2 = body.indexOf(',', c1 + 1);
  int c3 = body.indexOf(',', c2 + 1);

  if (c1 > 0 && c2 > c1 && c3 > c2) {
    String node = body.substring(0, c1);
    String latStr = body.substring(c1 + 1, c2);
    String lonStr = body.substring(c2 + 1, c3);
    String srvId = body.substring(c3 + 1);

    if (srvId == SERVER_ID) {
      latest.lat = latStr.toFloat();
      latest.lon = lonStr.toFloat();
      latest.rssi = -45; // fake good RSSI
      latest.test = false;
      latest.valid = true;
      latest.timestamp = millis();

      Serial.printf("WiFi Node %s → %.6f, %.6f\n", node.c_str(), latest.lat, latest.lon);

      if (inGeofence(latest.lat, latest.lon)) unlockDoor(); // This updates currentAngle
      else lockDoor(); // This updates currentAngle

      server.send(200, "text/plain", "ACK:" + String(NODE_ID));
      return;
    }
  }
  server.send(400, "text/plain", "Bad format");
}

// ===================================
// Setup & Loop
// ===================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(1000);
  Serial.println("\n=== PetSmartER Server Starting ===");

  TFLMSerial.begin(9600);
  Serial.printf("TFLM UART listener started on RX=%d, TX=%d\n", PIN_TFLM_RX, PIN_TFLM_TX);

  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);
  digitalWrite(B_LED, HIGH);

  // Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("AP Started → %s | http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  pinMode(PIN_LORA_EN, OUTPUT);
  digitalWrite(PIN_LORA_EN, HIGH);
  SPI.begin();
  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RESET, PIN_LORA_DIO0);

  if (LoRa.begin(LORA_FREQ_MHZ * 1E6)) {
    LoRa.setTxPower(20);
    Serial.println("LoRa initialized");
  } else {
    Serial.println("LoRa NOT found (OK if using WiFi nodes only)");
  }

  // Servo
  if (servoMotor.attach(SERVO_PIN, 500, 2500)) {
    servoMotor.write(ANGLE_LOCKED);
    currentAngle = ANGLE_LOCKED; // Set initial state
    Serial.println("Servo attached & locked");
  } else {
    Serial.println("Servo attach failed");
  }

  // Web server routes
  server.on("/", handleRoot);
  server.on("/unlock", HTTP_POST, handleUnlock);
  server.on("/lock", HTTP_POST, handleLock);
  server.on("/data", HTTP_POST, handleWiFiData);
  server.begin();
  Serial.printf("Web UI → http://%s\n", WiFi.softAPIP().toString().c_str());

  randomSeed(micros());
  if (testMode) nextTest = millis() + random(10, 60) * 1000UL;
}

void loop() {
  server.handleClient();
  servoMotor.write(currentAngle); // keep servo refreshed

  // === NEW: Check for TFLM UART commands ===
  if (TFLMSerial.available()) {
    String command = TFLMSerial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      Serial.print("TFLM UART Command RX: '");
      Serial.print(command);
      Serial.println("'");
      
      if (command == "unlock") {
        unlockDoor(); // This updates currentAngle
      } else if (command == "lock") {
        lockDoor(); // This updates currentAngle
      }
    }
  }
  // =========================================

  // Test mode generator
  if (testMode && millis() >= nextTest) {
    latest.lat = random(TEST_LAT_MIN * 1e6, TEST_LAT_MAX * 1e6) / 1e6;
    latest.lon = random(TEST_LON_MIN * 1e6, TEST_LON_MAX * 1e6) / 1e6;
    latest.rssi = random(-100, -40);
    latest.test = true;
    latest.valid = true;
    latest.timestamp = millis();

    Serial.printf("TEST → %.6f, %.6f\n", latest.lat, latest.lon);
    if (inGeofence(latest.lat, latest.lon)) unlockDoor(); // This updates currentAngle
    else lockDoor(); // This updates currentAngle

    nextTest = millis() + INTERVAL * 1000UL;
  }

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String packet = "";
    while (LoRa.available()) packet += (char)LoRa.read();
    packet.trim();
    Serial.println("LoRa RX: " + packet);

    int c1 = packet.indexOf(',');
    int c2 = packet.indexOf(',', c1 + 1);
    int c3 = packet.indexOf(',', c2 + 1);

    if (c1 > 0 && c2 > c1 && c3 > c2) {
      String srvId = packet.substring(c3 + 1);
      if (srvId == SERVER_ID) {
        latest.lat = packet.substring(c1 + 1, c2).toFloat();
        latest.lon = packet.substring(c2 + 1, c3).toFloat();
        latest.rssi = LoRa.packetRssi();
        latest.test = false;
        latest.valid = true;
        latest.timestamp = millis();

        Serial.printf("LoRa Node → %.6f, %.6f | RSSI %d\n", latest.lat, latest.lon, latest.rssi);
        if (inGeofence(latest.lat, latest.lon)) unlockDoor(); // This updates currentAngle
        else lockDoor(); // This updates currentAngle

        LoRa.beginPacket();
        LoRa.print("ACK:" + String(NODE_ID));
        LoRa.endPacket();
      }
    }
  }
}