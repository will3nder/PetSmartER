#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// ==================== CONFIG ====================
const char* ssid     = "PetSmartER";
const char* password = "pets1234";
const char* serverUrl = "http://192.168.42.1/data";   // ESP32 AP IP

#define NODE_ID       "7454.1974"
#define SERVER_ID     "0681.7478"

#define GPS_TX    D0
#define GPS_RX    D1
#define GPS_EN    D3

#define SEND_INTERVAL 180000   // 3 minutes in ms
// ================================================

SoftwareSerial GPS_SERIAL(GPS_TX, GPS_RX);
TinyGPSPlus gps;

unsigned long lastSendTime = 0;
bool wifiConnected = false;

void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  uint8_t attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {  // ~20 s timeout
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;
  } else {
    Serial.println("\nWiFi failed");
    wifiConnected = false;
  }
}

void sendPosition(double lat, double lon) {
  if (!wifiConnected) return;

  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "text/plain");

  char payload[80];
  snprintf(payload, sizeof(payload), "%s,%.6f,%.6f,%s",
           NODE_ID, lat, lon, SERVER_ID);

  Serial.print("HTTP POST: ");
  Serial.println(payload);

  int httpCode = http.POST(payload);

  if (httpCode > 0) {
    Serial.printf("HTTP %d\n", httpCode);
    if (httpCode == 200) {
      String response = http.getString();
      Serial.println("Server says: " + response);
    }
  } else {
    Serial.printf("HTTP failed: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Power on GPS module
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, HIGH);
  GPS_SERIAL.begin(9600);
  Serial.println("XIAO RA4M1 WiFi GPS Node Starting...");

  connectWiFi();
}

void loop() {
  // Feed GPS parser
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
  }

  // Reconnect if WiFi dropped
  if (WiFi.status() != WL_CONNECTED && millis() > 30000) {
    Serial.println("WiFi lost, reconnecting...");
    connectWiFi();
    delay(5000);
  }

  // Manual command from Serial Monitor: send(40.01666,-105.28000)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("send(") && cmd.endsWith(")")) {
      int s = cmd.indexOf('(') + 1;
      int e = cmd.indexOf(')');
      String data = cmd.substring(s, e);
      int comma = data.indexOf(',');
      if (comma != -1) {
        double lat = data.substring(0, comma).toDouble();
        double lon = data.substring(comma + 1).toDouble();
        sendPosition(lat, lon);
      }
    }
  }

  // Periodic send
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    if (gps.location.isValid()) {
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      sendPosition(lat, lon);
      lastSendTime = millis();
    } else {
      Serial.print("No GPS fix yet. Sats: ");
      Serial.println(gps.satellites.value());
    }
  }
}