#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Servo.h>

// ===================================
// Configuration
// ===================================
#define LORA_FREQ_MHZ       915.0
#define SERVER_ID           "0681.7478"
#define NODE_ID             "7454.1974"

#define PIN_LORA_RESET      15
#define PIN_LORA_CS         17
#define PIN_LORA_MOSI       19
#define PIN_LORA_MISO       16
#define PIN_LORA_SCK        18
#define PIN_LORA_DIO0       26
#define PIN_LORA_EN         27

#define AP_SSID             "PetSmartER"
#define AP_PASSWORD         "pets1234"

#define TEST_MODE           true
#define INTERVAL            180

#define TEST_DELAY_MIN      10
#define TEST_DELAY_MAX      120

#define TEST_LAT_MIN        40.01610
#define TEST_LAT_MAX        40.01730
#define TEST_LON_MIN        -105.28110
#define TEST_LON_MAX        -105.27920

#define ALERT_LAT_MIN       40.01650
#define ALERT_LAT_MAX       40.01700
#define ALERT_LON_MIN       -105.28050
#define ALERT_LON_MAX       -105.28000

#define SERVO_PIN           5
#define ANGLE_UNLOCKED      0
#define ANGLE_LOCKED        90
uint8_t currentAngle;

#define R_LED               13
#define G_LED               12
#define B_LED               11

// ===================================
// Global Objects
// ===================================
WebServer server(80);
Servo servoMotor;

struct Fix {
    float lat = 0.0;
    float lon = 0.0;
    int rssi = 0;
    bool test = false;
    bool valid = false;
} latest;

unsigned long nextTest = 0;
bool testMode = TEST_MODE;

const char* HTML_PAGE = R"=====(
<!DOCTYPE html>
<html>
<head>
<title>LoRa GPS</title>
<meta http-equiv="refresh" content="15">
<style>
body {
    background-color: #1e1e1e;
    color: #dcdcdc;
    font-family: Arial;
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    height: 100vh;
    margin: 0;
}
h1 {
    color: #0078d7;
    margin-bottom: 1rem;
}
table {
    border-collapse: collapse;
    min-width: 300px;
}
th, td {
    border: 1px solid #333;
    padding: 0.5rem 1rem;
    text-align: center;
}
th {
    background-color: #0078d7;
    color: #fff;
}
td {
    background-color: #2d2d2d;
}
button {
    margin: 1rem;
    padding: 1rem 2rem;
    font-size: 1.2rem;
    border: none;
    border-radius: 6px;
    cursor: pointer;
}
.unlock {
    background-color: #28a745;
    color: white;
}
.lock {
    background-color: #d73a49;
    color: white;
}
</style>
</head>
<body>
<h1>Latest GPS Fix</h1>
%s

<div>
    <form action="/unlock" method="post">
       <button class="unlock" type="submit">UNLOCK</button>
    </form>

    <form action="/lock" method="post">
       <button class="lock" type="submit">LOCK</button>
    </form>
</div>

</body>
</html>
)=====";

// ===================================
// Helper Functions
// ===================================
String generateTable() {
    if (!latest.valid) {
        return "<p>No fix yet.</p>";
    }
    char buf[256];
    snprintf(buf, sizeof(buf),
        "<table>"
        "<tr><th>Lat</th><th>Lon</th><th>RSSI</th></tr>"
        "<tr><td>%.6f</td><td>%.6f</td><td>%d</td></tr>"
        "</table>",
        latest.lat, latest.lon, latest.rssi
    );
    return String(buf);
}

bool inGeofence(float lat, float lon) {
    return (ALERT_LAT_MIN <= lat && lat <= ALERT_LAT_MAX &&
            ALERT_LON_MIN <= lon && lon <= ALERT_LON_MAX);
}

void generateTestFix() {
    latest.lat = random(TEST_LAT_MIN * 1e6, TEST_LAT_MAX * 1e6) / 1e6;
    latest.lon = random(TEST_LON_MIN * 1e6, TEST_LON_MAX * 1e6) / 1e6;
    latest.rssi = random(-110, -35);
    latest.test = true;
    latest.valid = true;
    Serial.printf("Test RX: NODE_1,%.6f,%.6f,%s\n", latest.lat, latest.lon, SERVER_ID);
}

void handleRoot() {
    char body[1024];
    String table = generateTable();
    snprintf(body, sizeof(body), HTML_PAGE, table.c_str());
    server.send(200, "text/html", body);
}

void handleUnlock() {
    currentAngle = ANGLE_UNLOCKED;
    servoMotor.write(currentAngle);
    Serial.println("Manual UNLOCK");
    server.sendHeader("Location", "/");
    server.send(303);
}

void handleLock() {
    currentAngle = ANGLE_LOCKED;
    servoMotor.write(currentAngle);
    Serial.println("Manual LOCK");
    server.sendHeader("Location", "/");
    server.send(303);
}

// ===================================
// Setup
// ===================================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(1000);

    pinMode(B_LED, OUTPUT);
    digitalWrite(B_LED, HIGH);

    Serial.println("Starting AP...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    IPAddress IP = WiFi.softAPIP();
    Serial.printf("AP: %s | http://%s\n", AP_SSID, IP.toString().c_str());

    pinMode(PIN_LORA_EN, OUTPUT);
    digitalWrite(PIN_LORA_EN, HIGH);

    SPI.begin();

    LoRa.setPins(PIN_LORA_CS, PIN_LORA_RESET, PIN_LORA_DIO0);
    if (!LoRa.begin(LORA_FREQ_MHZ * 1E6)) {
        Serial.println("LoRa init failed!");
        while (1);
    }
    LoRa.setTxPower(20);
    Serial.println("LoRa ready");

    if (!servoMotor.attach(SERVO_PIN, 1000, 2000)) {
        Serial.println("Servo attach failed!");
    } else {
        servoMotor.write(ANGLE_UNLOCKED);
        currentAngle = ANGLE_UNLOCKED;
        Serial.println("Servo UNLOCKED");
    }

    server.on("/", handleRoot);
    server.on("/unlock", HTTP_POST, handleUnlock);
    server.on("/lock",   HTTP_POST, handleLock);
    server.begin();
    Serial.printf("Web server: http://%s\n", IP.toString().c_str());

    if (testMode) {
        nextTest = millis() + random(TEST_DELAY_MIN, TEST_DELAY_MAX) * 1000;
    } else {
        nextTest = ULONG_MAX;
    }

    Serial.println("Listening...");
}

uint16_t counter = 0;

// ===================================
// Main Loop
// ===================================
void loop() {

    servoMotor.write(currentAngle);

    server.handleClient();

    unsigned long now = millis();

    // --------------------------
    // TEST MODE
    // --------------------------
    if (testMode && now >= nextTest) {
        generateTestFix();
        Serial.printf("Test update: %.6f, %.6f | RSSI: %d\n", latest.lat, latest.lon, latest.rssi);

        // NEW LOGIC: inside geofence = UNLOCK
        if (inGeofence(latest.lat, latest.lon)) {
            servoMotor.write(ANGLE_UNLOCKED);
            currentAngle = ANGLE_UNLOCKED;
            Serial.println("TEST: UNLOCKED (in zone)");
        } else {
            servoMotor.write(ANGLE_LOCKED);
            currentAngle = ANGLE_LOCKED;
            Serial.println("TEST: LOCKED (out)");
        }

        nextTest = now + INTERVAL * 1000;
    }

    // --------------------------
    // REAL LoRa RX
    // --------------------------
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        String packet = "";
        while (LoRa.available()) {
            packet += (char)LoRa.read();
        }
        packet.trim();
        Serial.println("RX: " + packet);

        int c1 = packet.indexOf(',');
        int c2 = packet.indexOf(',', c1 + 1);
        int c3 = packet.indexOf(',', c2 + 1);

        if (c1 > 0 && c2 > c1 && c3 > c2) {
            String node = packet.substring(0, c1);
            String latStr = packet.substring(c1 + 1, c2);
            String lonStr = packet.substring(c2 + 1, c3);
            String srvId = packet.substring(c3 + 1);

            if (srvId == SERVER_ID) {
                latest.lat = latStr.toFloat();
                latest.lon = lonStr.toFloat();
                latest.rssi = LoRa.packetRssi();
                latest.test = false;
                latest.valid = true;

                Serial.printf("From %s: %.6f, %.6f | RSSI: %d\n",
                              node.c_str(), latest.lat, latest.lon, latest.rssi);

                // NEW LOGIC: inside geofence = UNLOCK
                if (inGeofence(latest.lat, latest.lon)) {
                    servoMotor.write(ANGLE_UNLOCKED);
                    currentAngle = ANGLE_UNLOCKED;
                    Serial.println("UNLOCKED (in area)");
                } else {
                    servoMotor.write(ANGLE_LOCKED);
                    currentAngle = ANGLE_LOCKED;
                    Serial.println("LOCKED (outside)");
                }

                String ack = "ACK:" + String(NODE_ID) + ":" + String(SERVER_ID);
                LoRa.beginPacket();
                LoRa.print(ack);
                LoRa.endPacket();
                Serial.println("ACK sent");
            }
        } else {
            Serial.println("Parse error");
        }
    }

    // Debug LED tick
    counter++;
    if (counter % 65535 == 0) {
        digitalWrite(B_LED, LOW);
        delay(1000);
        digitalWrite(B_LED, HIGH);
    }
}