#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// Pin assignments for XIAO RA4M1
#define GPS_TX    D0
#define GPS_RX    D1
#define GPS_PPS   D2
#define GPS_EN    D3
#define LORA_RST  D4
#define LORA_EN   D5
#define LORA_G0   D6
#define LORA_CS   D7
#define LORA_SCK  D8
#define LORA_MISO D9
#define LORA_MOSI D10

// Node configuration
#define NODE_ID       "7454.1974"
#define SERVER_ID     "0681.7478"
#define LORA_FREQ     915E6
#define TX_POWER      20
#define GPS_BAUD      9600
#define SEND_INTERVAL 180000   // 3 minutes

SoftwareSerial GPS_SERIAL(GPS_TX, GPS_RX);
TinyGPSPlus gps;
unsigned long lastSendTime = 0;

// Function prototypes
void onReceive(int packetSize);
void sendPacket(double lat, double lon);
void checkSerialCommand();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("XIAO RA4M1 LoRa GPS Node Starting...");

  // GPS power and UART
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, HIGH);
  GPS_SERIAL.begin(GPS_BAUD);
  Serial.println("GPS UART initialized.");

  // LoRa power and SPI
  pinMode(LORA_EN, OUTPUT);
  digitalWrite(LORA_EN, HIGH);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_G0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa initialization Failed!");
    while (1);
  }

  LoRa.setTxPower(TX_POWER);
  LoRa.onReceive(onReceive);
  LoRa.receive();

  Serial.print("LoRa initialized OK on ");
  Serial.print(LORA_FREQ / 1000000);
  Serial.println(" MHz.");
}

void loop() {
  // Parse any incoming GPS data
  while (GPS_SERIAL.available() > 0) {
    gps.encode(GPS_SERIAL.read());
  }

  // Allow manual packet send via Serial Monitor
  checkSerialCommand();

  // Periodic automatic transmission
  if (millis() - lastSendTime > SEND_INTERVAL) {
    if (gps.location.isValid()) {
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      sendPacket(lat, lon);
      lastSendTime = millis();
    } else {
      Serial.print("No valid GPS fix. Satellites: ");
      Serial.println(gps.satellites.value());
    }
  }
}

// Build and send a LoRa packet: NODE_ID,lat,lon,SERVER_ID
void sendPacket(double lat, double lon) {
  char message[60];
  snprintf(message, sizeof(message), "%s,%.6f,%.6f,%s",
           NODE_ID, lat, lon, SERVER_ID);
  Serial.print("Sending packet: ");
  Serial.println(message);
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket(true);   // wait for completion
  Serial.println("Packet sent.");
}

// Handle 'send(lat,lon)' commands from Serial Monitor
void checkSerialCommand() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.startsWith("send(") && command.endsWith(")")) {
      Serial.println("Serial command recognized.");
      int start = command.indexOf('(') + 1;
      int end   = command.indexOf(')');
      if (start > 0 && end > start) {
        String data = command.substring(start, end);
        int commaPos = data.indexOf(',');
        if (commaPos != -1) {
          double lat = data.substring(0, commaPos).toDouble();
          double lon = data.substring(commaPos + 1).toDouble();
          Serial.print("Manual send: Lat=");
          Serial.print(lat, 6);
          Serial.print(", Lon=");
          Serial.println(lon, 6);
          sendPacket(lat, lon);
        } else {
          Serial.println("Error: Invalid format. Use send(lat,lon).");
        }
      }
    }
  }
}

// Process incoming LoRa packets (ACKs)
void onReceive(int packetSize) {
  if (packetSize) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    if (incoming.startsWith("ACK:") && incoming.indexOf(NODE_ID) != -1) {
      Serial.print("ACK received: ");
      Serial.println(incoming);
    } else {
      Serial.print("Unrecognized packet: ");
      Serial.println(incoming);
    }
  }
  LoRa.receive();   // back to receive mode
}