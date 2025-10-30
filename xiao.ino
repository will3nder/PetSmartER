// XIAO RA4M1 — GPS + RFM95W

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GPS.h>

// pinout
#define LORA_CS     D7   // CS / NSS
#define LORA_G0     D6   // DIO0 / IRQ
#define LORA_MOSI   D10  // MOSI
#define LORA_MISO   D9   // MISO
#define LORA_SCK    D8   // SCK

#define RADIO_EN_PIN  D5
#define GPS_EN_PIN    D3

#define GPS_PPS D2      // optional PPS
#define GPS_TX D1       // GPS TX
#define GPS_RX D0       // GPS RX

#define LORA_FREQUENCY 915E6
#define NODE_ID   "7454.1974"   // unique node identifier xxxx.xxxx
#define SERVER_ID "0681.7478"   // paired server ID xxxx.xxxx

// timing (ms)
const uint32_t GPS_COLLECTION_MS = 5000;
const uint32_t LORA_RX_WAIT_MS   = 2000;
const uint32_t POST_SLEEP_MS     = 171000; // use smaller for testing

// GPS object
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

// RA-FSP hook (optional). If you link RA-FSP LPM, define RA4M1_LPM_LINKED.
extern "C" void ra4m1_enter_low_power(uint32_t ms);

// -------- forward ----------
void powerOnGPS();
void powerOffGPS();
void powerOnRadio();
void powerOffRadio();
bool getGpsFix(float &lat, float &lon);
bool waitForAck(uint32_t timeout_ms);
void sendPacketWithRetry(const String &payload);
void enterDeepSleep(uint32_t ms);

// -------- setup/loop ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1);
  Serial.println(F("xiao: lora+gps starting"));

  // EN pins default HIGH (modules enabled)
  pinMode(GPS_EN_PIN, OUTPUT);
  pinMode(RADIO_EN_PIN, OUTPUT);
  digitalWrite(GPS_EN_PIN, HIGH);
  digitalWrite(RADIO_EN_PIN, HIGH);
  delay(50);

  // make SPI pins sensible immediately
  pinMode(LORA_SCK, OUTPUT);
  pinMode(LORA_MOSI, OUTPUT);
  pinMode(LORA_MISO, INPUT);

  // PPS as input if used
  pinMode(GPS_PPS, INPUT);
}

void loop() {
  // wake peripherals
  powerOnGPS();
  powerOnRadio();

  // collect a fix (5s window)
  float lat = 0.0, lon = 0.0;
  Serial.println(F("collecting GPS..."));
  bool ok = getGpsFix(lat, lon);
  if (ok) {
    Serial.print(F("fix: "));
    Serial.print(lat, 6); Serial.print(','); Serial.println(lon, 6);
  } else {
    Serial.println(F("no fix; sending 0,0"));
  }

  // send, wait ACK, retry once
  String payload = String(NODE_ID) + "," +
                   (ok ? String(lat, 6) + "," + String(lon, 6) : "0,0") +
                   "," + String(SERVER_ID);
  sendPacketWithRetry(payload);

  // shut peripherals down
  powerOffRadio();
  powerOffGPS();

  // sleep (RA-FSP hook if linked, otherwise WFI fallback)
  enterDeepSleep(POST_SLEEP_MS);
}

// -------- power control --------
void powerOnGPS() {
  digitalWrite(GPS_EN_PIN, HIGH);
  delay(50);
  GPSSerial.begin(9600);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // RMC + GGA
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(50);
  Serial.println(F("gps on"));
}

void powerOffGPS() {
  GPSSerial.end();
  digitalWrite(GPS_EN_PIN, LOW);
  delay(5);
  Serial.println(F("gps off"));
}

void powerOnRadio() {
  digitalWrite(RADIO_EN_PIN, HIGH);
  delay(25);

  // try to bind SPI to these pins so core picks right SERCOM; if core doesn't support, fallback to SPI.begin()
  #if defined(SPI_HAS_TRANSACTION) && defined(SPI_BEGIN_WITH_PINS)
    SPI.begin(LORA_SCK, LORA_MOSI, LORA_MISO);
  #else
    SPI.begin();
  #endif

  // pin sanity
  pinMode(LORA_SCK, OUTPUT);
  pinMode(LORA_MOSI, OUTPUT);
  pinMode(LORA_MISO, INPUT);
  pinMode(LORA_CS, OUTPUT);
  pinMode(LORA_G0, INPUT);

  // no RST under MCU control; pass -1
  LoRa.setPins(LORA_CS, -1, LORA_G0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println(F("warning: LoRa.begin failed"));
  } else {
    LoRa.idle();
    delay(5);
    Serial.println(F("radio on"));
  }
}

void powerOffRadio() {
  Serial.println(F("radio: sleep -> off"));
  // polite sleep (if LoRa was init'd)
  LoRa.idle();
  delay(5);
  LoRa.sleep();
  delay(10);

  // cut power
  digitalWrite(RADIO_EN_PIN, LOW);
  delay(5);

  // tri-state pins to avoid back-feed
  pinMode(LORA_CS, INPUT);
  pinMode(LORA_G0, INPUT);
  pinMode(LORA_SCK, INPUT);
  pinMode(LORA_MOSI, INPUT);
  pinMode(LORA_MISO, INPUT);
  Serial.println(F("radio off"));
}

// -------- GPS parsing --------
bool getGpsFix(float &outLat, float &outLon) {
  unsigned long t0 = millis();
  while (GPSSerial.available()) GPSSerial.read(); // flush
  while (millis() - t0 < GPS_COLLECTION_MS) {
    while (GPSSerial.available()) {
      GPS.encode((char)GPSSerial.read());
    }
    if (GPS.fix && GPS.latitude != 0.0 && GPS.longitude != 0.0) {
      outLat = GPS.latitudeDegrees;
      outLon = GPS.longitudeDegrees;
      return true;
    }
    delay(10);
  }
  if (GPS.fix) { outLat = GPS.latitudeDegrees; outLon = GPS.longitudeDegrees; return true; }
  return false;
}

// -------- LoRa comms --------
bool waitForAck(uint32_t timeout_ms) {
  unsigned long t0 = millis();
  String expect = String("ACK:") + NODE_ID + ":" + SERVER_ID;
  while (millis() - t0 < timeout_ms) {
    int sz = LoRa.parsePacket();
    if (sz > 0) {
      String s;
      while (LoRa.available()) s += (char)LoRa.read();
      s.trim();
      if (s == expect) return true;
    }
    delay(10);
  }
  return false;
}

void sendPacketWithRetry(const String &payload) {
  LoRa.idle();
  delay(5);
  Serial.print(F("TX -> ")); Serial.println(payload);
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();

  if (waitForAck(LORA_RX_WAIT_MS)) {
    Serial.println(F("ACK"));
    return;
  }
  // one retry
  Serial.println(F("no ACK, retrying"));
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();
  if (waitForAck(LORA_RX_WAIT_MS)) Serial.println(F("ACK on retry"));
  else Serial.println(F("no ACK after retry"));
}

// -------- deep sleep (WFI fallback) --------
void enterDeepSleep(uint32_t ms)
{
  #ifdef RA4M1_LPM_LINKED
    ra4m1_enter_low_power(ms);
    return;
  #endif

  // make sure pins won't back-feed powered-down modules
  pinMode(LORA_CS, INPUT);
  pinMode(LORA_G0, INPUT);
  pinMode(LORA_SCK, INPUT);
  pinMode(LORA_MOSI, INPUT);
  pinMode(LORA_MISO, INPUT);
  pinMode(GPS_EN_PIN, INPUT);
  pinMode(RADIO_EN_PIN, INPUT);
  pinMode(GPS_PPS, INPUT);

  delay(5); // settle

  unsigned long start = millis();

  // request deep sleep if core supports it
  #if defined(SCB) && defined(SCB_SCR_SLEEPDEEP_Msk)
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  #endif

  while (ms == 0 || (millis() - start < ms)) {
    __enable_irq();
    // dsb/isb before wfi
    #if defined(__DSB) && defined(__ISB)
      __DSB();
      __ISB();
    #else
      __asm__ volatile ("dsb");
      __asm__ volatile ("isb");
    #endif
    __WFI();
    if (ms != 0 && (millis() - start >= ms)) break;
    // otherwise re-enter
  }

  #if defined(SCB) && defined(SCB_SCR_SLEEPDEEP_Msk)
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
  #endif

  delay(10); // settle after wake
}
