/*
Important: This code is only for the DIY OUTDOOR OPEN AIR Presoldered Kit with
the ESP-C3.

It is a high quality outdoor air quality sensor with dual PM2.5 modules and can
send data over Wifi.

Kits are available: https://www.airgradient.com/open-airgradient/kits/

The codes needs the following libraries installed:
“WifiManager by tzapu, tablatronix” tested with version 2.0.11-beta
“pms” by Markusz Kakl version 1.1.0 (needs to be patched for 5003T model)

For built instructions and how to patch the PMS library:
https://www.airgradient.com/open-airgradient/instructions/diy-open-air-presoldered-v11/

Note that below code only works with both PM sensor modules connected.

If you have any questions please visit our forum at
https://forum.airgradient.com/

CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License

*/

#include "PMS.h"
#include <HTTPClient.h>
#include <HardwareSerial.h>
#include <WiFiManager.h>
#include <Wire.h>

#include "format.h"
#include "json.h"

#define NDEBUG

#ifndef NDEBUG
#define DEBUG(...) Serial.print(__VA_ARGS__)
#define DEBUGLN(...) Serial.println(__VA_ARGS__);
#else
#define DEBUG(...)
#define DEBUGLN(...)
#endif

char API_ROOT[] = "http://192.168.0.16:3000/api/restricted/submit/";
const int targetSamples = 20;

char API_ENDPOINT[128];
char CHIP_ID[16] = "";

HTTPClient client;

struct SensorReading {
  float pm01 = 0;
  float pm02 = 0;
  float pm10 = 0;
  float pCnt = 0;
  float temp = 0;
  float rhum = 0;
};

struct SensorData {
  explicit SensorData(Stream &stream) : pms(stream) {}

  PMS pms;
  PMS::DATA data;

  SensorReading reading;

  bool tryRead(int count);
  void accumulate();
  SensorReading computeAvgAndReset(int samples);
};

bool SensorData::tryRead(int timeout) { return pms.readUntil(data, timeout); }

void SensorData::accumulate() {
  reading.pm01 += data.PM_AE_UG_1_0;
  reading.pm02 += data.PM_AE_UG_2_5;
  reading.pm10 += data.PM_AE_UG_10_0;
  reading.pCnt += data.PM_RAW_0_3;
  reading.temp += data.AMB_TMP / 10.0;
  reading.rhum += data.AMB_HUM / 10.0;
}

SensorReading SensorData::computeAvgAndReset(int samples) {
  auto ret = reading;
  ret.pm01 /= samples;
  ret.pm02 /= samples;
  ret.pm10 /= samples;
  ret.pCnt /= samples;
  ret.temp /= samples;
  ret.rhum /= samples;
  reading = {};
  return ret;
}

SensorData sensors[] = {SensorData{Serial0}, SensorData{Serial1}};

int countPosition = 0;
int targetCount = 20;

int loopCount = 0;

int sampleCount = 0;

void IRAM_ATTR isr() { DEBUGLN("pushed"); }

void setupChipID() {
  std::uint8_t mac[6];
  WiFi.macAddress(mac);

  std::snprintf(CHIP_ID, sizeof(CHIP_ID), "%02x%02x%02x%02x%02x%02x", mac[0],
                mac[1], mac[2], mac[3], mac[4], mac[5]);

  DEBUG("Serial Number: ");
  DEBUGLN(CHIP_ID);
}

void setupApiEndpoint() {
  std::snprintf(API_ENDPOINT, sizeof(API_ENDPOINT), "%s%s", &API_ROOT,
                &CHIP_ID);

  DEBUG("API endpoint: ");
  DEBUGLN(API_ENDPOINT);
}

// select board LOLIN C3 mini to flash
void setup() {
  Serial.begin(115200);
  // see https://github.com/espressif/arduino-esp32/issues/6983
  Serial.setTxTimeoutMs(0); // <<<====== solves the delay issue

  DEBUGLN("Starting ...");

  setupChipID();
  setupApiEndpoint();

  // default hardware serial, PMS connector on the right side of the C3 mini on
  // the Open Air
  Serial0.begin(9600);

  // second hardware serial, PMS connector on the left side of the C3 mini on
  // the Open Air
  Serial1.begin(9600, SERIAL_8N1, 0, 1);

  // led
  pinMode(10, OUTPUT);

  // push button
  pinMode(9, INPUT_PULLUP);
  attachInterrupt(9, isr, FALLING);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  // give the PMSs some time to start
  countdown(3);

  connectToWifi();
  setLED(false);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (sensors[0].tryRead(2000) && sensors[1].tryRead(2000)) {
      sensors[0].accumulate();
      sensors[1].accumulate();
      sampleCount++;
      if (sampleCount >= targetCount) {
        postToApi(sensors[0].computeAvgAndReset(sampleCount),
                  sensors[1].computeAvgAndReset(sampleCount));
        sampleCount = 0;
      }
    }
  }

  countdown(2);
}

void setLED(boolean ledON) {
  digitalWrite(10, ledON ? HIGH : LOW);
}

void postToApi(const SensorReading &r0, const SensorReading &r1) {
  static char payload[512] = "";
  JsonFormatter json(payload);
  {
    auto root = json.object();
    root.addMember("wifi", WiFi.RSSI());
    root.addMember("pm01", 0.5 * (r0.pm01 + r1.pm01), 1);
    root.addMember("pm02", 0.5 * (r0.pm02 + r1.pm02), 1);
    root.addMember("pm10", 0.5 * (r0.pm10 + r1.pm10), 1);
    root.addMember("pmCnt", 0.5 * (r0.pCnt + r1.pCnt), 1);
    root.addMember("atmp", 0.5 * (r0.temp + r1.temp), 2);
    root.addMember("rhum", 0.5 * (r0.rhum + r1.rhum), 2);
    {
      auto channels = root.addArrayMember("channels");
      for (auto &r : {r0, r1}) {
        auto channel = channels.pushObject();
        channel.addMember("pm01", r.pm01, 1);
        channel.addMember("pm02", r.pm02, 1);
        channel.addMember("pm10", r.pm10, 1);
        channel.addMember("pmCnt", r.pCnt, 1);
        channel.addMember("atmp", r.temp, 2);
        channel.addMember("rhum", r.rhum, 2);
      }
    }
  }
  if (json.formatter()) {
    sendPayload(payload, json.formatter().peek() - payload);
  } else {
    DEBUGLN("Payload buffer overflow");
  }
}

void sendPayload(const char *payload, std::size_t length) {
  if (WiFi.status() != WL_CONNECTED) {
    DEBUGLN("No network connection");
    return;
  }

  DEBUGLN(payload);
  client.setConnectTimeout(5 * 1000);
  client.begin(API_ENDPOINT);
  client.addHeader("content-type", "application/json");
  // This function requiring a non-const pointer was not a good decision.
  int httpCode = client.POST((std::uint8_t *)payload, length);
  DEBUGLN(httpCode);
  client.end();
  resetWatchdog();
}

void countdown(int seconds) {
  while (seconds-- > 0) {
    DEBUG(seconds);
    DEBUG(" ");
    delay(1000);
  }
  DEBUGLN();
}

void resetWatchdog() {
  digitalWrite(2, HIGH);
  delay(20);
  digitalWrite(2, LOW);
}

// Wifi Manager
void connectToWifi() {
  WiFiManager wifiManager;
  setLED(true);
  // WiFi.disconnect(); //to delete previous saved hotspot
  char HOTSPOT[sizeof(CHIP_ID) + 4];
  std::snprintf(HOTSPOT, sizeof(HOTSPOT), "AG-%s", CHIP_ID);
  wifiManager.setTimeout(180);

  if (!wifiManager.autoConnect(HOTSPOT)) {
    setLED(false);
    Serial.println("failed to connect and hit timeout");
    delay(6000);
  }
}
