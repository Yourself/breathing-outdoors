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

#define ARDUHAL_LOG_LEVEL 3

#include <AirGradient.h>
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
AirGradient ag(OPEN_AIR_OUTDOOR);

using Callback = void (*)(void *data);

class Schedule {
public:
  Schedule(int period, Callback handler, void *data = nullptr)
      : period_(period), next_(0), handler_(handler), data_(data) {}

  void run() {
    if (next_ <= millis()) {
      handler_(data_);
      next_ += period_;
    }
  }

private:
  unsigned long period_;
  unsigned long next_;
  Callback handler_;
  void *data_;
};

struct PMSReading {
  float pm01 = 0;
  float pm02 = 0;
  float pm10 = 0;
  float pCnt = 0;
  float temp = 0;
  float rhum = 0;

  PMSReading &operator+=(const PMSReading &other) {
    pm01 += other.pm01;
    pm02 += other.pm02;
    pm10 += other.pm10;
    pCnt += other.pCnt;
    temp += other.temp;
    rhum += other.rhum;
    return *this;
  }

  PMSReading &operator/=(float s) {
    pm01 /= s;
    pm02 /= s;
    pm10 /= s;
    pCnt /= s;
    temp /= s;
    rhum /= s;
    return *this;
  }
};

class PMSensor {
public:
  explicit PMSensor(PMS5003T &pms)
      : accum_{}, pms_(&pms), samples_(0), active_(false) {}

  bool active() const { return active_; }
  bool begin(HardwareSerial &serial) { return active_ = pms_->begin(serial); }

  void update() {
    DEBUG("PMSensor ");
    DEBUG((size_t)(void *)this);
    DEBUG(" ");
    DEBUGLN(active_ ? "active" : "inactive");
    if (!(active_ && pms_->readData())) {
      return;
    }

    accum_.pCnt += pms_->getPm03ParticleCount();
    accum_.pm01 += pms_->getPm01Ae();
    accum_.pm02 += pms_->getPm25Ae();
    accum_.pm10 += pms_->getPm10Ae();
    accum_.temp += pms_->getRawTemperature();
    accum_.rhum += pms_->getRawRelativeHumidity();
    ++samples_;
    DEBUG("  Samples: ");
    DEBUGLN(samples_);
  }

  bool tryGetReading(PMSReading &out) {
    if (samples_ <= 0) {
      return false;
    }
    out = accum_;
    out /= samples_;
    accum_ = {};
    samples_ = 0;
    return true;
  }

private:
  PMSReading accum_;
  PMS5003T *pms_;
  int samples_;
  bool active_;
};

class S8Sensor {
public:
  explicit S8Sensor(S8 &s8)
      : s8_(&s8), co2ppm_(-1), samples_(0), active_(false) {}

  bool active() const { return active_; }
  bool begin(HardwareSerial &serial) { return active_ = s8_->begin(serial); }

  void update() {
    DEBUG("S8Sensor ");
    DEBUGLN(active_ ? "active" : "inactive");
    if (active_) {
      auto co2 = s8_->getCo2();
      if (co2 == 0) {
        return;
      }
      co2ppm_ += co2;
      ++samples_;
      DEBUG("  Samples: ");
      DEBUGLN(samples_);
    }
  }

  bool tryGetReading(float &co2ppm) {
    if (samples_ <= 0) {
      return false;
    }
    co2ppm = co2ppm_ / samples_;
    co2ppm_ = 0;
    samples_ = 0;
    return true;
  }

private:
  S8 *s8_;
  float co2ppm_;
  int samples_;
  bool active_;
};

struct SGPReading {
  float tvoc = -1;
  float tvocRaw = -1;
  float nox = -1;
};

class SGPSensor {
public:
  explicit SGPSensor(Sgp41 &sgp) : sgp_(&sgp), samples_(0), active_(false) {}

  bool active() const { return active_; }
  bool begin(TwoWire &wire) { return active_ = sgp_->begin(wire); }

  void update() {
    DEBUG("SGPSensor ");
    DEBUGLN(active_ ? "active" : "inactive");
    if (active_) {
      int tvoc = sgp_->getTvocIndex();
      int tvocRaw = sgp_->getTvocRaw();
      int nox = sgp_->getNoxIndex();
      if (tvoc < 0 || nox < 0) {
        return;
      }
      reading_.tvoc += tvoc;
      reading_.tvocRaw += tvocRaw;
      reading_.nox += nox;
      ++samples_;
      DEBUG("  Samples: ");
      DEBUGLN(samples_);
    }
  }

  bool tryGetReading(SGPReading &reading) {
    if (samples_ <= 0) {
      return false;
    }
    reading.tvoc = reading_.tvoc / samples_;
    reading.tvocRaw = reading_.tvocRaw / samples_;
    reading.nox = reading_.nox / samples_;
    reading_ = {};
    samples_ = 0;
    return true;
  }

private:
  SGPReading reading_;
  Sgp41 *sgp_;
  int samples_;
  bool active_;
};

constexpr unsigned long s8UpdatePeriod_ms = 4000;
constexpr unsigned long sgpUpdatePeriod_ms = 1000;
constexpr unsigned long pmsUpdatePeriod_ms = 2000;
constexpr unsigned long serverUpdatePeriod_ms = 60000;

PMSensor pmSensors[] = {PMSensor(ag.pms5003t_1), PMSensor(ag.pms5003t_2)};
S8Sensor s8Sensor(ag.s8);
SGPSensor sgpSensor(ag.sgp41);

void s8Update(void *) {
  DEBUGLN("Updating S8");
  s8Sensor.update();
}

void sgpUpdate(void *) {
  DEBUGLN("Updating SGP");
  sgpSensor.update();
}

void pmUpdate(void *) {
  DEBUGLN("Updating PMS");
  for (auto &sensor : pmSensors) {
    sensor.update();
  }
}

template <class T, size_t N> size_t size(const T (&)[N]) noexcept { return N; }

void serverUpdate(void *) {
  DEBUGLN("Sending server update...");
  static char payload[512] = "";
  bool hasData = false;
  JsonFormatter json(payload);
  {
    auto root = json.object();
    root.addMember("wifi", WiFi.RSSI());
    float co2;
    SGPReading sgpReading;
    size_t pmsCount = 0;
    PMSReading pmsAccum;
    PMSReading pmsReadings[size(pmSensors)];
    if (s8Sensor.tryGetReading(co2)) {
      root.addMember("rco2", co2, 0);
      hasData = true;
    }
    if (sgpSensor.tryGetReading(sgpReading)) {
      root.addMember("tvoc", sgpReading.tvoc, 0);
      root.addMember("tvocRaw", sgpReading.tvocRaw, 0);
      root.addMember("nox", sgpReading.nox, 0);
      hasData = true;
    }
    while (pmsCount < size(pmSensors) &&
           pmSensors[pmsCount].tryGetReading(pmsReadings[pmsCount])) {
      pmsAccum += pmsReadings[pmsCount];

      ++pmsCount;
    }
    if (pmsCount > 0) {
      hasData = true;
      pmsAccum /= pmsCount;
      root.addMember("pm01", pmsAccum.pm01, 1);
      root.addMember("pm02", pmsAccum.pm02, 1);
      root.addMember("pm10", pmsAccum.pm10, 1);
      root.addMember("pCnt", pmsAccum.pCnt, 1);
      root.addMember("atmp", pmsAccum.temp, 1);
      root.addMember("rhum", pmsAccum.rhum, 1);
      if (pmsCount > 1) {
        auto channels = root.addArrayMember("channels");
        for (size_t i = 0; i < pmsCount; ++i) {
          auto channel = channels.pushObject();
          channel.addMember("pm01", pmsReadings[i].pm01, 1);
          channel.addMember("pm02", pmsReadings[i].pm02, 1);
          channel.addMember("pm10", pmsReadings[i].pm10, 1);
          channel.addMember("pCnt", pmsReadings[i].pCnt, 1);
          channel.addMember("atmp", pmsReadings[i].temp, 1);
          channel.addMember("rhum", pmsReadings[i].rhum, 1);
        }
      }
    }
  }
  if (json.formatter()) {
    *json.formatter().peek() = '\0';
    DEBUGLN(payload);
    if (hasData) {
      sendPayload(payload, json.formatter().peek() - payload);
    }
  } else {
    DEBUGLN("Payload buffer overflow");
  }
}

Schedule schedules[] = {{s8UpdatePeriod_ms, &s8Update},
                        {sgpUpdatePeriod_ms, &sgpUpdate},
                        {pmsUpdatePeriod_ms, &pmUpdate},
                        {serverUpdatePeriod_ms, &serverUpdate}};

struct SerialEndpoint {
  HardwareSerial *serial;
  bool inUse;
};

SerialEndpoint serials[] = {{&Serial0, false}, {&Serial1, false}};

template <class Fn> bool connectSerial(Fn &&fn) {
  for (SerialEndpoint &ep : serials) {
    if (ep.inUse) {
      continue;
    }
    if (std::forward<Fn>(fn)(*ep.serial)) {
      ep.inUse = true;
      return true;
    }
  }
  return false;
}

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

void boardInit() {
  if (!Wire.begin(ag.getI2cSdaPin(), ag.getI2cSclPin())) {
    Serial.println("I2C initialization failed.");
  }

  ag.watchdog.begin();
  ag.button.begin();
  ag.statusLed.begin();

  if (!s8Sensor.begin(Serial1)) {
    Serial.println("CO2 sensor not found");
    Serial1.end();
  }
  if (!sgpSensor.begin(Wire)) {
    Serial.println("SGP sensor not found");
  }
  bool pmsFound = false;
  for (auto &pmSensor : pmSensors) {
    if (connectSerial([&](HardwareSerial &s) { return pmSensor.begin(s); })) {
      pmsFound = true;
    }
  }
  if (!pmsFound) {
    Serial.println("PMS sensor not found");
  }
}

// select board LOLIN C3 mini to flash
void setup() {
  Serial.begin(115200);
  // see https://github.com/espressif/arduino-esp32/issues/6983
  Serial.setTxTimeoutMs(0); // <<<====== solves the delay issue

  DEBUGLN("Starting ...");

  setupChipID();
  setupApiEndpoint();

  boardInit();

  connectToWifi();
  setLED(false);
  // give the PMSs some time to start
  countdown(3);
}

void loop() {
  for (auto &schedule : schedules) {
    schedule.run();
  }
  delay(500);
}

void setLED(boolean ledOn) {
  if (ledOn) {
    ag.statusLed.setOn();
  } else {
    ag.statusLed.setOff();
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
  ag.watchdog.reset();
}

void countdown(int seconds) {
  while (seconds-- > 0) {
    DEBUG(seconds);
    DEBUG(" ");
    delay(1000);
  }
  DEBUGLN();
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
