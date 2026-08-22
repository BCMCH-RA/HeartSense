/*
  ============================================================================
  HeartSense — XIAO ESP32S3 Firmware
  ============================================================================
  Hardware
    - Seeed XIAO ESP32S3
    - MPU6050 IMU on I2C     : SDA = D4, SCL = D5
    - Electret mic (analog, biased ~1.65V, capacitor-coupled) on D8 (ADC1)

  What it does
    1. Advertises a BLE GATT service.
    2. Once the browser/phone app connects, it writes the current UNIX epoch
       (ms) to the TIME characteristic. The firmware stores the offset
       between that epoch and its own millis() clock (a monotonic local
       reference), so every future sample can be time-stamped in real UTC
       time without needing an RTC. IST (UTC+5:30) conversion is done on the
       receiving app side (fixed offset, India has no DST).
    3. Samples the MPU6050 at a hardware-timed 200 Hz and the microphone at
       a configurable rate (default 2000 Hz — enough bandwidth for S1/S2
       heart sound content and murmurs).
    4. Batches samples into small self-describing binary packets sent as BLE
       notifications every `packetIntervalMs` (default 20 ms -> 50 pkt/s).
       Batching is the single biggest lever for BLE power + reliability:
       fewer, fuller radio events beat one notify per sample.
    5. Streaming (and audio rate / packet interval) is controlled from the
       app via the CONTROL characteristic, so the radio + ADC + I2C only run
       when you actually want data — saving battery between recordings.

  Power-saving choices made here
    - WiFi radio is explicitly disabled (WiFi.mode(WIFI_OFF)) — the ESP32S3
      shares RF front-end time between WiFi/BLE, and only BLE is needed.
    - Sample *acquisition* is decoupled from BLE *transmission*: esp_timer
      interrupts only increment a counter; actual I2C/ADC reads happen in
      loop(), and radio TX happens on its own cadence. This keeps sampling
      jitter low without holding the radio busy sample-by-sample.
    - Packets are self-describing (rates are embedded, not implicit), so no
      "keep-alive" chatter is needed to keep both sides in sync.
    - BLE TX power is set to a moderate level (adjust ESP_PWR_LVL below for
      your enclosure/range needs — lower = better battery life).
    - A drop-oldest ring buffer prevents unbounded memory growth / stalls if
      the link is briefly slow; dropped-sample counters are reported on the
      STATUS characteristic so data loss is visible, not silent.

  Library required
    - "NimBLE-Arduino" by h2zero (Library Manager). Tested against v1.4.x
      API (NimBLECharacteristicCallbacks::onWrite(NimBLECharacteristic*)).
      If you're on NimBLE-Arduino v2.x, callback signatures gain a
      NimBLEConnInfo& parameter — add it to onConnect/onDisconnect/onWrite.

  Board package: esp32 by Espressif, board "XIAO_ESP32S3".
  ============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include "esp_timer.h"

// ---------------------------------------------------------------------------
// Pin map
// ---------------------------------------------------------------------------
#define PIN_SDA   D4
#define PIN_SCL   D5
#define PIN_MIC   D8      // must be an ADC1-capable GPIO on your XIAO variant

// ---------------------------------------------------------------------------
// MPU6050 raw register driver (no external library dependency)
// ---------------------------------------------------------------------------
#define MPU_ADDR          0x68
#define REG_PWR_MGMT_1    0x6B
#define REG_CONFIG        0x1A   // DLPF
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
// Sensitivities for the ranges configured below (see mpuInit):
//   Accel ±2g   -> 16384 LSB/g
//   Gyro  ±250dps -> 131 LSB/(deg/s)

const uint16_t IMU_RATE_HZ   = 200;
const uint32_t IMU_PERIOD_US = 1000000UL / IMU_RATE_HZ;

void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuInit() {
  mpuWriteReg(REG_PWR_MGMT_1, 0x80); delay(50);  // reset
  mpuWriteReg(REG_PWR_MGMT_1, 0x01);             // wake, gyro-X clock ref
  mpuWriteReg(REG_CONFIG, 0x03);                 // DLPF: ~44Hz accel/42Hz gyro
  mpuWriteReg(REG_GYRO_CONFIG, 0x00);             // ±250 dps
  mpuWriteReg(REG_ACCEL_CONFIG, 0x00);            // ±2 g
}

struct ImuRaw { int16_t ax, ay, az, gx, gy, gz; };

bool mpuReadRaw(ImuRaw &s) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14, (int)true) != 14) return false;
  s.ax = (Wire.read() << 8) | Wire.read();
  s.ay = (Wire.read() << 8) | Wire.read();
  s.az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();               // skip temperature
  s.gx = (Wire.read() << 8) | Wire.read();
  s.gy = (Wire.read() << 8) | Wire.read();
  s.gz = (Wire.read() << 8) | Wire.read();
  return true;
}

// ---------------------------------------------------------------------------
// Microphone (analog electret) — rate is adjustable at runtime
// ---------------------------------------------------------------------------
volatile uint16_t micRateHz  = 2000;
volatile uint32_t micPeriodUs = 1000000UL / 2000;

// ---------------------------------------------------------------------------
// Packetization
// ---------------------------------------------------------------------------
volatile uint8_t packetIntervalMs = 20;     // 50 packets/sec by default
const uint8_t  MAX_IMU_PER_PKT   = 32;
const uint16_t MAX_AUDIO_PER_PKT = 160;

// Wire-format packet:
//   PacketHeader (14 bytes)
//   n_imu   * ImuSample (12 bytes each: ax,ay,az,gx,gy,gz int16 LE)
//   n_audio * uint8 (top 8 bits of the 12-bit ADC reading)
// Per-sample timestamps are NOT sent — both IMU and audio are sampled on a
// precise hardware timer, so the receiver reconstructs sample i's time as
// (packet epoch) + i * 1000 / rate_hz. This keeps packets small.
#pragma pack(push, 1)
struct PacketHeader {
  uint32_t epoch_s;       // unix seconds (UTC) at the start of this packet
  uint16_t epoch_ms;      // ms fraction, 0-999
  uint16_t seq;           // rolling packet sequence number
  uint8_t  n_imu;
  uint8_t  n_audio;
  uint16_t imu_rate_hz;
  uint16_t audio_rate_hz;
};
struct ImuSample { int16_t ax, ay, az, gx, gy, gz; };
#pragma pack(pop)

uint8_t pktBuf[sizeof(PacketHeader) + MAX_IMU_PER_PKT * sizeof(ImuSample) + MAX_AUDIO_PER_PKT];

// ---------------------------------------------------------------------------
// BLE UUIDs (custom 128-bit; reusing the well-known Nordic UART base pattern
// so the UUIDs are easy to eyeball as "our" service in a sniffer/app)
// ---------------------------------------------------------------------------
#define SVC_UUID       "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHR_DATA_UUID  "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // notify: sensor packets
#define CHR_TIME_UUID  "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // write : 8-byte epoch ms (LE)
#define CHR_CTRL_UUID  "6e400004-b5a3-f393-e0a9-e50e24dcca9e"  // write : commands (see below)
#define CHR_STAT_UUID  "6e400005-b5a3-f393-e0a9-e50e24dcca9e"  // notify/read: status text

// Control characteristic commands:
//   [0x01]                                -> start streaming
//   [0x00]                                -> stop streaming
//   [0x02, rateLo, rateHi, intervalMs]    -> set audio rate (u16 LE, Hz) and
//                                            packet interval (u8, ms)

NimBLECharacteristic *chData, *chTime, *chCtrl, *chStat;
NimBLEServer *bleServer;
volatile bool deviceConnected = false;

// ---------------------------------------------------------------------------
// Time sync
// ---------------------------------------------------------------------------
volatile uint64_t epochAtSyncMs   = 0;
volatile uint32_t localMillisAtSync = 0;
volatile bool     timeSynced = false;

uint64_t currentEpochMs() {
  if (!timeSynced) return 0;
  // Subtraction wraps correctly even across millis() overflow (~49 days)
  // as long as sync happened less than ~49 days ago.
  uint32_t elapsed = millis() - localMillisAtSync;
  return epochAtSyncMs + (uint64_t)elapsed;
}

// ---------------------------------------------------------------------------
// Streaming state
// ---------------------------------------------------------------------------
volatile bool streaming = false;
uint16_t seqCounter = 0;

// ---------------------------------------------------------------------------
// esp_timer sample "tick" counters — kept minimal so the timer callback
// (which runs in the esp_timer service task, not a true hardware ISR, but
// still time-critical) does almost no work.
// ---------------------------------------------------------------------------
esp_timer_handle_t imuTimer = nullptr;
esp_timer_handle_t micTimer = nullptr;
volatile uint32_t imuTicks = 0, micTicks = 0;
volatile uint32_t droppedImu = 0, droppedAudio = 0;

void IRAM_ATTR onImuTimer(void *arg) { imuTicks++; }
void IRAM_ATTR onMicTimer(void *arg) { micTicks++; }

// ---------------------------------------------------------------------------
// Ring buffers between acquisition and BLE transmission
// ---------------------------------------------------------------------------
#define IMU_BUF_LEN 64
#define AUD_BUF_LEN 512

ImuRaw   imuBuf[IMU_BUF_LEN];
volatile uint16_t imuHead = 0, imuTail = 0;

uint16_t audBuf[AUD_BUF_LEN];
volatile uint16_t audHead = 0, audTail = 0;

uint16_t ringCount(uint16_t head, uint16_t tail, uint16_t len) {
  return (head >= tail) ? (head - tail) : (len - tail + head);
}

void pushImu(const ImuRaw &s) {
  uint16_t next = (imuHead + 1) % IMU_BUF_LEN;
  if (next == imuTail) { imuTail = (imuTail + 1) % IMU_BUF_LEN; droppedImu++; }
  imuBuf[imuHead] = s;
  imuHead = next;
}
ImuRaw popImu() {
  ImuRaw s = imuBuf[imuTail];
  imuTail = (imuTail + 1) % IMU_BUF_LEN;
  return s;
}
void pushAudio(uint16_t v) {
  uint16_t next = (audHead + 1) % AUD_BUF_LEN;
  if (next == audTail) { audTail = (audTail + 1) % AUD_BUF_LEN; droppedAudio++; }
  audBuf[audHead] = v;
  audHead = next;
}
uint16_t popAudio() {
  uint16_t v = audBuf[audTail];
  audTail = (audTail + 1) % AUD_BUF_LEN;
  return v;
}

// ---------------------------------------------------------------------------
// Mic rate control (stop/restart the periodic timer with a new period)
// ---------------------------------------------------------------------------
void setMicRate(uint16_t hz) {
  hz = constrain(hz, (uint16_t)200, (uint16_t)4000);
  esp_timer_stop(micTimer);
  micRateHz = hz;
  micPeriodUs = 1000000UL / hz;
  esp_timer_start_periodic(micTimer, micPeriodUs);
}

// ---------------------------------------------------------------------------
// BLE callbacks
// ---------------------------------------------------------------------------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *pServer) override {
    deviceConnected = true;
    // Favor a shorter connection interval while connected for lower latency;
    // NimBLE will negotiate within what the central allows.
    pServer->updateConnParams(pServer->getPeerInfo(0).getConnHandle(), 12, 24, 0, 400);
    Serial.println("[BLE] connected");
  }
  void onDisconnect(NimBLEServer *pServer) override {
    deviceConnected = false;
    streaming = false;
    timeSynced = false;
    Serial.println("[BLE] disconnected, re-advertising");
    NimBLEDevice::getAdvertising()->start();
  }
};

class TimeCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) override {
    std::string v = c->getValue();
    if (v.size() >= 8) {
      uint64_t epoch_ms;
      memcpy(&epoch_ms, v.data(), 8);
      epochAtSyncMs = epoch_ms;
      localMillisAtSync = millis();
      timeSynced = true;
      Serial.printf("[TIME] synced to epoch_ms=%llu\n", (unsigned long long)epoch_ms);
    }
  }
};

class CtrlCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) override {
    std::string v = c->getValue();
    if (v.empty()) return;
    uint8_t cmd = (uint8_t)v[0];
    if (cmd == 0x01) {
      streaming = true;
      seqCounter = 0;
      droppedImu = droppedAudio = 0;
      imuHead = imuTail = 0;
      audHead = audTail = 0;
      Serial.println("[CTRL] start streaming");
    } else if (cmd == 0x00) {
      streaming = false;
      Serial.println("[CTRL] stop streaming");
    } else if (cmd == 0x02 && v.size() >= 4) {
      uint16_t newRate = (uint8_t)v[1] | ((uint8_t)v[2] << 8);
      uint8_t  newInterval = (uint8_t)v[3];
      setMicRate(newRate);
      packetIntervalMs = constrain(newInterval, (uint8_t)10, (uint8_t)100);
      Serial.printf("[CTRL] audio_rate=%u packet_interval_ms=%u\n", micRateHz, packetIntervalMs);
    }
  }
};

// ---------------------------------------------------------------------------
// Packet builder / sender
// ---------------------------------------------------------------------------
void sendPacket() {
  uint16_t availImu = ringCount(imuHead, imuTail, IMU_BUF_LEN);
  uint16_t availAud = ringCount(audHead, audTail, AUD_BUF_LEN);
  uint8_t  nImu = (uint8_t)min((uint16_t)availImu, (uint16_t)MAX_IMU_PER_PKT);
  uint16_t nAud = min(availAud, MAX_AUDIO_PER_PKT);
  if (nImu == 0 && nAud == 0) return;

  uint64_t nowEpochMs = currentEpochMs();

  PacketHeader hdr;
  hdr.epoch_s      = (uint32_t)(nowEpochMs / 1000);
  hdr.epoch_ms     = (uint16_t)(nowEpochMs % 1000);
  hdr.seq          = seqCounter++;
  hdr.n_imu        = nImu;
  hdr.n_audio      = (uint8_t)nAud;
  hdr.imu_rate_hz  = IMU_RATE_HZ;
  hdr.audio_rate_hz = micRateHz;

  size_t off = 0;
  memcpy(pktBuf + off, &hdr, sizeof(hdr)); off += sizeof(hdr);

  for (uint8_t i = 0; i < nImu; i++) {
    ImuRaw r = popImu();
    ImuSample s{ r.ax, r.ay, r.az, r.gx, r.gy, r.gz };
    memcpy(pktBuf + off, &s, sizeof(s)); off += sizeof(s);
  }
  for (uint16_t i = 0; i < nAud; i++) {
    uint16_t raw = popAudio();
    pktBuf[off++] = (uint8_t)(raw >> 4);  // 12-bit -> 8-bit, halves bandwidth
  }

  chData->setValue(pktBuf, off);
  chData->notify();
}

void sendStatus() {
  static uint32_t lastStat = 0;
  uint32_t now = millis();
  if (now - lastStat < 1000) return;
  lastStat = now;
  if (!deviceConnected) return;
  char buf[160];
  snprintf(buf, sizeof(buf),
           "synced=%d;streaming=%d;dropped_imu=%lu;dropped_audio=%lu;"
           "imu_rate=%u;audio_rate=%u;pkt_ms=%u;uptime_s=%lu",
           timeSynced ? 1 : 0, streaming ? 1 : 0,
           (unsigned long)droppedImu, (unsigned long)droppedAudio,
           IMU_RATE_HZ, micRateHz, packetIntervalMs, (unsigned long)(now / 1000));
  chStat->setValue((uint8_t *)buf, strlen(buf));
  chStat->notify();
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_OFF);   // BLE-only: saves power, avoids WiFi/BT coexistence hits

  pinMode(PIN_MIC, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MIC, ADC_11db);  // ~0-3.3V full scale

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  mpuInit();

  NimBLEDevice::init("HeartSense-XIAO");
  NimBLEDevice::setMTU(247);                    // request a larger ATT MTU
  NimBLEDevice::setPower(ESP_PWR_LVL_N0);        // moderate TX power; raise/lower for range vs battery

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  NimBLEService *svc = bleServer->createService(SVC_UUID);
  chData = svc->createCharacteristic(CHR_DATA_UUID, NIMBLE_PROPERTY::NOTIFY);
  chTime = svc->createCharacteristic(CHR_TIME_UUID, NIMBLE_PROPERTY::WRITE);
  chCtrl = svc->createCharacteristic(CHR_CTRL_UUID, NIMBLE_PROPERTY::WRITE);
  chStat = svc->createCharacteristic(CHR_STAT_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  chTime->setCallbacks(new TimeCallback());
  chCtrl->setCallbacks(new CtrlCallback());

  svc->start();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  adv->start();

  esp_timer_create_args_t imuArgs = { .callback = &onImuTimer, .arg = nullptr,
                                       .dispatch_method = ESP_TIMER_TASK, .name = "imu_tick" };
  esp_timer_create(&imuArgs, &imuTimer);
  esp_timer_start_periodic(imuTimer, IMU_PERIOD_US);

  esp_timer_create_args_t micArgs = { .callback = &onMicTimer, .arg = nullptr,
                                       .dispatch_method = ESP_TIMER_TASK, .name = "mic_tick" };
  esp_timer_create(&micArgs, &micTimer);
  esp_timer_start_periodic(micTimer, micPeriodUs);

  Serial.println("[BOOT] HeartSense ready, advertising as 'HeartSense-XIAO'");
}

void loop() {
  // Drain acquisition ticks as fast as possible; each read is a few hundred
  // microseconds (I2C) or a few microseconds (ADC), so this easily keeps up
  // with 200 Hz IMU + up to 4 kHz audio.
  while (imuTicks > 0) {
    imuTicks--;
    ImuRaw s;
    if (mpuReadRaw(s)) pushImu(s);
  }
  while (micTicks > 0) {
    micTicks--;
    pushAudio((uint16_t)analogRead(PIN_MIC));
  }

  static uint32_t lastPacketMs = 0;
  uint32_t now = millis();
  if (streaming && deviceConnected && (now - lastPacketMs >= packetIntervalMs)) {
    lastPacketMs = now;
    sendPacket();
  }

  sendStatus();
}
