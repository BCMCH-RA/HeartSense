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
       between that epoch and its own millis() clock, so every future
       sample can be time-stamped in real UTC time without an RTC. IST
       (UTC+5:30) conversion is done on the receiving app side.
    3. Samples the MPU6050 at a hardware-timed 200 Hz and the microphone at
       a configurable rate (default 2000 Hz).
    4. Batches samples into small self-describing binary packets sent as BLE
       notifications every `packetIntervalMs` (default 20 ms -> 50 pkt/s).
    5. Streaming (and audio rate / packet interval) is controlled from the
       app via the CONTROL characteristic.

  Library required
    - "NimBLE-Arduino" by h2zero (Library Manager).
    - THIS FILE TARGETS NimBLE-Arduino v2.x, whose callbacks all take an
      extra NimBLEConnInfo& parameter compared to v1.4.x, and whose
      getValue() returns a NimBLEAttValue rather than std::string.
      If you're on v1.4.x instead, drop the NimBLEConnInfo& parameters
      from onConnect/onDisconnect/onWrite below and change getValue()
      usage back to std::string.

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
// Shared type definitions — kept at the very top of the file, right after
// includes/pin defines and before ANY function. Arduino's IDE auto-generates
// forward prototypes for every function and inserts them near the top of the
// translation unit; if a struct used as a parameter/return type is defined
// further down (even just above the function that uses it), the inserted
// prototype ends up referencing an undeclared type and the build fails with
// "'StructName' was not declared in this scope". Defining structs first
// avoids that entirely.
// ---------------------------------------------------------------------------
struct ImuRaw { int16_t ax, ay, az, gx, gy, gz; };

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
// Microphone (analog electret via MAX4466) — rate is adjustable at runtime.
// Each output sample is the average of AUDIO_OVERSAMPLE back-to-back ADC
// reads, which knocks down the ESP32 ADC's inherent noise floor by roughly
// sqrt(AUDIO_OVERSAMPLE) with negligible time cost (each read is a few
// microseconds).
// ---------------------------------------------------------------------------
volatile uint16_t micRateHz  = 2000;
volatile uint32_t micPeriodUs = 1000000UL / 2000;
const uint8_t AUDIO_OVERSAMPLE = 4;

// ---------------------------------------------------------------------------
// Packetization
// ---------------------------------------------------------------------------
volatile uint8_t packetIntervalMs = 20;     // 50 packets/sec by default
const uint8_t  MAX_IMU_PER_PKT   = 32;
const uint16_t MAX_AUDIO_PER_PKT = 160;

// Wire-format packet:
//   PacketHeader (14 bytes)
//   n_imu   * ImuSample (12 bytes each: ax,ay,az,gx,gy,gz int16 LE)
//   n_audio * uint16 LE (full 12-bit ADC reading, 0-4095)
// Per-sample timestamps are NOT sent — both IMU and audio are sampled on a
// precise hardware timer, so the receiver reconstructs sample i's time as
// (packet epoch) + i * 1000 / rate_hz.
// Audio used to be truncated to 8 bits to save bandwidth, but that throws
// away 4 bits of resolution (256 vs 4096 quantization levels) which shows
// up as audible hiss on a low-amplitude signal like heart sounds. There's
// enough BLE throughput headroom at these sample rates to send full
// resolution instead -- e.g. 4 IMU + 40 audio samples per 20ms packet is
// 14 + 48 + 80 = 142 bytes, still comfortably under a 247-byte MTU.
uint8_t pktBuf[sizeof(PacketHeader) + MAX_IMU_PER_PKT * sizeof(ImuSample) + MAX_AUDIO_PER_PKT * sizeof(uint16_t)];

// ---------------------------------------------------------------------------
// BLE UUIDs
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
  uint32_t elapsed = millis() - localMillisAtSync;  // wraps correctly (~49 days)
  return epochAtSyncMs + (uint64_t)elapsed;
}

// ---------------------------------------------------------------------------
// Streaming state
// ---------------------------------------------------------------------------
volatile bool streaming = false;
uint16_t seqCounter = 0;

// ---------------------------------------------------------------------------
// esp_timer sample "tick" counters
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
// Mic rate control
// ---------------------------------------------------------------------------
void setMicRate(uint16_t hz) {
  hz = constrain(hz, (uint16_t)200, (uint16_t)4000);
  esp_timer_stop(micTimer);
  micRateHz = hz;
  micPeriodUs = 1000000UL / hz;
  esp_timer_start_periodic(micTimer, micPeriodUs);
}

// ---------------------------------------------------------------------------
// BLE callbacks (NimBLE-Arduino v2.x signatures — note the NimBLEConnInfo&)
// ---------------------------------------------------------------------------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override {
    deviceConnected = true;
    pServer->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 400);
    Serial.println("[BLE] connected");
  }
  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override {
    deviceConnected = false;
    streaming = false;
    timeSynced = false;
    Serial.println("[BLE] disconnected, re-advertising");
    NimBLEDevice::getAdvertising()->start();
  }
};

class TimeCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &connInfo) override {
    NimBLEAttValue v = c->getValue();
    if (v.length() >= 8) {
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
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &connInfo) override {
    NimBLEAttValue v = c->getValue();
    if (v.length() == 0) return;
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
    } else if (cmd == 0x02 && v.length() >= 4) {
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
    memcpy(pktBuf + off, &raw, sizeof(raw)); off += sizeof(raw);  // full 12-bit value, LE
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

  // A 128-bit UUID (18 bytes incl. AD header) + flags (3 bytes) already uses
  // 21 of the 31 bytes in a legacy advertising packet. Cramming the full
  // device name into the SAME packet overflows it -- NimBLE then either
  // truncates data or fails to advertise at all, which looks exactly like
  // "the device never shows up in the browser's pairing dialog". Fix: keep
  // the main advertising packet to just flags + service UUID (all the
  // browser's requestDevice({filters:[{services:[...]}]}) actually needs
  // to find it), and put the human-readable name in the separate
  // scan-response packet, which has its own independent 31-byte budget.
  NimBLEAdvertisementData scanResponse;
  scanResponse.setName("HeartSense");
  adv->setScanResponseData(scanResponse);

  adv->start();
  Serial.printf("[BLE] advertising as MAC=%s\n", NimBLEDevice::getAddress().toString().c_str());

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
  while (imuTicks > 0) {
    imuTicks--;
    ImuRaw s;
    if (mpuReadRaw(s)) pushImu(s);
  }
  while (micTicks > 0) {
    micTicks--;
    uint32_t acc = 0;
    for (uint8_t k = 0; k < AUDIO_OVERSAMPLE; k++) acc += analogRead(PIN_MIC);
    pushAudio((uint16_t)(acc / AUDIO_OVERSAMPLE));
  }

  static uint32_t lastPacketMs = 0;
  uint32_t now = millis();
  if (streaming && deviceConnected && (now - lastPacketMs >= packetIntervalMs)) {
    lastPacketMs = now;
    sendPacket();
  }

  sendStatus();
}
