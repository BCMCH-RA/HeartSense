/*
  ============================================================================
  HeartSense — ESP32-C3 SuperMini Firmware (revised)
  ============================================================================
  Hardware
    - ESP32-C3 SuperMini ("nologo")
    - MPU6050 IMU on I2C         : SDA = GPIO8, SCL = GPIO9   (EDIT BELOW)
    - MAX4466 mic module OUT     : GPIO4 (ADC1_CH4)           (EDIT BELOW)
        The mic MUST be on GPIO0..GPIO4 (ADC1). GPIO5 is ADC2, which does not
        work reliably while the BLE radio is active.

  Arduino IDE settings
    - Board package : esp32 by Espressif, version 3.x  (needs IDF 5.x because
                      this file uses the esp_adc/adc_continuous driver)
    - Board         : "ESP32C3 Dev Module"
    - USB CDC On Boot : Enabled   (so Serial prints appear over the USB port)
    - Library       : NimBLE-Arduino v2.x by h2zero

  What changed compared with the XIAO ESP32S3 version
    1. Microphone is now sampled by the ADC *continuous (DMA) driver* at a
       fixed 20 kHz and decimated in software by averaging down to the
       requested audio rate (default 2000 Hz). Previously analogRead() was
       polled from loop(), so sample timing wandered whenever BLE or I2C kept
       the loop busy. Averaging ~10 raw conversions per output sample also acts
       as a crude anti-alias filter and cuts ADC noise.
    2. IMU and microphone run in their own FreeRTOS tasks. loop() only builds
       and sends BLE packets.
    3. Every sample is time-stamped when it is ACQUIRED (esp_timer, microsecond
       clock), not when the packet is sent. The packet header carries the time
       of the oldest sample in the packet, so IMU/mic alignment error is now
       at most one IMU period (5 ms) instead of tens of milliseconds.
    4. Packet size is limited to the negotiated ATT MTU. The old code could
       build packets of up to 718 bytes, which silently fail on a 247-byte MTU.
    5. If a backlog builds up (e.g. radio busy), up to 3 packets are sent per
       pass to catch up.
    6. BLE TX power is set in dBm (NimBLE-Arduino v2 takes dBm; the old
       ESP_PWR_LVL_N0 constant is a v1-style enum and may have meant +4 dBm).
       Lower TX power = smaller supply current spikes next to the mic amp.
    7. The negotiated BLE connection interval is printed on the serial port
       and included in the status string (conn_ms=...). Use it to check
       whether the 45 ms interference period equals the connection interval.
    8. WiFi include removed (BLE-only firmware).

  The wire format of the data packet is UNCHANGED, so the existing web app
  should keep working:
      PacketHeader (14 bytes)
      n_imu   * ImuSample (12 bytes: ax,ay,az,gx,gy,gz int16 LE)
      n_audio * uint16 LE (12-bit ADC value 0..4095)

  Hardware fixes that go with this firmware (software cannot remove them):
    - 100-470 uF + 0.1 uF across 3V3/GND close to the ESP32-C3.
    - MAX4466 VCC through ~10 ohm (or a ferrite bead) with 47-100 uF + 0.1 uF
      right at the MAX4466 VCC/GND pins.
    - Turn the MAX4466 gain pot down until the signal no longer clips.
    - Keep the mic module and its OUT wire/trace away from the antenna and
      from the LiPo/regulator switching area.
  ============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h"

#if configTICK_RATE_HZ < 1000
#warning "FreeRTOS tick is below 1000 Hz; IMU task timing will be coarse."
#endif

// ---------------------------------------------------------------------------
// Pin map  — EDIT TO MATCH YOUR WIRING
// ---------------------------------------------------------------------------
#define PIN_SDA   8
#define PIN_SCL   9
#define PIN_MIC   4       // must be GPIO0..GPIO4 (ADC1) on the ESP32-C3

// BLE radio settings
#define BLE_TX_DBM          (-9)   // dBm. Phone is close; raise (0, +3, +9) only if range is poor.
#define CONN_MIN_UNITS      12     // x1.25 ms  -> 15 ms   (a request; the phone decides)
#define CONN_MAX_UNITS      24     // x1.25 ms  -> 30 ms

// ---------------------------------------------------------------------------
// Shared type definitions (kept above all functions for the Arduino IDE
// prototype generator)
// ---------------------------------------------------------------------------
struct ImuRaw { int16_t ax, ay, az, gx, gy, gz; };

#pragma pack(push, 1)
struct PacketHeader {
  uint32_t epoch_s;       // unix seconds (UTC) of the oldest sample in this packet
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
// MPU6050 raw register driver
// ---------------------------------------------------------------------------
#define MPU_ADDR          0x68
#define REG_PWR_MGMT_1    0x6B
#define REG_CONFIG        0x1A   // DLPF
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
//   Accel ±2g     -> 16384 LSB/g
//   Gyro  ±250dps -> 131 LSB/(deg/s)

const uint16_t IMU_RATE_HZ = 200;

void mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuInit() {
  mpuWriteReg(REG_PWR_MGMT_1, 0x80); delay(50);  // reset
  mpuWriteReg(REG_PWR_MGMT_1, 0x01);             // wake, gyro-X clock ref
  mpuWriteReg(REG_CONFIG, 0x03);                 // DLPF ~44 Hz accel / 42 Hz gyro
  mpuWriteReg(REG_GYRO_CONFIG, 0x00);            // ±250 dps
  mpuWriteReg(REG_ACCEL_CONFIG, 0x00);           // ±2 g
}

bool mpuReadRaw(ImuRaw &s) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14, (int)true) != 14) return false;
  s.ax = (Wire.read() << 8) | Wire.read();
  s.ay = (Wire.read() << 8) | Wire.read();
  s.az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                      // skip temperature
  s.gx = (Wire.read() << 8) | Wire.read();
  s.gy = (Wire.read() << 8) | Wire.read();
  s.gz = (Wire.read() << 8) | Wire.read();
  return true;
}

// ---------------------------------------------------------------------------
// Audio configuration
// ---------------------------------------------------------------------------
static const uint32_t ADC_FS_HZ       = 20000;   // raw ADC conversion rate
static const uint16_t ADC_FRAME_BYTES = 400;     // multiple of SOC_ADC_DIGI_RESULT_BYTES
volatile uint16_t micRateHz      = 2000;         // actual output rate (set by audio task)
volatile uint16_t pendingMicRate = 2000;         // requested output rate
adc_continuous_handle_t adcHandle = nullptr;
volatile uint32_t adcOverflows = 0, adcReadErr = 0;

// ---------------------------------------------------------------------------
// Packetization
// ---------------------------------------------------------------------------
volatile uint8_t packetIntervalMs = 20;          // 50 packets/sec by default
const uint8_t  MAX_IMU_PER_PKT   = 32;
const uint16_t MAX_AUDIO_PER_PKT = 160;
uint8_t pktBuf[sizeof(PacketHeader) + MAX_IMU_PER_PKT * sizeof(ImuSample) + MAX_AUDIO_PER_PKT * sizeof(uint16_t)];

// ---------------------------------------------------------------------------
// BLE UUIDs
// ---------------------------------------------------------------------------
#define SVC_UUID       "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHR_DATA_UUID  "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // notify: sensor packets
#define CHR_TIME_UUID  "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // write : 8-byte epoch ms (LE)
#define CHR_CTRL_UUID  "6e400004-b5a3-f393-e0a9-e50e24dcca9e"  // write : commands
#define CHR_STAT_UUID  "6e400005-b5a3-f393-e0a9-e50e24dcca9e"  // notify/read: status text

// Control characteristic commands:
//   [0x01]                              -> start streaming
//   [0x00]                              -> stop streaming
//   [0x02, rateLo, rateHi, intervalMs]  -> audio rate (u16 LE, Hz, 200..4000)
//                                          and packet interval (u8, ms, 10..100)

NimBLECharacteristic *chData, *chTime, *chCtrl, *chStat;
NimBLEServer *bleServer;
volatile bool     deviceConnected = false;
volatile uint16_t connHandle      = 0;
volatile uint16_t connIntervalUnits = 0;        // x1.25 ms
volatile uint32_t notifyFail = 0;

// ---------------------------------------------------------------------------
// Time sync (microsecond esp_timer clock <-> UNIX epoch)
// ---------------------------------------------------------------------------
volatile uint64_t epochAtSyncUs = 0;
volatile uint64_t localAtSyncUs = 0;
volatile bool     timeSynced    = false;

uint64_t epochUsFromLocal(uint64_t localUs) {
  if (!timeSynced) return 0;
  return epochAtSyncUs + (int64_t)(localUs - localAtSyncUs);
}

// ---------------------------------------------------------------------------
// Streaming state
// ---------------------------------------------------------------------------
volatile bool streaming = false;
uint16_t seqCounter = 0;

// ---------------------------------------------------------------------------
// Ring buffers (acquisition tasks -> loop). Every sample keeps its own
// acquisition timestamp (microseconds on the esp_timer clock).
// ---------------------------------------------------------------------------
#define IMU_BUF_LEN 128
#define AUD_BUF_LEN 1024

ImuRaw   imuBuf[IMU_BUF_LEN];
uint64_t imuT[IMU_BUF_LEN];
volatile uint16_t imuHead = 0, imuTail = 0;
volatile uint32_t droppedImu = 0;

uint16_t audBuf[AUD_BUF_LEN];
uint64_t audT[AUD_BUF_LEN];
volatile uint16_t audHead = 0, audTail = 0;
volatile uint32_t droppedAudio = 0;

portMUX_TYPE ringMux = portMUX_INITIALIZER_UNLOCKED;

static inline uint16_t ringCount(uint16_t head, uint16_t tail, uint16_t len) {
  return (head >= tail) ? (head - tail) : (len - tail + head);
}
static inline uint16_t u16min(uint16_t a, uint16_t b) { return a < b ? a : b; }

void pushImu(const ImuRaw &s, uint64_t tUs) {
  portENTER_CRITICAL(&ringMux);
  uint16_t next = (imuHead + 1) % IMU_BUF_LEN;
  if (next == imuTail) { imuTail = (imuTail + 1) % IMU_BUF_LEN; droppedImu++; }
  imuBuf[imuHead] = s;
  imuT[imuHead]   = tUs;
  imuHead = next;
  portEXIT_CRITICAL(&ringMux);
}

void pushAudio(uint16_t v, uint64_t tUs) {
  portENTER_CRITICAL(&ringMux);
  uint16_t next = (audHead + 1) % AUD_BUF_LEN;
  if (next == audTail) { audTail = (audTail + 1) % AUD_BUF_LEN; droppedAudio++; }
  audBuf[audHead] = v;
  audT[audHead]   = tUs;
  audHead = next;
  portEXIT_CRITICAL(&ringMux);
}

void clearRings() {
  portENTER_CRITICAL(&ringMux);
  imuHead = imuTail = 0;
  audHead = audTail = 0;
  portEXIT_CRITICAL(&ringMux);
}

// ---------------------------------------------------------------------------
// ADC continuous (DMA) microphone sampling
// ---------------------------------------------------------------------------
static bool IRAM_ATTR onAdcPoolOverflow(adc_continuous_handle_t handle,
                                        const adc_continuous_evt_data_t *edata,
                                        void *user_data) {
  adcOverflows++;
  return false;
}

bool adcInit() {
  adc_unit_t    unit;
  adc_channel_t channel;
  if (adc_continuous_io_to_channel(PIN_MIC, &unit, &channel) != ESP_OK || unit != ADC_UNIT_1) {
    Serial.println("[ADC] ERROR: PIN_MIC is not an ADC1 pin. Use GPIO0..GPIO4.");
    return false;
  }

  adc_continuous_handle_cfg_t hcfg = {};
  hcfg.max_store_buf_size = 8192;                // ~100 ms of raw data at 20 kHz
  hcfg.conv_frame_size    = ADC_FRAME_BYTES;
  if (adc_continuous_new_handle(&hcfg, &adcHandle) != ESP_OK) {
    Serial.println("[ADC] ERROR: adc_continuous_new_handle failed");
    return false;
  }

  adc_digi_pattern_config_t pat = {};
  pat.atten     = ADC_ATTEN_DB_11;               // alias of DB_12 on newer IDF; ~0-3.1 V
  pat.channel   = channel & 0x7;
  pat.unit      = unit;
  pat.bit_width = ADC_BITWIDTH_12;

  adc_continuous_config_t dcfg = {};
  dcfg.pattern_num    = 1;
  dcfg.adc_pattern    = &pat;
  dcfg.sample_freq_hz = ADC_FS_HZ;
  dcfg.conv_mode      = ADC_CONV_SINGLE_UNIT_1;
  dcfg.format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
  if (adc_continuous_config(adcHandle, &dcfg) != ESP_OK) {
    Serial.println("[ADC] ERROR: adc_continuous_config failed");
    return false;
  }

  adc_continuous_evt_cbs_t cbs = {};
  cbs.on_pool_ovf = onAdcPoolOverflow;
  adc_continuous_register_event_callbacks(adcHandle, &cbs, nullptr);
  return true;
}

// Reads DMA frames, averages raw conversions down to the output rate, and
// time-stamps each output sample from a running sample counter.
void audioTask(void *arg) {
  static uint8_t buf[ADC_FRAME_BYTES * 2];
  uint32_t got = 0;
  uint32_t acc = 0, cnt = 0, phase = 0;
  uint16_t rate = pendingMicRate;
  micRateHz = rate;
  uint64_t n = 0;                                 // output samples since t0
  uint64_t t0 = esp_timer_get_time();             // time of first conversion
  adc_continuous_start(adcHandle);

  for (;;) {
    if (pendingMicRate != rate) {                 // rate change: re-base time axis
      t0 += (n * 1000000ULL) / rate;
      n = 0; acc = 0; cnt = 0; phase = 0;
      rate = pendingMicRate;
      micRateHz = rate;
    }

    esp_err_t r = adc_continuous_read(adcHandle, buf, sizeof(buf), &got, 20);
    if (r == ESP_ERR_TIMEOUT) continue;
    if (r != ESP_OK) { adcReadErr++; continue; }

    for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= got; i += SOC_ADC_DIGI_RESULT_BYTES) {
      adc_digi_output_data_t *p = (adc_digi_output_data_t *)&buf[i];
      acc += p->type2.data;
      cnt++;
      phase += rate;
      if (phase >= ADC_FS_HZ) {
        phase -= ADC_FS_HZ;
        if (streaming) pushAudio((uint16_t)(acc / cnt), t0 + (n * 1000000ULL) / rate);
        n++;
        acc = 0; cnt = 0;
      }
    }
  }
}

void setMicRate(uint16_t hz) {
  pendingMicRate = constrain(hz, (uint16_t)200, (uint16_t)4000);
}

// ---------------------------------------------------------------------------
// IMU task (fixed 200 Hz via vTaskDelayUntil)
// ---------------------------------------------------------------------------
void imuTask(void *arg) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / IMU_RATE_HZ);
  for (;;) {
    vTaskDelayUntil(&last, period);
    if (!streaming) continue;
    uint64_t t = esp_timer_get_time();
    ImuRaw s;
    if (mpuReadRaw(s)) pushImu(s, t);
  }
}

// ---------------------------------------------------------------------------
// BLE callbacks (NimBLE-Arduino v2.x signatures)
// ---------------------------------------------------------------------------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override {
    deviceConnected = true;
    connHandle = connInfo.getConnHandle();
    connIntervalUnits = connInfo.getConnInterval();
    pServer->updateConnParams(connHandle, CONN_MIN_UNITS, CONN_MAX_UNITS, 0, 400);
    Serial.printf("[BLE] connected, interval=%.2f ms\n", connIntervalUnits * 1.25f);
  }
  // Called when the phone applies new connection parameters. (No 'override' on
  // purpose: if your NimBLE version names it differently this just never runs.)
  void onConnParamsUpdate(NimBLEConnInfo &connInfo) {
    connIntervalUnits = connInfo.getConnInterval();
    Serial.printf("[BLE] conn params updated, interval=%.2f ms\n", connIntervalUnits * 1.25f);
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
      localAtSyncUs = esp_timer_get_time();
      epochAtSyncUs = epoch_ms * 1000ULL;
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
      clearRings();
      seqCounter = 0;
      droppedImu = droppedAudio = 0;
      notifyFail = 0;
      streaming = true;
      Serial.println("[CTRL] start streaming");
    } else if (cmd == 0x00) {
      streaming = false;
      Serial.println("[CTRL] stop streaming");
    } else if (cmd == 0x02 && v.length() >= 4) {
      uint16_t newRate = (uint8_t)v[1] | ((uint8_t)v[2] << 8);
      uint8_t  newInterval = (uint8_t)v[3];
      setMicRate(newRate);
      packetIntervalMs = constrain(newInterval, (uint8_t)10, (uint8_t)100);
      Serial.printf("[CTRL] audio_rate=%u packet_interval_ms=%u\n", pendingMicRate, packetIntervalMs);
    }
  }
};

// ---------------------------------------------------------------------------
// Packet builder / sender. Returns true if a further backlog packet is waiting.
// ---------------------------------------------------------------------------
bool sendPacket() {
  uint16_t availImu, availAud;
  uint64_t tImu0 = 0, tAud0 = 0;
  portENTER_CRITICAL(&ringMux);
  availImu = ringCount(imuHead, imuTail, IMU_BUF_LEN);
  availAud = ringCount(audHead, audTail, AUD_BUF_LEN);
  if (availImu) tImu0 = imuT[imuTail];
  if (availAud) tAud0 = audT[audTail];
  portEXIT_CRITICAL(&ringMux);
  if (availImu == 0 && availAud == 0) return false;

  // Limit packet to the negotiated ATT payload (MTU - 3).
  uint16_t mtu = bleServer->getPeerMTU(connHandle);
  uint16_t payloadMax = (mtu > 3) ? (mtu - 3) : 20;
  if (payloadMax > sizeof(pktBuf)) payloadMax = sizeof(pktBuf);
  if (payloadMax < sizeof(PacketHeader) + 14) return false;   // MTU not negotiated yet

  uint16_t nImu = u16min(availImu, MAX_IMU_PER_PKT);
  uint16_t nAud = u16min(availAud, MAX_AUDIO_PER_PKT);
  uint32_t bodyMax = payloadMax - sizeof(PacketHeader);
  uint32_t need = (uint32_t)nImu * sizeof(ImuSample) + (uint32_t)nAud * 2;
  if (need > bodyMax) {                           // scale both streams down together
    nImu = (uint16_t)((uint32_t)nImu * bodyMax / need);
    nAud = (uint16_t)((uint32_t)nAud * bodyMax / need);
  }
  if (nImu == 0 && nAud == 0) return false;

  // Header time = acquisition time of the oldest sample going into this packet.
  uint64_t tFirst;
  if (nImu && nAud)      tFirst = (tImu0 < tAud0) ? tImu0 : tAud0;
  else if (nImu)         tFirst = tImu0;
  else                   tFirst = tAud0;
  uint64_t epochUs = epochUsFromLocal(tFirst);
  uint64_t epochMs = epochUs / 1000ULL;

  PacketHeader hdr;
  hdr.epoch_s       = (uint32_t)(epochMs / 1000ULL);
  hdr.epoch_ms      = (uint16_t)(epochMs % 1000ULL);
  hdr.seq           = seqCounter++;
  hdr.n_imu         = (uint8_t)nImu;
  hdr.n_audio       = (uint8_t)nAud;
  hdr.imu_rate_hz   = IMU_RATE_HZ;
  hdr.audio_rate_hz = micRateHz;

  size_t off = 0;
  memcpy(pktBuf + off, &hdr, sizeof(hdr)); off += sizeof(hdr);

  portENTER_CRITICAL(&ringMux);
  for (uint16_t i = 0; i < nImu; i++) {
    ImuRaw r = imuBuf[imuTail];
    imuTail = (imuTail + 1) % IMU_BUF_LEN;
    ImuSample s{ r.ax, r.ay, r.az, r.gx, r.gy, r.gz };
    memcpy(pktBuf + off, &s, sizeof(s)); off += sizeof(s);
  }
  for (uint16_t i = 0; i < nAud; i++) {
    uint16_t raw = audBuf[audTail];
    audTail = (audTail + 1) % AUD_BUF_LEN;
    memcpy(pktBuf + off, &raw, sizeof(raw)); off += sizeof(raw);
  }
  portEXIT_CRITICAL(&ringMux);

  chData->setValue(pktBuf, off);
  if (!chData->notify()) notifyFail++;

  uint16_t target = (uint16_t)((uint32_t)micRateHz * packetIntervalMs / 1000);
  return (availAud - nAud) >= target;
}

void sendStatus() {
  static uint32_t lastStat = 0;
  uint32_t now = millis();
  if (now - lastStat < 1000) return;
  lastStat = now;
  if (!deviceConnected) return;
  char buf[256];
  snprintf(buf, sizeof(buf),
           "synced=%d;streaming=%d;dropped_imu=%lu;dropped_audio=%lu;"
           "imu_rate=%u;audio_rate=%u;pkt_ms=%u;conn_ms=%.2f;mtu=%u;"
           "notify_fail=%lu;adc_ovf=%lu;adc_err=%lu;uptime_s=%lu",
           timeSynced ? 1 : 0, streaming ? 1 : 0,
           (unsigned long)droppedImu, (unsigned long)droppedAudio,
           IMU_RATE_HZ, micRateHz, packetIntervalMs,
           connIntervalUnits * 1.25f, (unsigned)bleServer->getPeerMTU(connHandle),
           (unsigned long)notifyFail, (unsigned long)adcOverflows,
           (unsigned long)adcReadErr, (unsigned long)(now / 1000));
  chStat->setValue((uint8_t *)buf, strlen(buf));
  chStat->notify();
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);                            // never hang on a stuck bus
  mpuInit();

  bool adcOk = adcInit();

  NimBLEDevice::init("HeartSense-C3");
  NimBLEDevice::setMTU(247);
  NimBLEDevice::setPower(BLE_TX_DBM);             // dBm (NimBLE-Arduino v2.x)

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

  // Main advertising packet: flags + 128-bit service UUID only. The readable
  // name goes in the scan response (separate 31-byte budget).
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  NimBLEAdvertisementData scanResponse;
  scanResponse.setName("HeartSense");
  adv->setScanResponseData(scanResponse);
  adv->start();
  Serial.printf("[BLE] advertising as MAC=%s\n", NimBLEDevice::getAddress().toString().c_str());

  if (adcOk) xTaskCreate(audioTask, "audio", 4096, nullptr, 5, nullptr);
  xTaskCreate(imuTask, "imu", 3072, nullptr, 4, nullptr);

  Serial.println("[BOOT] HeartSense ready, advertising as 'HeartSense'");
}

void loop() {
  static uint32_t lastPacketMs = 0;
  uint32_t now = millis();
  if (streaming && deviceConnected && (now - lastPacketMs >= packetIntervalMs)) {
    lastPacketMs = now;
    for (uint8_t k = 0; k < 3; k++) {             // catch up if a backlog built up
      if (!sendPacket()) break;
    }
  }
  sendStatus();
  delay(1);
}
