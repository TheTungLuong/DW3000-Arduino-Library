#include "DW3000.h"
#include <math.h>

const uint8_t PIN_SS = 10;    // DWS3000 CSn
const uint8_t PIN_IRQ = 2;    // DWS3000 IRQ
const uint8_t PIN_RST = 9;    // DWS3000 RSTn
const uint8_t PIN_WAKE = 7;   // Optional WAKEUP pin (active high)

const uint8_t UWB_CHANNEL = CHANNEL_5;
const uint8_t UWB_PREAMBLE = PREAMBLE_128;
const uint8_t UWB_PREAMBLE_CODE = 9;
const uint8_t UWB_PAC = PAC8;
const uint8_t UWB_DATARATE = DATARATE_6_8MB;
const uint8_t UWB_PHR_MODE = PHR_MODE_STANDARD;
const uint8_t UWB_PHR_RATE = PHR_RATE_850KB;

const uint16_t CIR_SAMPLE_COUNT = 256;    // Complex samples per frame to stream
const uint8_t CIR_BYTES_PER_SAMPLE = 4;   // 16-bit I + 16-bit Q
const uint16_t RX_WAIT_TIMEOUT_MS = 500;

int rxStatus = 0;

void configureUwbCommon() {
  DW3000.setChannel(UWB_CHANNEL);
  DW3000.setPreambleLength(UWB_PREAMBLE);
  DW3000.setPreambleCode(UWB_PREAMBLE_CODE);
  DW3000.setPACSize(UWB_PAC);
  DW3000.setDatarate(UWB_DATARATE);
  DW3000.setPHRMode(UWB_PHR_MODE);
  DW3000.setPHRRate(UWB_PHR_RATE);
}

bool waitForRxEvent() {
  unsigned long startMs = millis();
  rxStatus = 0;
  while ((millis() - startMs) < RX_WAIT_TIMEOUT_MS) {
    rxStatus = DW3000.receivedFrameSucc();
    if (rxStatus != 0) {
      return true;
    }
    yield();
  }
  return false;
}

void streamAccumulator() {
  // ACC_MEM is read-only. On each successful RX frame the DW3000 overwrites
  // the accumulator with new CIR data. We only burst-read it after RX OK;
  // we never write to ACC_MEM or try to "flush" it manually.
  Serial.println(F("FRAME_BEGIN"));

  uint8_t raw[4];
  for (uint16_t sampleIdx = 0; sampleIdx < CIR_SAMPLE_COUNT; sampleIdx++) {
    uint32_t byteOffset = (uint32_t)sampleIdx * CIR_BYTES_PER_SAMPLE;
    DW3000.readBytes(ACC_MEM_REG, byteOffset, raw, sizeof(raw));

    int16_t i = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
    int16_t q = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
    float mag = sqrtf((float)i * (float)i + (float)q * (float)q);

    // MATLAB parses one line per sample: idx,real,imag,mag
    Serial.print(sampleIdx);
    Serial.print(',');
    Serial.print(i);
    Serial.print(',');
    Serial.print(q);
    Serial.print(',');
    Serial.println(mag, 6);
  }

  Serial.println(F("FRAME_END"));
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_IRQ, INPUT_PULLUP);
  pinMode(PIN_WAKE, OUTPUT);
  digitalWrite(PIN_WAKE, HIGH);  // Keep the shield awake if connected

  DW3000.begin();
  configureUwbCommon();
  DW3000.hardReset();
  delay(200);

  if (!DW3000.checkSPI()) {
    Serial.println(F("[ERROR] Could not establish SPI connection to DW3000. Check wiring/pins."));
    while (1) {}
  }

  while (!DW3000.checkForIDLE()) {
    Serial.println(F("[ERROR] IDLE1 FAILED"));
    delay(1000);
  }

  DW3000.softReset();
  delay(200);

  if (!DW3000.checkForIDLE()) {
    Serial.println(F("[ERROR] IDLE2 FAILED"));
    while (1) {}
  }

  DW3000.init();
  DW3000.setupGPIO();
  DW3000.clearSystemStatus();

  Serial.println(F("[INFO] DW3000 CIR RX ready."));
}

void loop() {
  DW3000.standardRX();

  if (!waitForRxEvent()) {
    Serial.println(F("[WARN] RX timeout"));
    DW3000.clearSystemStatus();
    return;
  }

  uint32_t sysStatus = DW3000.read(GEN_CFG_AES_LOW_REG, 0x44);
  if (rxStatus == 1) {
    DW3000.pullLEDHigh(1);
    streamAccumulator();
    DW3000.pullLEDLow(1);
  } else {
    Serial.print(F("[ERROR] RX error, SYS_STATUS=0x"));
    Serial.println(sysStatus, HEX);
  }

  // Clear RX event/error bits before re-enabling receiver
  DW3000.clearSystemStatus();
}
