#include "DW3000.h"
#include <math.h>

static int rx_status;
static uint32_t frame_counter = 0;

const uint16_t CIR_TOTAL_SAMPLES = 256;      // Number of complex CIR samples per frame to stream
const uint16_t CIR_CHUNK_SAMPLES = 32;       // How many samples to read per SPI burst
const uint16_t CIR_FIRST_SAMPLE_OFFSET = 0;  // Offset inside the accumulator (in samples)
const uint8_t CIR_BYTES_PER_SAMPLE = 4;      // 16 bit I + 16 bit Q
const uint16_t RX_WAIT_TIMEOUT_MS = 500;

// Shared UWB configuration (must match the transmitter)
const uint8_t UWB_CHANNEL = CHANNEL_5;
const uint8_t UWB_PREAMBLE = PREAMBLE_128;
const uint8_t UWB_PREAMBLE_CODE = 9;
const uint8_t UWB_PAC = PAC8;
const uint8_t UWB_DATARATE = DATARATE_6_8MB;
const uint8_t UWB_PHR_MODE = PHR_MODE_STANDARD;
const uint8_t UWB_PHR_RATE = PHR_RATE_850KB;

void streamCIR(uint32_t frameIndex);
void configureUwbCommon();
bool waitForRx();

void setup() {
  Serial.begin(115200);
  DW3000.begin();
  configureUwbCommon();
  DW3000.hardReset();
  delay(200);

  if (!DW3000.checkSPI()) {
    Serial.println(F("[ERROR] Could not establish SPI Connection to DW3000! Please make sure that all pins are set correctly."));
    while (1) {}
  }

  while (!DW3000.checkForIDLE()) {
    Serial.println(F("[ERROR] IDLE1 FAILED\r"));
    delay(1000);
  }

  DW3000.softReset();
  delay(200);

  if (!DW3000.checkForIDLE()) {
    Serial.println(F("[ERROR] IDLE2 FAILED\r"));
    while (1) {}
  }

  DW3000.init();
  DW3000.setupGPIO();
  DW3000.clearSystemStatus();

  Serial.println(F("[INFO] CIR RX Stream setup complete."));
  Serial.print(F("[INFO] Streaming "));
  Serial.print(CIR_TOTAL_SAMPLES);
  Serial.println(F(" CIR samples per received frame."));
}

void loop() {
  DW3000.standardRX();

  if (!waitForRx()) {
    Serial.println(F("RX: frame receive timeout"));
    DW3000.clearSystemStatus();
    return;
  }

  if (rx_status == 1) {
    DW3000.pullLEDHigh(1);

    Serial.print(F("RX: frame received OK, frame_id = "));
    Serial.println(frame_counter);

    streamCIR(frame_counter);
    frame_counter++;

    DW3000.clearSystemStatus();
    DW3000.pullLEDLow(1);
  } else {
    Serial.print(F("RX: frame receive ERROR, code = "));
    Serial.println(rx_status);
    DW3000.clearSystemStatus();
  }
}

void configureUwbCommon() {
  DW3000.setChannel(UWB_CHANNEL);
  DW3000.setPreambleLength(UWB_PREAMBLE);
  DW3000.setPreambleCode(UWB_PREAMBLE_CODE);
  DW3000.setPACSize(UWB_PAC);
  DW3000.setDatarate(UWB_DATARATE);
  DW3000.setPHRMode(UWB_PHR_MODE);
  DW3000.setPHRRate(UWB_PHR_RATE);
}

bool waitForRx() {
  unsigned long startMs = millis();
  rx_status = 0;
  while ((millis() - startMs) < RX_WAIT_TIMEOUT_MS) {
    rx_status = DW3000.receivedFrameSucc();
    if (rx_status != 0) {
      return true;
    }
    yield();
  }
  return false;
}

void streamCIR(uint32_t frameIndex) {
  uint8_t raw[CIR_CHUNK_SAMPLES * CIR_BYTES_PER_SAMPLE];

  Serial.print(F("CIR_BEGIN,"));
  Serial.println(frameIndex);

  for (uint16_t sampleBase = 0; sampleBase < CIR_TOTAL_SAMPLES; sampleBase += CIR_CHUNK_SAMPLES) {
    uint16_t samplesThisChunk = CIR_CHUNK_SAMPLES;
    if (sampleBase + samplesThisChunk > CIR_TOTAL_SAMPLES) {
      samplesThisChunk = CIR_TOTAL_SAMPLES - sampleBase;
    }

    size_t bytesToRead = (size_t)samplesThisChunk * CIR_BYTES_PER_SAMPLE;
    uint16_t byteOffset = (CIR_FIRST_SAMPLE_OFFSET + sampleBase) * CIR_BYTES_PER_SAMPLE;

    DW3000.readBytes(ACC_MEM_REG, byteOffset, raw, bytesToRead);

    for (uint16_t i = 0; i < samplesThisChunk; i++) {
      uint16_t rawIndex = i * CIR_BYTES_PER_SAMPLE;
      int16_t realPart = (int16_t)((raw[rawIndex + 1] << 8) | raw[rawIndex]);
      int16_t imagPart = (int16_t)((raw[rawIndex + 3] << 8) | raw[rawIndex + 2]);
      float magnitude = sqrt((float)realPart * (float)realPart + (float)imagPart * (float)imagPart);
      uint16_t sampleIndex = CIR_FIRST_SAMPLE_OFFSET + sampleBase + i;

      Serial.print(F("CIR,"));
      Serial.print(frameIndex);
      Serial.print(',');
      Serial.print(sampleIndex);
      Serial.print(',');
      Serial.print(realPart);
      Serial.print(',');
      Serial.print(imagPart);
      Serial.print(',');
      Serial.println(magnitude, 6);
    }
  }

  Serial.print(F("CIR_END,"));
  Serial.println(frameIndex);
}
