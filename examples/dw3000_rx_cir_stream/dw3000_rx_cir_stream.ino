#include "DW3000.h"

static int rx_status;
static uint32_t frame_counter = 0;

const uint16_t CIR_TOTAL_SAMPLES = 256;      // Number of complex CIR samples per frame to stream
const uint16_t CIR_CHUNK_SAMPLES = 32;       // How many samples to read per SPI burst
const uint16_t CIR_FIRST_SAMPLE_OFFSET = 0;  // Offset inside the accumulator (in samples)
const uint8_t CIR_BYTES_PER_SAMPLE = 4;      // 16 bit I + 16 bit Q

void streamCIR(uint32_t frameIndex);

void setup() {
  Serial.begin(115200);
  DW3000.begin();
  DW3000.hardReset();
  delay(200);

  if (!DW3000.checkSPI()) {
    Serial.println("[ERROR] Could not establish SPI Connection to DW3000! Please make sure that all pins are set correctly.");
    while (1) {}
  }

  while (!DW3000.checkForIDLE()) {
    Serial.println("[ERROR] IDLE1 FAILED\r");
    delay(1000);
  }

  DW3000.softReset();
  delay(200);

  if (!DW3000.checkForIDLE()) {
    Serial.println("[ERROR] IDLE2 FAILED\r");
    while (1) {}
  }

  DW3000.init();
  DW3000.setupGPIO();
  Serial.println("[INFO] CIR RX Stream setup complete.");
  Serial.print("[INFO] Streaming ");
  Serial.print(CIR_TOTAL_SAMPLES);
  Serial.println(" CIR samples per received frame.");
}

void loop() {
  DW3000.standardRX();

  while (!(rx_status = DW3000.receivedFrameSucc())) {
  }

  if (rx_status == 1) {
    DW3000.pullLEDHigh(1);

    Serial.print("[INFO] Received frame #");
    Serial.println(frame_counter);

    streamCIR(frame_counter);
    frame_counter++;

    DW3000.clearSystemStatus();

    DW3000.pullLEDLow(1);
  } else {
    Serial.println("[ERROR] Receiver Error occured! Aborting event.");
    DW3000.clearSystemStatus();
  }
}

void streamCIR(uint32_t frameIndex) {
  uint8_t raw[CIR_CHUNK_SAMPLES * CIR_BYTES_PER_SAMPLE];

  Serial.print("CIR_BEGIN,");
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
      uint32_t magnitudeSq = (int32_t)realPart * realPart + (int32_t)imagPart * imagPart;

      Serial.print("CIR,");
      Serial.print(frameIndex);
      Serial.print(',');
      Serial.print((uint16_t)(CIR_FIRST_SAMPLE_OFFSET + sampleBase + i));
      Serial.print(',');
      Serial.print(realPart);
      Serial.print(',');
      Serial.print(imagPart);
      Serial.print(',');
      Serial.println(magnitudeSq);
    }
  }

  Serial.println("CIR_END");
}
