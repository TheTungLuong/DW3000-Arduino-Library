#include "DW3000.h"

static int rx_status;
static uint32_t frame_counter = 0;

const uint16_t CIR_TOTAL_SAMPLES = 256;      // Number of complex CIR samples per frame to stream
const uint16_t CIR_CHUNK_SAMPLES = 32;       // How many samples to read per SPI burst
const uint16_t CIR_FIRST_SAMPLE_OFFSET = 0;  // Offset inside the accumulator (in samples)

void streamCIR(uint32_t frameIndex);

void setup() {
  Serial.begin(115200);
  DW3000.begin();
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

  while (!(rx_status = DW3000.receivedFrameSucc())) {
    yield();
  }

  if (rx_status == 1) {
    DW3000.pullLEDHigh(1);

    Serial.print(F("[INFO] Received frame #"));
    Serial.println(frame_counter);

    streamCIR(frame_counter);
    frame_counter++;

    DW3000.clearSystemStatus();
    DW3000.standardRX();

    DW3000.pullLEDLow(1);
  } else {
    Serial.println(F("[ERROR] Receiver Error occured! Aborting event."));
    DW3000.clearSystemStatus();
    DW3000.standardRX();
  }
}

void streamCIR(uint32_t frameIndex) {
  DW3000CIRSample samples[CIR_CHUNK_SAMPLES];

  Serial.print(F("CIR_BEGIN,"));
  Serial.println(frameIndex);

  for (uint16_t sampleBase = 0; sampleBase < CIR_TOTAL_SAMPLES; sampleBase += CIR_CHUNK_SAMPLES) {
    uint16_t samplesThisChunk = CIR_CHUNK_SAMPLES;
    if (sampleBase + samplesThisChunk > CIR_TOTAL_SAMPLES) {
      samplesThisChunk = CIR_TOTAL_SAMPLES - sampleBase;
    }

    size_t readCount = DW3000.readCIRSamples(CIR_FIRST_SAMPLE_OFFSET + sampleBase, samples, samplesThisChunk);
    if (readCount == 0) {
      Serial.println(F("[ERROR] CIR read returned no samples."));
      break;
    }

    for (size_t i = 0; i < readCount; i++) {
      int16_t realPart = samples[i].i;
      int16_t imagPart = samples[i].q;
      uint32_t magnitudeSq = (int32_t)realPart * realPart + (int32_t)imagPart * imagPart;

      Serial.print(F("CIR,"));
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

  Serial.println(F("CIR_END"));
}
