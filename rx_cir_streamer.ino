#include <SPI.h>
#include "DW3000.h"
#include <math.h>

// Chip select pin for the DW3000 shield when used with Arduino UNO class boards.
#define DW3000_SS_PIN 10

// CIR parameters
const uint16_t CIR_TOTAL_SAMPLES = 256;       // Number of complex CIR samples per frame to stream
const uint16_t CIR_FIRST_SAMPLE_INDEX = 0;    // Starting sample index inside ACC_MEM
const uint8_t CIR_BYTES_PER_SAMPLE = 4;       // 16-bit I + 16-bit Q

const uint16_t RX_WAIT_TIMEOUT_MS = 500;

// Shared UWB configuration (must match the transmitter)
const uint8_t UWB_CHANNEL = CHANNEL_5;
const uint8_t UWB_PREAMBLE = PREAMBLE_128;
const uint8_t UWB_PREAMBLE_CODE = 9;
const uint8_t UWB_PAC = PAC8;
const uint8_t UWB_DATARATE = DATARATE_6_8MB;
const uint8_t UWB_PHR_MODE = PHR_MODE_STANDARD;
const uint8_t UWB_PHR_RATE = PHR_RATE_850KB;

static int rx_status;

void configureRadioCommon();
bool waitForFrame();
void streamCIRFrame(bool frameReceived);

void setup() {
  Serial.begin(115200);

  pinMode(DW3000_SS_PIN, OUTPUT);
  digitalWrite(DW3000_SS_PIN, HIGH);

  DW3000.begin();

  DW3000.hardReset();
  delay(200);

  if (!DW3000.checkSPI()) {
    Serial.println(F("[ERROR] Could not establish SPI Connection to DW3000! Please make sure that all pins are set correctly."));
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
  configureRadioCommon();
  DW3000.setupGPIO();
  DW3000.configureAsRX();

  Serial.println(F("[INFO] DW3000 RX + CIR streamer ready."));
  Serial.print(F("[INFO] Streaming "));
  Serial.print(CIR_TOTAL_SAMPLES);
  Serial.println(F(" CIR samples per frame."));
}

void loop() {
  DW3000.standardRX();

  bool frameReceived = waitForFrame();
  if (frameReceived && rx_status == 1) {
    Serial.println(F("RX: frame received OK"));
  } else if (!frameReceived) {
    Serial.println(F("RX: timeout, no frame"));
  } else {
    Serial.print(F("RX: frame receive ERROR, code = "));
    Serial.println(rx_status);
  }

  // Always read the accumulator fresh for this attempt and stream samples
  // (filled with zeros if the frame was not received correctly).
  streamCIRFrame(frameReceived && rx_status == 1);

  DW3000.clearSystemStatus();
}

void configureRadioCommon() {
  DW3000.setChannel(UWB_CHANNEL);
  DW3000.setPreambleLength(UWB_PREAMBLE);
  DW3000.setPreambleCode(UWB_PREAMBLE_CODE);
  DW3000.setPACSize(UWB_PAC);
  DW3000.setDatarate(UWB_DATARATE);
  DW3000.setPHRMode(UWB_PHR_MODE);
  DW3000.setPHRRate(UWB_PHR_RATE);
}

bool waitForFrame() {
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

// Read the accumulator and stream a single CIR frame to Serial wrapped in
// FRAME_BEGIN / FRAME_END markers. When no valid frame was received, zeros are
// transmitted to keep the MATLAB parser synchronized.
void streamCIRFrame(bool frameReceived) {
  const uint32_t firstByteOffset = (uint32_t)CIR_FIRST_SAMPLE_INDEX * CIR_BYTES_PER_SAMPLE;
  const size_t totalBytes = (size_t)CIR_TOTAL_SAMPLES * CIR_BYTES_PER_SAMPLE;

  static uint8_t cirBytes[CIR_TOTAL_SAMPLES * CIR_BYTES_PER_SAMPLE];
  DW3000.readBytes(ACC_MEM_REG, firstByteOffset, cirBytes, totalBytes);

  Serial.println(F("FRAME_BEGIN"));

  for (uint16_t n = 0; n < CIR_TOTAL_SAMPLES; n++) {
    int16_t I = 0;
    int16_t Q = 0;

    if (frameReceived) {
      size_t base = (size_t)n * CIR_BYTES_PER_SAMPLE;
      I = (int16_t)((uint16_t)cirBytes[base + 0] | ((uint16_t)cirBytes[base + 1] << 8));
      Q = (int16_t)((uint16_t)cirBytes[base + 2] | ((uint16_t)cirBytes[base + 3] << 8));
    }

    float mag = sqrtf((float)I * (float)I + (float)Q * (float)Q);

    Serial.print(n);
    Serial.print(',');
    Serial.print(I);
    Serial.print(',');
    Serial.print(Q);
    Serial.print(',');
    Serial.println(mag, 3);
  }

  Serial.println(F("FRAME_END"));
  Serial.flush();
}

