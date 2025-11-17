#include <SPI.h>
#include "DW3000.h"

// Chip select pin for the DW3000 shield when used with Arduino UNO class boards.
// The library already defaults to pin 10 for non-ESP32 targets, but we define it
// explicitly here for clarity.
#define DW3000_SS_PIN 10

// Shared UWB configuration (must match the receiver)
const uint8_t UWB_CHANNEL = CHANNEL_5;
const uint8_t UWB_PREAMBLE = PREAMBLE_128;
const uint8_t UWB_PREAMBLE_CODE = 9;
const uint8_t UWB_PAC = PAC8;
const uint8_t UWB_DATARATE = DATARATE_6_8MB;
const uint8_t UWB_PHR_MODE = PHR_MODE_STANDARD;
const uint8_t UWB_PHR_RATE = PHR_RATE_850KB;

const uint16_t TX_INTERVAL_MS = 150;     // Period between frames
const uint16_t TX_WAIT_TIMEOUT_MS = 200; // Wait time for TX complete status
const uint8_t TX_PAYLOAD_LEN = 8;        // Small frame (~8 bytes)

uint8_t txPayload[TX_PAYLOAD_LEN];
uint32_t frameCounter = 0;

void configureRadioCommon();
void buildPayload(uint32_t counter);
bool waitForTxDone();

void setup() {
  Serial.begin(115200);

  // Ensure the chip select is configured before SPI begins.
  pinMode(DW3000_SS_PIN, OUTPUT);
  digitalWrite(DW3000_SS_PIN, HIGH);

  DW3000.begin();

  // Perform the standard reset/idle checks used in the library examples.
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
  DW3000.configureAsTX();

  Serial.println(F("[INFO] DW3000 TX setup complete."));
}

void loop() {
  buildPayload(frameCounter);

  DW3000.pullLEDHigh(2);
  DW3000.writeTXBuffer(txPayload, TX_PAYLOAD_LEN);
  DW3000.setFrameLength(TX_PAYLOAD_LEN);
  DW3000.standardTX();

  bool txOk = waitForTxDone();
  if (txOk) {
    Serial.print(F("TX: frame sent OK, id="));
    Serial.println(frameCounter);
  } else {
    Serial.println(F("TX: frame send ERROR/timeout"));
  }

  DW3000.clearSystemStatus();
  DW3000.pullLEDLow(2);

  frameCounter++;
  delay(TX_INTERVAL_MS);
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

void buildPayload(uint32_t counter) {
  // Embed the 32-bit frame counter (little-endian) so the receiver can track IDs.
  txPayload[0] = (uint8_t)(counter & 0xFF);
  txPayload[1] = (uint8_t)((counter >> 8) & 0xFF);
  txPayload[2] = (uint8_t)((counter >> 16) & 0xFF);
  txPayload[3] = (uint8_t)((counter >> 24) & 0xFF);

  // Fill remaining bytes with a simple pattern for quick visual checks in a sniffer.
  txPayload[4] = 0xA5;
  txPayload[5] = 0x5A;
  txPayload[6] = 0xC3;
  txPayload[7] = 0x3C;
}

bool waitForTxDone() {
  unsigned long startMs = millis();
  while ((millis() - startMs) < TX_WAIT_TIMEOUT_MS) {
    if (DW3000.sentFrameSucc()) {
      return true;
    }
    yield();
  }
  return false;
}
