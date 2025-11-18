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

const uint16_t TX_INTERVAL_MS = 200;      // Period between frames
const uint16_t TX_WAIT_TIMEOUT_MS = 300;  // Wait time for TX complete status

const char TX_PAYLOAD[] = "CIR_TEST"; // Simple test frame payload (without null terminator)
const uint8_t TX_PAYLOAD_LEN = sizeof(TX_PAYLOAD) - 1;

void configureRadioCommon();
bool waitForTxDone(uint32_t &lastStatus);

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
  // Load the payload into the TX buffer.
  DW3000.writeTXBuffer((const uint8_t *)TX_PAYLOAD, TX_PAYLOAD_LEN);
  DW3000.setFrameLength(TX_PAYLOAD_LEN);

  // Trigger the transmission.
  DW3000.standardTX();

  uint32_t sysStatus = 0;
  bool txOk = waitForTxDone(sysStatus);
  if (txOk) {
    Serial.println(F("TX: frame sent OK"));
  } else {
    Serial.println(F("TX: frame send ERROR/timeout"));
    Serial.print(F("[DEBUG] SYS_STATUS on TX timeout: 0x"));
    Serial.println(sysStatus, HEX);
  }

  DW3000.clearSystemStatus();
  delay(TX_INTERVAL_MS);
}

// Apply a shared radio configuration compatible with the standard DW3000 examples.
void configureRadioCommon() {
  DW3000.setChannel(UWB_CHANNEL);
  DW3000.setPreambleLength(UWB_PREAMBLE);
  DW3000.setPreambleCode(UWB_PREAMBLE_CODE);
  DW3000.setPACSize(UWB_PAC);
  DW3000.setDatarate(UWB_DATARATE);
  DW3000.setPHRMode(UWB_PHR_MODE);
  DW3000.setPHRRate(UWB_PHR_RATE);
}

// Wait until the DW3000 reports a successful TX or a timeout occurs.
bool waitForTxDone(uint32_t &lastStatus) {
  unsigned long startMs = millis();
  lastStatus = 0;

  while ((millis() - startMs) < TX_WAIT_TIMEOUT_MS) {
    int txStatus = DW3000.sentFrameSucc();
    lastStatus = DW3000.read(GEN_CFG_AES_LOW_REG, 0x44);

    if (txStatus == 1) {
      return true;
    }

    yield();
  }
  return false;
}

