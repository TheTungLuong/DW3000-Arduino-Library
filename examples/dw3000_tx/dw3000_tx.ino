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

uint64_t txPayloadWord = 0;              // Pack the 8-byte payload into a single word
uint32_t frameCounter = 0;

void configureRadioCommon();
uint64_t buildPayloadWord(uint32_t counter);
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
  txPayloadWord = buildPayloadWord(frameCounter);

  // Clear stale status bits before arming a new transmission.
  DW3000.clearSystemStatus();
  DW3000.setMode(0); // standard frame type

  // Write the 8-byte payload via the helper used in the original examples.
  DW3000.pullLEDHigh(2);
  DW3000.setTXFrame(txPayloadWord);
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
  DW3000.writeSysConfig(); // apply the updated settings to SYS_CFG/CHAN_CTRL
}

uint64_t buildPayloadWord(uint32_t counter) {
  // Pack the 32-bit counter (little-endian) followed by a fixed pattern into a
  // single 64-bit word. This aligns with the helper that writes a full word to
  // the TX buffer in one go.
  uint64_t word = 0;
  word |= (uint64_t)(counter & 0xFF);
  word |= (uint64_t)((counter >> 8) & 0xFF) << 8;
  word |= (uint64_t)((counter >> 16) & 0xFF) << 16;
  word |= (uint64_t)((counter >> 24) & 0xFF) << 24;

  // Fill remaining bytes with a simple pattern for quick visual checks in a sniffer.
  word |= (uint64_t)0xA5 << 32;
  word |= (uint64_t)0x5A << 40;
  word |= (uint64_t)0xC3 << 48;
  word |= (uint64_t)0x3C << 56;

  return word;
}

bool waitForTxDone() {
  unsigned long startMs = millis();
  while ((millis() - startMs) < TX_WAIT_TIMEOUT_MS) {
    if (DW3000.sentFrameSucc()) {
      return true;
    }
    yield();
  }

  // Provide additional diagnostics so timeouts are easier to debug on hardware.
  uint32_t sysStat = DW3000.read(GEN_CFG_AES_LOW_REG, 0x44);
  Serial.print(F("[DEBUG] SYS_STATUS on TX timeout: 0x"));
  Serial.println(sysStat, HEX);
  return false;
}
