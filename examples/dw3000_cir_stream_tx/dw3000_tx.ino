#include "DW3000.h"

const uint8_t PIN_SS = 10;   // DWS3000 CSn
const uint8_t PIN_IRQ = 2;   // DWS3000 IRQ (unused, but declared for clarity)
const uint8_t PIN_RST = 9;   // DWS3000 RSTn

const uint8_t UWB_CHANNEL = CHANNEL_5;
const uint8_t UWB_PREAMBLE = PREAMBLE_128;
const uint8_t UWB_PREAMBLE_CODE = 9;
const uint8_t UWB_PAC = PAC8;
const uint8_t UWB_DATARATE = DATARATE_6_8MB;
const uint8_t UWB_PHR_MODE = PHR_MODE_STANDARD;
const uint8_t UWB_PHR_RATE = PHR_RATE_850KB;

const uint16_t TX_WAIT_TIMEOUT_MS = 250;
const uint16_t TX_SENT_DELAY_MS = 200;

uint16_t txSequence = 0;
uint8_t txPayload[12];

void configureUwbCommon() {
  DW3000.setChannel(UWB_CHANNEL);
  DW3000.setPreambleLength(UWB_PREAMBLE);
  DW3000.setPreambleCode(UWB_PREAMBLE_CODE);
  DW3000.setPACSize(UWB_PAC);
  DW3000.setDatarate(UWB_DATARATE);
  DW3000.setPHRMode(UWB_PHR_MODE);
  DW3000.setPHRRate(UWB_PHR_RATE);
}

bool waitForTxDone(unsigned long timeoutMs) {
  unsigned long startMs = millis();
  while ((millis() - startMs) < timeoutMs) {
    int status = DW3000.sentFrameSucc();
    if (status == 1) {
      return true;
    }
    yield();
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_IRQ, INPUT_PULLUP);

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
  DW3000.configureAsTX();
  DW3000.clearSystemStatus();

  Serial.println(F("[INFO] DW3000 TX ready."));
}

void loop() {
  // Build a simple payload that carries an incrementing sequence counter
  for (uint8_t i = 0; i < sizeof(txPayload); i++) {
    txPayload[i] = (uint8_t)(txSequence + i);
  }

  DW3000.pullLEDHigh(2);
  DW3000.writeTXBuffer(txPayload, sizeof(txPayload));
  DW3000.setFrameLength(sizeof(txPayload));
  DW3000.standardTX();

  bool txOk = waitForTxDone(TX_WAIT_TIMEOUT_MS);
  uint32_t sysStatus = DW3000.read(GEN_CFG_AES_LOW_REG, 0x44);

  if (txOk) {
    Serial.print(F("TX OK, seq="));
    Serial.println(txSequence);
  } else {
    Serial.print(F("TX ERROR, status=0x"));
    Serial.println(sysStatus, HEX);
  }

  DW3000.clearSystemStatus();
  DW3000.pullLEDLow(2);

  txSequence++;
  delay(TX_SENT_DELAY_MS);
}
