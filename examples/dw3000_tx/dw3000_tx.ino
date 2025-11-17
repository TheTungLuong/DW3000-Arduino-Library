#include "DW3000.h"

#define TX_SENT_DELAY 200
#define TX_WAIT_TIMEOUT_MS 250

// Shared UWB configuration (must match the receiver)
const uint8_t UWB_CHANNEL = CHANNEL_5;
const uint8_t UWB_PREAMBLE = PREAMBLE_128;
const uint8_t UWB_PREAMBLE_CODE = 9;
const uint8_t UWB_PAC = PAC8;
const uint8_t UWB_DATARATE = DATARATE_6_8MB;
const uint8_t UWB_PHR_MODE = PHR_MODE_STANDARD;
const uint8_t UWB_PHR_RATE = PHR_RATE_850KB;

struct CirSample {
  int16_t i;
  int16_t q;
};

const uint8_t CIR_SAMPLE_COUNT = 3;
const CirSample BASE_CIR_SAMPLES[CIR_SAMPLE_COUNT] = {
  { 320,  -120 },
  { -80,   260 },
  { 150,  -220 }
};

CirSample cirSamples[CIR_SAMPLE_COUNT];
uint8_t frameBuffer[3 + CIR_SAMPLE_COUNT * sizeof(CirSample)];
uint16_t frameCounter = 0;

void configureUwbCommon()
{
  // Ensure both TX and RX use identical radio parameters
  DW3000.setChannel(UWB_CHANNEL);
  DW3000.setPreambleLength(UWB_PREAMBLE);
  DW3000.setPreambleCode(UWB_PREAMBLE_CODE);
  DW3000.setPACSize(UWB_PAC);
  DW3000.setDatarate(UWB_DATARATE);
  DW3000.setPHRMode(UWB_PHR_MODE);
  DW3000.setPHRRate(UWB_PHR_RATE);
}

void prepareCirSamples(uint16_t counter)
{
  for (uint8_t idx = 0; idx < CIR_SAMPLE_COUNT; idx++) {
    cirSamples[idx].i = BASE_CIR_SAMPLES[idx].i + counter;
    cirSamples[idx].q = BASE_CIR_SAMPLES[idx].q - counter;
  }
}

size_t encodeCirFrame(uint16_t counter)
{
  frameBuffer[0] = CIR_SAMPLE_COUNT;      // Let the receiver know how many samples follow
  frameBuffer[1] = lowByte(counter);      // Frame counter LSB
  frameBuffer[2] = highByte(counter);     // Frame counter MSB

  size_t bufferIndex = 3;
  for (uint8_t idx = 0; idx < CIR_SAMPLE_COUNT; idx++) {
    frameBuffer[bufferIndex++] = lowByte(cirSamples[idx].i);
    frameBuffer[bufferIndex++] = highByte(cirSamples[idx].i);
    frameBuffer[bufferIndex++] = lowByte(cirSamples[idx].q);
    frameBuffer[bufferIndex++] = highByte(cirSamples[idx].q);
  }

  return bufferIndex;
}

bool waitForTxDone()
{
  unsigned long startMs = millis();
  while ((millis() - startMs) < TX_WAIT_TIMEOUT_MS) {
    if (DW3000.sentFrameSucc()) {
      return true;
    }
  }
  return false;
}

void setup()
{
  Serial.begin(115200); // Init Serial
  DW3000.begin(); // Init SPI
  configureUwbCommon(); // Apply radio config before init so TX/RX match
  DW3000.hardReset(); // hard reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up

  if(!DW3000.checkSPI())
  {
    Serial.println("[ERROR] Could not establish SPI Connection to DW3000! Please make sure that all pins are set correctly.");
    while(100);
  }

  while (!DW3000.checkForIDLE()) // Make sure that chip is in IDLE before continuing
  {
    Serial.println("[ERROR] IDLE1 FAILED\r");
    delay(1000);
  }

  DW3000.softReset(); // Reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up


  if (!DW3000.checkForIDLE())
  {
    Serial.println("[ERROR] IDLE2 FAILED\r");
    while (100);
  }


  DW3000.init(); // Initialize chip (write default values, calibration, etc.)
  DW3000.setupGPIO(); //Setup the DW3000s GPIO pins for use of LEDs
  Serial.println("[INFO] Setup is finished.");

  DW3000.configureAsTX(); // Configure basic settings for frame transmitting
}

void loop()
{
  prepareCirSamples(frameCounter);
  size_t frameLength = encodeCirFrame(frameCounter);

  DW3000.pullLEDHigh(2);
  DW3000.writeTXBuffer(frameBuffer, frameLength); // Write the CIR payload into the TX buffer
  DW3000.setFrameLength(frameLength); // Set frame length in bytes (FCS is added by hardware)

  DW3000.standardTX(); // Send fast command for transmitting

  bool txOk = waitForTxDone();
  uint32_t sysStatus = DW3000.read(GEN_CFG_AES_LOW_REG, 0x44);

  if (txOk) {
    Serial.print("TX: frame sent OK, frame_id = ");
    Serial.println(frameCounter);
  } else {
    Serial.print("TX: frame send ERROR, code = 0x");
    Serial.println(sysStatus, HEX);
  }

  DW3000.clearSystemStatus(); // Clear event status

  DW3000.pullLEDLow(2);

  frameCounter++;
  delay(TX_SENT_DELAY); // Give receiver time to process
}
