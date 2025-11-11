#include "DW3000.h"

#define SERIAL_BAUD    921600UL  // Match the CIR streaming tools for consistent capture
#define TX_SENT_DELAY  500

#define STREAM_TX_SAMPLES 1

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
static int tx_status; // Variable to store the current status of the transmitter operation
uint16_t frameCounter = 0;

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

static void printCsvHeader() {
#if STREAM_TX_SAMPLES
  Serial.println(F("#frame,index,I,Q"));
#endif
}

static void streamFrameSamples(uint16_t counter) {
#if STREAM_TX_SAMPLES
  for (uint8_t idx = 0; idx < CIR_SAMPLE_COUNT; idx++) {
    Serial.print(counter);
    Serial.print(',');
    Serial.print(idx);
    Serial.print(',');
    Serial.print(cirSamples[idx].i);
    Serial.print(',');
    Serial.println(cirSamples[idx].q);
  }
#else
  (void)counter;
#endif
}

void setup()
{
  Serial.begin(SERIAL_BAUD); // Init Serial fast enough for downstream capture tools
#if defined(USBCON) || defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_STM32)
  while (!Serial) {
    ;
  }
#endif
  DW3000.begin(); // Init SPI
  DW3000.hardReset(); // hard reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up

  if(!DW3000.checkSPI())
  {
    Serial.println(F("#ERROR Could not establish SPI Connection to DW3000! Please make sure that all pins are set correctly."));
    while(100);
  }

  while (!DW3000.checkForIDLE()) // Make sure that chip is in IDLE before continuing
  {
    Serial.println(F("#ERROR IDLE1 FAILED"));
    delay(1000);
  }

  DW3000.softReset(); // Reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up


  if (!DW3000.checkForIDLE())
  {
    Serial.println(F("#ERROR IDLE2 FAILED"));
    while (100);
  }


  DW3000.init(); // Initialize chip (write default values, calibration, etc.)
  DW3000.setupGPIO(); //Setup the DW3000s GPIO pins for use of LEDs
  Serial.println(F("#INFO Setup is finished."));

  printCsvHeader();

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
  delay(10); // Wait for frame to be sent

  while (!(tx_status = DW3000.sentFrameSucc()))
  {
    Serial.println(F("#ERROR Frame could not be sent succesfully!"));
  };

  DW3000.clearSystemStatus(); // Clear event status

  streamFrameSamples(frameCounter);
  DW3000.pullLEDLow(2);

  frameCounter++;
  delay(TX_SENT_DELAY);
}
