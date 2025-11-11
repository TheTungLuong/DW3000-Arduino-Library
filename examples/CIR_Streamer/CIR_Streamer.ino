#include <Arduino.h>
#include <SPI.h>
#include <math.h>

/*
 * Wiring:
 *  - Connect DW3000 module SPI pins to Arduino UNO default SPI: SCK=13, MISO=12, MOSI=11.
 *  - Chip select, reset and IRQ can be reassigned by editing DW_CS, DW_RST and DW_IRQ.
 *  - For other Arduino boards adjust the SPI pins or DW_* pin defines accordingly.
 *
 * Usage:
 *  - Open the Serial Monitor at 921600 baud to watch the streamed CIR samples.
 *  - Each capture prints "#index,I,Q,mag" followed by streamed samples for easy observation.
 *  - Send 'c' over the Serial Monitor to trigger another capture without resetting the board.
 */

#define DW_CS   10
#define DW_RST   9
#define DW_IRQ   2

// DW3000 register file addresses
#define DW_REG_PMSC             0x11    // Power management and system clock
#define DW_REG_ACC_MEM          0x15    // Accumulator memory
#define DW_REG_INDIRECT_PTR_A   0x1D    // Indirect pointer bank A

// PMSC sub-register offsets
#define DW_SUB_PMSC_CTRL0       0x04    // Clock control

// Indirect pointer A sub-registers
#define DW_SUB_PTR_A_WINDOW     0x00    // Data window for indirect access
#define DW_SUB_PTR_A_ADDR       0x04    // Pointer target register file address
#define DW_SUB_PTR_A_OFFSET     0x06    // Pointer offset (sample index)

// Indirect pointer control values
#define DW_PTR_ADDR_ACC_MEM     DW_REG_ACC_MEM

// ACC memory layout
#define ACC_BYTES_PER_SAMPLE    6u
#define ACC_DUMMY_BYTES         1u

// Sampling configuration
#define NUM_SAMPLES             1016u
#define STS_OFFSET              1024u
#define STS_SAMPLES             512u
#define READ_STS                0       // Set to 1 to also dump STS CIR taps

// Streaming granularity to limit RAM consumption on small boards (samples per read)
#define CIR_CHUNK_SAMPLES       16u

// SPI configuration for DW3000
#define DW_SPI_FREQUENCY        8000000u

static SPISettings dwSpiSettings(DW_SPI_FREQUENCY, MSBFIRST, SPI_MODE0);

static uint8_t gHeaderBuffer[3];
static uint8_t gAccBuffer[ACC_DUMMY_BYTES + CIR_CHUNK_SAMPLES * ACC_BYTES_PER_SAMPLE];

static void print_status(const __FlashStringHelper *message) {
  Serial.write('#');
  Serial.println(message);
}

static void print_status(const char *message) {
  Serial.write('#');
  Serial.println(message);
}

static void dw_select() {
  SPI.beginTransaction(dwSpiSettings);
  digitalWrite(DW_CS, LOW);
}

static void dw_deselect() {
  digitalWrite(DW_CS, HIGH);
  SPI.endTransaction();
}

static uint8_t build_spi_header(bool isRead, uint8_t regfile, uint16_t subaddress) {
  uint8_t index = 0;
  gHeaderBuffer[index] = (regfile & 0x3F) | (isRead ? 0x80 : 0x00);
  if (subaddress != 0) {
    gHeaderBuffer[index] |= 0x40;  // indicate sub-address follows
    index++;
    gHeaderBuffer[index] = (uint8_t)(subaddress & 0x7F);
    if (subaddress > 0x7F) {
      gHeaderBuffer[index] |= 0x80; // extended address flag
      index++;
      gHeaderBuffer[index] = (uint8_t)(subaddress >> 7);
    }
  }
  return index + 1;
}

static uint8_t spi_begin_read_header(uint8_t regfile, uint16_t subaddress) {
  uint8_t headerLength = build_spi_header(true, regfile, subaddress);
  for (uint8_t i = 0; i < headerLength; i++) {
    SPI.transfer(gHeaderBuffer[i]);
  }
  return headerLength;
}

static uint8_t spi_begin_write_header(uint8_t regfile, uint16_t subaddress) {
  uint8_t headerLength = build_spi_header(false, regfile, subaddress);
  for (uint8_t i = 0; i < headerLength; i++) {
    SPI.transfer(gHeaderBuffer[i]);
  }
  return headerLength;
}

static void dw_write(uint8_t regfile, uint16_t subaddress, const uint8_t *data, size_t length) {
  if (data == nullptr || length == 0) {
    return;
  }
  dw_select();
  spi_begin_write_header(regfile, subaddress);
  for (size_t i = 0; i < length; i++) {
    SPI.transfer(data[i]);
  }
  dw_deselect();
}

static void dw_write8(uint8_t regfile, uint16_t subaddress, uint8_t value) {
  dw_write(regfile, subaddress, &value, 1);
}

static void dw_write16(uint8_t regfile, uint16_t subaddress, uint16_t value) {
  uint8_t bytes[2];
  bytes[0] = (uint8_t)(value & 0xFF);
  bytes[1] = (uint8_t)((value >> 8) & 0xFF);
  dw_write(regfile, subaddress, bytes, sizeof(bytes));
}

static void dw_write32(uint8_t regfile, uint16_t subaddress, uint32_t value) {
  uint8_t bytes[4];
  bytes[0] = (uint8_t)(value & 0xFF);
  bytes[1] = (uint8_t)((value >> 8) & 0xFF);
  bytes[2] = (uint8_t)((value >> 16) & 0xFF);
  bytes[3] = (uint8_t)((value >> 24) & 0xFF);
  dw_write(regfile, subaddress, bytes, sizeof(bytes));
}

static void dw_read(uint8_t regfile, uint16_t subaddress, uint8_t *buffer, size_t length) {
  if (buffer == nullptr || length == 0) {
    return;
  }
  dw_select();
  spi_begin_read_header(regfile, subaddress);
  for (size_t i = 0; i < length; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  dw_deselect();
}

static uint16_t dw_read16(uint8_t regfile, uint16_t subaddress) {
  uint8_t bytes[2] = {0, 0};
  dw_read(regfile, subaddress, bytes, sizeof(bytes));
  return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
}

static void enable_acc_clocks(bool on) {
  uint16_t value = dw_read16(DW_REG_PMSC, DW_SUB_PMSC_CTRL0);
  const uint16_t accClkMask = (1u << 6);   // ACC_CLK_EN
  const uint16_t accMclkMask = (1u << 15); // ACC_MCLK_EN
  if (on) {
    value |= (accClkMask | accMclkMask);
  } else {
    value &= (uint16_t)~(accClkMask | accMclkMask);
  }
  dw_write16(DW_REG_PMSC, DW_SUB_PMSC_CTRL0, value);
}

static void program_indirect_pointer(uint16_t sampleIndex) {
  dw_write8(DW_REG_INDIRECT_PTR_A, DW_SUB_PTR_A_ADDR, DW_PTR_ADDR_ACC_MEM);
  dw_write16(DW_REG_INDIRECT_PTR_A, DW_SUB_PTR_A_OFFSET, sampleIndex);
}

static inline int32_t convert_acc_component(const uint8_t *raw) {
  int32_t value = (int32_t)raw[0] | ((int32_t)raw[1] << 8) | ((int32_t)raw[2] << 16);
  if (value & 0x00800000L) {
    value |= (int32_t)0xFF000000L;
  }
  value >>= 6;
  return value;
}

static void reset_dw3000() {
  pinMode(DW_RST, OUTPUT);
  digitalWrite(DW_RST, LOW);
  delay(10);
  pinMode(DW_RST, INPUT);
  delay(10);
}

static void emit_chunk(uint16_t baseSample, uint16_t sampleCount) {
  const uint8_t *p = gAccBuffer + ACC_DUMMY_BYTES;
  for (uint16_t i = 0; i < sampleCount; i++) {
    int32_t iVal = convert_acc_component(p);
    p += 3;
    int32_t qVal = convert_acc_component(p);
    p += 3;

    double magnitude = sqrt((double)iVal * (double)iVal +
                            (double)qVal * (double)qVal);

    uint16_t index = baseSample + i;
    Serial.print(index);
    Serial.write(',');
    Serial.print((long)iVal);
    Serial.write(',');
    Serial.print((long)qVal);
    Serial.write(',');
    Serial.println(magnitude, 6);
  }
}

static void stream_samples(uint16_t startSample, uint16_t sampleCount) {
  uint16_t processed = 0;
  while (processed < sampleCount) {
    uint16_t currentSample = startSample + processed;
    bool useDirect = (currentSample < 127u);

    uint16_t chunkLimit = CIR_CHUNK_SAMPLES;
    if (useDirect) {
      uint16_t directRemaining = 127u - currentSample;
      if (directRemaining < chunkLimit) {
        chunkLimit = directRemaining;
      }
    }

    uint16_t chunk = sampleCount - processed;
    if (chunk > chunkLimit) {
      chunk = chunkLimit;
    }
    if (chunk == 0) {
      useDirect = false;
      chunkLimit = CIR_CHUNK_SAMPLES;
      chunk = sampleCount - processed;
      if (chunk > chunkLimit) {
        chunk = chunkLimit;
      }
      if (chunk == 0) {
        break;
      }
    }

    if (useDirect) {
      dw_read(DW_REG_ACC_MEM, currentSample, gAccBuffer, ACC_DUMMY_BYTES + chunk * ACC_BYTES_PER_SAMPLE);
    } else {
      program_indirect_pointer(currentSample);
      dw_read(DW_REG_INDIRECT_PTR_A, DW_SUB_PTR_A_WINDOW, gAccBuffer, ACC_DUMMY_BYTES + chunk * ACC_BYTES_PER_SAMPLE);
    }

    emit_chunk(currentSample, chunk);
    processed += chunk;
  }
}

static void stream_full_capture() {
  print_status(F("capture-begin"));
  enable_acc_clocks(true);

  Serial.println(F("#index,I,Q,mag"));
  stream_samples(0, NUM_SAMPLES);

#if READ_STS
  stream_samples(STS_OFFSET, STS_SAMPLES);
#endif

  enable_acc_clocks(false);
  Serial.flush();
  print_status(F("capture-end"));
  print_status(F("ready-send-c-to-capture"));
}

void setup() {
  Serial.begin(921600);
#if defined(ARDUINO_AVR_UNO)
  // Allow time for the USB CDC bridge to enumerate after reset.
  delay(200);
#endif
  pinMode(DW_CS, OUTPUT);
  pinMode(DW_IRQ, INPUT_PULLUP);
  digitalWrite(DW_CS, HIGH);

  SPI.begin();

  reset_dw3000();
  delay(10);
  stream_full_capture();
}

void loop() {
  if (Serial.available()) {
    int incoming = Serial.read();
    if (incoming == 'c' || incoming == 'C') {
      stream_full_capture();
    }
  }
}

