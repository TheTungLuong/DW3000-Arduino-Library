#include <Arduino.h>
#include <SPI.h>
#include <math.h>

/*
 * Wiring:
 *  - Connect DW3000 module SPI pins to Arduino UNO default SPI: SCK=13, MISO=12, MOSI=11.
 *  - Chip select, reset and IRQ can be reassigned by editing DW_CS, DW_RST and DW_IRQ.
 *  - For other Arduino boards adjust the SPI pins or DW_* pin defines accordingly.
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

// SPI configuration for DW3000
#define DW_SPI_FREQUENCY        8000000u

static SPISettings dwSpiSettings(DW_SPI_FREQUENCY, MSBFIRST, SPI_MODE0);

static int32_t gPreambleI[NUM_SAMPLES];
static int32_t gPreambleQ[NUM_SAMPLES];
#if READ_STS
static int32_t gStsI[STS_SAMPLES];
static int32_t gStsQ[STS_SAMPLES];
#endif

static uint8_t gHeaderBuffer[3];

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

static void parse_samples(const uint8_t *raw, uint16_t count, int32_t *iBuffer, int32_t *qBuffer) {
  const uint8_t *p = raw + ACC_DUMMY_BYTES;
  for (uint16_t sample = 0; sample < count; sample++) {
    iBuffer[sample] = convert_acc_component(p);
    p += 3;
    qBuffer[sample] = convert_acc_component(p);
    p += 3;
  }
}

static void read_cir_direct(uint16_t startSample, uint16_t sampleCount, int32_t *iOut, int32_t *qOut) {
  const uint16_t kMaxChunk = 16;
  uint16_t processed = 0;
  while (processed < sampleCount) {
    uint16_t chunk = sampleCount - processed;
    if (chunk > kMaxChunk) {
      chunk = kMaxChunk;
    }
    uint8_t raw[ACC_DUMMY_BYTES + kMaxChunk * ACC_BYTES_PER_SAMPLE];
    uint16_t subaddress = startSample + processed;
    dw_read(DW_REG_ACC_MEM, subaddress, raw, ACC_DUMMY_BYTES + chunk * ACC_BYTES_PER_SAMPLE);
    parse_samples(raw, chunk, iOut + processed, qOut + processed);
    processed += chunk;
  }
}

static void read_cir_indirect(uint16_t startSample, uint16_t sampleCount, int32_t *iOut, int32_t *qOut) {
  const uint16_t kMaxChunk = 32;
  uint16_t processed = 0;
  while (processed < sampleCount) {
    uint16_t chunk = sampleCount - processed;
    if (chunk > kMaxChunk) {
      chunk = kMaxChunk;
    }
    program_indirect_pointer(startSample + processed);
    uint8_t raw[ACC_DUMMY_BYTES + kMaxChunk * ACC_BYTES_PER_SAMPLE];
    dw_read(DW_REG_INDIRECT_PTR_A, DW_SUB_PTR_A_WINDOW, raw, ACC_DUMMY_BYTES + chunk * ACC_BYTES_PER_SAMPLE);
    parse_samples(raw, chunk, iOut + processed, qOut + processed);
    processed += chunk;
  }
}

static void read_cir_block(uint16_t startSample, uint16_t sampleCount, int32_t *iOut, int32_t *qOut) {
  if (sampleCount == 0) {
    return;
  }
  uint16_t firstDirectCount = 0;
  if (startSample < 127) {
    uint16_t limit = 127 - startSample;
    firstDirectCount = sampleCount;
    if (firstDirectCount > limit) {
      firstDirectCount = limit;
    }
    read_cir_direct(startSample, firstDirectCount, iOut, qOut);
  }
  if (firstDirectCount < sampleCount) {
    uint16_t remaining = sampleCount - firstDirectCount;
    read_cir_indirect(startSample + firstDirectCount, remaining, iOut + firstDirectCount, qOut + firstDirectCount);
  }
}

static void reset_dw3000() {
  pinMode(DW_RST, OUTPUT);
  digitalWrite(DW_RST, LOW);
  delay(10);
  pinMode(DW_RST, INPUT);
  delay(10);
}

void setup() {
  Serial.begin(921600);
  Serial.println(F("#index,I,Q,mag"));

  pinMode(DW_CS, OUTPUT);
  pinMode(DW_IRQ, INPUT_PULLUP);
  digitalWrite(DW_CS, HIGH);

  SPI.begin();

  reset_dw3000();
  enable_acc_clocks(true);

  read_cir_block(0, NUM_SAMPLES, gPreambleI, gPreambleQ);

#if READ_STS
  read_cir_block(STS_OFFSET, STS_SAMPLES, gStsI, gStsQ);
#endif

  for (uint16_t i = 0; i < NUM_SAMPLES; i++) {
    double magnitude = sqrt((double)gPreambleI[i] * (double)gPreambleI[i] +
                            (double)gPreambleQ[i] * (double)gPreambleQ[i]);
    Serial.print(i);
    Serial.print(',');
    Serial.print(gPreambleI[i]);
    Serial.print(',');
    Serial.print(gPreambleQ[i]);
    Serial.print(',');
    Serial.println(magnitude, 6);
  }

#if READ_STS
  for (uint16_t i = 0; i < STS_SAMPLES; i++) {
    uint16_t index = STS_OFFSET + i;
    double magnitude = sqrt((double)gStsI[i] * (double)gStsI[i] +
                            (double)gStsQ[i] * (double)gStsQ[i]);
    Serial.print(index);
    Serial.print(',');
    Serial.print(gStsI[i]);
    Serial.print(',');
    Serial.print(gStsQ[i]);
    Serial.print(',');
    Serial.println(magnitude, 6);
  }
#endif

  enable_acc_clocks(false);
  Serial.flush();
}

void loop() {
  // Nothing to do in loop. Capture happens once during setup().
}

