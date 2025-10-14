#include "DW3000.h"

/*
   BE AWARE: Baud Rate got changed to 2.000.000!

   Approach based on the application note APS011 ("SOURCES OF ERROR IN DW1000 BASED
   TWO-WAY RANGING (TWR) SCHEMES")

   see chapter 2.4 figure 6 and the corresponding description for more information

   This approach tackles the problem of a big clock offset between the ping and pong side
   by reducing the clock offset to a minimum.

   This approach is a more advanced version of the classical ping and pong with timestamp examples.
*/

#define ROUND_DELAY 500 // Delay in milliseconds that the chip waits between PING requests

static int frame_buffer = 0; // Variable to store the transmitted message
static int rx_status; // Variable to store the current status of the receiver operation
static int tx_status; // Variable to store the current status of the receiver operation

/*
   valid stages:
   0 - default stage; starts ranging
   1 - ranging sent; awaiting response
   2 - response received; sending second range
   3 - second ranging sent; awaiting final answer
   4 - final answer received
*/
static int curr_stage = 0;

static int64_t t_roundA = 0;
static int64_t t_replyA = 0;
static int64_t rx = 0;
static int64_t tx = 0;

static int clock_offset = 0;

static int64_t ranging_time = 0;  // keep full precision
static float   distance = 0;

void setup()
{
  Serial.begin(2000000); // Init Serial

    // --- Pick your PHY ---
  DW3000.setChannel(CHANNEL_5);          // 6.5 GHz band (Ch 5)
  DW3000.setPreambleCode(9);             // 9..12 are valid; must match on both ends
  DW3000.setPreambleLength(PREAMBLE_128);// e.g., 128 symbols
  DW3000.setPACSize(PAC8);               // typical with 128 preamble
  DW3000.setDatarate(DATARATE_6_8MB);    // payload rate

  DW3000.begin(); // Init SPI
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

  DW3000.softReset();
  delay(200); // Wait for DW3000 chip to wake up


  if (!DW3000.checkForIDLE())
  {
    Serial.println("[ERROR] IDLE2 FAILED\r");
    while (100);
  }


  DW3000.init(); // Initialize chip (write default values, calibration, etc.)
  DW3000.setupGPIO(); //Setup the DW3000s GPIO pins for use of LEDs
  Serial.println("> double-sided PING with timestamp example <\n");
  Serial.println("[INFO] Setup is finished.");

  DW3000.configureAsTX(); // Configure basic settings for frame transmitting

  DW3000.clearSystemStatus();
  
}

// timeouts (tune as needed)
const uint32_t S1_TIMEOUT_MS = 200;
const uint32_t S3_TIMEOUT_MS = 200;
static uint32_t s1_deadline = 0;
static uint32_t s3_deadline = 0;


// Log tags (short strings to save SRAM):
//   "S0"   = entered S0 (POLL sent; starting a ranging cycle)
//   "S1OK" = RESPONSE received, stage==2 (good), proceed to S2
//   "S1ER" = RX error flags set (CRC/PHE/etc); restart
//   "S1ERF"= Got a frame but it's marked error by protocol (ds_isErrorFrame); restart
//   "S1STG"= Got a frame but stage field != 2 (peer out of sync); send error frame & restart
//   "S1TO" = Timeout waiting for RESPONSE; restart
//   "S2"   = FINAL sent and local spans captured; proceeding to S3
//   "S3OK" = ANSWER received (good); proceed to compute in S4
//   "S3ER" = RX error flags set while waiting ANSWER; restart
//   "S3ERF"= Got ANSWER frame but protocol marks it as error; restart
//   "S3TO" = Timeout waiting for ANSWER; restart
//   "D=..cm" printed distance in centimeters on each successful cycle
//   "S?"   = fell into an unknown state; reset to S0

void loop()
{
  switch (curr_stage) {
    case 0:  // Start ranging.
      Serial.println("S0");          // begin cycle
      t_roundA = 0;
      t_replyA = 0;

      DW3000.ds_sendFrame(1);        // POLL
      tx = DW3000.readTXTimestamp();

      curr_stage = 1;
      s1_deadline = millis() + S1_TIMEOUT_MS;
      break;

    case 1:  // Await first response (RESPONSE).
      rx_status = DW3000.receivedFrameSucc();
      if (rx_status != 0) {
        DW3000.clearSystemStatus();

        if (rx_status == 1) {        // RX OK
          bool err = DW3000.ds_isErrorFrame();
          int  stg = DW3000.ds_getStage();

          if (err) {
            Serial.println("S1ERF"); // error frame
            curr_stage = 0;
          } else if (stg != 2) {
            Serial.println("S1STG"); // wrong stage tag
            DW3000.ds_sendErrorFrame();
            curr_stage = 0;
          } else {
            Serial.println("S1OK");  // good response
            curr_stage = 2;
          }
        } else {                      // RX error flags
          Serial.println("S1ER");
          curr_stage = 0;
        }
      } else {
        if ((int32_t)(millis() - s1_deadline) >= 0) {
          Serial.println("S1TO");    // timeout waiting RESPONSE
          DW3000.clearSystemStatus();
          curr_stage = 0;
        }
      }
      break;

    case 2:  // RESPONSE received. Send FINAL.
      rx = DW3000.readRXTimestamp();

      t_roundA = rx - tx;

      DW3000.ds_sendFrame(3);        // FINAL
      tx = DW3000.readTXTimestamp();

      t_replyA = tx - rx;

      curr_stage = 3;
      s3_deadline = millis() + S3_TIMEOUT_MS;
      Serial.println("S2");
      break;

    case 3:  // Await ANSWER.
      rx_status = DW3000.receivedFrameSucc();
      if (rx_status != 0) {
        DW3000.clearSystemStatus();

        if (rx_status == 1) {        // RX OK
          bool err = DW3000.ds_isErrorFrame();
          if (err) {
            Serial.println("S3ERF"); // error frame
            curr_stage = 0;
          } else {
            clock_offset = DW3000.getRawClockOffset();
            curr_stage = 4;
            Serial.println("S3OK");
          }
        } else {                      // RX error flags
          Serial.println("S3ER");
          curr_stage = 0;
        }
      } else {
        if ((int32_t)(millis() - s3_deadline) >= 0) {
          Serial.println("S3TO");    // timeout waiting ANSWER
          DW3000.clearSystemStatus();
          curr_stage = 0;
        }
      }
      break;

    case 4:  // Compute result.
    {
      // peer spans from payload (adjust if your responder packs differently)
      int64_t t_roundB = (int64_t)DW3000.read(0x12, 0x04);
      int64_t t_replyB = (int64_t)DW3000.read(0x12, 0x08);

      ranging_time = DW3000.ds_processRTInfo(t_roundA, t_replyA, t_roundB, t_replyB, clock_offset);
      distance = DW3000.convertToCM(ranging_time);

      Serial.print("D=");            // final distance (cm)
      DW3000.printDouble(distance, 100, false);
      Serial.println("cm");

      curr_stage = 0;
      delay(ROUND_DELAY);
      break;
    }

    default:
      Serial.println("S?");          // unknown state
      curr_stage = 0;
      break;
  }
}





































// ---- 64-bit print helpers (AVR-safe, no F() macro) ----
static void print_u32_hex8(uint32_t x) {
  for (int i = 7; i >= 0; --i) {
    uint8_t nib = (x >> (i * 4)) & 0xF;
    Serial.print((char)(nib < 10 ? ('0' + nib) : ('A' + nib - 10)));
  }
}

static void print_u64ln_hex(const char* label, uint64_t v) {
  Serial.print(label);
  Serial.print(" = 0x");
  uint32_t hi = (uint32_t)(v >> 32);
  uint32_t lo = (uint32_t)(v & 0xFFFFFFFFULL);
  print_u32_hex8(hi);
  Serial.print("_");
  print_u32_hex8(lo);
  Serial.println();
}

// Decimal 64-bit (for spans like t_roundA/t_replyA)
static void print_i64ln(const char* label, int64_t v) {
  Serial.print(label);
  Serial.print(" = ");
  if (v == 0) { Serial.println(0); return; }
  if (v < 0) { Serial.print("-"); v = -v; }
  char buf[21]; // enough for 64-bit signed
  uint8_t i = 0;
  while (v && i < sizeof(buf)) {
    int digit = (int)(v % 10);
    buf[i++] = (char)('0' + digit);
    v /= 10;
  }
  while (i--) Serial.print(buf[i]);
  Serial.println();
}
// -------------------------------------------------------
