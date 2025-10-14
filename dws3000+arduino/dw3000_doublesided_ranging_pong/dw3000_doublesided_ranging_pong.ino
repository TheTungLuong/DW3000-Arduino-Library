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

static int frame_buffer = 0; // Variable to store the transmitted message
static int rx_status; // Variable to store the current status of the receiver operation
static int tx_status; // Variable to store the current status of the receiver operation

/*
   valid stages:
   0 - default stage; await ranging
   1 - ranging received; sending response
   2 - response sent; await second response
   3 - second response received; sending information frame
   4 - information frame sent
*/
static int curr_stage = 0;

static int t_roundB = 0;
static int t_replyB = 0;

static long long rx = 0;
static long long tx = 0;

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
  DW3000.softReset(); // Reset in case that the chip wasn't disconnected from power
  delay(200); // Wait for DW3000 chip to wake up

  if (!DW3000.checkForIDLE())
  {
    Serial.println("[ERROR] IDLE2 FAILED\r");
    while (100);
  }

  DW3000.init(); // Initialize chip (write default values, calibration, etc.)
  DW3000.setupGPIO(); //Setup the DW3000s GPIO pins for use of LEDs

  Serial.println("> double-sided PONG with timestamp example <\n");

  Serial.println("[INFO] Setup finished.");

  DW3000.configureAsTX(); // Configure basic settings for frame transmitting

  DW3000.clearSystemStatus();

  DW3000.standardRX();
}

// ---- counters ----
static uint32_t g_ok = 0, g_wrongStage = 0, g_errFrames = 0, g_rxErr = 0;

// (Using your helpers)
//   void print_u64ln_hex(const char* label, uint64_t v);
//   void print_i64ln(const char* label, int64_t v);

void loop()
{
  switch (curr_stage) {
    case 0: { // Await ranging (expect stage=1)
      t_roundB = 0;
      t_replyB = 0;

      int rs = DW3000.receivedFrameSucc();   // 0=no frame, 1=ok, 2=rx error
      if (rs) {
        rx_status = rs;
        DW3000.clearSystemStatus();

        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame()) {
            g_errFrames++;
            Serial.println("[S0] Warning: error frame → stay S0, RX");
            curr_stage = 0;
            DW3000.standardRX();
          } else {
            int stg = DW3000.ds_getStage();
            if (stg != 1) {
              g_wrongStage++;
              Serial.print("[S0] Wrong stage "); Serial.print(stg);
              Serial.println(" (expected 1). Send ErrorFrame, stay S0");
              DW3000.ds_sendErrorFrame();
              DW3000.standardRX();
              curr_stage = 0;
            } else {
              Serial.println("[S0] OK: POLL received (stage=1) → S1");
              curr_stage = 1;
            }
          }
        } else { // rx_status == 2
          g_rxErr++;
          Serial.println("[S0] RX error → cleared status, stay S0");
          DW3000.standardRX();
        }
      }
    } break;

    case 1: { // Send RESP (stage=2)
      Serial.println("[S1] TX RESP (stage=2)");
      DW3000.ds_sendFrame(2);

      rx = DW3000.readRXTimestamp();  // t2
      tx = DW3000.readTXTimestamp();  // t3
      t_replyB = tx - rx;             // Treply1 = t3 - t2

      print_u64ln_hex("t2_rx", rx);
      print_u64ln_hex("t3_tx", tx);
      print_i64ln("Treply1(t3-t2)", (int64_t)t_replyB);

      Serial.println("[S1] → S2 (await FINAL stage=3)");
      curr_stage = 2;
    } break;

    case 2: { // Await FINAL (expect stage=3)
      int rs = DW3000.receivedFrameSucc();
      if (rs) {
        rx_status = rs;
        DW3000.clearSystemStatus();

        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame()) {
            g_errFrames++;
            Serial.println("[S2] Warning: error frame → S0, RX");
            curr_stage = 0;
            DW3000.standardRX();
          } else {
            int stg = DW3000.ds_getStage();
            if (stg != 3) {
              g_wrongStage++;
              Serial.print("[S2] Wrong stage "); Serial.print(stg);
              Serial.println(" (expected 3). Send ErrorFrame → S0");
              DW3000.ds_sendErrorFrame();
              DW3000.standardRX();
              curr_stage = 0;
            } else {
              Serial.println("[S2] OK: FINAL received (stage=3) → S3");
              curr_stage = 3;
            }
          }
        } else {
          g_rxErr++;
          Serial.println("[S2] RX error → S0, RX");
          curr_stage = 0;
          DW3000.standardRX();
        }
      }
    } break;

    case 3: { // Compute, send RTInfo
      rx = DW3000.readRXTimestamp();   // t6
      t_roundB = rx - tx;              // Tround2 = t6 - t3

      print_u64ln_hex("t6_rx", rx);
      print_i64ln("Tround2(t6-t3)", (int64_t)t_roundB);
      print_i64ln("Treply1(prev)",  (int64_t)t_replyB);

      DW3000.ds_sendRTInfo(t_roundB, t_replyB);

      g_ok++;
      Serial.print("[S3] HANDSHAKE SUCCESS. ok=");
      Serial.print(g_ok);
      Serial.print(" wrongStage=");
      Serial.print(g_wrongStage);
      Serial.print(" rxErr=");
      Serial.print(g_rxErr);
      Serial.print(" errFrames=");
      Serial.println(g_errFrames);

      curr_stage = 0;
      DW3000.standardRX();
      Serial.println("[S3] → S0 (listening)");
    } break;

    default:
      Serial.print("[ERR] Unknown stage "); Serial.print(curr_stage);
      Serial.println(" → S0");
      curr_stage = 0;
      DW3000.standardRX();
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