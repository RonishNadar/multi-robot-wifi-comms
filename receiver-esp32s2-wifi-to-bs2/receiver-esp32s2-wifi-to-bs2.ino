// ================================================================
// receiver-esp32s2-wifi-to-bs2.ino
// ESP32-S2 : WiFi STA + TCP Client -> Binary frame to BS2
//
// DATA FLOW:
//   ESP32-S3 transmitter --TCP binary packets--> THIS --Serial1 binary frame--> BS2
//
// BS2 FRAME (7 bytes, sent via Serial1.write):
//   Byte 0: 0x21 '!'   sync marker 1
//   Byte 1: 0x21 '!'   sync marker 2
//   Byte 2: idLo       robot ID low byte (little-endian)
//   Byte 3: idHi       robot ID high byte
//   Byte 4: leftV      left servo  0..180 (90 = stop)
//   Byte 5: rightV     right servo 0..180 (90 = stop)
//   Byte 6: gripV      gripper     0..180 (0 = open, 180 = close)
//
// ACK PROTOCOL (GPIO, not serial):
//   BS2 pulses its ackPin (P1) HIGH for 10 ms after valid frame.
//   ESP32 reads this on ACK_PIN (GPIO13) via voltage divider.
//   If no ACK within ACK_TIMEOUT_MS, ESP32 resends same frame.
//   Latest command always wins: if new data arrives during retry,
//   the frame is updated and retry continues with new values.
//
// WIRING (ESP32-S2-DevKitC-1):
//   GPIO14 = Serial1 TX  -> BS2 P0 (SERIN rxPin)  [direct wire]
//   GPIO13 = ACK input   <- BS2 P1 (ackPin)        [voltage divider 5V->3.3V]
//   GPIO12 = Serial1 RX  (assigned, not connected)
//   5V     -> BS2 Vin
//   GND    -> BS2 GND
//
// TIMING:
//   BS2 loop: ~34 ms (20 ms SERIN + ~4 ms PULSOUT + 10 ms PAUSE)
//   CMD_INTERVAL_MS: 40 ms (just above one BS2 cycle, no backlog)
//   ACK_TIMEOUT_MS:  80 ms (~2.5 BS2 cycles, safe retry margin)
// ================================================================

#include <WiFi.h>

// ===================== CONFIGURATION =====================

static const char    *WIFI_SSID   = "ESP32S3_HOTSPOT";
static const char    *WIFI_PASS   = "esp32pass123";
static const char    *SERVER_IP   = "192.168.4.1";
static const uint16_t SERVER_PORT = 5000;
static const uint16_t MY_ID      = 147;

// BS2 UART pins
#define BS2_TX_PIN  14     // ESP32 TX -> BS2 P0 (serial in)
#define BS2_RX_PIN  12     // ESP32 RX (assigned but unconnected)
#define BS2_BAUD    9600

// ACK pin: BS2 P1 -> voltage divider -> GPIO13
#define ACK_PIN     13
static const uint32_t ACK_TIMEOUT_MS = 45;
static const uint16_t ACK_POLL_US    = 100;

// Rate limiter: min interval between BS2 sends
static const uint32_t CMD_INTERVAL_MS = 40;

// Reconnect timing
static const uint32_t RECONNECT_DELAY_MS    = 2000;
static const uint32_t WIFI_REJOIN_TIMEOUT_MS = 30000;

// Heartbeat watchdog
static const uint32_t HB_TIMEOUT_MS = 8000;

// Gripper angle mapping
static const uint8_t GRIPPER_OPEN_ANGLE  = 0;
static const uint8_t GRIPPER_CLOSE_ANGLE = 180;

// ===================== TCP PACKET DEFINITIONS =====================

enum PacketType : uint8_t {
  PKT_CONTROL   = 1,
  PKT_HEARTBEAT = 2
};

enum GripperCmd : uint8_t {
  GRIPPER_NONE  = 0,
  GRIPPER_OPEN  = 1,
  GRIPPER_CLOSE = 2
};

#pragma pack(push, 1)
struct ControlPacket {
  uint8_t  start;
  uint8_t  type;
  uint16_t id;
  uint8_t  left;
  uint8_t  right;
  uint8_t  gripper;
  uint8_t  seq;
  uint8_t  checksum;
};

struct HeartbeatPacket {
  uint8_t  start;
  uint8_t  type;
  uint16_t id;
  uint8_t  seq;
  uint8_t  checksum;
};
#pragma pack(pop)

// ===================== STATE =====================

WiFiClient tcpClient;

// BS2 frame: latest command to send
struct BS2Frame { uint8_t data[7]; };
BS2Frame lastFrame   = { {0} };
bool     hasCommand  = false;
bool     pendingFlag = false;

// Timing
uint32_t lastHbMs      = 0;
uint32_t lastCmdSentMs = 0;

// ===================== HELPERS =====================

uint8_t computeChecksum(const uint8_t *data, size_t len) {
  uint8_t x = 0;
  for (size_t i = 0; i < len; i++) x ^= data[i];
  return x;
}

uint8_t clamp180(int v) {
  if (v < 0)   return 0;
  if (v > 180) return 180;
  return (uint8_t)v;
}

// Convert gripper command enum (0/1/2) to servo angle (0..180)
uint8_t gripperToAngle(uint8_t cmd) {
  if (cmd == GRIPPER_OPEN)  return GRIPPER_OPEN_ANGLE;
  if (cmd == GRIPPER_CLOSE) return GRIPPER_CLOSE_ANGLE;
  return GRIPPER_OPEN_ANGLE;  // default: open
}

// ===================== LED (ESP32-S2 onboard WS2812) =====================

#define RGB_PIN 18
void ledOff()    { neopixelWrite(RGB_PIN, 0,  0,  0 ); }
void ledBlue()   { neopixelWrite(RGB_PIN, 0,  0,  40); }
void ledGreen()  { neopixelWrite(RGB_PIN, 0,  40, 0 ); }
void ledRed()    { neopixelWrite(RGB_PIN, 40, 0,  0 ); }
void ledYellow() { neopixelWrite(RGB_PIN, 40, 20, 0 ); }
void ledWhite()  { neopixelWrite(RGB_PIN, 10, 10, 10); }

uint32_t ledOffTime = 0;
void ledFlash(void (*fn)(), uint32_t ms) {
  fn();
  ledOffTime = millis() + ms;
  if (ledOffTime == 0) ledOffTime = 1;
}
void ledUpdate() {
  if (ledOffTime && millis() >= ledOffTime) { ledOff(); ledOffTime = 0; }
}

// ===================== WIFI / TCP =====================

bool rejoinWifi() {
  Serial.printf("[WiFi] Joining %s...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > WIFI_REJOIN_TIMEOUT_MS) {
      Serial.println("[WiFi] Timeout");
      return false;
    }
    delay(300);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected, IP: %s\n",
                WiFi.localIP().toString().c_str());
  return true;
}

bool connectToTransmitter() {
  Serial.printf("[TCP] Connecting %s:%u...\n", SERVER_IP, SERVER_PORT);
  ledBlue();
  if (!tcpClient.connect(SERVER_IP, SERVER_PORT, 5000)) {
    Serial.println("[TCP] Failed");
    ledOff();
    return false;
  }
  tcpClient.setNoDelay(true);

  // Register with transmitter
  tcpClient.printf("ID:%u\n", MY_ID);
  Serial.printf("[TCP] Connected, sent ID:%u\n", MY_ID);

  lastHbMs = millis();
  ledFlash(ledGreen, 300);
  return true;
}

void doReconnect() {
  tcpClient.stop();
  delay(RECONNECT_DELAY_MS);
  if (WiFi.status() != WL_CONNECTED) {
    while (!rejoinWifi()) delay(RECONNECT_DELAY_MS);
  }
  while (!connectToTransmitter()) delay(RECONNECT_DELAY_MS);
}

// ===================== TCP PACKET READING (non-blocking) =====================

enum RxState : uint8_t { RX_SYNC, RX_TYPE, RX_BODY };

RxState  rxState    = RX_SYNC;
uint8_t  rxType     = 0;
uint8_t  rxBuf[16];
size_t   rxExpected = 0;
size_t   rxGot      = 0;

void resetRx() { rxState = RX_SYNC; }

void handleControlPacket() {
  ControlPacket pkt;
  memcpy(&pkt, rxBuf, sizeof(pkt));

  uint8_t cs = computeChecksum(rxBuf, sizeof(pkt) - 1);
  if (cs != pkt.checksum) {
    Serial.printf("[PKT] Bad checksum: 0x%02X vs 0x%02X\n", pkt.checksum, cs);
    return;
  }
  if (pkt.id != MY_ID) return;

  // Build BS2 frame
  uint8_t leftV  = clamp180(pkt.left);
  uint8_t rightV = clamp180(pkt.right);
  uint8_t gripV  = gripperToAngle(pkt.gripper);

  lastFrame.data[0] = 0x21;  // '!'
  lastFrame.data[1] = 0x21;  // '!'
  lastFrame.data[2] = (uint8_t)(MY_ID & 0xFF);
  lastFrame.data[3] = (uint8_t)((MY_ID >> 8) & 0xFF);
  lastFrame.data[4] = leftV;
  lastFrame.data[5] = rightV;
  lastFrame.data[6] = gripV;

  pendingFlag = true;
  hasCommand  = true;

  Serial.printf("[PKT] CTRL L=%u R=%u G=%u (gripAngle=%u) seq=%u\n",
                pkt.left, pkt.right, pkt.gripper, gripV, pkt.seq);
}

void handleHeartbeatPacket() {
  HeartbeatPacket pkt;
  memcpy(&pkt, rxBuf, sizeof(pkt));

  uint8_t cs = computeChecksum(rxBuf, sizeof(pkt) - 1);
  if (cs != pkt.checksum) return;
  if (pkt.id != MY_ID) return;

  lastHbMs = millis();
  ledFlash(ledWhite, 30);
}

void serviceTcpRead() {
  while (tcpClient.available()) {
    uint8_t b = (uint8_t)tcpClient.read();

    switch (rxState) {
      case RX_SYNC:
        if (b == 0xAA) {
          rxBuf[0] = b;
          rxState  = RX_TYPE;
        }
        break;

      case RX_TYPE:
        rxBuf[1] = b;
        rxType   = b;   // <-- MUST store type for RX_BODY dispatch
        if (b == PKT_CONTROL) {
          rxExpected = sizeof(ControlPacket);
          rxGot = 2;
          rxState = RX_BODY;
        } else if (b == PKT_HEARTBEAT) {
          rxExpected = sizeof(HeartbeatPacket);
          rxGot = 2;
          rxState = RX_BODY;
        } else {
          resetRx();
        }
        break;

      case RX_BODY:
        rxBuf[rxGot++] = b;
        if (rxGot >= rxExpected) {
          if (rxType == PKT_CONTROL)        handleControlPacket();
          else if (rxType == PKT_HEARTBEAT) handleHeartbeatPacket();
          resetRx();
        }
        break;
    }
  }
}

// Also service TCP during ACK wait so we don't miss incoming data
void serviceTcpDuringWait() {
  while (tcpClient.available()) {
    uint8_t b = (uint8_t)tcpClient.read();
    switch (rxState) {
      case RX_SYNC:
        if (b == 0xAA) { rxBuf[0] = b; rxState = RX_TYPE; }
        break;
      case RX_TYPE:
        rxBuf[1] = b;
        rxType   = b;   // <-- MUST store type for RX_BODY dispatch
        if (b == PKT_CONTROL)        { rxExpected = sizeof(ControlPacket);   rxGot = 2; rxState = RX_BODY; }
        else if (b == PKT_HEARTBEAT) { rxExpected = sizeof(HeartbeatPacket); rxGot = 2; rxState = RX_BODY; }
        else resetRx();
        break;
      case RX_BODY:
        rxBuf[rxGot++] = b;
        if (rxGot >= rxExpected) {
          if (rxType == PKT_CONTROL)        handleControlPacket();
          else if (rxType == PKT_HEARTBEAT) handleHeartbeatPacket();
          resetRx();
        }
        break;
    }
  }
}

// ===================== BS2 SEND + ACK =====================

static const uint8_t MAX_RETRIES = 3;          // max retry attempts per command
static const uint32_t RETRY_GAP_MS = 35;       // gap between retries (~1 BS2 loop)

// Wait for ACK pulse on ACK_PIN, servicing TCP while waiting.
// Returns true if ACK received within ACK_TIMEOUT_MS.
bool waitForAck() {
  uint32_t t0 = millis();
  while (millis() - t0 < ACK_TIMEOUT_MS) {
    serviceTcpDuringWait();
    if (digitalRead(ACK_PIN) == HIGH) {
      // Wait for pulse to end so we don't double-trigger
      while (digitalRead(ACK_PIN) == HIGH) {
        delayMicroseconds(ACK_POLL_US);
      }
      Serial.printf("[ACK] received after %lu ms\n", millis() - t0);
      return true;
    }
    delayMicroseconds(ACK_POLL_US);
  }
  return false;
}

// Send frame to BS2 and wait for ACK. Retries up to MAX_RETRIES
// times with RETRY_GAP_MS between each attempt. Total worst case:
// (MAX_RETRIES+1) * (ACK_TIMEOUT_MS + RETRY_GAP_MS) ~ 320 ms
// but typically succeeds on 1st or 2nd try within ~45 ms.
bool sendAndWaitAck() {
  for (uint8_t attempt = 0; attempt <= MAX_RETRIES; attempt++) {

    if (attempt > 0) {
      // Gap before retry: let BS2 finish its current loop and re-enter SERIN.
      // Service TCP during the gap so we don't miss packets.
      uint32_t gapStart = millis();
      while (millis() - gapStart < RETRY_GAP_MS) {
        serviceTcpDuringWait();
        delayMicroseconds(500);
      }
    }

    // Send 7-byte binary frame
    Serial1.write(lastFrame.data, 7);

    if (attempt == 0) {
      Serial.printf("[BS2 TX] L=%u R=%u G=%u\n",
                    lastFrame.data[4], lastFrame.data[5], lastFrame.data[6]);
    } else {
      Serial.printf("[BS2 TX] retry #%u\n", attempt);
    }

    if (waitForAck()) {
      pendingFlag = false;
      return true;
    }
  }

  Serial.println("[BS2 TX] giving up after retries");
  // pendingFlag stays true: will try again on next loop if still pending
  return false;
}

// ===================== SETUP =====================

void setup() {
  ledOff();
  Serial.begin(115200);
  delay(300);

  pinMode(ACK_PIN, INPUT);
  Serial1.begin(BS2_BAUD, SERIAL_8N1, BS2_RX_PIN, BS2_TX_PIN);
  delay(50);

  WiFi.mode(WIFI_STA);
  while (!rejoinWifi())           delay(RECONNECT_DELAY_MS);
  while (!connectToTransmitter()) delay(RECONNECT_DELAY_MS);

  Serial.println();
  Serial.println("===== ESP32-S2 RECEIVER (ACK mode) =====");
  Serial.printf("MY_ID       : %u\n", MY_ID);
  Serial.printf("Transmitter : %s:%u\n", SERVER_IP, SERVER_PORT);
  Serial.printf("BS2 TX pin  : GPIO%d\n", BS2_TX_PIN);
  Serial.printf("ACK pin     : GPIO%d\n", ACK_PIN);
  Serial.printf("ACK timeout : %lu ms\n", ACK_TIMEOUT_MS);
  Serial.printf("Cmd interval: %lu ms\n", CMD_INTERVAL_MS);
  Serial.printf("HB timeout  : %lu ms\n", HB_TIMEOUT_MS);
  Serial.printf("Grip open=%u close=%u\n", GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE);
  Serial.println("=========================================");
  Serial.println();
}

// ===================== LOOP =====================

void loop() {
  ledUpdate();

  // Connection health
  if (!tcpClient.connected()) {
    Serial.println("[WARN] TCP lost");
    doReconnect();
    return;
  }
  if (millis() - lastHbMs > HB_TIMEOUT_MS) {
    Serial.println("[WARN] Heartbeat timeout");
    doReconnect();
    return;
  }

  // Read TCP packets
  serviceTcpRead();

  // Send pending command to BS2 with rate limiting
  if (hasCommand && pendingFlag) {
    if (millis() - lastCmdSentMs >= CMD_INTERVAL_MS) {
      bool acked = sendAndWaitAck();
      lastCmdSentMs = millis();  // rate limit from last attempt, success or not
    }
  }
}
