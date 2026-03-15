// ================================================================
// receiver-feathers3-to-propeller.ino
// UM FeatherS3 (ESP32-S3) : WiFi TCP Client -> UART to Propeller
//
// DATA FLOW:
//   ESP32-S3 transmitter --TCP binary--> THIS --UART--> Propeller --> Servos
//
// UART TO PROPELLER:
//   ASCII format: L<3digits>R<3digits>G<3digits>\n
//   Example: L090R090G000\n  (stop, gripper open)
//   Example: L120R060G180\n  (forward, gripper close)
//
//   Sent at most every 40ms (25 Hz), only when values change
//   or every 500ms as keepalive.
//
//   L/R = wheel servo angle 0..180 (90 = stop for continuous rotation)
//   G   = gripper servo angle 0..180 (0 = open, 180 = close)
//
// WIRING:
//   FeatherS3 TX (GPIO1) --> Propeller P7 (serial in)
//   FeatherS3 RX (GPIO2) <-- Propeller P8 (serial out, optional debug)
//   FeatherS3 GND        --> Propeller GND
//   Both boards powered independently (USB or battery)
//
// NOTE: ESP32-S3 is 3.3V, Propeller Activity Board is 3.3V.
//       Direct wire, no level shifting needed.
// ================================================================

#include <WiFi.h>

// ===================== CONFIGURATION =====================

static const char    *WIFI_SSID   = "ESP32S3_HOTSPOT";
static const char    *WIFI_PASS   = "esp32pass123";
static const char    *SERVER_IP   = "192.168.4.1";
static const uint16_t SERVER_PORT = 5000;
static const uint16_t ROBOT_ID   = 248;

// UART to Propeller (Serial1)
// UM FeatherS3: TX pin = GPIO43, RX pin = GPIO44
static const int      PROP_TX_PIN = 33;   // FeatherS3 GPIO33 -> Propeller P0
static const int      PROP_RX_PIN = 38;   // FeatherS3 GPIO38 <- Propeller P1
static const uint32_t PROP_BAUD   = 115200;

// Timing
static const uint32_t WIFI_RECONNECT_MS    = 3000;
static const uint32_t TCP_RECONNECT_MS     = 2000;
static const uint32_t PROP_SEND_INTERVAL_MS = 40;   // 25 Hz max
static const uint32_t PROP_KEEPALIVE_MS     = 500;
static const uint32_t HB_TIMEOUT_MS         = 8000;

// Gripper angle mapping
static const uint8_t GRIPPER_OPEN_ANGLE  = 0;
static const uint8_t GRIPPER_CLOSE_ANGLE = 180;

// ===================== RGB LED (UM FeatherS3 onboard NeoPixel) =====================
// NeoPixel WS2812 on GPIO40, powered by LDO2 enabled via GPIO39

#define RGB_PIN       40
#define RGB_POWER_PIN 39

void ledInit() {
  pinMode(RGB_POWER_PIN, OUTPUT);
  digitalWrite(RGB_POWER_PIN, HIGH);  // enable LDO2 to power the NeoPixel
}

void ledOff()    { neopixelWrite(RGB_PIN, 0,  0,  0 ); }
void ledBlue()   { neopixelWrite(RGB_PIN, 0,  0,  40); }
void ledGreen()  { neopixelWrite(RGB_PIN, 0,  40, 0 ); }
void ledRed()    { neopixelWrite(RGB_PIN, 40, 0,  0 ); }
void ledYellow() { neopixelWrite(RGB_PIN, 40, 20, 0 ); }
void ledWhite()  { neopixelWrite(RGB_PIN, 10, 10, 10); }

// Non-blocking flash: set color, auto-off after durationMs
uint32_t ledOffTime = 0;
void ledFlash(void (*colorFn)(), uint32_t durationMs) {
  colorFn();
  ledOffTime = millis() + durationMs;
  if (ledOffTime == 0) ledOffTime = 1;
}
void ledUpdate() {
  if (ledOffTime != 0 && millis() >= ledOffTime) {
    ledOff();
    ledOffTime = 0;
  }
}

// ===================== PACKET DEFINITIONS =====================

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

WiFiClient client;

uint8_t curLeft    = 90;
uint8_t curRight   = 90;
uint8_t curGrip    = GRIPPER_OPEN_ANGLE;

uint8_t sentLeft   = 255;
uint8_t sentRight  = 255;
uint8_t sentGrip   = 255;

uint32_t lastWifiAttemptMs = 0;
uint32_t lastTcpAttemptMs  = 0;
uint32_t lastPropSendMs    = 0;
uint32_t lastHbMs          = 0;

// ===================== HELPERS =====================

uint8_t computeChecksum(const uint8_t *data, size_t len) {
  uint8_t x = 0;
  for (size_t i = 0; i < len; i++) x ^= data[i];
  return x;
}

uint8_t clamp180(uint8_t v) {
  return v > 180 ? 180 : v;
}

uint8_t gripperToAngle(uint8_t cmd) {
  if (cmd == GRIPPER_OPEN)  return GRIPPER_OPEN_ANGLE;
  if (cmd == GRIPPER_CLOSE) return GRIPPER_CLOSE_ANGLE;
  return GRIPPER_OPEN_ANGLE;
}

// ===================== LINE-BASED LOGGING =====================
// Each data type prints on its own line, appending horizontally.
// When the type changes, a newline is inserted and a new label starts.

#define LINE_NONE 0
#define LINE_HB   1
#define LINE_CTRL 2
#define LINE_TX   3
#define LINE_SYS  4

uint8_t lastLine = LINE_NONE;

void switchLine(uint8_t next, const char *label) {
  if (lastLine != next) {
    if (lastLine != LINE_NONE) Serial.println();
    Serial.printf("[%s] ", label);
    lastLine = next;
  }
}

// ===================== WIFI / TCP =====================

void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  uint32_t now = millis();
  if (now - lastWifiAttemptMs < WIFI_RECONNECT_MS) return;
  lastWifiAttemptMs = now;

  switchLine(LINE_SYS, "WiFi");
  Serial.print("connecting.. ");
  ledBlue();
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void ensureTcp() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (client.connected()) return;
  uint32_t now = millis();
  if (now - lastTcpAttemptMs < TCP_RECONNECT_MS) return;
  lastTcpAttemptMs = now;

  switchLine(LINE_SYS, "TCP");
  Serial.printf("%s:%u.. ", SERVER_IP, SERVER_PORT);
  ledBlue();
  client.stop();
  if (client.connect(SERVER_IP, SERVER_PORT)) {
    client.setNoDelay(true);
    client.printf("ID:%u\n", ROBOT_ID);
    Serial.printf("OK(ID:%u) ", ROBOT_ID);
    lastHbMs = millis();
    ledFlash(ledGreen, 300);
  } else {
    Serial.print("FAIL ");
    ledOff();
  }
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
    switchLine(LINE_SYS, "ERR");
    Serial.printf("bad_chk:0x%02X/0x%02X ", pkt.checksum, cs);
    return;
  }
  if (pkt.id != ROBOT_ID) return;

  curLeft  = clamp180(pkt.left);
  curRight = clamp180(pkt.right);
  curGrip  = gripperToAngle(pkt.gripper);

  switchLine(LINE_CTRL, "CTRL");
  Serial.printf("L%u,R%u,G%u(=%u)#%u ", pkt.left, pkt.right, pkt.gripper, curGrip, pkt.seq);

  lastHbMs = millis();
}

void handleHeartbeatPacket() {
  HeartbeatPacket pkt;
  memcpy(&pkt, rxBuf, sizeof(pkt));

  uint8_t cs = computeChecksum(rxBuf, sizeof(pkt) - 1);
  if (cs != pkt.checksum) return;
  if (pkt.id != ROBOT_ID) return;

  lastHbMs = millis();
  ledFlash(ledWhite, 30);

  switchLine(LINE_HB, "HB");
  Serial.print(". ");
}

void serviceTcp() {
  if (!client.connected()) return;

  while (client.available()) {
    uint8_t b = (uint8_t)client.read();

    switch (rxState) {
      case RX_SYNC:
        if (b == 0xAA) {
          rxBuf[0] = b;
          rxState  = RX_TYPE;
        }
        break;

      case RX_TYPE:
        rxBuf[1] = b;
        rxType   = b;
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

// ===================== PROPELLER UART OUTPUT =====================

void servicePropOutput() {
  uint32_t now = millis();
  if (now - lastPropSendMs < PROP_SEND_INTERVAL_MS) return;

  bool changed = (curLeft != sentLeft) ||
                 (curRight != sentRight) ||
                 (curGrip != sentGrip);

  bool keepalive = (now - lastPropSendMs >= PROP_KEEPALIVE_MS);

  if (!changed && !keepalive) return;

  char buf[20];
  snprintf(buf, sizeof(buf), "L%03uR%03uG%03u\n", curLeft, curRight, curGrip);
  Serial1.print(buf);
  lastPropSendMs = now;

  sentLeft  = curLeft;
  sentRight = curRight;
  sentGrip  = curGrip;

  if (changed) {
    switchLine(LINE_TX, "TX");
    Serial.printf("%s ", buf);
  }
}

// ===================== SETUP / LOOP =====================

void setup() {
  Serial.begin(115200);
  delay(1000);

  ledInit();
  ledOff();

  Serial1.begin(PROP_BAUD, SERIAL_8N1, PROP_RX_PIN, PROP_TX_PIN);
  delay(50);

  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.println("===== FeatherS3 RECEIVER -> Propeller =====");
  Serial.printf("Robot ID : %u\n", ROBOT_ID);
  Serial.printf("WiFi     : %s\n", WIFI_SSID);
  Serial.printf("Server   : %s:%u\n", SERVER_IP, SERVER_PORT);
  Serial.printf("UART     : TX=GPIO%d RX=GPIO%d @ %u baud\n",
                PROP_TX_PIN, PROP_RX_PIN, PROP_BAUD);
  Serial.printf("Prop rate: every %u ms\n", PROP_SEND_INTERVAL_MS);
  Serial.println("============================================");
  Serial.println();
}

void loop() {
  ledUpdate();

  ensureWiFi();
  if (WiFi.status() == WL_CONNECTED) ensureTcp();

  serviceTcp();
  servicePropOutput();

  // Heartbeat timeout
  if (client.connected() && (millis() - lastHbMs > HB_TIMEOUT_MS)) {
    switchLine(LINE_SYS, "WARN");
    Serial.print("HB_timeout ");
    ledFlash(ledYellow, 80);
    client.stop();
  }

  delay(1);
}
