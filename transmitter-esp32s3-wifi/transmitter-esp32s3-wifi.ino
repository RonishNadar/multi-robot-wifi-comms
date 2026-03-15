// ================================================================
// transmitter-esp32s3-wifi.ino
// ESP32-S3 : WiFi Access Point + TCP Server
//
// PC --serial JSON--> ESP32-S3 --TCP binary--> ESP32-S2 receiver(s)
//
// JSON from PC (USB serial, 115200):
//   {"id":147,"left":90,"right":90,"gripper":"open"}
//   {"id":147,"left":120,"right":60}
//   {"id":147,"gripper":"close"}
//
// Binary packets over TCP:
//   ControlPacket  (9 bytes): 0xAA | type=1 | id(2) | L | R | G | seq | chk
//   HeartbeatPacket (6 bytes): 0xAA | type=2 | id(2) | seq | chk
// ================================================================

#include <WiFi.h>
#include <ArduinoJson.h>

// ===================== CONFIGURATION =====================

static const char    *AP_SSID     = "ESP32S3_HOTSPOT";
static const char    *AP_PASSWORD = "esp32pass123";
static const uint16_t TCP_PORT    = 5000;

static const int MAX_ROBOTS  = 16;
static const int MAX_CLIENTS = 8;

static const uint32_t CONTROL_SEND_INTERVAL_MS = 50;
static const uint32_t HEARTBEAT_INTERVAL_MS    = 1000;
static const uint32_t CLIENT_IDLE_TIMEOUT_MS   = 10000;

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
  uint8_t  start;      // 0xAA
  uint8_t  type;       // PKT_CONTROL
  uint16_t id;
  uint8_t  left;       // 0..180
  uint8_t  right;      // 0..180
  uint8_t  gripper;    // 0, 1, 2
  uint8_t  seq;
  uint8_t  checksum;
};

struct HeartbeatPacket {
  uint8_t  start;      // 0xAA
  uint8_t  type;       // PKT_HEARTBEAT
  uint16_t id;
  uint8_t  seq;
  uint8_t  checksum;
};
#pragma pack(pop)

// ===================== CLIENT SLOT =====================

struct ClientSlot {
  WiFiClient client;
  bool       active     = false;
  bool       registered = false;
  uint16_t   robotId    = 0;
  String     rxLine     = "";
  uint32_t   lastRxMs   = 0;
};

// ===================== ROBOT STATE =====================

struct RobotState {
  bool     used = false;
  uint16_t id   = 0;

  uint8_t left    = 90;
  uint8_t right   = 90;
  uint8_t gripper = GRIPPER_NONE;

  uint8_t lastSentLeft    = 255;
  uint8_t lastSentRight   = 255;
  uint8_t lastSentGripper = 255;

  uint8_t  seq             = 0;
  uint32_t lastSendMs      = 0;
  uint32_t lastHeartbeatMs = 0;

  int clientIndex = -1;
};

// ===================== GLOBALS =====================

WiFiServer server(TCP_PORT);
ClientSlot clientSlots[MAX_CLIENTS];
RobotState robots[MAX_ROBOTS];
String     serialLine;

// ===================== HELPERS =====================

uint8_t computeChecksum(const uint8_t *data, size_t len) {
  uint8_t x = 0;
  for (size_t i = 0; i < len; i++) x ^= data[i];
  return x;
}

uint8_t clampAngle(int v) {
  if (v < 0)   return 0;
  if (v > 180) return 180;
  return (uint8_t)v;
}

uint8_t parseGripper(const JsonVariantConst &v) {
  if (v.isNull()) return GRIPPER_NONE;
  if (v.is<const char*>()) {
    const char *s = v.as<const char*>();
    if (!s) return GRIPPER_NONE;
    if (strcasecmp(s, "open")  == 0) return GRIPPER_OPEN;
    if (strcasecmp(s, "close") == 0) return GRIPPER_CLOSE;
    return GRIPPER_NONE;
  }
  if (v.is<int>()) {
    int n = v.as<int>();
    if (n == 1) return GRIPPER_OPEN;
    if (n == 2) return GRIPPER_CLOSE;
  }
  return GRIPPER_NONE;
}

RobotState* findRobot(uint16_t id) {
  for (int i = 0; i < MAX_ROBOTS; i++) {
    if (robots[i].used && robots[i].id == id) return &robots[i];
  }
  return nullptr;
}

RobotState* getOrCreateRobot(uint16_t id) {
  RobotState *r = findRobot(id);
  if (r) return r;
  for (int i = 0; i < MAX_ROBOTS; i++) {
    if (!robots[i].used) {
      robots[i] = {};
      robots[i].used  = true;
      robots[i].id    = id;
      robots[i].left  = 90;
      robots[i].right = 90;
      robots[i].lastSentLeft    = 255;
      robots[i].lastSentRight   = 255;
      robots[i].lastSentGripper = 255;
      robots[i].clientIndex = -1;
      return &robots[i];
    }
  }
  return nullptr;
}

// ===================== CLIENT MANAGEMENT =====================

int findClientForRobot(uint16_t robotId) {
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (clientSlots[i].active && clientSlots[i].registered &&
        clientSlots[i].robotId == robotId && clientSlots[i].client.connected())
      return i;
  }
  return -1;
}

void closeClientSlot(int idx) {
  if (idx < 0 || idx >= MAX_CLIENTS) return;
  for (int i = 0; i < MAX_ROBOTS; i++) {
    if (robots[i].used && robots[i].clientIndex == idx)
      robots[i].clientIndex = -1;
  }
  if (clientSlots[idx].client) clientSlots[idx].client.stop();
  clientSlots[idx].active     = false;
  clientSlots[idx].registered = false;
  clientSlots[idx].robotId    = 0;
  clientSlots[idx].rxLine     = "";
  clientSlots[idx].lastRxMs   = 0;
  Serial.printf("[TCP] Closed slot %d\n", idx);
}

int getFreeSlot() {
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (!clientSlots[i].active) return i;
  }
  return -1;
}

void acceptNewClients() {
  while (server.hasClient()) {
    WiFiClient nc = server.accept();
    int slot = getFreeSlot();
    if (slot < 0) { nc.stop(); continue; }
    clientSlots[slot].client     = nc;
    clientSlots[slot].active     = true;
    clientSlots[slot].registered = false;
    clientSlots[slot].robotId    = 0;
    clientSlots[slot].rxLine     = "";
    clientSlots[slot].lastRxMs   = millis();
    Serial.printf("[TCP] Slot %d: client from %s\n",
                  slot, nc.remoteIP().toString().c_str());
  }
}

void handleRegistration(int idx, const String &line) {
  String s = line;
  s.trim();

  if (s.length() == 0) return;

  Serial.printf("[TCP] Slot %d raw line (%d chars): '", idx, s.length());
  for (int j = 0; j < (int)s.length() && j < 32; j++) {
    char c = s[j];
    if (c >= 32 && c <= 126) Serial.print(c);
    else Serial.printf("\\x%02X", (uint8_t)c);
  }
  Serial.println("'");

  if (!s.startsWith("ID:")) {
    Serial.printf("[TCP] Slot %d: not an ID line\n", idx);
    return;
  }
  long parsed = s.substring(3).toInt();
  if (parsed <= 0 || parsed > 65535) return;
  uint16_t rid = (uint16_t)parsed;

  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (i != idx && clientSlots[i].active &&
        clientSlots[i].registered && clientSlots[i].robotId == rid)
      closeClientSlot(i);
  }
  clientSlots[idx].registered = true;
  clientSlots[idx].robotId    = rid;
  clientSlots[idx].lastRxMs   = millis();
  RobotState *r = getOrCreateRobot(rid);
  if (r) r->clientIndex = idx;
  clientSlots[idx].client.print("OK\n");
  Serial.printf("[TCP] Slot %d = robot %u\n", idx, rid);
}

void serviceClients() {
  uint32_t now = millis();
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (!clientSlots[i].active) continue;
    WiFiClient &c = clientSlots[i].client;
    if (!c.connected()) { closeClientSlot(i); continue; }
    while (c.available()) {
      char ch = (char)c.read();
      clientSlots[i].lastRxMs = now;
      if (ch == '\r') continue;
      if (ch == '\n') {
        handleRegistration(i, clientSlots[i].rxLine);
        clientSlots[i].rxLine = "";
      } else {
        if (clientSlots[i].rxLine.length() < 64) clientSlots[i].rxLine += ch;
        else clientSlots[i].rxLine = "";
      }
    }
    if (!clientSlots[i].registered && (now - clientSlots[i].lastRxMs > CLIENT_IDLE_TIMEOUT_MS))
      closeClientSlot(i);
  }
  for (int i = 0; i < MAX_ROBOTS; i++) {
    if (robots[i].used)
      robots[i].clientIndex = findClientForRobot(robots[i].id);
  }
}

// ===================== PACKET SENDING =====================

bool sendControlPacket(RobotState &r) {
  if (r.clientIndex < 0 || !clientSlots[r.clientIndex].active) return false;
  WiFiClient &c = clientSlots[r.clientIndex].client;
  if (!c.connected()) return false;

  ControlPacket pkt;
  pkt.start    = 0xAA;
  pkt.type     = PKT_CONTROL;
  pkt.id       = r.id;
  pkt.left     = r.left;
  pkt.right    = r.right;
  pkt.gripper  = r.gripper;
  pkt.seq      = r.seq++;
  pkt.checksum = computeChecksum((const uint8_t*)&pkt, sizeof(pkt) - 1);
  if (c.write((const uint8_t*)&pkt, sizeof(pkt)) != sizeof(pkt)) return false;

  r.lastSentLeft    = r.left;
  r.lastSentRight   = r.right;
  r.lastSentGripper = r.gripper;
  r.lastSendMs      = millis();
  r.lastHeartbeatMs = r.lastSendMs;
  Serial.printf("[TX] CTRL id=%u L=%u R=%u G=%u seq=%u\n",
                r.id, r.left, r.right, r.gripper, pkt.seq);
  return true;
}

bool sendHeartbeat(RobotState &r) {
  if (r.clientIndex < 0 || !clientSlots[r.clientIndex].active) return false;
  WiFiClient &c = clientSlots[r.clientIndex].client;
  if (!c.connected()) return false;

  HeartbeatPacket pkt;
  pkt.start    = 0xAA;
  pkt.type     = PKT_HEARTBEAT;
  pkt.id       = r.id;
  pkt.seq      = r.seq++;
  pkt.checksum = computeChecksum((const uint8_t*)&pkt, sizeof(pkt) - 1);
  if (c.write((const uint8_t*)&pkt, sizeof(pkt)) != sizeof(pkt)) return false;
  r.lastHeartbeatMs = millis();
  return true;
}

// ===================== SERIAL JSON INPUT =====================

void processJsonLine(const String &line) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, line)) return;
  if (!doc["id"].is<int>()) return;
  int idVal = doc["id"].as<int>();
  if (idVal <= 0 || idVal > 65535) return;

  RobotState *r = getOrCreateRobot((uint16_t)idVal);
  if (!r) return;

  if (doc.containsKey("left"))    r->left    = clampAngle(doc["left"].as<int>());
  if (doc.containsKey("right"))   r->right   = clampAngle(doc["right"].as<int>());
  if (doc.containsKey("gripper")) r->gripper = parseGripper(doc["gripper"]);

  Serial.printf("[JSON] id=%u L=%u R=%u G=%u\n",
                r->id, r->left, r->right, r->gripper);
}

void serviceSerialInput() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      serialLine.trim();
      if (serialLine.length() > 0) processJsonLine(serialLine);
      serialLine = "";
    } else {
      if (serialLine.length() < 300) serialLine += ch;
      else serialLine = "";
    }
  }
}

// ===================== UPDATE DISPATCH =====================

void sendUpdates() {
  uint32_t now = millis();
  for (int i = 0; i < MAX_ROBOTS; i++) {
    if (!robots[i].used) continue;
    RobotState &r = robots[i];
    if (r.clientIndex < 0) continue;

    bool changed = (r.left    != r.lastSentLeft)  ||
                   (r.right   != r.lastSentRight)  ||
                   (r.gripper != r.lastSentGripper);

    if (changed && (now - r.lastSendMs >= CONTROL_SEND_INTERVAL_MS)) {
      // New values: send immediately (rate-limited)
      sendControlPacket(r);
    } else if (now - r.lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
      // Periodic: send BOTH a heartbeat AND a control packet refresh.
      // The heartbeat keeps the receiver's watchdog alive.
      // The control resend ensures the receiver has the latest state
      // even if a previous packet was lost or the receiver reconnected.
      sendHeartbeat(r);
      // Force a control resend by marking lastSent as stale
      r.lastSentLeft    = 255;
      r.lastSentRight   = 255;
      r.lastSentGripper = 255;
    }
  }
}

// ===================== SETUP / LOOP =====================

void setup() {
  Serial.begin(115200);
  delay(1200);

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.println("[WiFi] AP failed!");
    while (1) delay(1000);
  }
  server.begin();
  server.setNoDelay(true);

  Serial.println();
  Serial.println("===== ESP32-S3 TRANSMITTER =====");
  Serial.printf("SSID : %s\n", AP_SSID);
  Serial.printf("IP   : %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("Port : %u\n", TCP_PORT);
  Serial.println("================================");
  Serial.println("JSON: {\"id\":147,\"left\":90,\"right\":90,\"gripper\":\"open\"}");
  Serial.println();
}

void loop() {
  acceptNewClients();
  serviceClients();
  serviceSerialInput();
  sendUpdates();
  delay(1);
}
