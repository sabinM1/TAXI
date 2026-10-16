/*
 * TOF_Bridge.ino — Web Serial ↔ M60F ToF sensor bridge (transparent pass-through).
 *
 * Hardware:
 *   - 5x M60F ToF sensors on D4 (OneWire bus, IDs 1..5 @ 115200 baud)
 *   - USB Serial (Serial) for Web Serial communication from browser
 *
 * Webpage Protocol (matches SoBotz TOF Distance Sensor Tool):
 *   Host -> Bridge (via USB Serial, line-oriented):
 *     R<id><reg>        Read register (id=2 hex digits 01-05, reg=2 hex digits 00-FF)
 *                       Example: R0301 = sensor 3, reg 0x01
 *     W<id><reg><val>   Write register (val=variable hex digits, up to 4 = 16-bit)
 *                       Example: W01060064 = sensor 1, reg 0x06, val 0x064
 *                       Example: W03800 = sensor 3, reg 0x80, val 0x0
 *     S<id>             Save configuration (id=2 hex)
 *     U<id>             Restart sensor (id=2 hex)
 *     Z<id>             Reset configuration (id=2 hex)
 *     P                 Ping -> returns "PONG"
 *     V                 Version -> returns "VER<version>"
 *     H                 Help
 *
 *   Bridge -> Host (via USB Serial):
 *     <hexValue>        Read response (e.g., "0064")
 *     A                 Write/Action ack
 *     PONG              Ping response
 *     VER<version>      Version
 *     ERR<code>         Error (01=timeout, 03=invalid cmd, 04=invalid sensor)
 *
 * OneWire Sensor Protocol (native M60F, ASCII over SingleWireSerial):
 *   R<id_hex><reg_hex>\n        Read (id_hex=01-05, reg_hex=00-FF)
 *   W<id_hex><reg_hex><val_hex>\n  Write (val_hex=variable, up to FFFF)
 *   S<id_hex>\n                 Save
 *   U<id_hex>\n                 Restart
 *   Z<id_hex>\n                 Reset
 *   Response: <hexValue>\n or A\n
 *
 * Libraries: SingleWireSerial (Arduino Library Manager)
 */

#include <Arduino.h>
#include <SingleWireSerial.h>

// ============================================================================
// CONFIGURATION
// ============================================================================
#define PIN_TOF_BUS       4       // OneWire bus pin (shared SIG lines)
#define TOF_BUS_BAUD      115200  // Must match sensor register 0xB0
#define USB_BAUD          115200  // USB Serial baud (Web Serial)
#define TOF_SENSOR_COUNT  5       // Sensors 1..5
#define CMD_TIMEOUT_MS    50      // Per-command timeout on OneWire bus
#define MAX_CMD_LEN       32      // Max command line length

// Firmware version
const char* FIRMWARE_VERSION = "1.0.0";

// ============================================================================
// GLOBALS
// ============================================================================
static SingleWireSerial tofBus(false);  // Half-duplex on D4
static char cmdBuffer[MAX_CMD_LEN];
static uint8_t cmdPos = 0;

// ============================================================================
// HELPERS: HEX PARSING
// ============================================================================
static uint8_t parseHexByte(const char* s) {
  uint8_t v = 0;
  for (uint8_t i = 0; i < 2; i++) {
    char c = s[i];
    uint8_t d = (c <= '9') ? (c - '0') : ((c & 0x1F) + 9);
    v = (v << 4) | d;
  }
  return v;
}

static uint16_t parseHexVar(const char* s, uint8_t len) {
  uint16_t v = 0;
  for (uint8_t i = 0; i < len; i++) {
    char c = s[i];
    uint8_t d = (c <= '9') ? (c - '0') : ((c & 0x1F) + 9);
    v = (v << 4) | d;
  }
  return v;
}

// ============================================================================
// ONEWIRE COMMUNICATION (native M60F ASCII protocol)
// ============================================================================
bool sendToSensor(const char* cmd, char* response, uint8_t maxRespLen, bool expectAckOnly = false) {
  for (uint8_t i = 0; cmd[i]; i++) {
    tofBus.write(cmd[i]);
  }
  tofBus.flush();
  
  uint32_t deadline = millis() + CMD_TIMEOUT_MS;
  uint8_t respPos = 0;
  
  while (millis() < deadline && respPos < maxRespLen - 1) {
    if (tofBus.available()) {
      char c = tofBus.read();
      if (c == '\n') break;
      if (c != '\r') response[respPos++] = c;
    }
  }
  response[respPos] = '\0';
  
  if (expectAckOnly) {
    return (respPos == 1 && response[0] == 'A');
  }
  return (respPos > 0);
}

// ============================================================================
// COMMAND HANDLERS (transparent pass-through)
// ============================================================================
void handleReadRegister(uint8_t sensorId, uint8_t reg) {
  char cmd[10];
  cmd[0] = 'R';
  static const char hex[16] = "0123456789ABCDEF";
  cmd[1] = hex[(sensorId >> 4) & 0xF];
  cmd[2] = hex[sensorId & 0xF];
  cmd[3] = hex[(reg >> 4) & 0xF];
  cmd[4] = hex[reg & 0xF];
  cmd[5] = '\n';
  cmd[6] = '\0';
  
  char resp[16];
  if (sendToSensor(cmd, resp, sizeof(resp))) {
    Serial.println(resp);
  } else {
    Serial.println("ERR01");
  }
}

void handleWriteRegister(uint8_t sensorId, uint8_t reg, uint16_t val) {
  char cmd[14];  // Max: W + 2 + 2 + 4 + \n + \0 = 10
  cmd[0] = 'W';
  static const char hex[16] = "0123456789ABCDEF";
  cmd[1] = hex[(sensorId >> 4) & 0xF];
  cmd[2] = hex[sensorId & 0xF];
  cmd[3] = hex[(reg >> 4) & 0xF];
  cmd[4] = hex[reg & 0xF];
  // Value as variable hex (no leading zeros)
  uint8_t pos = 5;
  if (val == 0) {
    cmd[pos++] = '0';
  } else {
    // Find highest non-zero nibble
    int8_t start = 3;
    while (start > 0 && ((val >> (start * 4)) & 0xF) == 0) start--;
    for (int8_t i = start; i >= 0; i--) {
      cmd[pos++] = hex[(val >> (i * 4)) & 0xF];
    }
  }
  cmd[pos++] = '\n';
  cmd[pos] = '\0';
  
  char resp[4];
  if (sendToSensor(cmd, resp, sizeof(resp), true)) {
    Serial.println("A");
  } else {
    Serial.println("ERR01");
  }
}

void handleSimpleCommand(uint8_t sensorId, char cmdChar) {
  char cmd[6];
  cmd[0] = cmdChar;
  static const char hex[16] = "0123456789ABCDEF";
  cmd[1] = hex[(sensorId >> 4) & 0xF];
  cmd[2] = hex[sensorId & 0xF];
  cmd[3] = '\n';
  cmd[4] = '\0';
  
  char resp[4];
  if (sendToSensor(cmd, resp, sizeof(resp), true)) {
    Serial.println("A");
  } else {
    Serial.println("ERR01");
  }
}

void handlePing() {
  Serial.println("PONG");
}

void handleVersion() {
  Serial.print("VER");
  Serial.println(FIRMWARE_VERSION);
}

void handleHelp() {
  Serial.println("HELP Commands (Web Serial):");
  Serial.println("HELP R<id><reg>       Read register (id=01-05, reg=00-FF)");
  Serial.println("HELP W<id><reg><val>  Write register (val=variable hex)");
  Serial.println("HELP S<id>            Save configuration");
  Serial.println("HELP U<id>            Restart sensor");
  Serial.println("HELP Z<id>            Reset configuration");
  Serial.println("HELP P                Ping");
  Serial.println("HELP V                Version");
  Serial.println("HELP H                This help");
}

// ============================================================================
// COMMAND PARSER (webpage format: 2-digit hex sensor ID)
// ============================================================================
void parseAndExecute(const char* cmd) {
  if (cmd[0] == '\0') return;
  
  char c = cmd[0];
  
  // Single-char commands
  if (c == 'P' && cmd[1] == '\0') { handlePing(); return; }
  if (c == 'V' && cmd[1] == '\0') { handleVersion(); return; }
  if (c == 'H' && cmd[1] == '\0') { handleHelp(); return; }
  
  // Commands requiring 2-digit hex sensor ID (01-05)
  if (strlen(cmd) < 3) { Serial.println("ERR03"); return; }
  
  uint8_t sensorId = parseHexByte(cmd + 1);
  if (sensorId < 1 || sensorId > TOF_SENSOR_COUNT) { Serial.println("ERR04"); return; }
  
  switch (c) {
    case 'R': {  // Read: R<id><reg> (5 chars: R + 2 + 2)
      if (strlen(cmd) != 5) { Serial.println("ERR03"); return; }
      uint8_t reg = parseHexByte(cmd + 3);
      handleReadRegister(sensorId, reg);
      break;
    }
    
    case 'W': {  // Write: W<id><reg><val> (W + 2 + 2 + variable hex)
      uint8_t cmdLen = strlen(cmd);
      if (cmdLen < 6) { Serial.println("ERR03"); return; }  // At least W + 2 + 2 + 1
      uint8_t reg = parseHexByte(cmd + 3);
      // Parse remaining as hex value (variable length)
      uint16_t val = parseHexVar(cmd + 5, cmdLen - 5);
      handleWriteRegister(sensorId, reg, val);
      break;
    }
    
    case 'S':  // Save: S<id> (3 chars)
    case 'U':  // Restart: U<id>
    case 'Z':  // Reset: Z<id>
      if (strlen(cmd) != 3) { Serial.println("ERR03"); return; }
      handleSimpleCommand(sensorId, c);
      break;
    
    default:
      Serial.println("ERR03");
      break;
  }
}

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(USB_BAUD);
  while (!Serial && millis() < 2000);
  
  pinMode(PIN_TOF_BUS, INPUT);
  tofBus.begin(TOF_BUS_BAUD);
  tofBus.setTimeout(CMD_TIMEOUT_MS);
  
  delay(500);
  
  Serial.print("VER");
  Serial.println(FIRMWARE_VERSION);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (cmdPos > 0) {
        cmdBuffer[cmdPos] = '\0';
        parseAndExecute(cmdBuffer);
        cmdPos = 0;
      }
    } else if (cmdPos < MAX_CMD_LEN - 1) {
      cmdBuffer[cmdPos++] = c;
    } else {
      cmdPos = 0;
      Serial.println("ERR03");
    }
  }
}