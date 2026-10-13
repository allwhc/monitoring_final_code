/*
 * UART-Controlled Ultrasonic Sensor over RS485 Bridge — Test Sketch
 * (no cloud, no MQTT)
 *
 * Sensor: DYP-A06BLYT-V1.1 (UART Controlled output mode), 30-200cm range.
 * Per manufacturer datasheet (DYP-A21-Output-Interfaces, section 2.2):
 *   "When the trigger input lead 'RX' receives a trigger pulse with a
 *    falling edge OR ANY SERIAL PORT DATA, the falling edge will trigger
 *    the module to work once, and the output lead 'TX' will output one
 *    measurement data. The trigger cycle of the module must be greater
 *    than 150ms."
 *
 * Reply frame (same for UART Controlled and UART Automatic modes):
 *   0xFF, Data_H, Data_L, SUM   (4 bytes)
 *   SUM = (0xFF + Data_H + Data_L) & 0xFF
 *   Distance_mm = (Data_H << 8) | Data_L
 *
 * BAUD — bench-confirmed 9600, NOT the general A06 datasheet's 115200
 * default (that figure applies to the RS485-output-mode variant of this
 * chip family, not this specific UART-Controlled unit). Confirmed via
 * RAW command: trigger 0x55 -> reply FF 02 DC DD -> checksum verifies
 * ((0xFF+0x02+0xDC)&0xFF = 0xDD) -> 732mm / 73.2cm, well inside the
 * 30-200cm valid range.
 *
 * Bridge topology (confirmed hardware on hand):
 *   Sensor RX/TX <--> [Module A: TTL-to-RS485, HARDWARE AUTO CONTROL —
 *                       auto-direction, no DE/RE pin to drive]
 *                       <==A/B twisted pair==>
 *   [Module B: MAX485 — MANUAL DE/RE pin] <--> ESP32 UART2 (GPIO5/17)
 *
 * Wiring:
 *   ESP32 GPIO5   <- Module B RO  (ESP32 RX)
 *   ESP32 GPIO17  -> Module B DI  (ESP32 TX)
 *   ESP32 GPIO25  -> Module B DE+RE (shorted), LOW=receive HIGH=transmit
 *   Module B A/B  <==long cable run==> Module A A/B
 *   Module A TTL side <-> Sensor RX/TX (module auto-senses direction)
 *
 * Bench-confirmed working settings: BAUD 9600, TRIG 0x55, DIR NORMAL.
 * (General A06-series datasheet lists 115200 as default, but that's for
 * a different output-mode variant — this physical UART-Controlled unit
 * replies correctly at 9600.)
 *
 * Continuous polling is OFF by default — nothing transmits until you
 * type POLL ON, or run a one-shot RAW/TEST. All settings changeable live
 * over serial, no reflash needed.
 *
 * Serial commands (115200 baud, newline ending):
 *   HELP              - show commands + current settings
 *   BAUD <n>          - change RS485 baud (e.g. BAUD 9600, BAUD 115200)
 *   TRIG <hex|dec>    - change trigger byte (e.g. TRIG 0x55, TRIG 85)
 *   DIR INV|NORMAL    - flip Module B's DIR polarity
 *   RAW               - one-shot: send trigger, dump every raw byte that
 *                       comes back within 1.5s as hex, even garbage —
 *                       shows whether ANYTHING is arriving at all
 *   TEST              - one-shot: send trigger, try to parse a valid
 *                       distance frame from the reply
 *   POLL ON|OFF       - start/stop continuous polling every 500ms
 */

#include <HardwareSerial.h>

#define RS485_RX_PIN   5    // ESP32 <- Module B RO
#define RS485_TX_PIN   17   // ESP32 -> Module B DI
#define RS485_DIR_PIN  25   // Module B DE+RE (shorted)

// Per datasheet: blind zone / range for A06BLYT human-body wired-transducer
// variant is 30-200cm.
#define SENSOR_MIN_CM   30.0f
#define SENSOR_MAX_CM  200.0f

// ── Runtime-adjustable settings (all changeable via serial commands) ──
uint32_t currentBaud   = 9600;     // bench-confirmed for this unit (see header note)
uint8_t  triggerByte   = 0x55;
bool     dirInverted   = false;    // false = HIGH:transmit/LOW:receive (normal)

// Continuous polling — OFF by default (nothing auto-transmits until the
// user asks). POLL ON starts it, POLL OFF stops it. Trigger cycle must be
// > 150ms per datasheet section 2.2.
bool pollingEnabled = false;
const unsigned long POLL_INTERVAL_MS = 500;
unsigned long lastPoll = 0;
int32_t lastGoodDistMm = -1;
bool haveGoodReading = false;

HardwareSerial rs485(2);   // UART2

void setDir(bool transmit) {
  bool level = dirInverted ? !transmit : transmit;
  digitalWrite(RS485_DIR_PIN, level ? HIGH : LOW);
}

void printHex(const uint8_t* buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
}

// How long RAW waits for a reply after sending the trigger. Generous on
// purpose — datasheet worst case is ~140ms (+12ms if the sensor was
// dormant), so 1.5s gives huge margin while debugging. This is a manual,
// one-shot command (not the continuous poll loop), so a longer wait here
// costs nothing.
#define RAW_LISTEN_MS 1500

// Sends the trigger byte ONCE, dumps EVERY raw byte received within
// RAW_LISTEN_MS as hex — no framing/checksum assumptions. Use this to
// answer "is anything arriving at all" before trying to parse a real
// frame. This is the ONLY thing that transmits — nothing else in this
// sketch sends a trigger unless you explicitly run RAW or TEST.
void doRawDump() {
  while (rs485.available()) rs485.read();

  Serial.printf("--- RAW: baud=%lu trig=0x%02X dirInverted=%s  (listening up to %dms) ---\n",
                currentBaud, triggerByte, dirInverted ? "YES" : "NO", RAW_LISTEN_MS);
  Serial.print("TX: ");
  printHex(&triggerByte, 1);
  Serial.println();

  setDir(true);
  delayMicroseconds(100);
  rs485.write(triggerByte);
  rs485.flush();
  delayMicroseconds(100);
  setDir(false);

  uint8_t buf[128];
  uint8_t received = 0;
  unsigned long start = millis();
  while ((millis() - start) < RAW_LISTEN_MS && received < sizeof(buf)) {
    if (rs485.available()) {
      buf[received++] = rs485.read();
    }
  }

  if (received == 0) {
    Serial.println("RX: (nothing at all — 0 bytes)");
  } else {
    Serial.print("RX: ");
    printHex(buf, received);
    Serial.printf(" (%u bytes)\n", received);
    // Try to spot a valid frame anywhere in what came back
    bool foundValid = false;
    for (uint8_t i = 0; i + 3 < received; i++) {
      if (buf[i] == 0xFF) {
        uint8_t sum = (buf[i] + buf[i+1] + buf[i+2]) & 0xFF;
        if (sum == buf[i+3]) {
          uint16_t d = (buf[i+1] << 8) | buf[i+2];
          Serial.printf(">>> Valid frame at offset %u: %u mm (%.1f cm) <<<\n",
                        i, d, d / 10.0f);
          foundValid = true;
        }
      }
    }
    if (!foundValid) {
      Serial.println("(bytes received, but no valid 0xFF..checksum frame found in them)");
    }
  }
  Serial.println("--- RAW done ---\n");
}

// Single trigger-and-listen cycle, same generous RAW_LISTEN_MS window as
// doRawDump() (not a tight 250ms) — only fires when the user explicitly
// runs TEST, never automatically in loop().
int32_t triggerAndReadOnce() {
  while (rs485.available()) rs485.read();

  setDir(true);
  delayMicroseconds(100);
  rs485.write(triggerByte);
  rs485.flush();
  delayMicroseconds(100);
  setDir(false);

  uint8_t frame[4];
  uint8_t received = 0;
  unsigned long start = millis();
  unsigned long lastByteAt = start;
  const unsigned long INTERBYTE_TIMEOUT_MS = 100;   // generous while debugging

  while (received < 4 && (millis() - start) < RAW_LISTEN_MS) {
    if (rs485.available()) {
      uint8_t b = rs485.read();
      if (received == 0 && b != 0xFF) continue;
      frame[received++] = b;
      lastByteAt = millis();
    } else if (received > 0 && (millis() - lastByteAt) > INTERBYTE_TIMEOUT_MS) {
      break;
    }
  }

  if (received < 4) return -1;

  uint8_t sum = (frame[0] + frame[1] + frame[2]) & 0xFF;
  if (sum != frame[3]) return -1;

  uint16_t distMm = (frame[1] << 8) | frame[2];
  float distCm = distMm / 10.0f;
  if (distCm < SENSOR_MIN_CM || distCm > SENSOR_MAX_CM) return -1;

  return distMm;
}

void printHelp() {
  Serial.println();
  Serial.println("=== Commands ===");
  Serial.printf("  HELP               - this help. Current: baud=%lu trig=0x%02X dirInverted=%s\n",
                currentBaud, triggerByte, dirInverted ? "YES" : "NO");
  Serial.println("  BAUD <n>           - change RS485 baud, e.g. BAUD 9600");
  Serial.println("  TRIG <hex|dec>     - change trigger byte, e.g. TRIG 0x55 or TRIG 85");
  Serial.println("  DIR INV|NORMAL     - flip Module B DIR polarity");
  Serial.println("  RAW                - one-shot: send trigger, dump every raw byte received (hex)");
  Serial.println("  TEST               - one-shot: send trigger, try to parse a valid distance frame");
  Serial.printf("  POLL ON|OFF        - start/stop continuous polling every %lums (now: %s)\n",
                POLL_INTERVAL_MS, pollingEnabled ? "ON" : "OFF");
  Serial.println();
}

void applyBaud() {
  rs485.end();
  rs485.begin(currentBaud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
}

void handleSerialCommand() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  String cmd = line;
  String arg = "";
  int spaceIdx = line.indexOf(' ');
  if (spaceIdx > 0) {
    cmd = line.substring(0, spaceIdx);
    arg = line.substring(spaceIdx + 1);
  }
  cmd.toUpperCase();
  arg.trim();

  if (cmd == "HELP") {
    printHelp();
  } else if (cmd == "BAUD") {
    long b = strtol(arg.c_str(), nullptr, 10);
    if (b <= 0) { Serial.println("Invalid baud."); return; }
    currentBaud = (uint32_t)b;
    applyBaud();
    Serial.printf("Baud set to %lu\n", currentBaud);
  } else if (cmd == "TRIG") {
    long t;
    if (arg.startsWith("0x") || arg.startsWith("0X")) {
      t = strtol(arg.c_str() + 2, nullptr, 16);
    } else {
      t = strtol(arg.c_str(), nullptr, 10);
    }
    if (t < 0 || t > 255) { Serial.println("Invalid trigger byte (0-255)."); return; }
    triggerByte = (uint8_t)t;
    Serial.printf("Trigger byte set to 0x%02X\n", triggerByte);
  } else if (cmd == "DIR") {
    String a = arg; a.toUpperCase();
    if (a == "INV") { dirInverted = true; Serial.println("DIR inverted."); }
    else if (a == "NORMAL") { dirInverted = false; Serial.println("DIR normal."); }
    else Serial.println("Usage: DIR INV|NORMAL");
  } else if (cmd == "RAW") {
    doRawDump();
  } else if (cmd == "TEST") {
    Serial.printf("--- TEST: baud=%lu trig=0x%02X dirInverted=%s  (listening up to %dms) ---\n",
                  currentBaud, triggerByte, dirInverted ? "YES" : "NO", RAW_LISTEN_MS);
    int32_t distMm = triggerAndReadOnce();
    if (distMm >= 0) {
      Serial.printf(">>> Distance: %ld mm (%.1f cm) <<<\n", (long)distMm, distMm / 10.0f);
    } else {
      Serial.println("No valid frame received. Try RAW to see raw bytes.");
    }
    Serial.println("--- TEST done ---\n");
  } else if (cmd == "POLL") {
    String a = arg; a.toUpperCase();
    if (a == "ON") {
      pollingEnabled = true;
      haveGoodReading = false;
      lastPoll = 0;   // fire immediately on next loop() instead of waiting a full interval
      Serial.println("Continuous polling ON.");
    } else if (a == "OFF") {
      pollingEnabled = false;
      Serial.println("Continuous polling OFF.");
    } else {
      Serial.println("Usage: POLL ON|OFF");
    }
  } else {
    Serial.printf("Unknown command '%s'. Type HELP.\n", cmd.c_str());
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== UART-Controlled Sensor over RS485 Bridge — DIAGNOSTIC ===");
  Serial.println("Nothing runs automatically. Every trigger is sent only when you type RAW or TEST.");

  pinMode(RS485_DIR_PIN, OUTPUT);
  digitalWrite(RS485_DIR_PIN, LOW);

  applyBaud();
  printHelp();
  Serial.println("Bench-confirmed working: BAUD 9600, TRIG 0x55, DIR NORMAL.");
  Serial.println("Type POLL ON for continuous readings, or RAW/TEST for one-shot checks.\n");
}

void loop() {
  handleSerialCommand();

  if (pollingEnabled && millis() - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = millis();

    int32_t distMm = triggerAndReadOnce();
    if (distMm >= 0) {
      lastGoodDistMm = distMm;
      haveGoodReading = true;
    }

    if (haveGoodReading) {
      float distCm = lastGoodDistMm / 10.0f;
      bool stale = (distMm < 0);
      Serial.printf("Distance: %ld mm (%.1f cm)%s\n",
                    (long)lastGoodDistMm, distCm,
                    stale ? "  [stale — last good reading]" : "");
    } else {
      Serial.println("No reading yet — check wiring/bridge/baud.");
    }
  }
}
