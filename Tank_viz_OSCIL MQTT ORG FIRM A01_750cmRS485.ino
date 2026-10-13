/*
 * SenseFlow Firebase Sensor Firmware v19.0.0 (Ultrasonic)
 *
 * ESP32 sensor-only device that pushes data directly to Firebase RTDB.
 * Supports DIP (1-6 switches) and Ultrasonic (HC-SR04) sensors.
 * Uses MvsConnect for WiFi setup via Android app.
 *
 * COMPILE NOTE: This firmware exceeds the default 1.3 MB app partition
 * because both DIP and ultrasonic code paths live in the same file.
 * Use partition scheme: "Minimal SPIFFS (1.9MB APP / 190KB SPIFFS with OTA)"
 * (arduino-cli fqbn extra: :PartitionScheme=min_spiffs). This keeps OTA
 * working — we don't use SPIFFS on sensor devices anyway.
 *
 * Device Code: SF-XXXXXXXX-SN (generated once, stored in NVS)
 * Auth: Firebase anonymous authentication
 * Data Push: Change-driven + 5 minute heartbeat
 * Commands: refreshRequested, testRequested, restartRequested
 *
 * Device Class: 0x02 (Sensor only)
 * LED: Addressable WS2812B on GPIO15
 *   - Level color for 30s → WiFi status blink → repeat
 *   - Purple solid on sensor error
 *   - Rainbow on Firebase test command
 *   - Blue blink = WiFi connected, White blink = WiFi disconnected
 */

// Channel toggles must come before includes so the right libraries are pulled in.
#define ENABLE_CLOUD                 1
#define ENABLE_LOCAL_MQTT            0
// MQTT broker location: 0 = LAN Pi gateway, 1 = cloud broker (HiveMQ/EMQX/VPS).
// Detailed config block lives further down — these top-level defines exist
// here only so the WiFiClientSecure include can be conditionally pulled in.
#define USE_CLOUD_MQTT               0
#define CLOUD_MQTT_USE_TLS           0
#if ENABLE_CLOUD == 0 && ENABLE_LOCAL_MQTT == 0
  #warning "Both cloud and MQTT disabled — device will be local-AP-only"
#endif

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "esp_wifi.h"      // 21.1.6: esp_wifi_stop/start for soft recovery rungs
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <time.h>
#if ENABLE_CLOUD
  #include <Firebase_ESP_Client.h>
  #include <addons/TokenHelper.h>
#endif
#include <MvsConnect.h>
#include <mvsota_esp32.h>
#include <FastLED_min.h>
#if ENABLE_LOCAL_MQTT
  #include <PubSubClient.h>
  #if USE_CLOUD_MQTT && CLOUD_MQTT_USE_TLS
    #include <WiFiClientSecure.h>
  #endif
  #include "mbedtls/sha256.h"
#endif

// ══════════════════════════════════════════════════
//  CONFIGURATION — CHANGE THESE PER DEPLOYMENT
// ══════════════════════════════════════════════════

// Sensor mode: 0 = DIP switches, 1 = Ultrasonic HC-SR04
#define USE_ULTRASONIC    1

// DIP sensor count (1–4), ignored if USE_ULTRASONIC=1
#define SENSOR_COUNT      4

// Firebase project config
#define FIREBASE_API_KEY      "AIzaSyAyx29tFxNbERqbuM9iTFvWbVcehwtURw4"
#define FIREBASE_DB_URL       "https://senseflow-5a9bb-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_PROJECT_ID   "senseflow-5a9bb"

// Device info
// DEVICE_NAME kept SHORT because the AP SSID is:
//   <DEVICE_NAME>-<4-char code>_mvstech
// WiFi SSID max is 32 chars. "SenseFlow-US" (12) + "-XXXX" (5) +
// "_mvstech" (8) = 25 chars → fits with room to spare. The MvsConnect
// app requires the "_mvstech" suffix to recognise the AP — if the
// SSID is >32 chars ESP-IDF truncates and drops the suffix, breaking
// the app pairing. This was the 19.0.2 field-report bug fixed here.
// v21.0.4: shortened AP name. SSID format: "SFUSL-XXXX_mvstech" (18 chars).
// Old was "SenseFlow-USL-XXXX_mvstech" (25 chars) — some Android scan lists
// truncated it in list view. Kept "USL" so long-range devices remain
// distinguishable from short-range "SFUSR" units in the scan list.
#define DEVICE_NAME       "SFUSL"
#define FIRMWARE_VERSION  "21.1.6"
#define FIRMWARE_CODE     "SF-USL-2026"
#define AP_PASSWORD       "mvstech9867"

// ── MQTT broker config (LAN or cloud — selected by USE_CLOUD_MQTT above) ──
#define MQTT_PUBLISH_INTERVAL_MS   2000    // push every 2s + on change

#if USE_CLOUD_MQTT == 0
  // LAN mode — Pi runs Mosquitto, ESP32 auto-discovers via UDP, derived password.
  #define MQTT_PORT                  1883
  #define MQTT_DISCOVERY_PORT        1900
  #define MQTT_DISCOVERY_MSG         "SENSEFLOW_DISCOVER"
  #define MQTT_DISCOVERY_REPLY       "SENSEFLOW_HERE"
  #define MQTT_DISCOVERY_INTERVAL_MS 60000
  // Rotate MQTT_SECRET by re??flashing all devices + updating pi_gateway.py.
  #define MQTT_SECRET             "mvs_kalp_2026_xY9k_rotate_me"
#else
  // Cloud mode — direct connect to HiveMQ / EMQX / VPS Mosquitto via TLS.
  // Examples for CLOUD_MQTT_HOST:
  //   "abc123.s1.eu.hivemq.cloud"   (HiveMQ Cloud)
  //   "broker.emqx.io"              (public test, no auth)
  //   "mqtt.yourdomain.com"         (self-hosted Mosquitto on VPS)
  #define MQTT_PORT                  8883        // TLS port; 1883 for plain
  #define CLOUD_MQTT_HOST            "REPLACE_WITH_BROKER_HOST"
  #define CLOUD_MQTT_USER            "REPLACE_WITH_USERNAME"
  #define CLOUD_MQTT_PASS            "REPLACE_WITH_PASSWORD_OR_TOKEN"
  // CA root certificate (required when CLOUD_MQTT_USE_TLS=1).
  #if CLOUD_MQTT_USE_TLS
    static const char* CLOUD_MQTT_CA_CERT = R"PEM(
-----BEGIN CERTIFICATE-----
REPLACE_WITH_CA_CERT_PEM
-----END CERTIFICATE-----
)PEM";
  #endif
#endif

// ══════════════════════════════════════════════════
//  CONSTANTS
// ══════════════════════════════════════════════════

// Device classes (match RS485 protocol)
#define CLS_VALVE   0x01
#define CLS_SENSOR  0x02
#define CLS_MOTOR   0x03

// Sensor types
#define SNS_NONE        0x00
#define SNS_DIP         0x01
#define SNS_ULTRASONIC  0x02

// DIP sensor GPIOs (fixed order, same as RS485 version)
// Bottom→top: GPIO34=25%, GPIO35=50%, GPIO32=75%, GPIO33=100%
// All require external 10k pull-down resistors (34/35 have no internal pulls).
const int DIP_PINS[] = {34, 35, 32, 33};

// Common rod pin — driven by ESP32 GPIO regardless of mode.
#define DIP_COMMON_PIN       12   // EXCITE on PCB. Strapping pin — external pull-down ensures LOW at boot.

// Set to 1 when GPIO12 drives an NPN common-emitter transistor (BC547 or
// similar) that switches the probe-common rod. Schematic on the PCB:
//
//   EXCITE (GPIO12) ── EX_B (base resistor) ── Base
//                                              Collector ── EX_C (pull-up) ── 5V (or 3.3V via J2)
//                                                                          ── EX (probe common rod)
//                                              Emitter ── GND
//   EX_G keeps the base pulled down to GND during boot.
//
// Output is taken at the collector (common-emitter config), so the logic
// INVERTS:
//   GPIO12 LOW  → transistor OFF → collector pulled HIGH via EX_C → common HIGH (excite ON)
//   GPIO12 HIGH → transistor ON  → collector shorted to GND        → common LOW  (excite OFF)
//
// Boot is safe because EX_G holds the base LOW → transistor OFF → common
// sits at 5V via EX_C. No current is being driven from GPIO12 at boot.
//
// Set to 0 for direct GPIO drive (short cables only, ESP32 sources 3.3V).
#define DIP_COMMON_VIA_TRANSISTOR  1

#if DIP_COMMON_VIA_TRANSISTOR
  // NPN common-emitter inverts the GPIO sense (see schematic comment above).
  #define DIP_COMMON_DRIVE_ACTIVE   LOW    // GPIO LOW  → transistor OFF → common HIGH (excite)
  #define DIP_COMMON_DRIVE_IDLE     HIGH   // GPIO HIGH → transistor ON  → common LOW  (idle)
#else
  // Direct drive — GPIO IS the common rod.
  #define DIP_COMMON_DRIVE_ACTIVE   HIGH
  #define DIP_COMMON_DRIVE_IDLE     LOW
#endif

// Excitation mode for the common rod:
//   0 = CONSTANT DC — common held HIGH always; probes read with plain
//       digitalRead(). Simpler but causes electrolytic corrosion of probes
//       (months to a few months of life). Use only if probes are easily
//       replaceable or the system is short-lived.
//   1 = OCSIL PULSED — common pulsed only during read bursts, synchronous
//       sampling rejects leakage/stuck pins, probe life 2-10 yr.
#define EXCITATION_MODE      1

// How often the firmware re-reads the probes (applies to both modes).
#define DIP_READ_INTERVAL_MS_DEFAULT 2000

#if EXCITATION_MODE == 1
  // Probe life profile — only used in OCSIL mode.
  //   1 = ~2 years  (fastest UI)
  //   2 = ~4 years  (good balance, default)
  //   3 = ~8 years  (slower UI, lower duty)
  //   4 = MAX       (minimum duty cycle, slowest UI)
  #define PROBE_LIFE_PROFILE   2

  #if PROBE_LIFE_PROFILE == 1
    #define DIP_SETTLE_US         200
    #define DIP_SAMPLES_PER_READ   10
    #define DIP_AGREE_THRESHOLD     8
    #define DIP_READ_INTERVAL_MS 2000
  #elif PROBE_LIFE_PROFILE == 2
    // Tightened for EMI noise rejection on top-of-water probe:
    //   - 600 µs settle (was 300) — long probe cables (~100m) have higher
    //     capacitance; need ~2x time for the line to fully charge before sampling
    //   - 15 samples = more statistical confidence
    //   - 15/15 agreement (strict) — any noise-aligned glitch fails
    #define DIP_SETTLE_US         600
    #define DIP_SAMPLES_PER_READ   15
    #define DIP_AGREE_THRESHOLD    15
    #define DIP_READ_INTERVAL_MS 3000
  #elif PROBE_LIFE_PROFILE == 3
    #define DIP_SETTLE_US          50
    #define DIP_SAMPLES_PER_READ    5
    #define DIP_AGREE_THRESHOLD     4
    #define DIP_READ_INTERVAL_MS 5000
  #elif PROBE_LIFE_PROFILE == 4
    #define DIP_SETTLE_US          50
    #define DIP_SAMPLES_PER_READ    3
    #define DIP_AGREE_THRESHOLD     3
    #define DIP_READ_INTERVAL_MS 10000
  #else
    #error "PROBE_LIFE_PROFILE must be 1, 2, 3 or 4"
  #endif
#elif EXCITATION_MODE == 0
  // Constant DC — only DIP_READ_INTERVAL_MS is meaningful.
  #define DIP_READ_INTERVAL_MS DIP_READ_INTERVAL_MS_DEFAULT
#else
  #error "EXCITATION_MODE must be 0 (constant DC) or 1 (OCSIL pulsed)"
#endif

// Ultrasonic pins — reused for BOTH sensor types (v20.0.0):
//   HC-SR04:   GPIO 32 = TRIG (output), GPIO 34 = ECHO  (input-only)
//   DYP UART:  GPIO 32 = UART TX      , GPIO 34 = UART RX (input-only, ok for RX)
// GPIO 34 is ESP32 input-only which is exactly what UART RX needs, so no
// PCB rework is required to switch between HC-SR04 and DYP sensors.
#define US_TRIG_PIN     32   // HC-SR04 TRIG, doubles as DYP UART TX
#define US_ECHO_PIN     34   // HC-SR04 ECHO, doubles as DYP UART RX
#define US_UART_TX_PIN  32
#define US_UART_RX_PIN  34

// RS485/Modbus sensor — separate GPIOs from DYP/HC-SR04 above (RS485 needs
// a third DE/RE direction-control pin, and always runs at a fixed 9600
// baud/8N1/no-parity per the sensor's Modbus-RTU spec, unlike DYP which
// auto-detects across several bauds). Confirmed working wiring from bench
// validation — RO(module)->RX, DI(module)->TX, DE+RE(shorted)->DIR.
#define US_RS485_RX_PIN  5    // ESP32 <- RS485 module RO
#define US_RS485_TX_PIN  17   // ESP32 -> RS485 module DI
#define US_RS485_DIR_PIN 25   // RS485 module DE+RE (shorted), LOW=receive HIGH=transmit
#define US_RS485_BAUD    9600

// Sensor type IDs (stored in NVS key "usType"). Detected at boot.
// 0=unknown → force autodetect. Autodetect saves the result so subsequent
// boots are fast. Installer can override via /setustype AP endpoint.
#define US_TYPE_UNKNOWN  0
#define US_TYPE_HCSR04   1   // legacy HC-SR04 (pulseIn path)
#define US_TYPE_DYP_A01  2   // DYP-A01 controlled — send 0x55, read 4-byte frame
#define US_TYPE_DYP_A21  3   // DYP-A21 auto — sensor streams frames, we read
// RS485/Modbus-RTU — NOT part of the auto-detect ladder in initUltrasonic().
// Modbus requires knowing the slave address up front to address a request,
// and there's no self-revealing "always streaming" fallback the way DYP-A21
// has, so blind autodetection would be slow/unreliable and risks writing to
// unrelated devices sharing the bus. Installer must select it manually via
// /setustype and enter the slave address via /setrs485addr.
#define US_TYPE_RS485    4

// Modbus registers for RS485 mode. Firmware always reads REAL-TIME
// (0x0101) — NOT installer-selectable. Field testing showed the
// processed register (0x0100, ~500ms worst-case response) produced
// false/stale readings under this firmware's burst-polling cadence;
// real-time (~100ms) is far more reliable, and the existing histogram/
// hysteresis/jump-limit filter pipeline (shared with the DYP path)
// already handles the extra raw noise from skipping the sensor's own
// internal smoothing. US_RS485_REG_PROCESSED is kept defined for
// reference/future use but is not read by any code path.
#define US_RS485_REG_PROCESSED  0x0100
#define US_RS485_REG_REALTIME   0x0101

// Addressable LED
#define LED_PIN      15
#define LED_COUNT    1

// Timing
#define HEARTBEAT_INTERVAL    300000   // 5 minutes
#define COMMAND_CHECK_INTERVAL 30000   // 30 seconds — bumped from 5s to cut
                                        // RTDB command polling by 6x. Cost
                                        // driver: 5 boolean reads every 5s
                                        // per device ≈ 26 MB/day/device of
                                        // download bandwidth. At 30s that
                                        // drops to ~4 MB/day. Trade-off: a
                                        // Restart / Refresh / Test command
                                        // from cloud takes up to 30s to
                                        // fire instead of 5s. Acceptable
                                        // for admin-triggered actions.
#define DIP_DEBOUNCE_MS          0     // disabled — sync sampling + 2s read gap = natural filter
#define US_READ_INTERVAL      2000     // Ultrasonic read interval — 2s. No probe wear cost
                                        // (unlike DIP electrolysis), so cadence is set by
                                        // dashboard-response goals not sensor lifetime.
#define LED_CYCLE_DURATION    30000    // 30 seconds level display
#define WIFI_BLINK_DURATION   2000     // WiFi status blink (2s per sticker spec)

// Ultrasonic — HC-SR04 physical limits (NOT tank-height dependent).
// Blind zone <21 cm = transducer still ringing after transmit.
// Max range >450 cm = echo too weak to detect reliably.
// 30-ms pulseIn timeout supports round-trip of 514 cm, so 450 cm max
// leaves 14 cm safety margin.
#define US_SAMPLES        25       // 25 pings per burst — more data for
                                   // histogram to find true mode. Burst
                                   // takes 625 ms best / 1000 ms worst
                                   // (still <50% of 2-sec cycle budget).
#define US_MAD_MULTIPLIER 2.5
#define US_BLIND_ZONE     28.0    // cm — DYP-A01 blind zone per datasheet
#define US_MAX_RANGE      750.0   // cm — DYP-A01 max range per datasheet (was 450 for HC-SR04)

// ── Layer 1: Histogram-mode burst screen (19.1.0) ──────────────────
// Reflections off walls / edges show up as CLUSTERED noise: ping
// hits wall first, returns 195 cm; next ping catches real target
// at 260 cm; next hits wall again, 200 cm. MAD averages them (bad).
// Histogram-mode counts which distance appears most often — the
// consistent target signal wins over sporadic reflections.
//
// Algorithm:
//   1. Take 25 pings, keep valid
//   2. MAD-reject wild outliers (|d - median| > 2.5 × MAD)
//   3. Bin remaining into 5 cm buckets
//   4. Find mode (bucket with most pings)
//   5. If mode has ≥ 60% of surviving pings → accept bin center
//      Otherwise → burst UNRELIABLE, fall back to zone memory
#define US_HIST_BIN_CM         5.0    // 5 cm buckets (~sensor noise floor)
#define US_HIST_MODE_MIN_PCT   60     // mode must have ≥60% of pings
#define US_HIST_MIN_VALID      10     // need ≥10 of 25 pings valid to attempt

// ── Layer 2: Confirmed-mode hysteresis (was 3-strike, now 8) ───────
// Even if Layer 1 accepts a wrong value one cycle, Layer 2 needs
// N=8 consecutive mode-values in the new range before committing.
// 8 strikes × 2 sec = 16 sec of consistent wrong readings needed.
// Physical reflections cluster for ≤ 15 sec typically, so this
// stops them from ever being committed.
// Tolerance widened from 10 → 20 cm because Layer 1 has already
// killed the wild outliers — this layer only filters legitimate
// but ambiguous readings near the true value.

// Zone boundaries — fixed by SENSOR hardware limits, not tank size:
//   UPPER  (21-25):   sensor near-blind → tank is nearly FULL
//   MIDDLE (26-389):  normal usable range
//   LOWER  (390+):    near max range → tank is nearly EMPTY
// When readings become invalid, the last-known zone tells us which
// direction to fall back to: UPPER→100%, LOWER→0%, MIDDLE→last known
// good pct. This means the device NEVER goes offline visually — cloud
// only stops seeing updates when the physical device loses power (and
// then heartbeat-staleness at cloud level marks it offline naturally).
// Zones scaled for DYP-A01 750 cm range (was 429/430 for HC-SR04):
#define ZONE_UPPER_MIN    28     // A01 blind zone starts higher
#define ZONE_UPPER_MAX    35
#define ZONE_MIDDLE_MIN   36
#define ZONE_MIDDLE_MAX   729
#define ZONE_LOWER_MIN    730

// Hysteresis on Layer 1's mode output. Widened from 10→20 cm and
// strikes bumped 3→8 because Layer 1 already rejects wild outliers;
// this layer only needs to protect against sustained wrong-mode
// clusters (which last ≤15 sec in practice).
#define US_HYST_TOLERANCE_CM      20.0
#define US_HYST_LIVE_STRIKES      8     // 8 × 2 sec = 16 sec to commit a change
#define US_HYST_HISTORY_PCT       5     // ≥5% pct delta since last history push (was 3)

// ── Layer 3: Rate limit on cloud pushes ────────────────────────────
// Never push /live more than once every 20 sec — caps damage from
// any noise that somehow makes it through Layers 1+2. Real fills/
// drains happen over minutes so this doesn't affect them.
// /history additionally requires ≥5% pct delta since last history
// entry (via US_HYST_HISTORY_PCT) so the analytics chart stays clean.
#define US_LIVE_PUSH_MIN_GAP_MS   20000UL   // 20 sec between /live pushes
#define US_HISTORY_PUSH_MIN_GAP_MS 20000UL  // 20 sec between /history pushes

// v21.1.1: Layer 4 — JUMP-LIMIT FILTER
// Field CSV analysis (Aug 6-10) showed the sensor occasionally producing
// spikes 40-50% higher than actual water level, lasting 1-3 minutes then
// snapping back. Root cause is likely intermittent reflections from a
// fixed structure ~60 cm from sensor (support beam, ladder, pipe) OR
// splash / steam / brief obstruction. Layers 1-3 miss these because the
// spike lasts long enough that an entire burst reads the wrong value
// consistently, so histogram-mode + hysteresis both accept it.
//
// Layer 4: any jump larger than US_JUMP_LIMIT_PCT must persist for
// US_JUMP_SUSTAIN_STRIKES bursts (each burst = US_READ_INTERVAL) before
// we accept it as real. Rejected spikes fall back to last committed pct.
//
// Real refills in the CSV take hours (43% -> 87% overnight), so a
// 60-sec sustain requirement never rejects genuine changes.
#define US_JUMP_LIMIT_PCT         15    // % delta that triggers the check
// v21.1.3: bumped from 30 (60 sec) to 60 (120 sec) after field CSV
// showed 30-40 sec spikes still slipping through when the fallback path
// bypassed Layer 4 entirely. Now the fallback path also runs through
// Layer 4, and the longer sustain gives more time to reject transient
// interference (splash, condensation, brief obstruction, wall reflection).
#define US_JUMP_SUSTAIN_STRIKES   60    // 60 × 2 sec = 120 sec sustained

// v21.1.2: Boot warmup gate. For the first BOOT_WARMUP_MS after power-on,
// don't publish anything to /live. Collect BOOT_WARMUP_SAMPLES bursts,
// then publish the MEDIAN as the first stable reading. Prevents user
// from seeing the initial 20-30 sec of settling values dance on the app.
// Device still shows ONLINE (heartbeat updates devices/{code}/info) —
// only /live is held back.
#define US_BOOT_WARMUP_MS         60000UL   // 60 sec warmup after boot
#define US_BOOT_WARMUP_SAMPLES    30        // 30 bursts × 2 sec = ~60 sec

// v21.0.6: US_STEP_SIZE removed — was dead code, never referenced.
// Percentages are NOT snapped to 5% steps. distanceToIntegerPct() returns
// every whole percentage 0-100 via ceilf() rounding.
                                  //   (kept as compile-time constant; unused
                                  //    in the redesign — we now publish integer
                                  //    percent via ceilf() instead of snapping)

// ══════════════════════════════════════════════════
//  DIP PERCENT TABLE (same as RS485 version)
// ══════════════════════════════════════════════════

const uint8_t DIP_PCT_1[] = {100};
const uint8_t DIP_PCT_2[] = {50, 100};
const uint8_t DIP_PCT_3[] = {33, 67, 100};
const uint8_t DIP_PCT_4[] = {25, 50, 75, 100};
const uint8_t DIP_PCT_5[] = {20, 40, 60, 80, 100};
const uint8_t DIP_PCT_6[] = {17, 33, 50, 67, 83, 100};

const uint8_t* DIP_PCT_TABLE[] = {
  NULL, DIP_PCT_1, DIP_PCT_2, DIP_PCT_3, DIP_PCT_4, DIP_PCT_5, DIP_PCT_6
};

// ══════════════════════════════════════════════════
//  GLOBALS
// ══════════════════════════════════════════════════

Preferences prefs;
MvsConnect mvs(DEVICE_NAME, FIRMWARE_VERSION);
MvsOTA mvsota;

#if ENABLE_LOCAL_MQTT
  #if USE_CLOUD_MQTT && CLOUD_MQTT_USE_TLS
    WiFiClientSecure mqttNetClient;
  #else
    WiFiClient       mqttNetClient;
  #endif
  PubSubClient   mqttClient(mqttNetClient);
  #if USE_CLOUD_MQTT == 0
    WiFiUDP      mqttUdp;
    IPAddress    mqttBrokerIp;             // resolved by discovery
  #endif
  bool          mqttBrokerKnown   = false; // also used in cloud mode to skip DNS retry
  unsigned long lastMqttDiscovery = 0;
  unsigned long lastMqttPublish   = 0;
  unsigned long lastMqttReconnect = 0;
#endif

// Firebase
#if ENABLE_CLOUD
FirebaseData fbdo;
FirebaseAuth fbAuth;
FirebaseConfig fbConfig;
#endif
bool firebaseReady = false;   // stays false forever when ENABLE_CLOUD==0

// Device identity
String deviceCode = "";
String apName = "";
// User-assigned physical-identity label. Set by installer via the AP
// page, persisted to NVS (senseflow namespace, key "userName"), and
// mirrored to /devices/<code>/info/userAssignedName in Firebase so
// dashboard + admin viewer can display it. NEVER pulled FROM Firebase —
// NVS is the source of truth. If firmware is reflashed the label is
// lost (by design — it's a physical property of THIS chip's install).
String userAssignedName = "";

// Sensor state
uint8_t sensorBits = 0;
uint8_t confirmedPct = 0;
uint8_t flags = 0;        // bit0=sensorError, bit5=sensorOffline
bool    sensorError = false;

// Analytics — write history on data change
bool analyticsOn = false;

// Notifications — when ON, change-driven pushes are mirrored to
// /devices/<code>/notify_trigger. A Cloud Function watches that path and
// dispatches FCM. Free devices keep this OFF → zero Cloud Function
// invocations for non-paying customers. Admin sets via /admin/notifications.
bool notifyOn = false;

// Diagnostics — when ON, firmware uploads boot log to RTDB on boot. Pure
// data (no Cloud Function watching). Admin-set per device for debugging.
// Independent of notifyOn — admin can troubleshoot a free customer's device
// without granting them notifications.
bool diagnosticsOn = false;

// Last sent values (for change detection)
uint8_t lastSentBits = 0xFF;
uint8_t lastSentPct = 0xFF;
uint8_t lastSentFlags = 0xFF;

// Last value that passed the confirmation gate. Heartbeat + idle-history
// fallback use these so cloud never sees a transient glitch.
uint8_t lastConfirmedBits  = 0;
uint8_t lastConfirmedPct   = 0;
uint8_t lastConfirmedFlags = 0;
bool    haveConfirmedValue = false;
unsigned long lastHistoryWriteAt = 0;

// Last values actually written to /history. Separate from lastSent* (which
// gates /live pushes). Guarantees we never write a duplicate row even if
// some other path (boot, reboot, NVS reset) would otherwise force one.
// 0xFF sentinel = nothing written yet → first push always goes through.
uint8_t lastHistoryBits  = 0xFF;
uint8_t lastHistoryPct   = 0xFF;
uint8_t lastHistoryFlags = 0xFF;

// ── Diagnostics: boot log ──────────────────────────────────────────
// 50-entry circular buffer in NVS records every restart with its reason
// and how long the previous session ran. Retrievable via cloud (when
// diagnosticsOn=true) or always via serial `BOOTLOG` command. Lets admin
// diagnose flaky devices remotely — brownouts → bad PSU, repeated WDT
// → firmware hang, etc.
#define BOOTLOG_CAPACITY 50

struct BootLogEntry {
  uint32_t epoch;          // unix timestamp at boot (0 if NTP wasn't synced yet)
  uint32_t uptimeBefore;   // seconds the previous session ran before dying
  uint32_t freeHeapAtBoot; // bytes free heap right after setup()
  uint8_t  reason;         // esp_reset_reason_t code (1=POWERON, 7=WDT, 15=BROWNOUT, etc.)
  uint8_t  fwMajor;        // firmware version captured at this boot
  uint8_t  fwMinor;
  uint8_t  fwPatch;
};   // 16 bytes per entry

// In-RAM circular buffer; mirrors the NVS-persisted version. We don't
// reload all 50 on boot — only the new entry is appended. On cloud upload
// we read the latest from RAM (boot path) or all 50 from NVS (admin pull).
uint8_t boot_logIdx = 0;     // next slot to write (0..49)
uint32_t bootCount  = 0;     // total boots ever — survives NVS persistence

// Most recent boot log entry (in RAM, written by checkConfig on first
// diagnosticsOn=true read, then used by uploadLatestBootLog).
BootLogEntry latestBootEntry = {0};
bool hasLatestBootEntry = false;

// RAM-only capture from setup() — held until checkConfig() learns whether
// diagnosticsOn is true. If yes → materialised into NVS+RTDB. If no →
// discarded (free customers never write to NVS for diagnostics). Once
// materialised, pendingBootCaptured is cleared so we don't double-log
// when admin flips the flag mid-session.
uint8_t  pendingBootReason     = 0;
uint32_t pendingBootPrevUptime = 0;
bool     pendingBootCaptured   = false;

// Set true once the current boot has been logged to NVS (either at first
// diagnosticsOn=true read OR — if diagnosticsOn was already true at boot
// and stayed true — at any later flip). Prevents duplicate writes.
bool diagnosticsLoggedThisBoot = false;

// While true, loop() persists current uptime to NVS adaptively. Mirrors
// diagnosticsOn but only flips true after we've seen the flag at least
// once (so the very first config read doesn't lose data).
bool diagnosticsLoggingActive = false;

// Forward decls — defined later, called by checkConfig (which lives in
// the Firebase block above the boot log section in the file).
void uploadLatestBootLog();
void persistCurrentUptime();

// ── OTA + NTP state ────────────────────────────────────────────────
uint32_t firstBootAt    = 0;   // epoch seconds — set once, persisted in NVS
uint32_t lastUpdatedAt  = 0;   // epoch seconds — last successful OTA (or firstBootAt if never)
unsigned long lastNtpSyncAt = 0;  // millis()
bool          ntpSynced     = false;
unsigned long lastOtaCheckAt = 0;  // millis()
const unsigned long OTA_CHECK_INTERVAL_MS = 30000;   // poll trigger every 30 s
const unsigned long NTP_RESYNC_INTERVAL   = 24UL * 60UL * 60UL * 1000UL;  // 24 h
#define OTA_MAX_RETRIES 3
#define NTP_SERVER      "pool.ntp.org"
#define NTP_TZ_OFFSET_S (5 * 3600 + 30 * 60)   // IST = UTC+5:30

// DIP debounce
uint8_t rawBits = 0;
uint8_t pendingBits = 0;
unsigned long debounceStart = 0;
bool debouncing = false;

// Instant (unfiltered) level view + safeRestart() live OUTSIDE the DIP
// #if block because they're used unconditionally by handleLED(), the AP
// page /sstatus endpoint, and every code path that reboots the device.
// DIP mode populates instantPct/instantBits from the latest raw read;
// ultrasonic mode mirrors confirmedPct into instantPct (US doesn't
// distinguish "instant vs committed" the way DIP does) and sets
// instantBits to 0 (no probe pattern).
uint8_t instantPct = 0;
uint8_t instantBits = 0;

// CRITICAL: GPIO12 is the MTDI strapping pin. On boot it MUST be LOW —
// HIGH would force the ESP32 bootloader to set internal flash voltage to
// 1.8V instead of 3.3V, soft-bricking the chip until power-cycle.
// safeRestart() drives it LOW before rebooting so the next boot sees
// the correct strapping value. Used in both DIP and ultrasonic modes.
inline void safeRestart() {
  // Graceful WiFi cleanup BEFORE restart. Send the deassociation frame
  // so the router clears its DHCP lease state, then let the radio settle.
  //
  // CRITICAL: pass FALSE here, NOT true. WiFi.disconnect(true) erases
  // the ESP-IDF WiFi config from internal flash — which on the very
  // next boot causes reconnect to fail forever. Fleet-blocker bug from
  // 17.0.11. NVS 'mvswifi' namespace stays intact either way; the IDF
  // needs its own copy for some code paths. disconnect(false) sends
  // the deassoc frame WITHOUT erasing.
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect(false);
    delay(200);
  }
  pinMode(DIP_COMMON_PIN, OUTPUT);
  digitalWrite(DIP_COMMON_PIN, LOW);
  delay(50);
  ESP.restart();
}

// Ultrasonic
float usRawDistance = 0;
float usFilteredDistance = 0;
float usLastSentDistance = 0;  // For min change threshold
uint8_t usLastSentPct = 0xFF;
float usTankHeight = 100.0;   // cm, configurable from AP page
// v21.0.2: overflow pipe distance from sensor (cm). 0 = disabled (use full
// tank height, backward-compatible). When >0, anything closer than this
// is treated as 100% (water is spilling out the overflow pipe) and the
// usable range for percentage math becomes [usOverflow, usTankHeight].
uint16_t usOverflow = 0;
// v21.0.5: pump suction pipe offset from tank BOTTOM (cm). 0 = disabled.
// When >0, water below the suction inlet is treated as 0% (pump can't
// draw it dry anyway). Effective usable range shrinks to
// [usOverflow, usTankHeight - usSuction]. Distance-from-sensor equivalent:
// anything > (usTankHeight - usSuction) → clip to 0%.
uint16_t usSuction  = 0;
uint32_t tankCapacityLitres = 0;  // 0 = not configured, hides litres in UI
// AP mode: 0 = always on, 1 = on for 10 min after first successful STA connect (default)
uint8_t apMode = 1;
unsigned long apTimerStart = 0;          // millis() when 10-min countdown begins (0 = not started)
unsigned long apTimerDeadline = 0;       // millis() when AP will shut off (extends on activity)
bool apTimerEnded = false;               // true once AP has been shut down
const unsigned long AP_AUTO_OFF_MS = 10UL * 60UL * 1000UL;   // 10 min
const unsigned long AP_EXTEND_MS   =  5UL * 60UL * 1000UL;   //  +5 min on web activity
int   usFailCount = 0;
bool  usSensorOffline = false;
unsigned long lastUsRead = 0;

// 21.1.5: sensor-missing detection. If N consecutive bursts return zero
// valid pings, flag sensor as physically disconnected. In that state we:
//   1) Cap per-ping timeout to 30ms (from 150-1200ms) so processUltrasonic()
//      returns in ~450ms instead of 2.4sec → task WDT never fires.
//   2) Expose "sensorConnected":false in /sstatus so AP page shows red
//      "sensor missing" banner for the installer.
// Recovery: any burst that produces >=1 valid ping resets both counters.
// Cloud data + LED unchanged — zone-memory fallback keeps working as before.
uint16_t usConsecFailBursts = 0;
const uint16_t US_MISSING_BURST_THRESHOLD = 15;  // ~30 sec of nothing → flag missing

// ── Zone memory + hysteresis state (redesign 19.0.1) ────────────────
// Zone the sensor was last KNOWN to be in (persisted to NVS). Used
// as the fallback when readings become invalid (transducer blind
// zone, out-of-range, or transient bad ping burst) — device keeps
// publishing UPPER→100%, LOWER→0%, MIDDLE→lastValidPct instead of
// going silent or purple-error.
// Zone codes — stored as uint8_t (not an enum type) so Arduino's
// auto-prototype generation compiles even though function signatures
// referencing these values appear before this line.
#define US_ZONE_NONE    0
#define US_ZONE_UPPER   1   // near sensor (tank ~full)
#define US_ZONE_MIDDLE  2   // normal usable range
#define US_ZONE_LOWER   3   // near max range (tank ~empty)
uint8_t usLastZone = US_ZONE_NONE;
uint8_t usLastValidPct = 0;          // last cleanly-computed integer pct
bool    usHaveValidHistory = false;  // false until first successful read

// v21.1.1: Layer 4 jump-limit filter state
uint8_t  usLastCommittedPct = 0;     // last pct actually published
bool     usHaveCommittedPct = false; // false at boot until first commit
uint16_t usJumpCandidatePct = 0;     // pct we're currently considering
uint16_t usJumpStrikes      = 0;     // how many consecutive bursts said the same jump
uint32_t usSpikesRejected   = 0;     // diag counter — visible on AP page

// v21.1.2: Boot warmup gate state
uint32_t usBootStartMs      = 0;     // millis() at boot (initUltrasonic)
uint8_t  usBootPctRing[US_BOOT_WARMUP_SAMPLES] = {0};
uint8_t  usBootPctCount     = 0;     // how many bursts collected so far
bool     usBootWarmupDone   = false; // true once first stable value published

// Three-strike hysteresis state (only active in MIDDLE zone).
float   usHystLastGood = 0.0f;
uint8_t usHystOutCounter = 0;
bool    usHystInitialised = false;

// History-push tracking — /history writes only when integer pct
// changes by ≥ US_HYST_HISTORY_PCT since the last history write.
uint8_t usLastHistoryPct = 0xFF;     // 0xFF = nothing written yet

// Layer 3 rate-limit timestamps. Cap /live to 1 push per 20 sec and
// /history to 1 push per 20 sec (with additional 5% delta gate).
// Prevents any noise that slips through Layers 1+2 from spamming
// the cloud with hundreds of pushes/minute (see CSV analysis for
// exactly this failure mode in 19.0.3).
unsigned long usLastLivePushMs    = 0;
unsigned long usLastHistoryPushMs = 0;

// Timing
unsigned long lastHeartbeat = 0;
unsigned long lastCommandCheck = 0;

// Manual WiFi flag — pauses auto-reconnect
bool manualWiFiInProgress = false;
unsigned long manualWiFiStart = 0;

// Push fail tracking
int consecutiveFailCount = 0;
bool pushFailFlash = false;
unsigned long pushFailFlashStart = 0;
unsigned long lastSuccessfulPush = 0;
unsigned long lastDataPush = 0;

// LED state
unsigned long ledCycleStart = 0;
bool ledShowingWifi = false;
unsigned long wifiBlinkStart = 0;
bool testBlinkActive = false;
unsigned long testBlinkStart = 0;

// ══════════════════════════════════════════════════
//  DEVICE CODE GENERATION
// ══════════════════════════════════════════════════

String generateRandomCode() {
  const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
  String code = "SF-";
  for (int i = 0; i < 8; i++) {
    code += charset[random(0, 36)];
  }
  code += "-SN";
  return code;
}

void loadOrCreateDeviceCode() {
  prefs.begin("senseflow", false);
  deviceCode = prefs.getString("devcode", "");

  if (deviceCode.length() == 0) {
    // Seed random from analog noise + MAC
    uint8_t mac[6];
    WiFi.macAddress(mac);
    uint32_t seed = esp_random();
    randomSeed(seed);

    deviceCode = generateRandomCode();
    prefs.putString("devcode", deviceCode);
    Serial.println("Generated new device code: " + deviceCode);
  } else {
    Serial.println("Loaded device code from NVS: " + deviceCode);
  }

  // Load ultrasonic tank height
  usTankHeight = prefs.getFloat("tankh", 100.0);
  usOverflow   = prefs.getUShort("usOvfl", 0);   // v21.0.2 — 0 = disabled
  // v21.0.3: sanitize on load — if a bad value was written by v21.0.2
  // (which had no floor check), reset to 0 rather than trust it.
  if (usOverflow > 0 && usOverflow < 35) {
    Serial.printf("[BOOT] Overflow=%u cm is below 35cm floor — disabling\n", usOverflow);
    usOverflow = 0;
  }
  // v21.0.5: pump suction offset from tank bottom.
  usSuction    = prefs.getUShort("usSuct", 0);
  // Sanity: suction can't be >= tank height. If corrupt, disable.
  if (usSuction > 0 && usSuction >= (uint16_t)usTankHeight) {
    Serial.printf("[BOOT] Suction=%u cm >= tank height %u — disabling\n",
                  usSuction, (uint16_t)usTankHeight);
    usSuction = 0;
  }

  // Load last-known zone + last-valid pct — used by the never-offline
  // fallback so a device that reboots when tank is full or empty
  // immediately reports the correct 100% or 0% instead of waiting for
  // a first valid reading (which won't come — the blind zone / far
  // range is what triggered the reboot scenario in the first place).
  usLastZone = prefs.getUChar("usZone", US_ZONE_NONE);
  usLastValidPct = prefs.getUChar("usLastPct", 0);
  usHaveValidHistory = (usLastZone != US_ZONE_NONE);
  Serial.printf("[BOOT] Ultrasonic zone memory: zone=%u lastPct=%u\n",
                usLastZone, usLastValidPct);

  // Load tank capacity in litres (0 = not configured → litres hidden in UI)
  tankCapacityLitres = prefs.getUInt("capL", 0);

  // Load user-assigned name (installer-typed label like "3F West Flush").
  // Empty string if never set. Kept in the same senseflow namespace as
  // deviceCode + capL so all device-identity state lives together.
  userAssignedName = prefs.getString("userName", "");

  // Load AP-always-on flag (default true)
  // Load AP mode. New key apMode (0=always-on, 1=10-min).
  // Migrate from old apOn bool if present (true=always-on, false=10-min).
  if (prefs.isKey("apMode")) {
    apMode = prefs.getUChar("apMode", 1);
  } else if (prefs.isKey("apOn")) {
    apMode = prefs.getBool("apOn", true) ? 0 : 1;
    prefs.putUChar("apMode", apMode);
    prefs.remove("apOn");
  } else {
    apMode = 1;   // default: 10-min auto-off
  }

  // Load OTA timestamps (stored as epoch seconds)
  firstBootAt   = prefs.getUInt("firstBoot", 0);
  lastUpdatedAt = prefs.getUInt("lastUpd", 0);
  Serial.printf("[BOOT] NVS firstBootAt=%u  lastUpdatedAt=%u\n",
                firstBootAt, lastUpdatedAt);

  // Last-sent values from previous boot — used to skip a redundant push if
  // the new confirmed reading matches what cloud already has. Prevents
  // double-history entries on reboot when sensor state hasn't changed.
  // 0xFF sentinel = "nothing stored yet" → force first push.
  lastSentBits  = prefs.getUChar("lsBits",  0xFF);
  lastSentPct   = prefs.getUChar("lsPct",   0xFF);
  lastSentFlags = prefs.getUChar("lsFlags", 0xFF);
  Serial.printf("[BOOT] NVS lastSent bits=%u pct=%u flags=%u\n",
                lastSentBits, lastSentPct, lastSentFlags);

  // Last values pushed to /history — separate gate so we never write
  // duplicate rows even when /live state changes are valid.
  lastHistoryBits  = prefs.getUChar("lhBits",  0xFF);
  lastHistoryPct   = prefs.getUChar("lhPct",   0xFF);
  lastHistoryFlags = prefs.getUChar("lhFlags", 0xFF);
  Serial.printf("[BOOT] NVS lastHistory bits=%u pct=%u flags=%u\n",
                lastHistoryBits, lastHistoryPct, lastHistoryFlags);
  // firstBootAt + lastUpdatedAt finalized later, after NTP sync

  prefs.end();

  apName = DEVICE_NAME;
  apName += "-";
  apName += deviceCode.substring(3, 7);  // First 4 chars of random part
  apName += "_mvstech";
  // Sanity guard: WiFi SSID limit is 32 chars. If we ever grow
  // DEVICE_NAME so the composed SSID exceeds 32, ESP-IDF silently
  // truncates AND drops the "_mvstech" suffix that MvsConnect app
  // uses to identify the AP. Refuse to boot in that case — better
  // than shipping a "won't pair" firmware to the field.
  if (apName.length() > 32) {
    Serial.printf("[FATAL] AP SSID '%s' is %u chars > 32 max. "
                  "Shorten DEVICE_NAME.\n", apName.c_str(), apName.length());
    // Fall back to a minimum-length SSID that DOES fit so the device
    // is still recoverable via a shorter name — still ends in
    // _mvstech so the app can find it.
    apName = "SF-" + deviceCode.substring(3, 7) + "_mvstech";
  }
  Serial.printf("[BOOT] AP SSID: '%s' (%u chars)\n", apName.c_str(), apName.length());
  mvs.setDeviceName(String(DEVICE_NAME) + "-" + deviceCode.substring(3, 7));
}

// ══════════════════════════════════════════════════
//  SERIAL REGISTRATION OUTPUT (FACTORY)
// ══════════════════════════════════════════════════

void printRegistrationInfo() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("  SENSEFLOW DEVICE REGISTRATION INFO");
  Serial.println("========================================");
  Serial.print("  Code:           "); Serial.println(deviceCode);
  Serial.print("  Class:          SENSOR (0x02)");
  Serial.println();
  Serial.print("  Sensor Type:    ");
  #if USE_ULTRASONIC
    Serial.println("ULTRASONIC (0x02)");
  #else
    Serial.println("DIP (0x01)");
  #endif
  Serial.print("  Sensor Count:   "); Serial.println(SENSOR_COUNT);
  Serial.print("  Firmware:       "); Serial.println(FIRMWARE_VERSION);
  Serial.print("  MAC:            "); Serial.println(WiFi.macAddress());
  Serial.print("  AP Name:        "); Serial.println(apName);
  Serial.println("========================================");
  Serial.println();
}

// ══════════════════════════════════════════════════
//  ADDRESSABLE LED (WS2812B via FastLED_min)
// ══════════════════════════════════════════════════

CRGB rgbLeds[1];
bool internetAvailable = false;          // result of DNS chain check
bool firebaseHealthy   = false;          // false after consecutive Firebase fails
unsigned long lastInternetCheck      = 0;
unsigned long lastFirebaseHealthRetry = 0;
unsigned long wifiLastConnectedAt    = 0;
unsigned long wifiLastDisconnectedAt = 0;
int   wifiReconnectAttempts = 0;

// 21.1.6: three-rung recovery ladder for stuck-pink / silent-wedge states.
//
// Triggers (either fires the ladder):
//   A) Pink state (WiFi connected but internet check fails) for > 10 min
//   B) No successful heartbeat push in > 30 min
//
// Ladder rungs (10-min gap between each — total 30 min from wedge to hard reset):
//   Rung 1: esp_wifi_stop() + esp_wifi_start() + restore AP + WiFi.begin()
//           → ~2 sec AP outage, clears WiFi driver state
//   Rung 2: same as Rung 1 (some routers need a second try)
//   Rung 3: hardware WDT reset (deeper chip re-init than ESP.restart())
//           → 30-40 sec full boot outage
//
// After Rung 3: wait 1 hour, then start ladder over from Rung 1. Prevents
// reboot loops while giving router/ISP time to recover on their end.
//
// Success signal for Trigger B: only the periodic heartbeat push counts
// (not /live change pushes) — heartbeat is our proof-of-life to cloud.
unsigned long pinkStateSinceMs         = 0;   // millis() when pink state started (0 = not pink)
unsigned long lastHeartbeatSuccessMs   = 0;   // millis() of last successful heartbeat push
uint8_t       recoveryRungExecuted     = 0;   // 0 = none, 1 = did rung 1, 2 = did rung 2, 3 = did rung 3
unsigned long recoveryLadderStartMs    = 0;   // millis() when ladder was armed (first trigger fired)
unsigned long recoveryCooldownUntilMs  = 0;   // millis() before which no new ladder attempts
const unsigned long PINK_TRIGGER_MS      = 10UL * 60UL * 1000UL;  // Trigger A: 10 min pink
const unsigned long SILENT_TRIGGER_MS    = 30UL * 60UL * 1000UL;  // Trigger B: 30 min no heartbeat
const unsigned long RUNG_GAP_MS          = 10UL * 60UL * 1000UL;  // 10 min between rungs
const unsigned long POST_LADDER_COOLDOWN = 60UL * 60UL * 1000UL;  // 1 hour after Rung 3

// v21.0.10: Non-blocking SSID visibility check with WDT feeding.
//
// The previous v21.0.8 implementation used WiFi.scanNetworks(false,...)
// which is BLOCKING and takes 3-4 sec. Combined with the MvsConnect
// library's own boot-time scan, total blocking time exceeded the ESP32
// Task WDT (5 sec) → crash loop. That's what v21.0.9 field-tested
// as: reboots every ~11 sec, AP flickers, sensor never gets to run.
//
// New approach:
//   - Sync scan is still fine (used sparingly), but we feed the WDT
//     via esp_task_wdt_reset() before and after.
//   - Boot skips the extra scan entirely — trust MvsConnect's scan
//     that already happened, OR skip visibility check and rely on
//     WiFi.begin() natural timeout (with our 60s reconnect gate).
bool isSsidVisible(const String& targetSsid) {
  if (targetSsid.length() == 0) return false;
  esp_task_wdt_reset();
  int16_t n = WiFi.scanNetworks(false, false);
  esp_task_wdt_reset();
  if (n <= 0) {
    Serial.printf("[SCAN] No APs visible (n=%d)\n", (int)n);
    WiFi.scanDelete();
    return false;
  }
  bool found = false;
  for (int16_t i = 0; i < n; i++) {
    if (WiFi.SSID(i) == targetSsid) { found = true; break; }
  }
  Serial.printf("[SCAN] Scanned %d APs, target '%s' %s\n",
                (int)n, targetSsid.c_str(), found ? "FOUND" : "not visible");
  WiFi.scanDelete();
  esp_task_wdt_reset();
  return found;
}

// Force Google DNS — fixes broken router DNS
void setGoogleDNS() {
  IPAddress dns1(8, 8, 8, 8);
  IPAddress dns2(8, 8, 4, 4);
  WiFi.config(WiFi.localIP(), WiFi.gatewayIP(), WiFi.subnetMask(), dns1, dns2);
  Serial.println("DNS set to 8.8.8.8 / 8.8.4.4");
}

// Check internet with DNS-server fallback chain.
// Tries Google → Cloudflare → Quad9 on port 53 with 1s timeout each.
// Returns true on first success, false only if all three fail.
// Works on virtually any consumer router (port 53 rarely blocked).
// 21.1.6: internet check via hostByName("firebaseio.com") — tests the
// FULL path we actually care about (DNS + WAN + our target host reachable)
// in one call. Falls back to raw TCP-to-DNS if hostByName fails, so a
// broken corporate DNS server doesn't false-negative us. If BOTH fail
// after 3-sec budget, internet is genuinely down.
//
// Why this is better than the old TCP-to-port-53 test:
//   - old test just proved packets reach 8.8.8.8, not that DNS works
//   - after LWIP wedge, opening a socket to 8.8.8.8:53 keeps failing
//     even though the actual internet is fine → false negative → pink LED
//   - hostByName uses the resolver stack, catches broken DNS explicitly
//   - direct test against firebaseio.com validates our real dependency
bool checkInternet() {
  if (WiFi.status() != WL_CONNECTED) return false;
  // Primary check: hostname resolution of our actual Firebase host.
  IPAddress ip;
  if (WiFi.hostByName("firebaseio.com", ip) && (uint32_t)ip != 0) {
    return true;
  }
  // Fallback: raw TCP to public DNS servers, in case hostByName is
  // wedged but socket layer works.
  const char* dnsServers[] = { "8.8.8.8", "1.1.1.1", "9.9.9.9" };
  for (int i = 0; i < 3; i++) {
    WiFiClient client;
    if (client.connect(dnsServers[i], 53, 1000)) {
      client.stop();
      return true;
    }
    client.stop();
  }
  return false;
}

// ══════════════════════════════════════════════════
//  21.1.6 — 3-RUNG RECOVERY LADDER
// ══════════════════════════════════════════════════
// Two triggers can arm the ladder:
//   A) Pink state persistent > PINK_TRIGGER_MS (10 min)
//      = WiFi connected but internet check fails → likely LWIP wedge or
//        broken DNS.
//   B) No successful heartbeat push > SILENT_TRIGGER_MS (30 min)
//      = "silent wedge" where all health checks look OK but cloud isn't
//        seeing us. Catches cases Trigger A misses.
//
// Rungs execute sequentially with RUNG_GAP_MS (10 min) between each so
// we give each attempt time to actually help before escalating. After
// Rung 3 (hard reset), POST_LADDER_COOLDOWN (1 hour) blocks new ladder
// attempts to avoid reboot-loop pathology when the problem is external.

// Rung 1 & 2: soft WiFi driver reset. Keeps SoftAP interface config so
// technician AP page comes back in ~2 sec instead of a full 30-40 sec boot.
void recoveryRungSoft(uint8_t rungNum) {
  Serial.printf("\n[RECOVERY] Rung %u: soft WiFi driver reset (esp_wifi_stop/start)\n", rungNum);
  esp_task_wdt_reset();
  // Stop WiFi driver — both STA and SoftAP go down for ~500 ms
  esp_wifi_stop();
  delay(500);
  esp_wifi_start();
  delay(200);
  // Restore AP first so technician retains reach even if STA fails
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apName.c_str(), AP_PASSWORD);
  esp_task_wdt_reset();
  // Retry STA using saved creds
  Preferences wp; wp.begin("mvswifi", true);
  String savedSsid = wp.getString("ssid", "");
  String savedPass = wp.getString("password", "");
  bool   savedVal  = wp.getBool("valid", false);
  wp.end();
  if (savedSsid.length() > 0 && savedVal) {
    // WiFi.disconnect(false) — never true (fleet rule: preserves ESP-IDF config)
    WiFi.disconnect(false);
    delay(200);
    WiFi.begin(savedSsid.c_str(), savedPass.c_str());
    Serial.printf("[RECOVERY] Rung %u: WiFi.begin('%s') issued\n", rungNum, savedSsid.c_str());
  } else {
    Serial.printf("[RECOVERY] Rung %u: no saved WiFi, AP-only mode\n", rungNum);
  }
}

// Rung 3: hardware WDT reset. Deeper than ESP.restart() — chip-level
// hardware reset via RWDT, closest software equivalent to a power-cycle.
// Clears LWIP state, driver internal state, radio hardware registers.
// AP is down for the full ~30-40 sec boot cycle.
void recoveryRungHard() {
  Serial.println("\n[RECOVERY] Rung 3: hardware WDT reset (deeper than ESP.restart)");
  Serial.flush();
  delay(1000);  // let serial output land before we die
  // Arm a fresh 1-sec panic Task WDT and don't feed it → chip-level HW
  // reset. ESP32 Arduino core 3.x uses the config-struct API.
  esp_task_wdt_deinit();
  esp_task_wdt_config_t wdtCfg = {
      .timeout_ms     = 1000,
      .idle_core_mask = 0,
      .trigger_panic  = true
  };
  esp_task_wdt_init(&wdtCfg);
  esp_task_wdt_add(NULL);
  // Don't feed the WDT — let it fire. Belt-and-braces ESP.restart() in
  // case some ESP-IDF core version doesn't honor the panic path.
  unsigned long deadline = millis() + 3000;
  while (millis() < deadline) { delay(100); }
  ESP.restart();   // never reached if WDT worked; fallback if not
}

// Called from loop() every iteration. Cheap when not armed. Manages the
// ladder timing + rung execution. Uses millis() throughout — overflow
// after ~49 days is not a concern because a wedge lasting that long has
// bigger problems.
void driveRecoveryLadder() {
  unsigned long now = millis();

  // Cooldown after Rung 3 — no new ladder attempts for 1 hour
  if (recoveryCooldownUntilMs != 0 && now < recoveryCooldownUntilMs) return;

  // Detect triggers
  bool pinkNow = (WiFi.status() == WL_CONNECTED) && !internetAvailable;
  if (pinkNow) {
    if (pinkStateSinceMs == 0) pinkStateSinceMs = now;
  } else {
    pinkStateSinceMs = 0;   // any recovery from pink resets the timer
  }
  bool triggerA = (pinkStateSinceMs != 0) && (now - pinkStateSinceMs > PINK_TRIGGER_MS);
  bool triggerB = (lastHeartbeatSuccessMs != 0) &&
                  (now - lastHeartbeatSuccessMs > SILENT_TRIGGER_MS);

  // Arm ladder on first trigger
  if ((triggerA || triggerB) && recoveryLadderStartMs == 0) {
    recoveryLadderStartMs = now;
    recoveryRungExecuted  = 0;
    Serial.printf("\n[RECOVERY] Ladder ARMED — trigger%s%s\n",
                  triggerA ? " A(pink>10m)" : "",
                  triggerB ? " B(no-heartbeat>30m)" : "");
  }
  // If neither trigger active AND ladder was armed, cancel it — we healed on our own
  if (!triggerA && !triggerB && recoveryLadderStartMs != 0 && recoveryRungExecuted == 0) {
    Serial.println("[RECOVERY] Ladder disarmed — device self-recovered before Rung 1");
    recoveryLadderStartMs = 0;
    return;
  }

  if (recoveryLadderStartMs == 0) return;   // not armed

  unsigned long elapsed = now - recoveryLadderStartMs;

  // Rung 1: fire immediately when armed (elapsed >= 0)
  if (recoveryRungExecuted == 0) {
    recoveryRungExecuted = 1;
    recoveryRungSoft(1);
    return;
  }
  // Rung 2: 10 min after Rung 1
  if (recoveryRungExecuted == 1 && elapsed >= RUNG_GAP_MS) {
    // Cancel escalation if we've recovered in the meantime
    if (!triggerA && !triggerB) {
      Serial.println("[RECOVERY] Recovered after Rung 1 — ladder complete");
      recoveryLadderStartMs = 0;
      recoveryRungExecuted  = 0;
      return;
    }
    recoveryRungExecuted = 2;
    recoveryRungSoft(2);
    return;
  }
  // Rung 3: 20 min after Rung 1 (10 min after Rung 2) — hard reset
  if (recoveryRungExecuted == 2 && elapsed >= 2 * RUNG_GAP_MS) {
    if (!triggerA && !triggerB) {
      Serial.println("[RECOVERY] Recovered after Rung 2 — ladder complete");
      recoveryLadderStartMs = 0;
      recoveryRungExecuted  = 0;
      return;
    }
    recoveryRungExecuted = 3;
    recoveryCooldownUntilMs = now + POST_LADDER_COOLDOWN;   // set BEFORE hard reset so post-boot state respects it
    recoveryRungHard();   // does not return
    return;
  }
  // After Rung 3 — cooldown active, ladder disarmed
  if (recoveryRungExecuted == 3) {
    // Should never actually reach here (Rung 3 reboots), but if it does,
    // arm cooldown and clear state for next cycle.
    recoveryLadderStartMs = 0;
    recoveryRungExecuted  = 0;
  }
}

void setLED(uint8_t r, uint8_t g, uint8_t b) {
  rgbLeds[0] = CRGB(r, g, b);
  FastLED_min<LED_PIN>.show();
}

void setLEDOff() { setLED(0, 0, 0); }

// Level colors matching LED sticker spec
void setLevelColor(uint8_t pct) {
  if (pct == 0)       setLED(255, 0, 0);       // Red - Empty
  else if (pct <= 25) setLED(255, 80, 0);       // Orange - Low
  else if (pct <= 50) setLED(255, 200, 0);      // Yellow - Half
  else if (pct <= 75) setLED(0, 229, 255);      // Cyan - Good
  else                setLED(0, 200, 0);        // Green - Full
}

// ══════════════════════════════════════════════════
//  DIP SENSOR LOGIC
// ══════════════════════════════════════════════════

#if !USE_ULTRASONIC

void initDipSensors() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(DIP_PINS[i], INPUT_PULLDOWN);  // External 10k pull-down required on 34/35
  }
  pinMode(DIP_COMMON_PIN, OUTPUT);
#if EXCITATION_MODE == 0
  // Constant DC mode — common held active permanently. Probes read with plain digitalRead.
  digitalWrite(DIP_COMMON_PIN, DIP_COMMON_DRIVE_ACTIVE);
#else
  // OCSIL pulsed — common parked idle between bursts (no current flow, no electrolysis).
  digitalWrite(DIP_COMMON_PIN, DIP_COMMON_DRIVE_IDLE);
#endif
}

// safeRestart() moved to the shared section above so ultrasonic mode
// can call it too. GPIO12 strap-safety comment lives with the function.

#if EXCITATION_MODE == 0
// CONSTANT DC read — common is held HIGH all the time; just sample each probe.
uint8_t readDipRaw() {
  uint8_t bits = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (digitalRead(DIP_PINS[i]) == HIGH) bits |= (1 << i);
  }
  return bits;
}
#else
// Synchronous (unipolar square-wave) read on common pin.
// Drives common HIGH then LOW for DIP_SAMPLES_PER_READ cycles. A probe is
// counted as "wet" for a cycle only if it reads HIGH while common is HIGH
// AND LOW while common is LOW — i.e. it tracks the excitation. Stuck-HIGH
// pins (leakage, plating film, residual moisture) fail the LOW-phase check
// and get rejected. Returns a bitmask where bit i = 1 if probe i was wet
// for at least DIP_AGREE_THRESHOLD of DIP_SAMPLES_PER_READ cycles.
uint8_t readDipRaw() {
  uint8_t agreeCount[6] = {0, 0, 0, 0, 0, 0};

  for (int s = 0; s < DIP_SAMPLES_PER_READ; s++) {
    digitalWrite(DIP_COMMON_PIN, DIP_COMMON_DRIVE_ACTIVE);
    delayMicroseconds(DIP_SETTLE_US);
    uint8_t highSample = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
      if (digitalRead(DIP_PINS[i]) == HIGH) highSample |= (1 << i);
    }

    digitalWrite(DIP_COMMON_PIN, DIP_COMMON_DRIVE_IDLE);
    delayMicroseconds(DIP_SETTLE_US);
    uint8_t lowSample = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
      if (digitalRead(DIP_PINS[i]) == LOW) lowSample |= (1 << i);
    }

    uint8_t agree = highSample & lowSample;
    for (int i = 0; i < SENSOR_COUNT; i++) {
      if (agree & (1 << i)) agreeCount[i]++;
    }
  }

  // Park common idle between reads (minimises DC bias, slows electrolysis)
  digitalWrite(DIP_COMMON_PIN, DIP_COMMON_DRIVE_IDLE);

  uint8_t bits = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (agreeCount[i] >= DIP_AGREE_THRESHOLD) bits |= (1 << i);
  }
  return bits;
}
#endif  // EXCITATION_MODE

// Count consecutive ON sensors from bottom (bit 0)
int countConsecutive(uint8_t bits, int count) {
  int consecutive = 0;
  for (int i = 0; i < count; i++) {
    if (bits & (1 << i)) {
      consecutive++;
    } else {
      break;
    }
  }
  return consecutive;
}

// Check for physics violation (non-consecutive sensors)
bool checkSensorError(uint8_t bits, int count) {
  int totalOn = 0;
  for (int i = 0; i < count; i++) {
    if (bits & (1 << i)) totalOn++;
  }
  int consecutive = countConsecutive(bits, count);
  return (totalOn != consecutive);  // Error if non-consecutive
}

uint8_t bitsToPercent(uint8_t bits, int count) {
  // Count consecutive ON sensors from bottom (bit 0 = GPIO 32 = bottom)
  // 0001 = 25%, 0011 = 50%, 0111 = 75%, 1111 = 100%
  int consecutive = countConsecutive(bits, count);
  if (consecutive == 0) return 0;
  if (count >= 1 && count <= 6) {
    return DIP_PCT_TABLE[count][consecutive - 1];
  }
  return 0;
}

// ── Majority-vote level stability algorithm ─────────────────────────
// Field devices kept showing 25↔0 and 75↔100 flicker at probe boundaries
// even after the earlier "time-dominant" attempt. Root cause: the
// fallback "most recent stable run" rule let occasional transient runs
// commit — so once a wave held one level for 15 sec, it committed, then
// another wave held the other level for 15 sec, it committed again.
//
// New rule (per Vishal's clarification): ONLY commit when a level wins
// a clean majority of the full window. If no clean majority exists,
// don't commit anything — cloud dashboard keeps interpolating the last
// committed value, so the analytics chart stays clean.
//
// Discrete-level physics: the 4-probe DIP produces exactly 5 possible
// resolved levels (0/25/50/75/100). Majority voting fits naturally.
//
// Effects:
//   - Wave train at boundary (samples ~50/50 split): no level wins
//     majority → no push → cloud stays flat at previous value ✓
//   - Genuine fill / drain: new level appears in ~90% of samples once
//     water settles → majority triggers → single clean commit ✓
//   - Fast pump-in that stays at 100%: takes ~1 min to reach majority
//     but then commits cleanly, one event in history, not a burst ✓
//   - Non-consecutive fault glitch: resolver still computes highest-wet
//     level so brief faults count as their resolved value (75%) — same
//     bucket as normal 75% reads, no dominance shift ✓

// Tune here if boundary flicker on a specific site needs stricter
// settings. Larger STABLE_SAMPLES = longer window = more delay before
// real changes reach cloud, but stronger noise rejection. Higher
// MAJORITY_PCT = stricter — needs cleaner consensus to commit.
#define STABLE_SAMPLES    30         // full window = ~90 sec at 3-sec read interval
#define MAJORITY_PCT      70         // level must win >60% of the votes to commit
#define STABLE_WINDOW_MS  (uint32_t)((uint32_t)STABLE_SAMPLES * (uint32_t)DIP_READ_INTERVAL_MS)

struct SampleRec {
  uint32_t ts;    // millis() when this sample was taken
  uint8_t  pct;   // resolved level from readDipRaw() at that moment
};

static SampleRec sampleBuf[STABLE_SAMPLES];
static uint8_t   sampleHead = 0;   // next write slot
static uint8_t   sampleCount = 0;  // how many valid samples are in the buffer

// instantPct / instantBits moved above the #if !USE_ULTRASONIC block
// so they're visible in both sensor modes (handleLED + /sstatus need them).

// One-shot flag: on very first Firebase-ready moment we push instantPct
// so the cloud dashboard shows the device online immediately, then hand
// off to the majority-vote gate for all subsequent pushes.
bool committedOnce = false;

// Resolve a raw bit pattern to a percentage using the highest-wet-probe
// rule (physically correct: water above touching a probe means water
// exists at all levels below it). This deliberately IGNORES non-
// consecutive faults so a brief 0110 glitch resolves as 75%, not 0%,
// matching the cloud resolver's behaviour and preventing false "empty"
// reports from a bad lower probe.
uint8_t highestWetPct(uint8_t bits, int count) {
  for (int i = count - 1; i >= 0; i--) {
    if (bits & (1 << i)) {
      if (count >= 1 && count <= 6) return DIP_PCT_TABLE[count][i];
      return 0;
    }
  }
  return 0;
}

// Compute the committed level using majority voting over a FULL sample
// window. Returns 0xFF ("no majority") if no level clears MAJORITY_PCT
// or the buffer isn't full yet — caller must interpret that as "don't
// push, keep old committed value in cloud."
uint8_t computeMajorityLevel() {
  // Wait for a full window before any commit. Prevents 3-sample early
  // consensus from producing a false first commit.
  if (sampleCount < STABLE_SAMPLES) return 0xFF;

  // Discrete levels: 0/25/50/75/100 = 5 buckets. Any non-standard
  // resolved value (shouldn't happen with the resolver) falls into
  // bucket 4 (100%) via clamping.
  uint16_t votes[5] = {0, 0, 0, 0, 0};
  uint16_t total = 0;
  for (uint8_t k = 0; k < sampleCount; k++) {
    uint8_t idx = (sampleHead + STABLE_SAMPLES - sampleCount + k) % STABLE_SAMPLES;
    uint8_t p = sampleBuf[idx].pct;
    uint8_t bucket = p / 25;
    if (bucket > 4) bucket = 4;
    votes[bucket]++;
    total++;
  }
  if (total == 0) return 0xFF;

  // Winner: strictly more than MAJORITY_PCT of the window
  for (uint8_t i = 0; i < 5; i++) {
    if ((uint32_t)votes[i] * 100 > (uint32_t)total * MAJORITY_PCT) {
      return (uint8_t)(i * 25);
    }
  }
  return 0xFF;   // no majority — do not commit, cloud keeps old value
}

void processDipSensors() {
  static unsigned long lastDipRead = 0;

  if (millis() - lastDipRead < DIP_READ_INTERVAL_MS) return;
  lastDipRead = millis();

  uint8_t currentRaw = readDipRaw();

  // Instant view — populated every read for LED + AP page live tank viz.
  instantBits = currentRaw;
  instantPct  = highestWetPct(currentRaw, SENSOR_COUNT);

  // Append to rolling buffer
  sampleBuf[sampleHead] = { millis(), instantPct };
  sampleHead = (sampleHead + 1) % STABLE_SAMPLES;
  if (sampleCount < STABLE_SAMPLES) sampleCount++;

  // On the very first Firebase-ready reading, seed the committed value
  // from the instant read so the cloud dashboard sees the device online
  // with SOME value before the majority-vote gate kicks in ~90 sec later.
  // Only fires once per boot; after that only majority-vote can change
  // the committed value.
  if (!committedOnce && firebaseReady) {
    committedOnce = true;
    if (confirmedPct != instantPct) {
      confirmedPct = instantPct;
      sensorBits   = 0;
      if (SENSOR_COUNT >= 1 && SENSOR_COUNT <= 6) {
        const uint8_t *tbl = DIP_PCT_TABLE[SENSOR_COUNT];
        for (int i = 0; i < SENSOR_COUNT; i++) {
          sensorBits |= (1 << i);
          if (tbl[i] == confirmedPct) break;
        }
        if (confirmedPct == 0) sensorBits = 0;
      }
    }
  }

  // Majority-vote decision. 0xFF = "no clear winner" → keep last
  // committed value; cloud dashboard shows a flat line instead of flicker.
  uint8_t majority = computeMajorityLevel();
  if (majority != 0xFF && majority != confirmedPct) {
    confirmedPct = majority;
    // Canonical consecutive-from-bottom bits for the new level. Keeps
    // /live's sensorBits internally consistent with confirmedPct for
    // downstream code + the dashboard resolver.
    sensorBits = 0;
    if (SENSOR_COUNT >= 1 && SENSOR_COUNT <= 6) {
      const uint8_t *tbl = DIP_PCT_TABLE[SENSOR_COUNT];
      for (int i = 0; i < SENSOR_COUNT; i++) {
        sensorBits |= (1 << i);
        if (tbl[i] == confirmedPct) break;
      }
      if (confirmedPct == 0) sensorBits = 0;
    }
    Serial.printf("[LEVEL] Majority committed: %d%%\n", confirmedPct);
  }

  // sensor_error flag: only set if the INSTANT raw pattern is genuinely
  // non-consecutive AND has been for the whole window (fault-persistent).
  // Brief flickers are absorbed by the dominant-level algorithm above and
  // never surface as ERR to the customer.
  sensorError = checkSensorError(currentRaw, SENSOR_COUNT);
  if (sensorError) flags |= 0x01; else flags &= ~0x01;
  pendingBits = currentRaw;
}

#endif

// ══════════════════════════════════════════════════
//  ULTRASONIC SENSOR LOGIC
// ══════════════════════════════════════════════════

#if USE_ULTRASONIC

// ── Sensor type + UART (v20.0.0) ────────────────────────────────────
// One firmware supports HC-SR04 (legacy), DYP-A01 (UART controlled),
// DYP-A21 (UART auto-stream). Detected once at boot, cached in NVS.
HardwareSerial usSerial(2);   // UART2 — pins remapped in begin()
uint8_t usSensorType   = US_TYPE_UNKNOWN;
uint32_t usDypBaud     = 9600;   // A01=9600 always, A21 defaults 115200

// RS485/Modbus config — installer-set via AP page, persisted to NVS.
// Manual-only (see US_TYPE_RS485 comment above) so these are never
// touched by the DYP autodetect ladder.
uint8_t usRs485Addr = 1;                          // Modbus slave address, 1-247
// Register is fixed to REAL-TIME (0x0101), not installer-selectable.
// Field testing showed the PROCESSED register (0x0100) has a ~500ms
// worst-case response time and produced false/stale-looking readings
// under this firmware's burst-polling cadence — real-time (0x0101) is
// ~100ms and far more reliable. The existing histogram/hysteresis/
// jump-limit filter stack (same pipeline the DYP path already uses)
// handles the extra raw noise that comes with skipping the sensor's
// own internal smoothing, so there's no accuracy tradeoff in practice.
uint16_t usRs485Reg = US_RS485_REG_REALTIME;

// v21.0.11: RAW UART debug. For the first 5 sec after boot, every byte
// read from the sensor is printed to Serial as hex. Lets us see (a)
// whether ANY bytes arrive on GPIO 34 (wiring/baud check), (b) if they
// look like the expected 0xFF-prefixed frames.
static uint32_t g_dbgUartUntilMs = 0;
static uint16_t g_dbgUartBytesLogged = 0;
static void dbgUartByte(uint8_t b) {
  if (millis() > g_dbgUartUntilMs || g_dbgUartBytesLogged > 200) return;
  Serial.printf("[UART-DBG] %02X\n", b);
  g_dbgUartBytesLogged++;
}

// Read exactly one DYP 4-byte distance frame (0xFF HH LL SUM) from
// usSerial with the given per-byte timeout. Returns distance in cm, or
// -1 if timeout / bad checksum / out-of-range. Drains one byte at a
// time so we resync on partial frames without blocking the whole loop.
float readDypFrameCm(uint32_t timeoutMs) {
  // 21.1.5: adaptive timeout — once sensor has been flagged missing (15+
  // consecutive empty bursts), cap the per-ping wait to 30ms. That drops
  // a full 15-ping burst from ~2400ms to ~450ms, keeping loop() well
  // under the 8-sec task-WDT threshold. Any successful frame clears
  // usSensorOffline via processUltrasonic() and full timeouts return.
  if (usSensorOffline && timeoutMs > 30) timeoutMs = 30;
  uint32_t deadline = millis() + timeoutMs;
  while (millis() < deadline) {
    if (usSerial.available() < 4) { delay(1); continue; }
    // Find start byte 0xFF
    uint8_t peek = usSerial.peek();
    if (peek != 0xFF) { dbgUartByte(peek); usSerial.read(); continue; }
    uint8_t frame[4];
    usSerial.readBytes(frame, 4);
    for (int i = 0; i < 4; i++) dbgUartByte(frame[i]);
    uint8_t sum = (frame[0] + frame[1] + frame[2]) & 0xFF;
    if (sum != frame[3]) {
      Serial.printf("[UART-DBG] BAD_SUM want=%02X got=%02X frame=%02X %02X %02X %02X\n",
                    sum, frame[3], frame[0], frame[1], frame[2], frame[3]);
      continue;
    }
    uint16_t distMm = (frame[1] << 8) | frame[2];
    Serial.printf("[UART-DBG] OK_FRAME dist=%u mm\n", distMm);
    if (distMm == 0xFFFE) continue;                          // A21 same-freq interference flag
    float distCm = distMm / 10.0f;
    if (distCm < US_BLIND_ZONE || distCm > US_MAX_RANGE) {
      Serial.printf("[UART-DBG] OUT_OF_RANGE %.1f cm\n", distCm);
      return -1;
    }
    return distCm;
  }
  return -1;
}

// DYP-A01 controlled trigger + read.
// v21.0.9: switched from usSerial.write(0x55) to a pin-toggle trigger
// per uart_auto.pdf section 3.2: "When pin(RX) receives a falling edge
// pulse, the module will perform a measurement." The datasheet does NOT
// specify a magic byte — the sensor triggers on a falling-edge line
// event, not a UART character. Approach:
//   1) end() UART so we own the pin
//   2) drive TX pin (= sensor RX) HIGH, wait, LOW, wait, HIGH → falling edge
//   3) re-begin() UART @ 9600 for the response
//   4) read 4-byte frame (sensor takes ~50-60 ms to reply per T2 spec)
// Trigger cycle must be >70 ms per datasheet — we're gated to 2 sec by
// US_READ_INTERVAL so that's satisfied.
float readDypA01Cm() {
  usSerial.end();
  pinMode(US_UART_TX_PIN, OUTPUT);
  digitalWrite(US_UART_TX_PIN, HIGH);
  delayMicroseconds(200);
  digitalWrite(US_UART_TX_PIN, LOW);
  delayMicroseconds(500);                                    // >100 us pulse per timing diagram
  digitalWrite(US_UART_TX_PIN, HIGH);
  usSerial.begin(9600, SERIAL_8N1, US_UART_RX_PIN, US_UART_TX_PIN);
  return readDypFrameCm(150);                                // T2=50-60ms + margin
}

// ── RS485/Modbus-RTU read (bench-validated protocol) ────────────────
// CRC-16/MODBUS: poly 0xA001, init 0xFFFF. Verified against sensor
// datasheet's worked example (01 03 01 00 00 01 -> CRC 85 F6).
static uint16_t rs485ModbusCRC16(const uint8_t* buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else              { crc >>= 1; }
    }
  }
  return crc;
}

// Sends a Modbus-RTU "Read Holding Register" request (function 0x03) for
// a single register at usRs485Addr/usRs485Reg and returns the raw 16-bit
// value (mm for either distance register), or -1 on timeout/CRC/format
// error. Single attempt — caller (readUltrasonicRaw / burst loop in
// processUltrasonic) already retries via US_SAMPLES pings same as DYP.
float readRs485Cm() {
  uint8_t req[8];
  req[0] = usRs485Addr;
  req[1] = 0x03;
  req[2] = (usRs485Reg >> 8) & 0xFF;
  req[3] = usRs485Reg & 0xFF;
  req[4] = 0x00;
  req[5] = 0x01;                     // read 1 register
  uint16_t crc = rs485ModbusCRC16(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  while (usSerial.available()) usSerial.read();   // flush stale bytes

  digitalWrite(US_RS485_DIR_PIN, HIGH);            // transmit mode
  delayMicroseconds(100);
  usSerial.write(req, 8);
  usSerial.flush();
  delayMicroseconds(100);
  digitalWrite(US_RS485_DIR_PIN, LOW);             // back to receive mode

  // Expected reply: addr(1) + func(1) + bytecount(1) + data(2) + crc(2) = 7 bytes.
  // 500ms overall cap (sensor's documented worst-case response time for the
  // processed register), 30ms inter-byte gap (abandon a stalled/noisy
  // partial frame early rather than waiting out the full window).
  uint8_t resp[7];
  uint8_t received = 0;
  unsigned long start = millis();
  unsigned long lastByteAt = start;
  const unsigned long INTERBYTE_TIMEOUT_MS = 30;
  while (received < 7 && (millis() - start) < 500) {
    if (usSerial.available()) {
      resp[received++] = usSerial.read();
      lastByteAt = millis();
    } else if (received > 0 && (millis() - lastByteAt) > INTERBYTE_TIMEOUT_MS) {
      break;
    }
  }
  if (received < 7) return -1;
  if (resp[0] != usRs485Addr || resp[1] != 0x03 || resp[2] != 0x02) return -1;

  uint16_t respCrc = rs485ModbusCRC16(resp, 5);
  uint16_t recvCrc = resp[5] | (resp[6] << 8);
  if (respCrc != recvCrc) return -1;

  uint16_t distanceMm = (resp[3] << 8) | resp[4];
  float distCm = distanceMm / 10.0f;
  // Same blind-zone/max-range validation as the DYP path (readDypFrameCm)
  // so downstream zone-memory fallback logic sees a consistent contract:
  // out-of-range readings come back as -1, never a raw out-of-band value.
  if (distCm < US_BLIND_ZONE || distCm > US_MAX_RANGE) return -1;
  return distCm;
}

// v21.0.0 A01_750cm branch: HC-SR04 path and detectSensorType() removed.
// initUltrasonic() runs its own Auto-vs-Controlled detection inline.

void initUltrasonic() {
  // v21.1.2: mark boot start for warmup gate — used to hold /live
  // pushes for US_BOOT_WARMUP_MS while we collect stable samples.
  usBootStartMs    = millis();
  usBootPctCount   = 0;
  usBootWarmupDone = false;

  // v21.0.0 A01_750cm branch: DYP-only firmware. HC-SR04 not supported.
  // Detects whether the connected DYP horn sensor is Auto-output
  // (A01ANYUB — streams frames every 100 ms) or Controlled-output
  // (A01ANYTB — replies only when we send 0x55). Both use identical
  // 4-byte frames (0xFF HH LL SUM) and same pins.
  //
  // Detection sequence @ 9600 baud (both A01 auto/controlled use 9600):
  //   1. Listen 500 ms → if a frame arrives on its own → AUTO (A21 path)
  //   2. Send 0x55, wait 200 ms → if a frame arrives → CONTROLLED (A01 path)
  //   3. Otherwise → mark unknown, retry every boot (or installer overrides
  //      via /setustype on AP page)
  //
  // Cached in NVS "usType" for fast boot after first successful detect.
  Preferences p;
  p.begin("senseflow", true);
  usSensorType = p.getUChar("usType", US_TYPE_UNKNOWN);
  usDypBaud    = p.getUInt("usBaud", 9600);
  uint8_t detectVer = p.getUChar("usDetV", 0);
  usRs485Addr  = p.getUChar("usRs485A", 1);
  // Register is no longer installer-selectable — always real-time (see
  // comment at usRs485Reg's declaration). Ignore any value previously
  // stored in NVS from before this change; don't even read the key.
  usRs485Reg   = US_RS485_REG_REALTIME;
  p.end();

  // RS485/Modbus is manual-only (installer-selected via /setustype, never
  // part of the DYP autodetect ladder below — see US_TYPE_RS485 comment).
  // Handle it here and return early: none of the DYP-specific cache-version
  // guard, TX-pin power-on-state trick, or baud/protocol autodetection
  // below apply to a Modbus sensor.
  if (usSensorType == US_TYPE_RS485) {
    usSerial.end();
    pinMode(US_RS485_DIR_PIN, OUTPUT);
    digitalWrite(US_RS485_DIR_PIN, LOW);   // receive mode by default
    usSerial.begin(US_RS485_BAUD, SERIAL_8N1, US_RS485_RX_PIN, US_RS485_TX_PIN);
    Serial.printf("[US] RS485 Modbus mode: addr=%u reg=0x%04X\n", usRs485Addr, usRs485Reg);
    // Boot-time health ping, WDT-bracketed same as isSsidVisible()'s scan —
    // a single Modbus transaction at 9600 baud is ~20-50ms, well under any
    // watchdog threshold, but bracket it anyway per the file's established
    // defensive idiom in case the sensor hangs the UART read.
    esp_task_wdt_reset();
    float bootDist = readRs485Cm();
    esp_task_wdt_reset();
    if (bootDist > 0) {
      Serial.printf("[US] RS485 boot ping OK: %.1f cm\n", bootDist);
    } else {
      Serial.println("[US] RS485 boot ping: no reply (check wiring/address)");
    }
    return;
  }

  // v21.0.13: force fresh detection ONE more time. v21.0.12 wrote
  // usDetV=12 during test runs even when the "detection" was actually
  // using stale AUTO cache, so version 12 cache is untrustworthy.
  if (detectVer < 13) {
    Serial.printf("[US] Cache detectVer=%u < 13 — clearing to force fresh baud detection\n", detectVer);
    usSensorType = US_TYPE_UNKNOWN;
    Preferences pw;
    pw.begin("senseflow", false);
    pw.putUChar("usDetV", 13);
    pw.end();
  }

  // v21.0.11: Enable raw UART byte debug for first 15 sec after boot
  g_dbgUartUntilMs = millis() + 15000;
  Serial.println("[UART-DBG] Byte logging enabled for 15 sec");

  // v21.0.9: Force TX pin (= sensor's RX) HIGH BEFORE anything else.
  // Per uart_auto.pdf section 2.2: A01ANYUB only checks its RX-pin
  // level at POWER-ON. High/floating → sensor outputs "processed value"
  // every 300-500 ms (stable, temperature-compensated, filtered). Low →
  // real-time mode, 100 ms cycle, less stable. We want processed mode.
  //
  // Note: this only helps if we boot BEFORE the sensor does (e.g. shared
  // power rail comes up together). In practice ESP32 boots ~0.5 sec
  // faster than sensor MCU, so this pin state is what sensor sees at
  // its power-up window.
  pinMode(US_UART_TX_PIN, OUTPUT);
  digitalWrite(US_UART_TX_PIN, HIGH);
  delay(10);

  // v21.0.12: try BOTH baud rates. v21.0.11 diagnostic showed sensor
  // emitting only 0xFE bytes at 9600 baud — classic wrong-baud signature.
  // A21 twoEyes datasheet says default is 115200 (modifiable). A01 auto
  // datasheet says 9600. Since we can't be sure which sensor is wired
  // (A21 vs A01), try 115200 first (more likely for the auto-stream
  // sensor the user has), then fall back to 9600.
  //
  // Detection order:
  //   1. 115200 baud, listen for auto-stream (1 sec)
  //   2. 9600 baud, listen for auto-stream (1 sec)
  //   3. 9600 baud, try controlled trigger
  // First to produce a valid frame wins, and we cache the baud.

  bool fromCache = (usSensorType == US_TYPE_DYP_A01 || usSensorType == US_TYPE_DYP_A21);
  if (fromCache) {
    const char* modeName = (usSensorType == US_TYPE_DYP_A21) ? "AUTO stream" : "CONTROLLED";
    Serial.printf("[US] Using cached DYP mode: %s @ %u baud\n", modeName, usDypBaud);
    usSerial.end();
    usSerial.begin(usDypBaud, SERIAL_8N1, US_UART_RX_PIN, US_UART_TX_PIN);
    return;
  }

  Serial.println("[US] v21.0.12: detecting sensor + baud rate...");
  uint32_t candidates[] = { 115200, 9600 };
  float d = -1.0f;
  uint32_t winningBaud = 0;

  // Try each baud for AUTO-STREAM detection
  for (uint32_t baud : candidates) {
    Serial.printf("[US] Trying AUTO @ %u baud...\n", baud);
    usSerial.end();
    usSerial.begin(baud, SERIAL_8N1, US_UART_RX_PIN, US_UART_TX_PIN);
    delay(50);
    while (usSerial.available()) usSerial.read();
    d = readDypFrameCm(1200);
    if (d > 0) {
      Serial.printf("[US] Detected AUTO stream @ %u baud: %.1f cm\n", baud, d);
      usSensorType = US_TYPE_DYP_A21;
      winningBaud = baud;
      break;
    }
  }

  // Try controlled trigger only at 9600 (A01 controlled always 9600)
  if (usSensorType == US_TYPE_UNKNOWN) {
    Serial.println("[US] Trying CONTROLLED @ 9600 baud...");
    usSerial.end();
    usSerial.begin(9600, SERIAL_8N1, US_UART_RX_PIN, US_UART_TX_PIN);
    delay(50);
    d = readDypA01Cm();
    if (d > 0) {
      Serial.printf("[US] Detected CONTROLLED: %.1f cm\n", d);
      usSensorType = US_TYPE_DYP_A01;
      winningBaud = 9600;
    }
  }

  if (usSensorType != US_TYPE_UNKNOWN && winningBaud > 0) {
    usDypBaud = winningBaud;
    Preferences pw;
    pw.begin("senseflow", false);
    pw.putUChar("usType", usSensorType);
    pw.putUInt("usBaud",  usDypBaud);
    pw.end();
  } else {
    Serial.println("[US] No DYP sensor detected on any baud rate");
  }
}

// v21.0.0 A01_750cm: DYP-only dispatch. No HC-SR04 path.
// If sensor type is UNKNOWN (detection failed at boot), returns -1
// so processUltrasonic() falls back to zone memory instead of reading
// garbage from an unconfigured UART.
float readUltrasonicRaw() {
  switch (usSensorType) {
    case US_TYPE_DYP_A01: return readDypA01Cm();               // controlled: send 0x55
    case US_TYPE_DYP_A21: return readDypFrameCm(150);          // auto: drain stream
    case US_TYPE_RS485:   return readRs485Cm();                // Modbus-RTU read holding register
    default:              return -1;                            // unknown → fallback
  }
}

// Median Absolute Deviation filter
float madFilter(float* samples, int count) {
  // Sort for median
  float sorted[US_SAMPLES];
  memcpy(sorted, samples, count * sizeof(float));
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (sorted[j] < sorted[i]) {
        float tmp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = tmp;
      }
    }
  }

  float median = sorted[count / 2];

  // Calculate MAD
  float deviations[US_SAMPLES];
  for (int i = 0; i < count; i++) {
    deviations[i] = abs(samples[i] - median);
  }
  // Sort deviations
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (deviations[j] < deviations[i]) {
        float tmp = deviations[i];
        deviations[i] = deviations[j];
        deviations[j] = tmp;
      }
    }
  }
  float mad = deviations[count / 2];

  // Filter outliers, average inliers
  float sum = 0;
  int inliers = 0;
  for (int i = 0; i < count; i++) {
    if (abs(samples[i] - median) <= US_MAD_MULTIPLIER * mad + 0.001) {
      sum += samples[i];
      inliers++;
    }
  }

  return (inliers > 0) ? (sum / inliers) : median;
}

// ── Zone classifier ────────────────────────────────────────────────
// v21.0.1: zones now scale with usTankHeight (installer-entered on AP
// page). Blind zone (UPPER) is a physics constant tied to the sensor
// (28-35 cm for DYP-A01), but MIDDLE/LOWER boundaries move with the
// tank height:
//   - UPPER : 28-35 cm     → 100% (near sensor = tank full)
//   - MIDDLE: 36 cm to tankHeight → linear map to percentage
//   - LOWER : > tankHeight → 0% (empty — beam went past tank floor)
// Sensor hardware cap (US_MAX_RANGE=750) still applies as an outer
// bound so nonsense readings from open sky don't count.
uint8_t classifyZone(float distanceCm) {
  // v21.0.5: zones bounded by overflow (top) and suction cutoff (bottom).
  //   UPPER  : blind zone to overflow pipe. Hysteresis bypassed — spill
  //            events commit as 100% immediately.
  //   MIDDLE : overflow to suction cutoff. Normal range, hysteresis applies.
  //   LOWER  : below suction cutoff. Commits as 0% immediately (pump dry).
  float upperMax        = (usOverflow > 0 && usOverflow < usTankHeight)
                          ? (float)usOverflow : (float)ZONE_UPPER_MAX;
  float suctionCutoffCm = (usSuction > 0 && usSuction < (uint16_t)usTankHeight)
                          ? (usTankHeight - (float)usSuction)
                          : usTankHeight;
  if (distanceCm >= ZONE_UPPER_MIN && distanceCm <= upperMax)           return US_ZONE_UPPER;
  if (distanceCm > upperMax        && distanceCm <= suctionCutoffCm)    return US_ZONE_MIDDLE;
  if (distanceCm > suctionCutoffCm && distanceCm <= US_MAX_RANGE)       return US_ZONE_LOWER;
  return US_ZONE_NONE;
}

// Persist zone + last valid pct so on reboot we resume with correct
// fallback (e.g. tank was full at power-cut → boot with 100% not 0%).
// Only writes when values actually change — NVS wear stays negligible
// even for a device that runs for years. Zone parameter is uint8_t
// (holding UsZone values) for Arduino auto-prototype compatibility.
void saveZoneMemory(uint8_t zone, uint8_t pct) {
  static uint8_t lastPersistedZone = US_ZONE_NONE;
  static uint8_t lastPersistedPct = 0xFF;
  if (zone == lastPersistedZone && pct == lastPersistedPct) return;
  lastPersistedZone = zone;
  lastPersistedPct  = pct;
  Preferences zp;
  zp.begin("senseflow", false);
  zp.putUChar("usZone",    zone);
  zp.putUChar("usLastPct", pct);
  zp.end();
}

// Three-strike hysteresis for MIDDLE zone. Rejects single-frame
// spikes; accepts a real change after N consecutive out-of-range
// reads. Returns the value to use (either the incoming reading if
// accepted, or the last good value if the spike was rejected).
float applyThreeStrikeHysteresis(float newValue) {
  if (!usHystInitialised) {
    usHystInitialised = true;
    usHystLastGood = newValue;
    usHystOutCounter = 0;
    return newValue;
  }
  float diff = fabsf(newValue - usHystLastGood);
  if (diff <= US_HYST_TOLERANCE_CM) {
    // Within tolerance — accept, reset counter
    usHystOutCounter = 0;
    usHystLastGood = newValue;
    return newValue;
  }
  // Out of tolerance — need N confirmations before accepting
  usHystOutCounter++;
  if (usHystOutCounter >= US_HYST_LIVE_STRIKES) {
    usHystLastGood = newValue;
    usHystOutCounter = 0;
    return newValue;    // confirmed real change
  }
  return usHystLastGood; // spike — hold last value
}

void resetHysteresisState() {
  // Reset counter when we leave MIDDLE zone. Keep usHystInitialised
  // true and usHystLastGood as-is so re-entering MIDDLE has a
  // reasonable starting point instead of re-initialising to whatever
  // the first read happens to be.
  usHystOutCounter = 0;
}

// Compute integer percent from a validated distance. Always rounds UP
// (ceilf) — customer never gets "short-changed" on a reading. E.g.
// 37.1% displays as 38, 37.9% displays as 38, 38.0% displays as 38.
uint8_t distanceToIntegerPct(float distanceCm) {
  // Effective usable range shrinks based on overflow (top) and pump
  // suction (bottom). Both are optional (0 = disabled).
  //   overflowCm: distance FROM SENSOR to overflow pipe. Water at/above
  //               here → capped 100%. If 0, uses full tank height.
  //   suctionCm : distance from tank BOTTOM to pump inlet. Water below
  //               here → 0% (pump can't draw dry). Converted internally
  //               to a distance-from-sensor cutoff = (tankHeight - suction).
  float pipeCm    = (usOverflow > 0 && usOverflow < usTankHeight) ? (float)usOverflow : 0.0f;
  float suctionCutoffCm = (usSuction > 0 && usSuction < (uint16_t)usTankHeight)
                          ? (usTankHeight - (float)usSuction)
                          : usTankHeight;
  if (distanceCm <= pipeCm)          return 100;        // above/at overflow → capped
  if (distanceCm >= suctionCutoffCm) return 0;          // below suction inlet → dead pool
  float usable = suctionCutoffCm - pipeCm;              // effective range
  if (usable <= 0) return 0;                            // sanity
  float waterHeight = suctionCutoffCm - distanceCm;     // measured up from suction cutoff
  if (waterHeight < 0) waterHeight = 0;
  if (waterHeight > usable) waterHeight = usable;
  float exactPct = (waterHeight / usable) * 100.0f;
  int pct = (int) ceilf(exactPct);
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t) pct;
}

// ── Layer 1: MAD-cleaned histogram-mode filter (19.1.0) ────────────
// Reflection noise on HC-SR04 is CLUSTERED: pulse hits a wall/edge
// early and returns as a false short-distance reading; next pulse
// reaches the real target further away. The two paths interleave.
//
// Pure MAD (previous algorithm) AVERAGES these into a wrong middle
// value. Histogram-mode COUNTS which distance appears most often
// and returns that — the consistent target signal wins.
//
// Two-stage pipeline:
//   a) MAD rejects wild outliers (|d - median| > 2.5 × MAD)
//   b) Remaining pings binned into 5 cm buckets
//   c) Find the bucket with most pings (mode)
//   d) If mode has ≥ 60% of the CLEANED pings → return bin center
//   e) Otherwise → return -1 (burst UNRELIABLE)
//
// Returns -1 on failure so caller can fall back to zone memory.
float histogramModeFilter(float* samples, int count) {
  if (count < US_HIST_MIN_VALID) return -1.0f;

  // Stage a — MAD reject wild outliers
  float sorted[US_SAMPLES];
  memcpy(sorted, samples, count * sizeof(float));
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (sorted[j] < sorted[i]) { float t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t; }
    }
  }
  float median = sorted[count / 2];
  float devs[US_SAMPLES];
  for (int i = 0; i < count; i++) devs[i] = fabsf(samples[i] - median);
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (devs[j] < devs[i]) { float t = devs[i]; devs[i] = devs[j]; devs[j] = t; }
    }
  }
  float mad = devs[count / 2];
  float madThr = US_MAD_MULTIPLIER * mad + 0.001f;

  // Stage b — bin cleaned samples into 5 cm buckets. Max distance
  // 450 cm / 5 cm bin = 90 bins.
  const int BIN_COUNT = 92;   // headroom for edge values
  uint8_t bins[BIN_COUNT];
  memset(bins, 0, sizeof(bins));
  int cleanCount = 0;
  for (int i = 0; i < count; i++) {
    if (fabsf(samples[i] - median) > madThr) continue;   // MAD rejected
    int b = (int)(samples[i] / US_HIST_BIN_CM);
    if (b >= 0 && b < BIN_COUNT) { bins[b]++; cleanCount++; }
  }
  if (cleanCount < US_HIST_MIN_VALID) return -1.0f;

  // Stage c — find mode
  int modeBin = 0;
  uint8_t modeCount = 0;
  for (int b = 0; b < BIN_COUNT; b++) {
    if (bins[b] > modeCount) { modeCount = bins[b]; modeBin = b; }
  }

  // Stage d — 60% agreement threshold
  int required = (cleanCount * US_HIST_MODE_MIN_PCT + 99) / 100;   // ceil
  if (modeCount < required) {
    Serial.printf("[US] Layer1 REJECT: mode=%dcm has %u/%u pings (%u%%<60%%)\n",
                  (int)(modeBin * US_HIST_BIN_CM), modeCount, cleanCount,
                  (uint32_t)((modeCount * 100) / cleanCount));
    return -1.0f;
  }

  // Return bin center (bin start + half bin width)
  float result = (modeBin * US_HIST_BIN_CM) + (US_HIST_BIN_CM * 0.5f);
  Serial.printf("[US] Layer1 mode=%.1fcm (%u/%u pings, %u%%)\n",
                result, modeCount, cleanCount,
                (uint32_t)((modeCount * 100) / cleanCount));
  return result;
}

// ── Main processing (Layer 1 + Layer 2, Layer 3 at push site) ──────
void processUltrasonic() {
  if (millis() - lastUsRead < US_READ_INTERVAL) return;
  lastUsRead = millis();

  // Collect burst — count and per-ping gap tuned per sensor type.
  // HC-SR04: 25 pings × (15–30 ms pulseIn + 10 ms) ≈ 625–1000 ms
  // A01:     15 pings × (90 ms reply + 10 ms)      ≈ 1500 ms
  //          (fewer needed — sensor MCU already picked best echo)
  // A21:     drain stream — up to 15 frames in ~150 ms (10 fps stream)
  // RS485:   each ping is a full Modbus request/response (worst case
  //          500ms timeout on a miss per readRs485Cm()). 15 pings × up to
  //          500ms could reach 7.5 sec, uncomfortably close to the 8-sec
  //          WDT budget mentioned above — use fewer pings than DYP.
  float samples[US_SAMPLES];
  int validCount = 0;
  int burstN = (usSensorType == US_TYPE_HCSR04) ? US_SAMPLES
             : (usSensorType == US_TYPE_RS485)  ? 8
             : 15;
  int gapMs  = (usSensorType == US_TYPE_DYP_A21) ? 0 : 10;
  for (int i = 0; i < burstN; i++) {
    float d = readUltrasonicRaw();
    if (d > 0) samples[validCount++] = d;
    if (gapMs) delay(gapMs);
    // 21.1.5: feed WDT every 3 pings so a hung sensor UART can never
    // block us long enough to trigger the 8-sec task watchdog. Even
    // with adaptive-timeout below, this is cheap insurance.
    if ((i % 3) == 2) esp_task_wdt_reset();
  }

  // 21.1.5: sensor-missing detection.
  // Zero valid pings in a burst → increment consecutive-fail counter.
  // Threshold (15 bursts × ~2 sec = ~30 sec) → flip usSensorOffline flag.
  // Any burst with >=1 valid ping → reset both counters.
  if (validCount == 0) {
    if (usConsecFailBursts < 0xFFFF) usConsecFailBursts++;
    if (!usSensorOffline && usConsecFailBursts >= US_MISSING_BURST_THRESHOLD) {
      usSensorOffline = true;
      Serial.printf("[US] SENSOR MISSING — %u consecutive failed bursts. "
                    "Adaptive timeout active. Check wiring.\n",
                    usConsecFailBursts);
    }
  } else {
    if (usSensorOffline) {
      Serial.printf("[US] Sensor RECOVERED after %u failed bursts — normal timeouts restored.\n",
                    usConsecFailBursts);
    }
    usConsecFailBursts = 0;
    usSensorOffline    = false;
  }

  // Layer 1: histogram-mode filter (rejects clustered reflections).
  // Returns -1 if burst can't establish a >60% mode → treat as invalid.
  float filtered = (validCount >= US_HIST_MIN_VALID)
                   ? histogramModeFilter(samples, validCount)
                   : -1.0f;

  // v21.1.3: compute `candidateFromBurst` from EITHER the good-burst
  // path OR the fallback path. Both paths converge into Layer 4 below.
  // No path writes confirmedPct/instantPct outside the Layer 4 gate.
  uint8_t candidateFromBurst;

  if (filtered <= 0) {
    // Bad burst — derive candidate from zone memory
    usFailCount++;
    switch (usLastZone) {
      case US_ZONE_UPPER:  candidateFromBurst = 100; break;
      case US_ZONE_LOWER:  candidateFromBurst = 0;   break;
      case US_ZONE_MIDDLE: candidateFromBurst = usLastValidPct; break;
      default:             candidateFromBurst = usLastValidPct; break;
    }
    Serial.printf("[US] Burst FAIL — fallback candidate=%u%% (from zone %u)\n",
                  candidateFromBurst, usLastZone);
    usRawDistance = -1.0f;
    flags       &= ~0x20;
    // usSensorOffline managed by burst-collection block above (21.1.5).
  } else {
    // Good burst — full Layer 1+2 pipeline
    usFailCount = 0;
    flags &= ~0x20;
    usRawDistance = filtered;

    uint8_t zone = classifyZone(filtered);
    float accepted;
    if (zone == US_ZONE_MIDDLE) {
      accepted = applyThreeStrikeHysteresis(filtered);
    } else if (zone == US_ZONE_UPPER || zone == US_ZONE_LOWER) {
      accepted = filtered;
      resetHysteresisState();
    } else {
      accepted = applyThreeStrikeHysteresis(filtered);
      zone = US_ZONE_MIDDLE;
    }
    usFilteredDistance = accepted;

    uint8_t newPct = distanceToIntegerPct(accepted);
    usLastValidPct = newPct;
    usHaveValidHistory = true;
    if (zone != usLastZone) {
      usLastZone = zone;
      saveZoneMemory(zone, newPct);
      Serial.printf("[US] Zone: %u pct=%u (distance=%.1f cm)\n",
                    (uint8_t) zone, newPct, filtered);
    } else {
      saveZoneMemory(zone, newPct);
    }
    candidateFromBurst = newPct;
  }
  // ── Layer 4: Jump-limit filter (v21.1.3) ────────────────────────────
  // v21.1.3: rewritten so BOTH good-burst and fallback paths flow
  // through here. `candidateFromBurst` is what the sensor "wants" to
  // publish this cycle. Layer 4 either accepts (commits + publishes)
  // or rejects (holds last committed value). NOTHING publishes to
  // confirmedPct outside this gate.
  //
  // Big jump (>= US_JUMP_LIMIT_PCT from last committed) — either up or
  // down, including 0% dropouts — must persist for US_JUMP_SUSTAIN_STRIKES
  // consecutive bursts before commit. Field CSV showed 30-40 sec spikes
  // slipping past v21.1.2's 60-sec sustain via the fallback bypass;
  // v21.1.3 closes both loopholes.
  uint8_t publishPct = candidateFromBurst;
  if (usHaveCommittedPct) {
    int16_t delta = (int16_t)candidateFromBurst - (int16_t)usLastCommittedPct;
    if (delta < 0) delta = -delta;
    if (delta >= US_JUMP_LIMIT_PCT) {
      int16_t candDelta = (int16_t)candidateFromBurst - (int16_t)usJumpCandidatePct;
      if (candDelta < 0) candDelta = -candDelta;
      if (usJumpStrikes == 0 || candDelta > (US_JUMP_LIMIT_PCT / 2)) {
        // New candidate (first hit, or previous candidate broken)
        usJumpCandidatePct = candidateFromBurst;
        usJumpStrikes      = 1;
      } else {
        usJumpStrikes++;
      }
      if (usJumpStrikes < US_JUMP_SUSTAIN_STRIKES) {
        usSpikesRejected++;
        Serial.printf("[US] Layer4 REJECT jump %u%%->%u%% (strike %u/%u, spikes=%u)\n",
                      usLastCommittedPct, candidateFromBurst, usJumpStrikes,
                      US_JUMP_SUSTAIN_STRIKES, usSpikesRejected);
        publishPct = usLastCommittedPct;
      } else {
        Serial.printf("[US] Layer4 ACCEPT sustained jump %u%%->%u%% after %u strikes\n",
                      usLastCommittedPct, candidateFromBurst, usJumpStrikes);
        usJumpStrikes = 0;
      }
    } else {
      // Small change — normal path, reset strike counter
      usJumpStrikes = 0;
    }
  }
  usLastCommittedPct = publishPct;
  usHaveCommittedPct = true;

  // v21.1.2: BOOT WARMUP GATE.
  // For the first US_BOOT_WARMUP_MS after boot, do NOT publish to
  // confirmedPct/instantPct. Instead collect each burst's pct into a
  // ring, and once we have US_BOOT_WARMUP_SAMPLES entries, publish the
  // MEDIAN as the first stable value. This gives the app a rock-solid
  // first reading instead of the settling values dancing 0-30% during
  // the first minute after power-on.
  if (!usBootWarmupDone) {
    if (usBootPctCount < US_BOOT_WARMUP_SAMPLES) {
      usBootPctRing[usBootPctCount++] = publishPct;
      Serial.printf("[US] Warmup %u/%u: pct=%u (holding /live)\n",
                    usBootPctCount, US_BOOT_WARMUP_SAMPLES, publishPct);
      // Update instantPct so the AP page shows current reading — only
      // /live to cloud is held back.
      instantPct  = publishPct;
      instantBits = 0;
      sensorBits  = 0;
      return;
    }
    // Ring full — compute median of the 30 samples
    uint8_t sorted[US_BOOT_WARMUP_SAMPLES];
    memcpy(sorted, usBootPctRing, sizeof(sorted));
    for (int i = 0; i < US_BOOT_WARMUP_SAMPLES - 1; i++) {
      for (int j = i + 1; j < US_BOOT_WARMUP_SAMPLES; j++) {
        if (sorted[j] < sorted[i]) {
          uint8_t t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t;
        }
      }
    }
    uint8_t medianPct = sorted[US_BOOT_WARMUP_SAMPLES / 2];
    Serial.printf("[US] Warmup DONE: median=%u%% of %u samples → first /live\n",
                  medianPct, US_BOOT_WARMUP_SAMPLES);
    usBootWarmupDone = true;
    // Also reset Layer 4 anchor so the median is the new baseline
    usLastCommittedPct = medianPct;
    publishPct = medianPct;
  }

  // Publish to /live via the standard change-detection path
  confirmedPct = publishPct;
  instantPct   = publishPct;
  instantBits  = 0;
  sensorBits   = 0;
}

#endif

// ══════════════════════════════════════════════════
//  FIREBASE
// ══════════════════════════════════════════════════

#if !ENABLE_CLOUD
// Stubs so call sites compile when cloud is disabled. All no-ops.
void  initFirebase() {}
bool  checkFirebaseReady() { return false; }
void  writePendingDevice() {}
bool  pushLiveData() { return false; }
void  updateDeviceInfo(bool) {}
void  checkCommands() {}
void  writeHistory() {}
void  checkConfig() {}
#else

void initFirebase() {
  Serial.println("[FB] initFirebase called");
  // Ensure Google DNS is set before any Firebase connection
  if (WiFi.status() == WL_CONNECTED) setGoogleDNS();
  Serial.println("[FB] DB URL: " + String(FIREBASE_DB_URL));

  fbConfig.api_key = FIREBASE_API_KEY;
  fbConfig.database_url = FIREBASE_DB_URL;
  fbConfig.token_status_callback = tokenStatusCallback;
  // Aggressive timeouts so a dead network doesn't block the main loop.
  // Default is 15000 ms which freezes AP page + mDNS + MQTT.
  fbConfig.timeout.serverResponse  = 2000;
  fbConfig.timeout.socketConnection = 2000;
  fbConfig.timeout.sslHandshake     = 3000;

  Serial.println("[FB] Calling Firebase.begin...");
  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectNetwork(true);
  Serial.println("[FB] Firebase.begin done");

  Serial.println("[FB] Calling signUp for anonymous auth...");
  if (Firebase.signUp(&fbConfig, &fbAuth, "", "")) {
    Serial.println("[FB] Anonymous auth OK!");
  } else {
    Serial.println("[FB] Auth FAILED: " + String(fbConfig.signer.signupError.message.c_str()));
  }

  Serial.println("[FB] Firebase.ready() = " + String(Firebase.ready() ? "true" : "false"));
}

bool checkFirebaseReady() {
  if (Firebase.ready()) {
    if (!firebaseReady) {
      firebaseReady = true;
      firebaseHealthy = true;
      consecutiveFailCount = 0;
      Serial.println("Firebase ready!");
      writePendingDevice();
      updateDeviceInfo(true);
      reportFirmwareInfoIfChanged();   // push firmwareVersion + firstBootAt + lastUpdatedAt (only if changed)
      // No initial pushLiveData() — wait for the 3-confirm gate to produce
      // first stable reading. Avoids polluting /history with boot-default 0%.
      // Boot log entry: NOT uploaded here. checkConfig() materialises it
      // from the in-RAM pendingBoot* values once it reads diagnosticsOn,
      // so free customers' devices stay quiet.
      Serial.println("Waiting for first confirmed sensor read");
    }
    return true;
  }
  return false;
}

// Write to /pendingDevices/{deviceCode} — capability info only
void writePendingDevice() {
  String path = "pendingDevices/" + deviceCode;

  FirebaseJson json;
  json.set("deviceClass", CLS_SENSOR);
  #if USE_ULTRASONIC
    json.set("sensorType", SNS_ULTRASONIC);
  #else
    json.set("sensorType", SNS_DIP);
  #endif
  json.set("sensorCount", SENSOR_COUNT);
  json.set("firmwareVersion", FIRMWARE_VERSION);
  json.set("macAddress", WiFi.macAddress());
  json.set("firstSeenAt/.sv", "timestamp");

  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
    Serial.println("Pending device info written to Firebase");
  } else {
    Serial.println("Failed to write pending device: " + fbdo.errorReason());
  }
}

// Push live sensor data to /devices/{deviceCode}/live/
// Gated by firebaseHealthy — if we know cloud is down, skip silently without
// hitting the 2-3s TLS timeout that would stall everything else.
// Internal: actually push supplied values. Used by both change-driven path
// (currents) and heartbeat (last-confirmed-stable values).
bool pushLiveDataValues(uint8_t bits, uint8_t pct, uint8_t flg) {
  // Layered gate: WiFi → internet → firebase
  if (WiFi.status() != WL_CONNECTED) return false;
  if (!internetAvailable)             return false;
  if (!firebaseHealthy)               return false;

  String path = "devices/" + deviceCode + "/live";

  FirebaseJson json;
  json.set("sensorBits", bits);
  json.set("confirmedPct", pct);
  json.set("stateVal", 0);   // Sensor-only, no valve state
  json.set("flags", flg);
  json.set("rssi", WiFi.RSSI());
  // v21.0.5: expose the raw distance (filtered, cm) so the dashboard can
  // show cm alongside %. usFilteredDistance is the post-histogram value
  // (what the pct is derived from). One float per /live push.
  #if USE_ULTRASONIC
    json.set("rawCm", usFilteredDistance);
  #endif
  json.set("timestamp/.sv", "timestamp");

  esp_task_wdt_reset();
  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
    // Persist lastSent* to NVS ONLY when value actually changes vs RAM
    // copy — otherwise heartbeat (288/day) would wear out NVS in ~1.8 yr.
    bool valueChanged = (bits != lastSentBits) ||
                        (pct  != lastSentPct)  ||
                        (flg  != lastSentFlags);
    lastSentBits  = bits;
    lastSentPct   = pct;
    lastSentFlags = flg;
    if (valueChanged) {
      Preferences lp;
      lp.begin("senseflow", false);
      lp.putUChar("lsBits",  bits);
      lp.putUChar("lsPct",   pct);
      lp.putUChar("lsFlags", flg);
      lp.end();
    }
    lastDataPush = millis();
    consecutiveFailCount = 0;
    lastSuccessfulPush = millis();
    return true;
  } else {
    consecutiveFailCount++;
    Serial.printf("Push FAILED (%d): %s\n", consecutiveFailCount, fbdo.errorReason().c_str());
    // After 3 strikes mark cloud unhealthy. LED state machine will show this
    // via the existing 5-second blink slot (red). No solid-red spam.
    if (consecutiveFailCount >= 3) {
      Serial.println("[FB] 3 consecutive fails — marking cloud unhealthy");
      firebaseHealthy = false;
      consecutiveFailCount = 0;
    }
    return false;
  }
}

// Change-driven push — uses current (possibly mid-glitch) values; gated by
// hasDataChanged() at the call site so only confirmed-stable values reach here.
bool pushLiveData() {
  return pushLiveDataValues(sensorBits, confirmedPct, flags);
}

// Premium-tier mirror — writes a tiny snapshot to /notify_trigger so the
// dispatcher Cloud Function fires. Only called when notifyOn=true, only
// on change-driven pushes (NOT heartbeat — heartbeat would spam the
// dispatcher with no real event behind it). Minimal payload: just pct,
// flags, ts. The Cloud Function reads /live for full context if it needs
// more. Path mirrors trigger pattern from project memory.
void pushNotifyTrigger(uint8_t pct, uint8_t flg) {
  if (!notifyOn)        return;
  if (!firebaseHealthy) return;
  String path = "devices/" + deviceCode + "/notify_trigger";
  FirebaseJson json;
  json.set("pct",       pct);
  json.set("flags",     flg);
  json.set("ts/.sv",    "timestamp");
  esp_task_wdt_reset();
  Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json);
  // No retry on fail — the Cloud Function dispatcher is best-effort
  // anyway; missing a single ping for a free-paying customer is fine,
  // the next change will fire it.
}

// Heartbeat push — always sends last-confirmed-stable value (or current
// if no confirmed value yet). Cloud sees device online without absorbing glitch.
bool pushLiveDataHeartbeat() {
  // v21.1.4: block heartbeat push during boot warmup — we don't have a
  // real value yet, don't send stale/default 0% to cloud.
  #if USE_ULTRASONIC
    if (!usBootWarmupDone) return false;
  #endif
  if (haveConfirmedValue) {
    return pushLiveDataValues(lastConfirmedBits, lastConfirmedPct, lastConfirmedFlags);
  }
  return pushLiveDataValues(sensorBits, confirmedPct, flags);
}

// Update device info node
void updateDeviceInfo(bool online) {
  if (!firebaseHealthy) return;
  String path = "devices/" + deviceCode + "/info";

  FirebaseJson json;
  json.set("online", online);
  json.set("lastSeen/.sv", "timestamp");
  json.set("firmwareVersion", FIRMWARE_VERSION);
  json.set("deviceClass", CLS_SENSOR);
  #if USE_ULTRASONIC
    json.set("sensorType", SNS_ULTRASONIC);
  #else
    json.set("sensorType", SNS_DIP);
  #endif
  json.set("sensorCount", SENSOR_COUNT);
  // Mirror the physical-identity label so admin/subscribers see it
  // alongside firmware version + WiFi status. NVS is the source of
  // truth — this write just keeps the /info mirror in sync.
  json.set("userAssignedName", userAssignedName);

  // Ultrasonic geometry mirror — lets the dashboard show water depth
  // in ft/in alongside the percentage. NVS remains authoritative on
  // the firmware side (installer edits via AP page → NVS); this
  // pushes the current NVS values so the browser can compute display
  // values without needing to know the tank shape itself. Written
  // every /info update (cheap — no separate poll). Only meaningful
  // for ultrasonic devices — DIP has no configurable geometry.
  #if USE_ULTRASONIC
    json.set("tankHeightCm", (int)usTankHeight);
    json.set("overflowCm",   (int)usOverflow);
    json.set("suctionCm",    (int)usSuction);
  #endif

  // updateNode merges into /info instead of overwriting — preserves
  // firstBootAt, lastUpdatedAt, lastOtaStatus, otaRetryCount that other
  // code paths wrote there.
  esp_task_wdt_reset();
  Firebase.RTDB.updateNode(&fbdo, path.c_str(), &json);
}

// Check commands node
void checkCommands() {
  if (!firebaseHealthy) return;
  String basePath = "devices/" + deviceCode + "/commands/";

  esp_task_wdt_reset();
  if (Firebase.RTDB.getBool(&fbdo, (basePath + "refreshRequested").c_str())) {
    if (fbdo.boolData()) {
      Serial.println("Refresh requested — force pushing data");
      pushLiveData();
      Firebase.RTDB.setBool(&fbdo, (basePath + "refreshRequested").c_str(), false);
    }
  }
  esp_task_wdt_reset();
  if (Firebase.RTDB.getBool(&fbdo, (basePath + "testRequested").c_str())) {
    if (fbdo.boolData()) {
      Serial.println("Test requested — blinking LED");
      testBlinkActive = true;
      testBlinkStart = millis();
      Firebase.RTDB.setBool(&fbdo, (basePath + "testRequested").c_str(), false);
    }
  }
  esp_task_wdt_reset();
  if (Firebase.RTDB.getBool(&fbdo, (basePath + "restartRequested").c_str())) {
    if (fbdo.boolData()) {
      Serial.println("Restart requested — rebooting...");
      Firebase.RTDB.setBool(&fbdo, (basePath + "restartRequested").c_str(), false);
      // Intentionally NOT calling updateDeviceInfo(false) here. Old code
      // pre-emptively marked the device offline in cloud BEFORE rebooting.
      // If WiFi reconnect then failed (e.g. router stuck on old DHCP lease),
      // nobody was around to flip online back to true, so the cloud showed
      // the device permanently offline. Now we let the cloud detect offline
      // through heartbeat staleness (15 min). When the device boots back
      // up and reaches Firebase, IT writes online=true via updateDeviceInfo
      // in checkFirebaseReady() and the cloud picks up immediately.
      delay(500);
      safeRestart();
    }
  }
  // Diagnostics: admin wants a fresh /diagnostics/now snapshot.
  esp_task_wdt_reset();
  if (Firebase.RTDB.getBool(&fbdo, (basePath + "refreshDiagRequested").c_str())) {
    if (fbdo.boolData()) {
      Firebase.RTDB.setBool(&fbdo, (basePath + "refreshDiagRequested").c_str(), false);
      uploadDiagnosticsNow();
      Serial.println("[DIAG] On-demand snapshot uploaded");
    }
  }
  // Diagnostics: admin wants the boot log wiped (both NVS and RTDB).
  esp_task_wdt_reset();
  if (Firebase.RTDB.getBool(&fbdo, (basePath + "clearDiagLogRequested").c_str())) {
    if (fbdo.boolData()) {
      Firebase.RTDB.setBool(&fbdo, (basePath + "clearDiagLogRequested").c_str(), false);
      clearBootLog();
      clearBootLogRtdb();
      Serial.println("[DIAG] Boot log cleared (NVS + RTDB)");
    }
  }

  // Cloud-side geometry update (ultrasonic only — DIP has no
  // geometry). OrgAdmin edits Tank Height / Overflow / Suction on
  // the cloud dashboard and hits Send → cloud writes a JSON at
  //   /devices/<code>/commands/updateGeometry = {tankHeight, overflow, suction}
  // We pick it up on the next commands poll (up to 30 sec), validate,
  // write NVS, update in-memory, push /info (so cloud sees the new
  // values reflected — no separate ACK needed), then delete the
  // command node so we don't re-apply on the next poll.
  //
  // NO_QUEUE_ON_MISUSE: any field out of range → whole update is
  // rejected as a group. Partial application (e.g. tank height ok
  // but suction bad) would leave geometry in an inconsistent state.
  //
  // Idempotent — admin re-sending the same values is safe (NVS write
  // is cheap, /info push is one write, command deletion always runs).
  #if USE_ULTRASONIC
  esp_task_wdt_reset();
  if (Firebase.RTDB.getJSON(&fbdo, (basePath + "updateGeometry").c_str())) {
    FirebaseJson &json = fbdo.jsonObject();
    FirebaseJsonData jd;
    // Missing fields → keep current NVS value (partial updates
    // legitimate — admin might only want to change overflow). Absent
    // fields simply aren't touched.
    float   newTank    = usTankHeight;
    int     newOverflow = usOverflow;
    int     newSuction  = usSuction;
    bool    hasAny = false;
    bool    valid  = true;

    if (json.get(jd, "tankHeight") && jd.typeNum != FirebaseJson::JSON_NULL) {
      float v = jd.floatValue;
      if (v >= 36 && v <= 750) { newTank = v; hasAny = true; }
      else { valid = false; Serial.printf("[GEOM] Rejected tankHeight=%.1f (must be 36-750 cm)\n", v); }
    }
    if (valid && json.get(jd, "overflow") && jd.typeNum != FirebaseJson::JSON_NULL) {
      int v = jd.intValue;
      // Same validation rules as /setoverflow AP-page handler.
      if (v == 0 || (v >= 35 && v < (int)newTank - 10)) {
        newOverflow = v;
        hasAny = true;
      } else {
        valid = false;
        Serial.printf("[GEOM] Rejected overflow=%d (must be 0 or 35..tank-10)\n", v);
      }
    }
    if (valid && json.get(jd, "suction") && jd.typeNum != FirebaseJson::JSON_NULL) {
      int v = jd.intValue;
      // Same validation as /setsuction AP-page handler.
      if (v == 0 || (v > 0 && v < (int)newTank - 35)) {
        newSuction = v;
        hasAny = true;
      } else {
        valid = false;
        Serial.printf("[GEOM] Rejected suction=%d (must be 0 or 1..tank-35)\n", v);
      }
    }

    if (valid && hasAny) {
      usTankHeight = newTank;
      usOverflow   = (uint16_t)newOverflow;
      usSuction    = (uint16_t)newSuction;
      Preferences p;
      p.begin("senseflow", false);
      p.putFloat("tankh",  usTankHeight);
      p.putUShort("usOvfl", usOverflow);
      p.putUShort("usSuct", usSuction);
      p.end();
      Serial.printf("[GEOM] Cloud-applied: tank=%.0f overflow=%u suction=%u\n",
                    usTankHeight, usOverflow, usSuction);
      // Mirror new geometry to /info. Cloud dashboard's live listener
      // sees the change and its Tank Geometry rows auto-refresh — this
      // is the implicit ACK, no separate confirmation channel needed.
      updateDeviceInfo(true);
    }

    // Always delete the command node so it doesn't re-apply next poll.
    // Applies whether the values were valid, invalid, or empty — the
    // command has been "handled" either way.
    Firebase.RTDB.deleteNode(&fbdo, (basePath + "updateGeometry").c_str());
  }
  #endif
}

// Internal: write a history entry with explicit values + tag.
void writeHistoryValues(uint8_t bits, uint8_t pct, uint8_t flg, const char* tag) {
  if (!firebaseHealthy) return;

  // Dedupe guard — if this exact value is already the most recent /history
  // entry, skip the write entirely. The change-driven path and idle-1h
  // fallback both come through here, so this is the single funnel that
  // guarantees /history can never have two consecutive identical rows.
  // 0xFF sentinel = nothing written yet → first push always goes through.
  if (lastHistoryPct   != 0xFF &&
      bits == lastHistoryBits &&
      pct  == lastHistoryPct  &&
      flg  == lastHistoryFlags) {
    Serial.printf("[HISTORY] Skipped duplicate (%s, pct=%u)\n", tag, pct);
    // Update lastHistoryWriteAt anyway so the idle-1h fallback restarts its
    // hour clock from this skipped attempt — keeps the fallback rhythm
    // honest instead of trying again on the next loop iteration.
    lastHistoryWriteAt = millis();
    return;
  }

  FirebaseJson json;
  json.set("pct", pct);
  json.set("bits", bits);
  json.set("flags", flg);
  json.set("ts/.sv", "timestamp");
  esp_task_wdt_reset();
  if (Firebase.RTDB.pushJSON(&fbdo, ("devices/" + deviceCode + "/history").c_str(), &json)) {
    lastHistoryWriteAt = millis();
    lastHistoryBits  = bits;
    lastHistoryPct   = pct;
    lastHistoryFlags = flg;
    Preferences hp;
    hp.begin("senseflow", false);
    hp.putUChar("lhBits",  bits);
    hp.putUChar("lhPct",   pct);
    hp.putUChar("lhFlags", flg);
    hp.end();
    Serial.printf("[HISTORY] Entry recorded (%s)\n", tag);
  }
}

// Spacing for IDLE-only history writes (heartbeat fallback so the chart
// isn't blank when level is steady). Change-driven writes ignore this —
// real level changes go to history immediately, otherwise the chart would
// show the wrong value for up to an hour after a fill/drain event.
#define HISTORY_MIN_INTERVAL (60UL * 60UL * 1000UL)   // 1 hour

// Change-driven write — fires on every confirmed level change. NO time
// gate: history must reflect what live just published, or the chart lags.
//
// Ultrasonic build (19.0.1+): additional 3% integer-pct gate so the
// history chart stays clean. /live is unaffected (still every confirmed
// change), but /history only writes when the reported pct moved ≥3%
// since the last history entry. DIP mode has discrete 25% steps so
// this gate never blocks anything for DIP.
void writeHistory() {
  if (!analyticsOn) return;
#if USE_ULTRASONIC
  // Ultrasonic /history gate (Layer 3):
  //   1. ≥ US_HYST_HISTORY_PCT (5%) pct delta since last history push
  //   2. ≥ US_HISTORY_PUSH_MIN_GAP_MS (20 sec) since last history push
  // First push (usLastHistoryPct == 0xFF) always writes.
  if (usLastHistoryPct != 0xFF) {
    int delta = (int) confirmedPct - (int) usLastHistoryPct;
    if (delta < 0) delta = -delta;
    if (delta < US_HYST_HISTORY_PCT) {
      Serial.printf("[HISTORY] Skipped: delta=%d%% < %d%% threshold\n",
                    delta, US_HYST_HISTORY_PCT);
      return;
    }
    unsigned long now = millis();
    if (usLastHistoryPushMs != 0 &&
        now - usLastHistoryPushMs < US_HISTORY_PUSH_MIN_GAP_MS) {
      Serial.printf("[HISTORY] Skipped: %lu ms since last (need %u)\n",
                    now - usLastHistoryPushMs,
                    (unsigned)US_HISTORY_PUSH_MIN_GAP_MS);
      return;
    }
  }
  usLastHistoryPct   = confirmedPct;
  usLastHistoryPushMs = millis();
#endif
  writeHistoryValues(sensorBits, confirmedPct, flags, "change");
}

// Idle fallback — if no history written for 1 hour, push last-confirmed
// values so chart isn't blank. Never the boot-default 0%.
void writeHistoryIdleIfDue() {
  if (!analyticsOn) return;
  if (!haveConfirmedValue) return;
  if (lastHistoryWriteAt != 0 && millis() - lastHistoryWriteAt < HISTORY_MIN_INTERVAL) return;
  writeHistoryValues(lastConfirmedBits, lastConfirmedPct, lastConfirmedFlags, "idle-1h");
}

// Check config — read analyticsOn, notifyOn, diagnosticsOn flags.
// Polled every 30 sec from the main loop. Off → on transitions trigger
// one-shot side effects (boot log upload on diagnosticsOn flip).
void checkConfig() {
  if (!firebaseHealthy) return;
  String base = "devices/" + deviceCode + "/config/";

  esp_task_wdt_reset();
  if (Firebase.RTDB.getBool(&fbdo, (base + "analyticsOn").c_str())) {
    bool newVal = fbdo.boolData();
    if (newVal != analyticsOn) {
      analyticsOn = newVal;
      Serial.printf("[CONFIG] analyticsOn = %s\n", analyticsOn ? "ON" : "OFF");
    }
  }

  esp_task_wdt_reset();
  if (Firebase.RTDB.getBool(&fbdo, (base + "notifyOn").c_str())) {
    bool newVal = fbdo.boolData();
    if (newVal != notifyOn) {
      notifyOn = newVal;
      Serial.printf("[CONFIG] notifyOn = %s\n", notifyOn ? "ON" : "OFF");
    }
  }

  esp_task_wdt_reset();
  if (Firebase.RTDB.getBool(&fbdo, (base + "diagnosticsOn").c_str())) {
    bool newVal = fbdo.boolData();
    if (newVal != diagnosticsOn) {
      diagnosticsOn = newVal;
      Serial.printf("[CONFIG] diagnosticsOn = %s\n", diagnosticsOn ? "ON" : "OFF");
    }
    // Materialise the pending boot capture if (a) diagnostics is now on,
    // and (b) we haven't already logged this boot. Triggers on first
    // confirmation after boot AND on later off→on flips within the same
    // session. Free-customer device that never flips on → nothing happens.
    if (diagnosticsOn && pendingBootCaptured && !diagnosticsLoggedThisBoot) {
      appendBootLogEntry(pendingBootReason, pendingBootPrevUptime);
      // Reset NVS uptime to 0 so the next persistCurrentUptime() records
      // THIS session's uptime, not the previous one.
      persistCurrentUptime();
      uploadLatestBootLog();
      diagnosticsLoggedThisBoot = true;
      pendingBootCaptured       = false;
      diagnosticsLoggingActive  = true;
    } else if (diagnosticsOn && !diagnosticsLoggingActive) {
      // Already logged this boot but admin re-enabled later — restart the
      // periodic uptime save loop so we still catch the next crash.
      diagnosticsLoggingActive = true;
    } else if (!diagnosticsOn && diagnosticsLoggingActive) {
      // Admin turned it off — stop persistent uptime saves. Existing log
      // entries stay in NVS + RTDB until cleared.
      diagnosticsLoggingActive = false;
    }
  }
}

#endif  // ENABLE_CLOUD

// ══════════════════════════════════════════════════
//  CHANGE DETECTION — only push when values change
// ══════════════════════════════════════════════════

// Cloud-push change detection.
// processDipSensors() now owns the stability logic via a 60-sec dominant-
// level rolling window — anything reaching (sensorBits, confirmedPct, flags)
// is ALREADY vetted for stability. This function is just a "changed since
// last push?" check that also stamps last-confirmed for heartbeat use.
bool hasDataChanged() {
  // v21.1.4: BOOT WARMUP GATE — during the first ~60 sec after boot,
  // confirmedPct hasn't been set by the sensor yet (it's the default 0
  // OR whatever value was last committed before reboot). If we return
  // true here, the main loop will publish that stale/default value to
  // /live immediately after Firebase connects. Field CSV showed this
  // as a 0% spike at boot (v21.1.3 flash on 2026-08-11 11:06:15).
  //
  // Fix: while warmup gate is still holding, refuse to acknowledge any
  // change. haveConfirmedValue also stays false so heartbeat has
  // nothing to send. Once warmup completes and Layer 4 has a real
  // committed value, this returns to normal change-detection.
  #if USE_ULTRASONIC
    if (!usBootWarmupDone) return false;
  #endif

  // Always mark the current committed reading as the "last known good"
  // that heartbeat pushes should send.
  lastConfirmedBits  = sensorBits;
  lastConfirmedPct   = confirmedPct;
  lastConfirmedFlags = flags;
  haveConfirmedValue = true;

  return (sensorBits    != lastSentBits) ||
         (confirmedPct  != lastSentPct)  ||
         (flags         != lastSentFlags);
}

// ══════════════════════════════════════════════════
//  LED STATE MACHINE
// ══════════════════════════════════════════════════

void handleLED() {
  unsigned long now = millis();

  // pushFailFlash deliberately removed — was causing constant-red misleading
  // signal when internet drops. Cloud-fail state now shown via the 5s blink
  // slot (red blink) so user sees it as one of the proper status colors.

  // Priority 1: Test blink (Firebase command)
  if (testBlinkActive) {
    unsigned long elapsed = now - testBlinkStart;
    if (elapsed < 1800) {
      int phase = (elapsed / 200) % 3;
      if (phase == 0) setLED(255, 0, 0);
      else if (phase == 1) setLED(0, 255, 0);
      else setLED(0, 0, 255);
    } else { testBlinkActive = false; }
    return;
  }

  // LED cycle: 30s level color → 5s system status blink → repeat
  unsigned long cycleElapsed = now - ledCycleStart;
  if (cycleElapsed >= 35000) { ledCycleStart = now; ledShowingWifi = false; }

  if (cycleElapsed >= 30000) {
    // Stage 2: System status blink (5s) — HIGHEST PRIORITY, overrides sensor error
    if (!ledShowingWifi) { ledShowingWifi = true; wifiBlinkStart = now; }
    int blinkPhase = ((now - wifiBlinkStart) / 250) % 2;
    bool wifiUp = (WiFi.status() == WL_CONNECTED);
    if (!wifiUp) {
      // White blink = WiFi disconnected
      if (blinkPhase == 0) setLED(255, 255, 255); else setLEDOff();
    } else if (!internetAvailable) {
      // Pink blink = WiFi OK, no internet (DNS chain failed)
      if (blinkPhase == 0) setLED(255, 0, 100); else setLEDOff();
    } else if (ENABLE_CLOUD && !firebaseHealthy) {
      // Red blink = internet OK but cloud unreachable (Firebase failing)
      if (blinkPhase == 0) setLED(255, 0, 0); else setLEDOff();
    } else {
      // Blue blink = all good
      if (blinkPhase == 0) setLED(0, 0, 255); else setLEDOff();
    }
  } else {
    // Stage 1: Tank level color (30s)
    ledShowingWifi = false;
#if USE_ULTRASONIC
    // Ultrasonic 19.0.1+: NO purple sensor-error state. Zone-fallback
    // ensures instantPct always holds a sensible value (100/0/lastGood
    // even during a bad ping burst) so the level color band is what
    // the LED shows, always.
    setLevelColor(instantPct);
#else
    // DIP mode: purple on non-consecutive probe fault (physics error).
    if (sensorError) {
      setLED(148, 51, 234);  // Purple - sensor error (DIP only)
    } else {
      setLevelColor(instantPct);
    }
#endif
  }
}

// ══════════════════════════════════════════════════
//  MVSCONNECT CUSTOM PAGE (AP MODE)
// ══════════════════════════════════════════════════

// Auto-format litres: <1000 = "L", 1000-999,999 = "KL" (1 decimal),
// >=1,000,000 = "ML" (2 decimal). Returns e.g. "12.0 KL" or "750 L".
String fmtLitres(uint32_t v) {
  if (v >= 1000000UL) {
    return String(v / 1000000.0, 2) + " ML";
  }
  if (v >= 1000UL) {
    return String(v / 1000.0, 1) + " KL";
  }
  return String(v) + " L";
}

String buildCustomHTML() {
  String html = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>SenseFlow Device</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:system-ui;background:#f5f5f5;color:#333;padding:16px}
.card{background:#fff;border-radius:12px;padding:16px;margin-bottom:12px;box-shadow:0 1px 3px rgba(0,0,0,0.1)}
h1{font-size:20px;color:#2563eb;margin-bottom:4px}
h2{font-size:14px;font-weight:600;color:#666;margin-bottom:8px}
.code{font-family:monospace;font-size:18px;font-weight:bold;color:#111;letter-spacing:1px}
.row{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #f0f0f0;font-size:13px}
.row:last-child{border:none}
.label{color:#888}
.val{font-weight:600;color:#333}
.qr{text-align:center;padding:16px}
.qr img{border:8px solid #fff;box-shadow:0 2px 8px rgba(0,0,0,0.15);border-radius:4px}
.link{word-break:break-all;font-size:11px;color:#2563eb;margin-top:8px;display:block}
.dip-row{display:flex;gap:6px;margin:8px 0}
.dip-dot{width:28px;height:28px;border-radius:50%;border:2px solid #ccc;display:flex;align-items:center;justify-content:center;font-size:10px;font-weight:bold}
.dip-on{background:#3b82f6;border-color:#2563eb;color:#fff}
.dip-off{background:#e5e7eb;border-color:#d1d5db;color:#999}
.btn{display:block;width:100%;padding:12px;border:none;border-radius:8px;font-size:14px;font-weight:600;cursor:pointer;margin-top:8px}
.btn-blue{background:#2563eb;color:#fff}
.btn-red{background:#ef4444;color:#fff}
.btn-gray{background:#e5e7eb;color:#333}
/* Tank visualization */
.tank-wrap{display:flex;gap:24px;align-items:stretch}
/* Tapered tank silhouette (top is narrower, like a real water tank with a lid) */
.tank{position:relative;width:120px;height:240px;flex-shrink:0;overflow:visible}
.tank-svg{width:100%;height:100%;display:block;overflow:visible}
.tank-water{transition:height .6s ease-out}
.tank-info{flex:1;min-width:0;display:flex;flex-direction:column;justify-content:center;gap:6px;overflow:hidden}
.big-pct{font-size:40px;font-weight:800;color:#2563eb;line-height:1;word-break:break-word}
.litres{font-size:15px;color:#475569;font-weight:600;word-break:break-word}
.tank-sub{font-size:11px;color:#64748b;word-break:break-word}
.err-banner{background:#faf5ff;border:1px solid #d8b4fe;color:#7e22ce;padding:6px 10px;border-radius:8px;font-size:11px;font-weight:600;margin-top:6px}
@media (max-width:380px){.big-pct{font-size:32px}.litres{font-size:13px}}
</style>
</head><body>
)rawliteral";

  // WiFi status banner
  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  if (wifiOk) {
    html += "<div style='background:#f0fdf4;border:1px solid #86efac;border-radius:10px;padding:10px 14px;margin-bottom:12px;display:flex;align-items:center;gap:8px'>";
    html += "<div style='width:10px;height:10px;border-radius:50%;background:#22c55e;box-shadow:0 0 6px #22c55e'></div>";
    html += "<div><div style='font-size:12px;font-weight:700;color:#166534'>WiFi Connected</div>";
    html += "<div style='font-size:10px;color:#15803d'>" + WiFi.SSID() + " &bull; " + WiFi.localIP().toString() + " &bull; RSSI " + String(WiFi.RSSI()) + "dBm</div></div></div>";
  } else {
    html += "<div style='background:#fef2f2;border:1px solid #fca5a5;border-radius:10px;padding:10px 14px;margin-bottom:12px;display:flex;align-items:center;gap:8px'>";
    html += "<div style='width:10px;height:10px;border-radius:50%;background:#ef4444;box-shadow:0 0 6px #ef4444'></div>";
    html += "<div><div style='font-size:12px;font-weight:700;color:#991b1b'>WiFi Not Connected</div>";
    html += "<div style='font-size:10px;color:#dc2626'>Enter credentials below or use MvsConnect app</div></div></div>";
  }

  // Tank visualization card (DIP only — ultrasonic has its own raw-distance card lower down)
  #if !USE_ULTRASONIC
  {
    html += "<div class='card'>";
    html += "<h1>SenseFlow Device</h1>";
    if (userAssignedName.length() > 0) {
      html += "<p style='font-size:15px;font-weight:600;color:#334155;margin-bottom:2px'>" + userAssignedName + "</p>";
    }
    html += "<p class='code'>" + deviceCode + "</p>";
    // User-assigned physical-identity label. Installer edits inline; save
    // is a plain GET to /setusername that redirects back to /.
    html += "<form action='/setusername' method='GET' style='margin-top:8px;display:flex;gap:6px;align-items:center'>";
    html += "<span style='font-size:11px;color:#666;white-space:nowrap'>Name:</span>";
    html += "<input type='text' name='n' maxlength='32' value='" + userAssignedName + "' placeholder='e.g. 3F West Flush' style='flex:1;padding:6px 8px;border:1px solid #ddd;border-radius:6px;font-size:12px'>";
    html += "<button class='btn btn-blue' type='submit' style='margin:0;padding:6px 12px;font-size:12px'>Save</button>";
    html += "</form>";

    html += "<div class='tank-wrap' style='margin-top:14px'>";
    // Tank graphic — SVG with tapered shoulders + lid, water clipped inside body
    // viewBox 0 0 150 260. Body inner area: x 15-135, y 50-240 (190 tall)
    static const uint8_t PCT_1[] = {100};
    static const uint8_t PCT_2[] = {50, 100};
    static const uint8_t PCT_3[] = {33, 67, 100};
    static const uint8_t PCT_4[] = {25, 50, 75, 100};
    static const uint8_t PCT_5[] = {20, 40, 60, 80, 100};
    static const uint8_t PCT_6[] = {17, 33, 50, 67, 83, 100};
    const uint8_t* PCT = NULL;
    switch (SENSOR_COUNT) {
      case 1: PCT = PCT_1; break; case 2: PCT = PCT_2; break;
      case 3: PCT = PCT_3; break; case 4: PCT = PCT_4; break;
      case 5: PCT = PCT_5; break; case 6: PCT = PCT_6; break;
    }
    int innerTop = 50, innerBottom = 240;       // y range of water fill inside SVG
    int innerHeight = innerBottom - innerTop;   // 190
    // Initial render uses INSTANT reading (matches /sstatus poll updates).
    int waterYStart = innerBottom - (innerHeight * instantPct / 100);
    int waterH = innerBottom - waterYStart;

    html += "<div class='tank' id='tank'>";
    html += "<svg class='tank-svg' viewBox='0 0 150 260' preserveAspectRatio='xMidYMid meet'>";
    // Clip path matching inner body shape (tapered shoulders, rounded bottom)
    html += "<defs><clipPath id='bodyClip'>";
    html += "<path d='M 15 60 Q 15 50 25 50 L 125 50 Q 135 50 135 60 L 135 220 Q 135 240 115 240 L 35 240 Q 15 240 15 220 Z'/>";
    html += "</clipPath>";
    html += "<linearGradient id='waterGrad' x1='0' y1='0' x2='0' y2='1'>";
    html += "<stop offset='0%' stop-color='#60a5fa'/><stop offset='100%' stop-color='#2563eb'/>";
    html += "</linearGradient></defs>";
    // Tank body outline
    html += "<path d='M 15 60 Q 15 50 25 50 L 125 50 Q 135 50 135 60 L 135 220 Q 135 240 115 240 L 35 240 Q 15 240 15 220 Z' fill='#f8fafc' stroke='#475569' stroke-width='3'/>";
    // Lid (sits on top, slightly wider)
    html += "<rect x='10' y='30' width='130' height='14' rx='4' fill='#475569'/>";
    html += "<rect x='35' y='20' width='80' height='12' rx='3' fill='#64748b'/>";
    // Water (clipped to body shape)
    html += "<g clip-path='url(#bodyClip)'>";
    html += "<rect class='tank-water' id='tankWater' x='0' y='" + String(waterYStart) + "' width='150' height='" + String(waterH) + "' fill='url(#waterGrad)'/>";
    // Water surface highlight
    html += "<ellipse id='tankSurface' cx='75' cy='" + String(waterYStart) + "' rx='65' ry='3' fill='#93c5fd' opacity='0.7'/>";
    html += "</g>";
    // Probe markers (drawn outside clip so they show even above water)
    for (int i = 0; i < SENSOR_COUNT; i++) {
      bool on = (instantBits >> i) & 1;
      int pct = PCT ? PCT[i] : 0;
      int probeY = innerBottom - (innerHeight * pct / 100);
      String color = on ? "#2563eb" : "#cbd5e1";
      String tabFill = on ? "#2563eb" : "#e5e7eb";
      String tabText = on ? "#ffffff" : "#64748b";
      // Dashed line across tank
      html += "<line class='probe' data-bit='" + String(i) + "' x1='15' y1='" + String(probeY) + "' x2='135' y2='" + String(probeY) + "' stroke='" + color + "' stroke-width='1.5' stroke-dasharray='3,2'/>";
      // % tab on the right outside the tank
      html += "<rect class='probe-tab-bg' data-bit='" + String(i) + "' x='138' y='" + String(probeY - 7) + "' width='28' height='14' rx='3' fill='" + tabFill + "'/>";
      html += "<text class='probe-tab-tx' data-bit='" + String(i) + "' x='152' y='" + String(probeY + 4) + "' text-anchor='middle' font-size='10' font-weight='700' fill='" + tabText + "'>" + String(pct) + "%</text>";
    }
    html += "</svg>";
    html += "</div>";  // .tank

    // Right-side readout
    html += "<div class='tank-info'>";
    html += "<div class='big-pct' id='bigPct'>" + String(instantPct) + "%</div>";
    if (tankCapacityLitres > 0) {
      uint32_t litres = (uint32_t)(((uint64_t)instantPct * tankCapacityLitres) / 100ULL);
      html += "<div class='litres' id='litres'>" + fmtLitres(litres) + " / " + fmtLitres(tankCapacityLitres) + "</div>";
    } else {
      html += "<div class='litres' id='litres' style='display:none'></div>";
    }
    if (sensorError) {
      html += "<div class='err-banner' id='errBanner'>Sensor pattern fault &mdash; check wiring</div>";
    } else {
      html += "<div class='err-banner' id='errBanner' style='display:none'></div>";
    }
    // 21.1.5: ultrasonic sensor-missing banner. Shown when >=15 consecutive
    // bursts returned no valid pings (sensor unplugged / wiring broken).
    #if USE_ULTRASONIC
      bool showMissing = usSensorOffline;
      html += "<div class='err-banner' id='sensorMissingBanner' style='";
      html += showMissing ? "" : "display:none;";
      html += "background:#dc2626;color:#fff;padding:10px 12px;border-radius:6px;margin-top:8px;font-weight:600;font-size:13px;'>";
      html += "&#9888; No sensor detected &mdash; check wiring: Red&rarr;5V, Black&rarr;GND, Yellow&rarr;GPIO32, White&rarr;GPIO34";
      html += "</div>";
    #endif
    html += "</div>";  // .tank-info
    html += "</div>";  // .tank-wrap

    // Tank capacity config
    html += "<form action='/setcapacity' method='GET' style='margin-top:14px;display:flex;gap:6px;align-items:center'>";
    html += "<span style='font-size:12px;color:#666;white-space:nowrap'>Capacity (L):</span>";
    html += "<input type='number' name='c' min='0' max='1000000' value='" + String(tankCapacityLitres) + "' style='flex:1;padding:8px;border:1px solid #ddd;border-radius:6px;font-size:13px'>";
    html += "<button class='btn btn-blue' type='submit' style='margin:0;padding:8px 16px'>Save</button>";
    html += "</form>";
    html += "</div>";  // card
  }
  #else
  // Device Info Card (ultrasonic)
  html += "<div class='card'>";
  html += "<h1>SenseFlow Device</h1>";
  if (userAssignedName.length() > 0) {
    html += "<p style='font-size:15px;font-weight:600;color:#334155;margin-bottom:2px'>" + userAssignedName + "</p>";
  }
  html += "<p class='code'>" + deviceCode + "</p>";
  html += "<form action='/setusername' method='GET' style='margin-top:8px;display:flex;gap:6px;align-items:center'>";
  html += "<span style='font-size:11px;color:#666;white-space:nowrap'>Name:</span>";
  html += "<input type='text' name='n' maxlength='32' value='" + userAssignedName + "' placeholder='e.g. 3F West Flush' style='flex:1;padding:6px 8px;border:1px solid #ddd;border-radius:6px;font-size:12px'>";
  html += "<button class='btn btn-blue' type='submit' style='margin:0;padding:6px 12px;font-size:12px'>Save</button>";
  html += "</form>";
  html += "</div>";
  #endif

  // Device code for admin reference
  html += "<div class='card' style='text-align:center'>";
  html += "<h2>Device Code</h2>";
  html += "<p class='code' style='font-size:20px;margin:10px 0;user-select:all'>" + deviceCode + "</p>";
  html += "<p style='font-size:10px;color:#888'>Register this code in admin panel to generate QR</p>";
  html += "</div>";

  // Device Details Card — collapsed by default, click header to expand
  {
    int rssi = WiFi.RSSI();
    String sig;
    if (WiFi.status() != WL_CONNECTED) sig = "Not connected";
    else if (rssi >= -65) sig = "Excellent";
    else if (rssi >= -70) sig = "Good";
    else if (rssi >= -98) sig = "Fair";
    else                  sig = "Weak";

    String lastUpd;
    if (lastSuccessfulPush == 0) lastUpd = "Never";
    else {
      unsigned long secs = (millis() - lastSuccessfulPush) / 1000;
      if (secs < 60)      lastUpd = String(secs) + "s ago";
      else if (secs < 3600) lastUpd = String(secs / 60) + "m ago";
      else                 lastUpd = String(secs / 3600) + "h ago";
    }

    html += "<div class='card'>";
    html += "<h2 onclick=\"var d=document.getElementById('devDet');d.style.display=d.style.display==='none'?'block':'none';this.querySelector('span').textContent=d.style.display==='none'?'\\u25BC':'\\u25B2'\" style='cursor:pointer;display:flex;justify-content:space-between;align-items:center;margin:0'>Device Details <span style='font-size:11px;color:#888'>&#9660;</span></h2>";
    html += "<div id='devDet' style='display:none;margin-top:10px'>";
    #if USE_ULTRASONIC
      html += "<div class='row'><span class='label'>Sensor</span><span class='val'>Ultrasonic</span></div>";
      html += "<div class='row'><span class='label'>Tank Height</span><span class='val'>" + String(usTankHeight, 0) + " cm</span></div>";
    #else
      html += "<div class='row'><span class='label'>Sensor</span><span class='val'>" + String(SENSOR_COUNT) + "-Probe DIP</span></div>";
    #endif
    html += "<div class='row'><span class='label'>WiFi Signal</span><span class='val'>" + sig + "</span></div>";
    html += "<div class='row'><span class='label'>Cloud Status</span><span class='val' style='color:" + String(firebaseReady ? "#16a34a" : "#dc2626") + "'>" + String(firebaseReady ? "Connected" : "Not connected") + "</span></div>";
    html += "<div class='row'><span class='label'>Last Update</span><span class='val'>" + lastUpd + "</span></div>";
    html += "<div class='row'><span class='label'>Firmware</span><span class='val'>v" + String(FIRMWARE_VERSION) + "</span></div>";
    html += "</div></div>";
  }

  // (DIP live dots card removed — tank visualisation already shows probe state)

  // Tank height config (ultrasonic only)
  #if USE_ULTRASONIC
    html += "<div class='card'>";
    html += "<h2>Tank Settings</h2>";
    html += "<form action='/settank' method='GET' style='display:flex;gap:6px;align-items:center'>";
    html += "<span style='font-size:12px;color:#666;white-space:nowrap'>Height (cm):</span>";
    html += "<input type='number' name='h' min='36' max='750' value='" + String((int)usTankHeight) + "' style='flex:1;padding:8px;border:1px solid #ddd;border-radius:6px;font-size:13px' required>";
    html += "<button class='btn btn-blue' type='submit' style='margin:0;padding:8px 16px'>Save</button>";
    html += "</form>";
    // v21.0.2: overflow pipe (optional). Blank / 0 = disabled → uses full tank height.
    html += "<form action='/setoverflow' method='GET' style='display:flex;gap:6px;align-items:center;margin-top:8px'>";
    html += "<span style='font-size:12px;color:#666;white-space:nowrap'>Overflow (cm):</span>";
    html += "<input type='number' name='o' min='0' max='500' value='" + String(usOverflow) + "' placeholder='0 = disabled' style='flex:1;padding:8px;border:1px solid #ddd;border-radius:6px;font-size:13px'>";
    html += "<button class='btn btn-blue' type='submit' style='margin:0;padding:8px 16px'>Save</button>";
    html += "</form>";
    html += "<div style='font-size:10px;color:#888;margin:4px 0 6px'>Optional. Distance from sensor to overflow pipe. When set, tank shows 100% once water reaches the pipe. <b>Sensor must be mounted ≥35 cm above the overflow pipe</b> (28 cm sensor blind zone + 7 cm margin). Leave 0 to disable.</div>";
    // v21.0.5: pump suction pipe distance FROM TANK BOTTOM (optional).
    // Below this level tank shows 0% because the pump can't draw dry.
    html += "<form action='/setsuction' method='GET' style='display:flex;gap:6px;align-items:center;margin-top:8px'>";
    html += "<span style='font-size:12px;color:#666;white-space:nowrap'>Suction (cm):</span>";
    html += "<input type='number' name='s' min='0' max='500' value='" + String(usSuction) + "' placeholder='0 = disabled' style='flex:1;padding:8px;border:1px solid #ddd;border-radius:6px;font-size:13px'>";
    html += "<button class='btn btn-blue' type='submit' style='margin:0;padding:8px 16px'>Save</button>";
    html += "</form>";
    html += "<div style='font-size:10px;color:#888;margin:4px 0 6px'>Optional. Distance from tank <b>bottom</b> to pump suction inlet. Water below this level counts as 0% (pump can't draw dry). Leave 0 to disable.</div>";
    // Show live effective range so installer sees the effect of their config
    {
      float pipeCm    = (usOverflow > 0 && usOverflow < usTankHeight) ? (float)usOverflow : 0.0f;
      float suctionCm = (usSuction  > 0 && usSuction  < (uint16_t)usTankHeight)
                        ? (usTankHeight - (float)usSuction) : usTankHeight;
      html += "<div style='font-size:11px;color:#334155;background:#f1f5f9;border-radius:6px;padding:6px 8px;margin:6px 0'>";
      html += "Effective range: <b>" + String(pipeCm, 0) + " cm</b> (full) to <b>" + String(suctionCm, 0) + " cm</b> (empty)";
      html += "</div>";
    }
    html += "<div class='row' style='margin-top:6px'><span class='label'>Raw Distance</span><span class='val'>" + String(usRawDistance, 1) + " cm</span></div>";
    // v21.1.5: AP page shows THREE pct values so installer can see the full
    // pipeline in action — no more confusion when the cloud value seems stuck.
    //   Live raw pct : what sensor is reading right now (updates every 2 sec)
    //   Committed   : Layer 4 approved value (only moves after 120s sustain)
    //   Sent to cloud: what /live has pushed to Firebase (frozen during warmup)
    html += "<div class='row'><span class='label'>Live raw %</span><span class='val' style='font-size:16px;color:#2563eb;font-weight:700'>" + String(instantPct) + " %</span></div>";
    html += "<div class='row'><span class='label'>Committed %</span><span class='val'>" + String(usLastCommittedPct) + " %</span></div>";
    html += "<div class='row'><span class='label'>Sent to cloud</span><span class='val'>" + String(confirmedPct) + " %</span></div>";
    // Show effective geometry math so installer can verify pct is right
    html += "<div class='row'><span class='label'>Geometry</span><span class='val' style='font-size:11px'>tank " + String((int)usTankHeight) + "cm, ovfl " + String(usOverflow) + "cm, suct " + String(usSuction) + "cm</span></div>";
    // Zone indicator (19.0.1) — installer commissioning aid.
    const char* zoneName;
    switch (usLastZone) {
      case US_ZONE_UPPER:  zoneName = "UPPER (near blind — tank ~full)"; break;
      case US_ZONE_MIDDLE: zoneName = "MIDDLE (normal range)"; break;
      case US_ZONE_LOWER:  zoneName = "LOWER (near max — tank ~empty)"; break;
      default:             zoneName = "NONE (no valid reading yet)"; break;
    }
    html += "<div class='row'><span class='label'>Zone</span><span class='val'>" + String(zoneName) + "</span></div>";
    html += "<div class='row'><span class='label'>Hysteresis</span><span class='val'>" + String((int)US_HYST_TOLERANCE_CM) + " cm / " + String(US_HYST_LIVE_STRIKES) + " strikes</span></div>";
    html += "<div class='row'><span class='label'>History push</span><span class='val'>&ge; " + String(US_HYST_HISTORY_PCT) + "% change</span></div>";
    // v21.1.1: Layer 4 jump-limit spike counter
    html += "<div class='row'><span class='label'>Jump filter</span><span class='val'>&gt;" + String(US_JUMP_LIMIT_PCT) + "% needs " + String(US_JUMP_SUSTAIN_STRIKES * 2) + "s sustain</span></div>";
    html += "<div class='row'><span class='label'>Spikes rejected</span><span class='val'>" + String(usSpikesRejected) + " since boot</span></div>";
    // v21.1.2: boot warmup status
    if (!usBootWarmupDone) {
      html += "<div class='row'><span class='label'>Warmup</span><span class='val' style='color:#f59e0b'>" + String(usBootPctCount) + "/" + String(US_BOOT_WARMUP_SAMPLES) + " samples (holding /live)</span></div>";
    } else {
      html += "<div class='row'><span class='label'>Warmup</span><span class='val' style='color:#16a34a'>DONE</span></div>";
    }
    // v20.0.0: sensor type + manual override + on-demand refresh.
    const char* usTypeName;
    switch (usSensorType) {
      case US_TYPE_DYP_A01: usTypeName = "DYP-A01 Controlled (A01ANYTB)"; break;
      case US_TYPE_DYP_A21: usTypeName = "DYP-A01 Auto (A01ANYUB)"; break;
      case US_TYPE_RS485:   usTypeName = "RS485 Modbus-RTU"; break;
      default:              usTypeName = "not detected — check wiring"; break;
    }
    html += "<div class='row'><span class='label'>Sensor</span><span class='val'>" + String(usTypeName) + "</span></div>";
    // Force-refresh button — fires immediate 15-ping burst, shows result inline
    html += "<button class='btn btn-blue' onclick=\"fetch('/refreshus').then(r=>r.json()).then(j=>{document.getElementById('rufR').innerText=j.ok?('Reading: '+j.cm+' cm, '+j.pct+'% ('+j.valid+' valid pings)'):('Refresh failed — valid='+j.valid);}).catch(()=>{document.getElementById('rufR').innerText='Request error';})\">Take Reading Now</button>";
    html += "<div id='rufR' style='font-size:12px;color:#334155;margin-top:6px;text-align:center'></div>";
    // Manual sensor-type override (installer). onchange toggles the RS485
    // config block below — same show/hide idiom as the devDet collapsible
    // toggle elsewhere on this page.
    html += "<form action='/setustype' method='GET' style='margin-top:8px;display:flex;gap:6px;align-items:center'>";
    html += "<span style='font-size:11px;color:#666;white-space:nowrap'>Force type:</span>";
    html += "<select name='t' onchange=\"document.getElementById('rs485Cfg').style.display=(this.value=='4')?'flex':'none'\" style='flex:1;padding:6px;border:1px solid #ddd;border-radius:6px;font-size:12px'>";
    html += "<option value='0'>Auto-detect on next boot</option>";
    html += "<option value='2'" + String(usSensorType == US_TYPE_DYP_A01 ? " selected" : "") + ">DYP-A01 Controlled (A01ANYTB)</option>";
    html += "<option value='3'" + String(usSensorType == US_TYPE_DYP_A21 ? " selected" : "") + ">DYP-A01 Auto (A01ANYUB)</option>";
    html += "<option value='4'" + String(usSensorType == US_TYPE_RS485 ? " selected" : "") + ">RS485 Modbus-RTU</option>";
    html += "</select>";
    html += "<button class='btn btn-gray' type='submit' style='margin:0;padding:6px 12px;font-size:12px'>Save</button>";
    html += "</form>";
    // RS485 address config — separate form/endpoint (same one-form-per-
    // endpoint convention as overflow/suction/capacity below), hidden
    // unless RS485 is the currently-selected type. Register is no longer
    // a field here — always real-time (0x0101), not installer-selectable
    // (see usRs485Reg comment: processed register was slower/less
    // reliable under this firmware's burst-polling cadence).
    html += "<form action='/setrs485addr' method='GET' id='rs485Cfg' style='margin-top:8px;display:" +
            String(usSensorType == US_TYPE_RS485 ? "flex" : "none") + ";gap:6px;align-items:center;flex-wrap:wrap'>";
    html += "<span style='font-size:11px;color:#666;white-space:nowrap'>Modbus addr:</span>";
    html += "<input type='number' name='a' min='1' max='247' value='" + String(usRs485Addr) + "' style='width:60px;padding:6px;border:1px solid #ddd;border-radius:6px;font-size:12px' required>";
    html += "<button class='btn btn-gray' type='submit' style='margin:0;padding:6px 12px;font-size:12px'>Save</button>";
    html += "</form>";
    html += "</div>";
  #endif

  // Actions Card
  html += "<div class='card'>";
  html += "<h2>Actions</h2>";

  // AP mode countdown banner (only when 10-min mode + timer running)
  if (apMode == 1 && apTimerStart != 0 && !apTimerEnded && millis() < apTimerDeadline) {
    unsigned long left = (apTimerDeadline - millis()) / 1000;
    html += "<div id='apBanner' style='background:#fef3c7;border:1px solid #fcd34d;color:#92400e;padding:8px 10px;border-radius:8px;font-size:12px;margin-bottom:10px;display:flex;justify-content:space-between;align-items:center;gap:8px'>";
    html += "<span>AP turns off in <b id='apLeftTxt'>" + String(left / 60) + "m " + String(left % 60) + "s</b></span>";
    html += "<a href='/extendap' style='background:#fbbf24;color:#fff;padding:4px 10px;border-radius:6px;text-decoration:none;font-weight:600;font-size:11px'>+5 min</a>";
    html += "</div>";
  }

  html += "<p style='font-size:11px;color:#666;margin-bottom:4px;font-weight:600'>AP WiFi visibility</p>";
  html += "<label style='display:flex;align-items:center;gap:6px;cursor:pointer;padding:4px 0;font-size:13px'>";
  html += "<input type='radio' name='apModeRdo' value='0' " + String(apMode == 0 ? "checked" : "") + " onchange=\"location.href='/setapmode?mode=0'\">";
  html += "<span>Keep AP always on</span></label>";
  html += "<label style='display:flex;align-items:center;gap:6px;cursor:pointer;padding:4px 0;font-size:13px'>";
  html += "<input type='radio' name='apModeRdo' value='1' " + String(apMode == 1 ? "checked" : "") + " onchange=\"location.href='/setapmode?mode=1'\">";
  html += "<span>AP active for 10 min after each boot</span></label>";
  html += "<p style='font-size:11px;color:#888;margin:6px 0 10px'>10-min mode auto-extends while you're on this page.</p>";

  // Test LED button — installer identifies which physical device this AP
  // belongs to. Uses fetch() so the page doesn't navigate away.
  html += "<button class='btn btn-blue' id='testLedBtn' onclick=\"var b=this;b.disabled=true;var o=b.textContent;b.textContent='Blinking...';fetch('/testled').finally(()=>{setTimeout(()=>{b.disabled=false;b.textContent=o},2000)})\">Test LED (Identify Device)</button>";
  html += "<a href='/restart'><button class='btn btn-red'>Restart Device</button></a>";
  html += "</div>";

  // Manual WiFi entry
  html += "<div class='card'>";
  html += "<h2>WiFi Setup</h2>";
  html += "<form action='/setwifi' method='GET'>";
  html += "<input type='text' name='ssid' placeholder='WiFi SSID' style='width:100%;margin-bottom:6px;padding:8px;border:1px solid #ddd;border-radius:6px;font-size:13px' required>";
  html += "<div style='position:relative'>";
  html += "<input type='password' id='wpass' name='pass' placeholder='Password' style='width:100%;padding:8px;border:1px solid #ddd;border-radius:6px;font-size:13px;padding-right:40px'>";
  html += "<button type='button' onclick=\"var p=document.getElementById('wpass');p.type=p.type==='password'?'text':'password'\" style='position:absolute;right:6px;top:50%;transform:translateY(-50%);background:none;border:none;color:#888;font-size:14px;cursor:pointer'>&#128065;</button>";
  html += "</div>";
  html += "<button class='btn btn-blue' type='submit' style='width:100%;margin-top:8px'>Connect WiFi</button>";
  html += "</form>";
  html += "</div>";

  // Live polling — updates tank viz without page reload so user can fill / drain and watch
  html += "<script>";
  html += "const INNER_TOP=50, INNER_BOTTOM=240, INNER_H=190;";
  html += "function fmtOne(v){if(v>=1000000)return (v/1000000).toFixed(2)+' ML';if(v>=1000)return (v/1000).toFixed(1)+' KL';return v+' L'}";
  html += "function fmtL(p,c){return c>0?fmtOne(Math.round(p*c/100))+' / '+fmtOne(c):''}";
  html += "async function pollStatus(){";
  html += "try{const r=await fetch('/sstatus');if(!r.ok)return;const d=await r.json();";
  // Water rect: recompute y + height in SVG units
  html += "const yStart=INNER_BOTTOM-(INNER_H*d.level/100);";
  html += "const w=document.getElementById('tankWater');if(w){w.setAttribute('y',yStart);w.setAttribute('height',INNER_BOTTOM-yStart)}";
  html += "const sf=document.getElementById('tankSurface');if(sf)sf.setAttribute('cy',yStart);";
  html += "const bp=document.getElementById('bigPct');if(bp)bp.textContent=d.level+'%';";
  html += "const lt=document.getElementById('litres');if(lt){const s=fmtL(d.level,d.capL);if(s){lt.textContent=s;lt.style.display='block'}else{lt.style.display='none'}}";
  html += "const eb=document.getElementById('errBanner');if(eb){if(d.error){eb.textContent='Sensor pattern fault \\u2014 check wiring';eb.style.display='block'}else{eb.style.display='none'}}";
  // 21.1.5: live sensor-missing banner update
  html += "const sm=document.getElementById('sensorMissingBanner');if(sm&&d.sensorConnected!==undefined){sm.style.display=d.sensorConnected?'none':'block'}";
  // Probe colors (line + tab bg + tab text)
  html += "document.querySelectorAll('.probe').forEach(el=>{const b=parseInt(el.dataset.bit);const on=!!(d.bits&(1<<b));el.setAttribute('stroke',on?'#2563eb':'#cbd5e1')});";
  html += "document.querySelectorAll('.probe-tab-bg').forEach(el=>{const b=parseInt(el.dataset.bit);const on=!!(d.bits&(1<<b));el.setAttribute('fill',on?'#2563eb':'#e5e7eb')});";
  html += "document.querySelectorAll('.probe-tab-tx').forEach(el=>{const b=parseInt(el.dataset.bit);const on=!!(d.bits&(1<<b));el.setAttribute('fill',on?'#ffffff':'#64748b')});";
  // AP-countdown live update
  html += "const ab=document.getElementById('apBanner');const alt=document.getElementById('apLeftTxt');";
  html += "if(ab&&alt&&d.apMode===1){if(d.apLeft>0){alt.textContent=Math.floor(d.apLeft/60)+'m '+(d.apLeft%60)+'s';ab.style.display='flex'}else{ab.style.display='none'}}";
  html += "}catch(e){}}";
  html += "setInterval(pollStatus,2000);";
  // Periodic page reload (every 30 s) to refresh non-tank cards (Last Push, RSSI, etc.) — only when not typing
  html += "setInterval(()=>{if(!document.activeElement||document.activeElement.tagName==='BODY')location.reload()},30000);";
  html += "</script>";
  html += "</body></html>";

  return html;
}

// ══════════════════════════════════════════════════
//  LOCAL MQTT (Phase 1: publish only)
// ══════════════════════════════════════════════════

#if ENABLE_LOCAL_MQTT

#if USE_CLOUD_MQTT == 0
// ── LAN-only helpers ──
void mqttBroadcastDiscovery() {
  if (WiFi.status() != WL_CONNECTED) return;
  IPAddress bcast = WiFi.localIP();
  bcast[3] = 255;
  mqttUdp.beginPacket(bcast, MQTT_DISCOVERY_PORT);
  mqttUdp.print(MQTT_DISCOVERY_MSG);
  mqttUdp.endPacket();
  Serial.println("[MQTT] Broadcast discovery sent");
}

void mqttHandleDiscoveryReply() {
  int pktSize = mqttUdp.parsePacket();
  if (pktSize <= 0) return;
  char buf[64] = {0};
  int len = mqttUdp.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  if (strncmp(buf, MQTT_DISCOVERY_REPLY, strlen(MQTT_DISCOVERY_REPLY)) == 0) {
    mqttBrokerIp = mqttUdp.remoteIP();
    mqttBrokerKnown = true;
    mqttClient.setServer(mqttBrokerIp, MQTT_PORT);
    Serial.print("[MQTT] Gateway discovered at "); Serial.println(mqttBrokerIp);
  }
}

String mqttDerivePassword() {
  String input = deviceCode + MQTT_SECRET;
  uint8_t hash[32];
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, (const uint8_t*)input.c_str(), input.length());
  mbedtls_sha256_finish(&ctx, hash);
  mbedtls_sha256_free(&ctx);
  char hex[17];
  for (int i = 0; i < 8; i++) sprintf(hex + i * 2, "%02x", hash[i]);
  hex[16] = 0;
  return String(hex);
}
#else
// ── Cloud-only setup ──
void mqttCloudInit() {
  if (mqttBrokerKnown) return;
  #if CLOUD_MQTT_USE_TLS
    mqttNetClient.setCACert(CLOUD_MQTT_CA_CERT);
  #endif
  mqttClient.setServer(CLOUD_MQTT_HOST, MQTT_PORT);
  mqttBrokerKnown = true;
  Serial.printf("[MQTT] Cloud broker set: %s:%d (TLS=%d)\n",
                CLOUD_MQTT_HOST, MQTT_PORT, CLOUD_MQTT_USE_TLS);
}
#endif

void mqttPublishLive();   // forward decl

// Command handler: dispatches messages on senseflow/<deviceCode>/cmd/+
void mqttCommandCallback(char* topic, byte* payload, unsigned int len) {
  String t = String(topic);
  String p;
  for (unsigned int i = 0; i < len; i++) p += (char)payload[i];
  Serial.printf("[MQTT] cmd %s = %s\n", t.c_str(), p.c_str());

  String base = "senseflow/" + deviceCode + "/cmd/";
  if (t == base + "refresh") {
    mqttPublishLive();
  }
  else if (t == base + "restart") {
    delay(500);
    safeRestart();
  }
  else if (t == base + "analytics") {
    bool on = (p == "1" || p == "true" || p == "ON");
    analyticsOn = on;
    Serial.printf("[MQTT] analyticsOn -> %s\n", on ? "ON" : "OFF");
  }
}

bool mqttEnsureConnected() {
  if (!mqttBrokerKnown) return false;
  if (mqttClient.connected()) return true;
  if (millis() - lastMqttReconnect < 5000) return false;
  lastMqttReconnect = millis();
  String clientId = "sf-" + deviceCode;
  #if USE_CLOUD_MQTT
    String username = CLOUD_MQTT_USER;
    String password = CLOUD_MQTT_PASS;
  #else
    String username = deviceCode;
    String password = mqttDerivePassword();
  #endif
  // LWT: when device disconnects, broker tells subscribers it went offline
  String willTopic = "senseflow/" + deviceCode + "/info/online";
  if (mqttClient.connect(clientId.c_str(), username.c_str(), password.c_str(),
                         willTopic.c_str(), 0, true, "false")) {
    Serial.println("[MQTT] Connected to gateway (authenticated)");
    // Announce online (retained)
    mqttClient.publish(willTopic.c_str(), "true", true);
    // Subscribe to all command topics for this device
    String cmdSub = "senseflow/" + deviceCode + "/cmd/+";
    mqttClient.subscribe(cmdSub.c_str());
    return true;
  }
  Serial.printf("[MQTT] connect failed rc=%d (5=bad credentials)\n", mqttClient.state());
  return false;
}

void mqttPublishLive() {
  if (!mqttClient.connected()) return;
  String topic = "senseflow/" + deviceCode + "/live";
  String payload = "{";
  payload += "\"pct\":" + String(confirmedPct) + ",";
  payload += "\"bits\":" + String(sensorBits) + ",";
  payload += "\"flags\":" + String(flags) + ",";
  payload += "\"error\":" + String(sensorError ? "true" : "false") + ",";
  payload += "\"rssi\":" + String(WiFi.RSSI());
  payload += "}";
  mqttClient.publish(topic.c_str(), payload.c_str(), true);   // retained
}

void mqttLoop() {
  if (WiFi.status() != WL_CONNECTED) return;

#if USE_CLOUD_MQTT
  // Cloud mode: no discovery, just set server once
  if (!mqttBrokerKnown) mqttCloudInit();
#else
  // LAN mode: broadcast UDP discovery until gateway replies
  if (!mqttBrokerKnown) {
    if (millis() - lastMqttDiscovery >= MQTT_DISCOVERY_INTERVAL_MS || lastMqttDiscovery == 0) {
      lastMqttDiscovery = millis();
      mqttUdp.begin(MQTT_DISCOVERY_PORT);
      mqttBroadcastDiscovery();
    }
    mqttHandleDiscoveryReply();
    return;
  }
#endif

  if (mqttEnsureConnected()) {
    mqttClient.loop();
    if (millis() - lastMqttPublish >= MQTT_PUBLISH_INTERVAL_MS) {
      lastMqttPublish = millis();
      mqttPublishLive();
    }
  } else {
#if !USE_CLOUD_MQTT
    // LAN: if broker stays unreachable >2 min, maybe Pi changed IP.
    // Force re-discovery so we pick up the new one.
    static unsigned long firstDisconnectAt = 0;
    if (firstDisconnectAt == 0) firstDisconnectAt = millis();
    if (millis() - firstDisconnectAt > 120000) {
      Serial.println("[MQTT] >2 min disconnect — forcing re-discovery");
      mqttBrokerKnown = false;
      lastMqttDiscovery = 0;
      firstDisconnectAt = 0;
    }
#endif
  }
}

#endif  // ENABLE_LOCAL_MQTT

// ══════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════

// Boot-reason flag — if last shutdown was unclean, blink purple briefly so
// installer/user notices something happened. Set during setup, used by LED.
bool crashedLastBoot = false;
unsigned long crashIndicatorStart = 0;

// ══════════════════════════════════════════════════
//  NTP TIME SYNC
// ══════════════════════════════════════════════════

uint32_t nowEpoch() {
  time_t now = time(nullptr);
  return (now > 1700000000) ? (uint32_t)now : 0;   // sanity: post-2023
}

void syncNTPIfDue() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (ntpSynced && (millis() - lastNtpSyncAt < NTP_RESYNC_INTERVAL)) return;

  configTime(NTP_TZ_OFFSET_S, 0, NTP_SERVER);
  // Wait up to 3 sec for first sync (boot path) or just kick off (resync)
  for (int i = 0; i < 30 && nowEpoch() == 0; i++) delay(100);
  uint32_t e = nowEpoch();
  if (e > 0) {
    ntpSynced = true;
    lastNtpSyncAt = millis();
    Serial.printf("[NTP] Sync OK, epoch=%u\n", e);

    // On the first successful sync after boot, stamp firstBootAt if unset
    Preferences p;
    p.begin("senseflow", false);
    if (firstBootAt == 0) {
      firstBootAt = e;
      p.putUInt("firstBoot", firstBootAt);
      Serial.printf("[BOOT] firstBootAt = %u\n", firstBootAt);
    }
    // If never OTA'd, lastUpdatedAt mirrors firstBootAt
    if (lastUpdatedAt == 0) {
      lastUpdatedAt = firstBootAt;
      p.putUInt("lastUpd", lastUpdatedAt);
    }
    p.end();

    // Now that timestamps are real (not 0), push to Firebase if changed.
    // Write-once cache ensures this is a no-op on subsequent reboots.
    reportFirmwareInfoIfChanged();
  }
}

// ══════════════════════════════════════════════════
//  OTA — admin-triggered firmware update
// ══════════════════════════════════════════════════

#if ENABLE_CLOUD

// Report firmware info to Firebase /devices/<code>/info/
void reportFirmwareInfo() {
  if (!firebaseHealthy) return;
  FirebaseJson json;
  json.set("firmwareVersion", FIRMWARE_VERSION);
  json.set("firstBootAt",     firstBootAt);
  json.set("lastUpdatedAt",   lastUpdatedAt);
  String path = "devices/" + deviceCode + "/info";
  Firebase.RTDB.updateNode(&fbdo, path.c_str(), &json);
}

// Write-once optimisation — only push to Firebase if firmware version OR
// lastUpdatedAt has changed since the last reported snapshot (stored in NVS).
// Saves a Firebase write per boot/NTP sync on devices that haven't changed.
void reportFirmwareInfoIfChanged() {
  if (!firebaseHealthy) return;

  // Build a stable hash of {firmwareVersion, lastUpdatedAt}. djb2 on the
  // version string XOR lastUpdatedAt is enough — collisions are harmless
  // (worst case: one extra write).
  uint32_t verHash = 5381;
  const char* s = FIRMWARE_VERSION;
  while (*s) { verHash = ((verHash << 5) + verHash) + (uint8_t)(*s++); }
  uint32_t key = verHash ^ lastUpdatedAt;

  Preferences p;
  p.begin("senseflow", false);
  uint32_t lastKey = p.getUInt("rptKey", 0);
  if (key == lastKey) {
    p.end();
    return;   // nothing changed — skip the Firebase write
  }
  reportFirmwareInfo();
  p.putUInt("rptKey", key);
  p.end();
  Serial.printf("[FB] reportFirmwareInfo pushed (key=%u)\n", key);
}

// Set OTA status fields on Firebase
void setOtaStatus(const String& status, uint8_t retries) {
  if (!firebaseHealthy) return;
  FirebaseJson json;
  json.set("lastOtaStatus", status);
  json.set("otaRetryCount", retries);
  String path = "devices/" + deviceCode + "/info";
  Firebase.RTDB.updateNode(&fbdo, path.c_str(), &json);
}

// Clear the trigger so we don't re-fire on next poll
void clearOtaTrigger() {
  if (!firebaseHealthy) return;
  FirebaseJson json;
  json.set("otaTrigger", false);
  String path = "devices/" + deviceCode + "/config";
  Firebase.RTDB.updateNode(&fbdo, path.c_str(), &json);
}

// Execute the actual HTTP(S) firmware download + flash. Blocks for the
// duration of the download — watchdog is fed via progress callback.
// Returns reason string on failure, empty string on success (won't return
// at all on success because device reboots).
String runHttpUpdate(const String& url, const String& md5) {
  // Disable watchdog during long download
  esp_task_wdt_delete(NULL);

  // Pick client based on URL scheme
  WiFiClient        plainClient;
  WiFiClientSecure  tlsClient;
  WiFiClient*       chosen = nullptr;
  if (url.startsWith("https://")) {
    tlsClient.setInsecure();   // accept self-signed / unknown CA; admin owns the URL
    chosen = &tlsClient;
  } else {
    chosen = &plainClient;
  }

  httpUpdate.setLedPin(LED_PIN, LOW);
  httpUpdate.rebootOnUpdate(false);   // we'll reboot ourselves after Firebase write
  if (md5.length() == 32) {
    httpUpdate.setMD5sum(md5);
  }
  // Feed watchdog during download via progress callback
  httpUpdate.onProgress([](int progress, int total){
    static uint32_t lastFeed = 0;
    if (millis() - lastFeed > 2000) {
      lastFeed = millis();
      Serial.printf("[OTA] %d / %d bytes\n", progress, total);
    }
  });

  t_httpUpdate_return ret = httpUpdate.update(*chosen, url);

  // Re-arm watchdog
  esp_task_wdt_add(NULL);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      return "fail:" + String(httpUpdate.getLastError()) + ":" + httpUpdate.getLastErrorString();
    case HTTP_UPDATE_NO_UPDATES:
      return "fail:no_update";
    case HTTP_UPDATE_OK:
      return "";   // success, but execution won't reach here if we rebooted
    default:
      return "fail:unknown";
  }
}

// Check Firebase for an OTA trigger and execute if due. Runs from main
// loop on a 30-sec timer.
void checkOtaTrigger() {
  if (!firebaseHealthy) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (!ntpSynced) return;   // need real time to honor schedules

  String basePath = "devices/" + deviceCode + "/config/";

  // Each Firebase call can stall up to ~2s on a flaky network. Feed the
  // watchdog between calls so a chain of slow reads doesn't add up to >60s.
  esp_task_wdt_reset();
  if (!Firebase.RTDB.getBool(&fbdo, (basePath + "otaTrigger").c_str())) return;
  if (!fbdo.boolData()) return;   // not triggered

  // Read scheduled time
  uint32_t scheduledAt = 0;
  esp_task_wdt_reset();
  if (Firebase.RTDB.getInt(&fbdo, (basePath + "otaScheduledAt").c_str())) {
    scheduledAt = (uint32_t)fbdo.intData();
  }
  uint32_t epoch = nowEpoch();
  if (scheduledAt > 0 && epoch < scheduledAt) {
    // Not yet time
    return;
  }
  // Stale trigger guard — if scheduled time is more than 7 days in the past,
  // assume the admin meant a different update window and don't auto-flash.
  const uint32_t SEVEN_DAYS = 7UL * 24UL * 3600UL;
  if (scheduledAt > 0 && epoch > scheduledAt + SEVEN_DAYS) {
    Serial.println("[OTA] Trigger expired (>7 days past scheduled time) — clearing");
    setOtaStatus("fail:expired", 0);
    clearOtaTrigger();
    return;
  }

  // Read retry count + URL + MD5
  uint8_t retries = 0;
  String infoPath = "devices/" + deviceCode + "/info/";
  esp_task_wdt_reset();
  if (Firebase.RTDB.getInt(&fbdo, (infoPath + "otaRetryCount").c_str())) {
    retries = (uint8_t)fbdo.intData();
  }
  if (retries >= OTA_MAX_RETRIES) {
    setOtaStatus("fail:max_retries", retries);
    clearOtaTrigger();
    Serial.println("[OTA] Max retries reached — giving up");
    return;
  }

  String url, md5;
  esp_task_wdt_reset();
  if (Firebase.RTDB.getString(&fbdo, (basePath + "otaTargetUrl").c_str())) url = fbdo.stringData();
  if (url.length() == 0) {
    setOtaStatus("fail:no_url", retries);
    clearOtaTrigger();
    return;
  }
  esp_task_wdt_reset();
  if (Firebase.RTDB.getString(&fbdo, (basePath + "otaTargetMd5").c_str())) md5 = fbdo.stringData();

  Serial.printf("[OTA] Starting download: %s\n", url.c_str());
  setOtaStatus("in_progress", retries);

  String result = runHttpUpdate(url, md5);

  if (result.length() == 0) {
    // Success — record then reboot. Clear the report-cache key so the
    // freshly-booted new firmware re-pushes timestamps to Firebase.
    Preferences p;
    p.begin("senseflow", false);
    lastUpdatedAt = nowEpoch();
    p.putUInt("lastUpd", lastUpdatedAt);
    p.remove("rptKey");   // force re-push on next boot
    p.end();

    setOtaStatus("success", 0);
    clearOtaTrigger();
    Serial.println("[OTA] Success — rebooting into new firmware");
    delay(1000);
    safeRestart();
  } else {
    // Fail — increment retry, set next scheduled time with random backoff
    retries++;
    setOtaStatus(result, retries);
    if (retries < OTA_MAX_RETRIES) {
      uint32_t backoffSec = 60 + (esp_random() % 540);   // 1-10 min
      FirebaseJson json;
      json.set("otaScheduledAt", epoch + backoffSec);
      Firebase.RTDB.updateNode(&fbdo, ("devices/" + deviceCode + "/config").c_str(), &json);
      Serial.printf("[OTA] Failed (%s), retry %u/%u in %u sec\n",
                    result.c_str(), retries, OTA_MAX_RETRIES, backoffSec);
    } else {
      clearOtaTrigger();
      Serial.println("[OTA] Final fail — trigger cleared");
    }
  }
}

#else
void reportFirmwareInfo() {}
void checkOtaTrigger() {}
#endif  // ENABLE_CLOUD

// ══════════════════════════════════════════════════
//  DIAGNOSTICS — BOOT LOG
// ══════════════════════════════════════════════════
//
// Tracks every restart so admin can diagnose flaky devices remotely.
// Storage: 50-entry circular buffer in NVS, ~800 bytes total. Indexed by
// `bootCount % 50` so we never overflow and the latest 50 are always there.
// Cloud upload is gated by `diagnosticsOn` to avoid bandwidth waste on
// healthy devices.
//
// Uptime tracking: persistCurrentUptime() saves millis()/1000 to NVS
// every 10 min (every 60 s during first 10 min after boot, to catch
// early-boot brownouts). Next boot reads this — that's how we know how
// long the previous session ran before dying.

#define BOOTLOG_NVS_NS "diag"

// Parse "17.0.9" → 17, 0, 9
void parseFwVersion(const char* v, uint8_t* maj, uint8_t* min, uint8_t* patch) {
  *maj = *min = *patch = 0;
  int dots = 0;
  uint8_t acc = 0;
  for (const char* p = v; *p; p++) {
    if (*p == '.') {
      if (dots == 0) *maj = acc;
      else if (dots == 1) *min = acc;
      dots++;
      acc = 0;
    } else if (*p >= '0' && *p <= '9') {
      acc = acc * 10 + (*p - '0');
    }
  }
  if (dots >= 2) *patch = acc;
}

// Append a new boot log entry to NVS. Called once from setup() after we
// know the reset reason + previous uptime. Updates the in-RAM
// latestBootEntry so the cloud upload path can grab it.
void appendBootLogEntry(uint8_t reason, uint32_t uptimeBefore) {
  Preferences p;
  p.begin(BOOTLOG_NVS_NS, false);
  boot_logIdx = p.getUChar("idx", 0);
  bootCount   = p.getUInt("count", 0);

  BootLogEntry e;
  e.epoch          = nowEpoch();              // 0 if NTP not yet synced — Cloud Function tolerates
  e.uptimeBefore   = uptimeBefore;
  e.freeHeapAtBoot = ESP.getFreeHeap();
  e.reason         = (uint8_t)reason;
  parseFwVersion(FIRMWARE_VERSION, &e.fwMajor, &e.fwMinor, &e.fwPatch);

  // Persist this slot
  char key[8];
  snprintf(key, sizeof(key), "e%u", boot_logIdx);
  p.putBytes(key, &e, sizeof(e));

  // Advance index circularly
  boot_logIdx = (boot_logIdx + 1) % BOOTLOG_CAPACITY;
  bootCount++;
  p.putUChar("idx", boot_logIdx);
  p.putUInt("count", bootCount);
  p.end();

  latestBootEntry    = e;
  hasLatestBootEntry = true;
  Serial.printf("[DIAG] Boot logged: reason=%u uptimeBefore=%us heap=%u (#%u)\n",
                e.reason, e.uptimeBefore, e.freeHeapAtBoot, bootCount);
}

// Persist current millis()/1000 to NVS so the next boot knows how long
// this session ran. Adaptive cadence — caller decides when to call.
void persistCurrentUptime() {
  uint32_t up = (uint32_t)(millis() / 1000);
  Preferences p;
  p.begin(BOOTLOG_NVS_NS, false);
  p.putUInt("up", up);
  p.end();
}

// Read previous session's uptime from NVS (set by persistCurrentUptime()
// during last run). Returns 0 if NVS is fresh (first boot ever).
uint32_t readPreviousUptime() {
  Preferences p;
  p.begin(BOOTLOG_NVS_NS, true);   // read-only
  uint32_t prev = p.getUInt("up", 0);
  p.end();
  return prev;
}

// Reset reason → human-readable string for log + serial dump.
const char* resetReasonStr(uint8_t r) {
  switch ((esp_reset_reason_t)r) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_EXT:      return "external-pin";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "panic";
    case ESP_RST_INT_WDT:  return "int-wdt";
    case ESP_RST_TASK_WDT: return "task-wdt";
    case ESP_RST_WDT:      return "other-wdt";
    case ESP_RST_DEEPSLEEP:return "deep-sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO:     return "sdio";
    default:               return "unknown";
  }
}

// Read entry at slot i (0..49). Returns true if a valid entry was loaded.
bool readBootLogEntry(uint8_t i, BootLogEntry* out) {
  if (i >= BOOTLOG_CAPACITY) return false;
  Preferences p;
  p.begin(BOOTLOG_NVS_NS, true);
  char key[8];
  snprintf(key, sizeof(key), "e%u", i);
  size_t n = p.getBytes(key, out, sizeof(BootLogEntry));
  p.end();
  return (n == sizeof(BootLogEntry));
}

// Wipe NVS boot log + RTDB upload. Triggered by clearDiagLogRequested
// command from admin or `DIAG CLEAR` serial command.
void clearBootLog() {
  Preferences p;
  p.begin(BOOTLOG_NVS_NS, false);
  p.clear();
  p.end();
  boot_logIdx = 0;
  bootCount   = 0;
  hasLatestBootEntry = false;
  Serial.println("[DIAG] Boot log NVS cleared");
}

// Dump the last 50 entries to Serial (human-readable). Bound to the
// `BOOTLOG` serial command — always available regardless of diagnosticsOn,
// since this is local-only and helps installer diagnose USB-attached devices.
void dumpBootLogToSerial() {
  Preferences p;
  p.begin(BOOTLOG_NVS_NS, true);
  uint8_t  idx   = p.getUChar("idx", 0);
  uint32_t total = p.getUInt("count", 0);
  p.end();

  Serial.println("\n--- BOOT LOG (oldest first, max 50) ---");
  Serial.printf("Total boots ever: %u\n", total);
  if (total == 0) {
    Serial.println("(empty)\n");
    return;
  }

  uint8_t toShow = total < BOOTLOG_CAPACITY ? (uint8_t)total : BOOTLOG_CAPACITY;
  // Walk circularly starting from the oldest slot.
  uint8_t start = (total < BOOTLOG_CAPACITY) ? 0 : idx;
  for (uint8_t k = 0; k < toShow; k++) {
    uint8_t slot = (start + k) % BOOTLOG_CAPACITY;
    BootLogEntry e;
    if (!readBootLogEntry(slot, &e)) continue;
    Serial.printf("%2u) epoch=%u  ran=%us  reason=%u (%s)  heap=%u  fw=%u.%u.%u\n",
                  k + 1, e.epoch, e.uptimeBefore, e.reason,
                  resetReasonStr(e.reason), e.freeHeapAtBoot,
                  e.fwMajor, e.fwMinor, e.fwPatch);
  }
  Serial.println("--------------------------------------\n");
}

#if ENABLE_CLOUD
// Upload the most recent boot log entry to RTDB. Path:
//   /devices/<code>/diagnostics/boots/<slot>
// where <slot> = (bootCount - 1) % 50. Admin UI reads from there.
void uploadLatestBootLog() {
  if (!firebaseHealthy) return;
  if (!diagnosticsOn)   return;
  if (!hasLatestBootEntry) return;

  BootLogEntry& e = latestBootEntry;
  uint8_t slot = (bootCount == 0) ? 0 : (uint8_t)((bootCount - 1) % BOOTLOG_CAPACITY);

  char path[128];
  snprintf(path, sizeof(path), "devices/%s/diagnostics/boots/%u",
           deviceCode.c_str(), slot);

  FirebaseJson json;
  json.set("epoch",          e.epoch);
  json.set("uptimeBefore",   e.uptimeBefore);
  json.set("freeHeapAtBoot", e.freeHeapAtBoot);
  json.set("reason",         e.reason);
  json.set("reasonStr",      resetReasonStr(e.reason));
  json.set("fwVersion",      String(e.fwMajor) + "." + String(e.fwMinor) + "." + String(e.fwPatch));
  json.set("bootNumber",     bootCount);

  esp_task_wdt_reset();
  if (Firebase.RTDB.setJSON(&fbdo, path, &json)) {
    Serial.printf("[DIAG] Boot log uploaded to slot %u\n", slot);
  } else {
    Serial.printf("[DIAG] Boot log upload failed: %s\n", fbdo.errorReason().c_str());
  }
}

// Snapshot live diagnostics to RTDB. Called on refreshDiagRequested
// command (admin clicks button in UI). Cheap, single write.
void uploadDiagnosticsNow() {
  if (!firebaseHealthy) return;
  if (!diagnosticsOn)   return;

  String path = "devices/" + deviceCode + "/diagnostics/now";

  FirebaseJson json;
  json.set("uptime",                 (uint32_t)(millis() / 1000));
  json.set("freeHeap",               ESP.getFreeHeap());
  json.set("rssi",                   WiFi.RSSI());
  json.set("internetAvailable",      internetAvailable);
  json.set("firebaseHealthy",        firebaseHealthy);
  json.set("consecutivePushFails",   consecutiveFailCount);
  json.set("lastNtpSyncAt",          (uint32_t)(lastNtpSyncAt / 1000));
  json.set("ts/.sv",                 "timestamp");

  esp_task_wdt_reset();
  Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json);
}

// Wipe RTDB boot log path. Called from clearDiagLogRequested command.
void clearBootLogRtdb() {
  if (!firebaseHealthy) return;
  String path = "devices/" + deviceCode + "/diagnostics/boots";
  esp_task_wdt_reset();
  Firebase.RTDB.deleteNode(&fbdo, path.c_str());
}

#else
void uploadLatestBootLog() {}
void uploadDiagnosticsNow() {}
void clearBootLogRtdb()    {}
#endif  // ENABLE_CLOUD

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== SenseFlow Firebase Sensor v" FIRMWARE_VERSION " ===\n");

  // Log reset reason so installer/log can see if device crashed
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.printf("[BOOT] Reset reason: %d (%s)\n", (int)reason, resetReasonStr((uint8_t)reason));
  if (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
      reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT ||
      reason == ESP_RST_BROWNOUT) {
    Serial.println("[BOOT] Previous run ended abnormally");
    crashedLastBoot = true;
    crashIndicatorStart = millis();
  }

  // Capture reset reason + previous-session uptime to RAM only. NO NVS
  // write here — we don't know yet whether diagnosticsOn is true (config
  // hasn't been read). If it IS on, checkConfig() will materialise this
  // into NVS + cloud later. Free customers' devices that never enable
  // diagnostics will never write to NVS for diagnostics. Zero wear.
  pendingBootReason       = (uint8_t)reason;
  pendingBootPrevUptime   = readPreviousUptime();
  pendingBootCaptured     = true;

  // Hardware watchdog — reboot if main loop ever blocks > 30 s.
  // (Firebase TLS at worst case ~3s now thanks to timeout tightening.)
  // ESP32 core 3.x uses a config struct; older cores use (timeout, panic).
  #if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t wdt_cfg = {
      .timeout_ms = 60000,
      .idle_core_mask = 0,
      .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_cfg);
  #else
    esp_task_wdt_init(60, true);
  #endif
  esp_task_wdt_add(NULL);

  // LED
  FastLED_min<LED_PIN>.addLeds(rgbLeds, 1);
  FastLED_min<LED_PIN>.setBrightness(80);
  setLED(255, 100, 0);  // Orange on boot

  // Load or generate device code
  loadOrCreateDeviceCode();

  // Print registration info (factory serial output)
  printRegistrationInfo();

  // Initialize sensors
  #if USE_ULTRASONIC
    initUltrasonic();
    Serial.println("Ultrasonic sensor initialized");
  #else
    initDipSensors();
    Serial.println("DIP sensors initialized (" + String(SENSOR_COUNT) + " switches)");
  #endif

  // Start MvsConnect AP
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apName.c_str(), AP_PASSWORD);
  Serial.println("AP started: " + apName);
  // Let the AP fully initialise before the STA subsystem starts its
  // scan/associate cycle. Without this settle time, the concurrent
  // AP+STA startup produces a race in ESP-IDF where the AP briefly
  // drops when STA reconfigures the radio — visible in the field as
  // "MvsConnect app can't connect, AP flapping". See feedback memory
  // and 19.0.2 fix.
  delay(1000);

  // MvsConnect setup
  mvs.setCustomHTML([](){ return buildCustomHTML(); });

  mvs.onWiFiCredentialsReceived([](const String& ssid) {
    Serial.println("WiFi credentials received: " + ssid);
    // Fix: disconnect first to avoid "sta is connecting, cannot set config"
    WiFi.disconnect(false);
    delay(200);
  });

  // begin() MUST be called before addEndpoint()
  mvs.begin();

  // Manual WiFi entry endpoint
  mvs.addEndpoint("/setwifi", []() {
    WebServer* srv = mvs.getServer();
    String ssid = srv->arg("ssid");
    String pass = srv->arg("pass");
    if (ssid.length() == 0) {
      srv->send(400, "text/html", "<html><body><h2>SSID required</h2></body></html>");
      return;
    }
    // Confirmation page polls /sstatus every 2s and only redirects once
    // WiFi.status() reports Connected. Hard fallback redirect at 45s so
    // the page never hangs forever if the creds were wrong.
    String html =
      "<html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
      "<style>body{font-family:sans-serif;text-align:center;padding:30px;background:#111;color:#eee}"
      ".dot{display:inline-block;width:14px;height:14px;border-radius:50%;background:#f44;margin-right:8px;vertical-align:middle}"
      ".dot.ok{background:#4c4}.spin{animation:sp 1s linear infinite;display:inline-block}"
      "@keyframes sp{to{transform:rotate(360deg)}}h2{margin:20px 0}p{opacity:.8}</style></head><body>"
      "<h2>Connecting to " + ssid + "</h2>"
      "<p><span class='dot' id='d'></span><span id='s'>Handshaking with router\xE2\x80\xA6</span></p>"
      "<p class='spin' id='sp'>\xE2\x9A\x99</p>"
      "<p id='t' style='font-size:12px;opacity:.6'>0s</p>"
      "<script>"
      "let n=0,done=false;"
      "function poll(){if(done)return;n+=2;document.getElementById('t').textContent=n+'s';"
      "fetch('/sstatus').then(r=>r.json()).then(j=>{"
      "if(j.wifi&&j.wifi.toLowerCase().indexOf('connect')===0&&j.wifi.toLowerCase().indexOf('not')<0){"
      "done=true;document.getElementById('d').classList.add('ok');"
      "document.getElementById('s').textContent='Connected! Redirecting\xE2\x80\xA6';"
      "document.getElementById('sp').style.display='none';"
      "setTimeout(()=>location.href='/',1500);}}).catch(()=>{});}"
      "setInterval(poll,2000);poll();"
      "setTimeout(()=>{if(!done)location.href='/';},45000);"
      "</script></body></html>";
    srv->send(200, "text/html", html);
    Serial.println("Manual WiFi: " + ssid);
    // Pause auto-reconnect for 30s so it doesn't fight
    manualWiFiInProgress = true;
    manualWiFiStart = millis();
    // disconnect(false) — never true (erases ESP-IDF config + drops AP).
    WiFi.disconnect(false);
    delay(200);
    // Save new credentials to NVS first
    Preferences wifiPrefs;
    wifiPrefs.begin("mvswifi", false);
    wifiPrefs.putString("ssid", ssid);
    wifiPrefs.putString("password", pass);
    wifiPrefs.putBool("valid", true);
    wifiPrefs.end();
    // Now connect with new credentials
    WiFi.begin(ssid.c_str(), pass.c_str());
    Serial.println("WiFi.begin called for: " + ssid);
  });

  // API endpoints — use mvs.getServer() inside handlers
  // Tank height setting (ultrasonic)
  mvs.addEndpoint("/settank", []() {
    WebServer* srv = mvs.getServer();
    float h = srv->arg("h").toFloat();
    if (h >= 36 && h <= 750) {                         // v21.0.2: extended to A01 range
      usTankHeight = h;
      Preferences tankPrefs;
      tankPrefs.begin("senseflow", false);
      tankPrefs.putFloat("tankh", h);
      tankPrefs.end();
      Serial.println("Tank height set to: " + String(h) + " cm");
      // Mirror to /info so the cloud dashboard shows the updated
      // geometry immediately (used for computing water depth in ft/in
      // on the tile). Otherwise the change waits until the next
      // heartbeat, up to 5 min lag.
      if (firebaseHealthy) updateDeviceInfo(true);
    }
    srv->sendHeader("Location", "/"); srv->send(302);
  });

  // v21.0.3: overflow pipe distance (optional). Rules:
  //   o = 0            → disabled (uses full tank height, backward-compat)
  //   1 <= o < 35      → REJECTED. Sensor blind zone is 28 cm; anything
  //                      closer than ~35 cm is unmeasurable. Installer
  //                      must remount sensor at least 35 cm above the
  //                      overflow pipe. Error page explains why.
  //   35 <= o < tank-10 → accepted, saved to NVS
  //   o >= tank-10     → REJECTED. Would leave <10 cm usable range.
  mvs.addEndpoint("/setoverflow", []() {
    WebServer* srv = mvs.getServer();
    int o = srv->arg("o").toInt();
    if (o == 0) {                                       // disable
      usOverflow = 0;
      Preferences p; p.begin("senseflow", false);
      p.putUShort("usOvfl", 0); p.end();
      Serial.println("Overflow DISABLED (using full tank height)");
      if (firebaseHealthy) updateDeviceInfo(true);
      srv->sendHeader("Location", "/"); srv->send(302);
      return;
    }
    if (o < 35) {
      srv->send(400, "text/html",
        "<html><body style='font-family:sans-serif;padding:24px;background:#fef2f2'>"
        "<h2 style='color:#991b1b'>Overflow rejected</h2>"
        "<p>You entered <b>" + String(o) + " cm</b>, but the sensor blind zone is 28 cm "
        "and needs a 7 cm safety margin.</p>"
        "<p><b>Please remount the sensor at least 35 cm above the overflow pipe</b>, "
        "then enter that distance here.</p>"
        "<p><a href='/'>&larr; Back</a></p></body></html>");
      Serial.printf("[OVFL] Rejected %d cm — below sensor blind zone + margin (35 cm)\n", o);
      return;
    }
    if (o >= (int)usTankHeight - 10) {
      srv->send(400, "text/html",
        "<html><body style='font-family:sans-serif;padding:24px;background:#fef2f2'>"
        "<h2 style='color:#991b1b'>Overflow too high</h2>"
        "<p>Overflow (" + String(o) + " cm) must be at least 10 cm less than tank height ("
        + String((int)usTankHeight) + " cm).</p>"
        "<p><a href='/'>&larr; Back</a></p></body></html>");
      Serial.printf("[OVFL] Rejected %d cm — leaves <10 cm usable range (tank=%d)\n", o, (int)usTankHeight);
      return;
    }
    usOverflow = (uint16_t)o;
    Preferences p; p.begin("senseflow", false);
    p.putUShort("usOvfl", usOverflow); p.end();
    Serial.printf("Overflow set to: %u cm\n", usOverflow);
    // Push new geometry to cloud immediately (see /settank note).
    if (firebaseHealthy) updateDeviceInfo(true);
    srv->sendHeader("Location", "/"); srv->send(302);
  });

  // v21.0.5: pump suction offset from tank bottom.
  //   s = 0            → disable (backward-compat, tank empty at floor)
  //   1 <= s < tank-35 → accepted (need enough range above suction to
  //                       still measure — sensor blind zone + overflow room)
  //   s >= tank-35     → rejected (leaves too little usable range)
  mvs.addEndpoint("/setsuction", []() {
    WebServer* srv = mvs.getServer();
    int s = srv->arg("s").toInt();
    if (s == 0) {
      usSuction = 0;
      Preferences p; p.begin("senseflow", false);
      p.putUShort("usSuct", 0); p.end();
      Serial.println("Suction DISABLED");
      if (firebaseHealthy) updateDeviceInfo(true);
      srv->sendHeader("Location", "/"); srv->send(302);
      return;
    }
    if (s < 0 || s >= (int)usTankHeight - 35) {
      srv->send(400, "text/html",
        "<html><body style='font-family:sans-serif;padding:24px;background:#fef2f2'>"
        "<h2 style='color:#991b1b'>Suction rejected</h2>"
        "<p>You entered <b>" + String(s) + " cm</b>. Must be between 1 and "
        + String((int)usTankHeight - 36) + " cm (leaves room for sensor blind zone + overflow).</p>"
        "<p><a href='/'>&larr; Back</a></p></body></html>");
      Serial.printf("[SUCTION] Rejected %d cm — out of range\n", s);
      return;
    }
    usSuction = (uint16_t)s;
    Preferences p; p.begin("senseflow", false);
    p.putUShort("usSuct", usSuction); p.end();
    Serial.printf("Suction set to: %u cm from bottom\n", usSuction);
    if (firebaseHealthy) updateDeviceInfo(true);
    srv->sendHeader("Location", "/"); srv->send(302);
  });

  mvs.addEndpoint("/restart", []() {
    WebServer* srv = mvs.getServer();
    srv->send(200, "text/html", "<html><body><h2>Restarting...</h2><script>setTimeout(()=>history.back(),3000)</script></body></html>");
    delay(1000);
    safeRestart();
  });

  // Test LED — triggers the same rainbow blink as the cloud test command.
  // Lets an installer confirm which physical device they're connected to
  // when multiple SenseFlow APs are visible in the same area.
  mvs.addEndpoint("/testled", []() {
    WebServer* srv = mvs.getServer();
    testBlinkActive = true;
    testBlinkStart  = millis();
    srv->send(200, "application/json", "{\"ok\":true}");
    Serial.println("[TEST] LED rainbow blink triggered from AP page");
  });

  #if USE_ULTRASONIC
  // Force-refresh: installer button. Fires 10 back-to-back reads,
  // runs the histogram filter with strict 70% agreement, returns raw
  // stats without touching hysteresis / rate limits so the installer
  // can hit refresh repeatedly during sensor positioning.
  mvs.addEndpoint("/refreshus", []() {
    WebServer* srv = mvs.getServer();
    float samples[15];
    int valid = 0;
    for (int i = 0; i < 15; i++) {
      float d = readUltrasonicRaw();
      if (d > 0) samples[valid++] = d;
      if (usSensorType != US_TYPE_DYP_A21) delay(10);
    }
    float chosen = (valid >= 5) ? histogramModeFilter(samples, valid) : -1.0f;
    uint8_t pct  = (chosen > 0) ? distanceToIntegerPct(chosen) : 0;

    String json = "{";
    json += "\"ok\":" + String(chosen > 0 ? "true" : "false") + ",";
    json += "\"valid\":" + String(valid) + ",";
    json += "\"cm\":" + String(chosen, 1) + ",";
    json += "\"pct\":" + String(pct) + ",";
    json += "\"usType\":" + String(usSensorType) + ",";
    json += "\"pings\":[";
    for (int i = 0; i < valid; i++) {
      if (i) json += ",";
      json += String(samples[i], 1);
    }
    json += "]}";
    srv->send(200, "application/json", json);
    Serial.printf("[REFRESH] valid=%d chosen=%.1f cm pct=%u\n", valid, chosen, pct);
  });

  // Manual sensor-type override. Also clears cached baud so autodetect
  // reruns on next boot for type=0. Values: 0=autodetect, 1=HC-SR04,
  // 2=DYP-A01, 3=DYP-A21, 4=RS485 Modbus-RTU.
  mvs.addEndpoint("/setustype", []() {
    WebServer* srv = mvs.getServer();
    int t = srv->arg("t").toInt();
    if (t < 0 || t > 4) { srv->send(400, "text/plain", "bad type"); return; }
    Preferences p;
    p.begin("senseflow", false);
    p.putUChar("usType", (uint8_t)t);
    p.end();
    srv->send(200, "text/html",
      "<html><body><h2>Sensor type saved. Restarting…</h2></body></html>");
    delay(500);
    safeRestart();
  });

  // RS485 Modbus address select. No restart needed — takes effect on the
  // next burst read since usRs485Addr is the live global readRs485Cm()
  // consults directly. Register is no longer settable here — always
  // real-time (see usRs485Reg comment).
  mvs.addEndpoint("/setrs485addr", []() {
    WebServer* srv = mvs.getServer();
    int a = srv->arg("a").toInt();
    if (a < 1 || a > 247) {
      srv->send(400, "text/html",
        "<html><body style='font-family:sans-serif;padding:24px;background:#fef2f2'>"
        "<h2 style='color:#991b1b'>Address rejected</h2>"
        "<p>Modbus address must be between 1 and 247.</p>"
        "<p><a href='/'>&larr; Back</a></p></body></html>");
      return;
    }
    usRs485Addr = (uint8_t)a;
    Preferences p;
    p.begin("senseflow", false);
    p.putUChar("usRs485A", usRs485Addr);
    p.end();
    Serial.printf("[RS485] Config saved: addr=%u (register fixed: real-time 0x%04X)\n",
                  usRs485Addr, US_RS485_REG_REALTIME);
    srv->sendHeader("Location", "/"); srv->send(302);
  });
  #endif

  mvs.addEndpoint("/sstatus", []() {
    WebServer* srv = mvs.getServer();
    // Auto-extend AP timer: any active poll proves a user is on the page
    if (apMode == 1 && apTimerStart != 0 && !apTimerEnded) {
      unsigned long remaining = (millis() < apTimerDeadline) ? (apTimerDeadline - millis()) : 0;
      if (remaining < 60000) {   // < 1 min left → extend
        apTimerDeadline = millis() + AP_EXTEND_MS;
      }
    }
    unsigned long apLeft = 0;
    if (apMode == 1 && apTimerStart != 0 && !apTimerEnded && millis() < apTimerDeadline) {
      apLeft = (apTimerDeadline - millis()) / 1000;
    }
    // AP page shows INSTANT reading so an installer sees real-time water
    // motion. Cloud dashboard uses the committed (dominant-level) value —
    // see processDipSensors(). committedLevel exposed as `stableLevel`
    // so installer can see both when needed.
    String json = "{";
    json += "\"code\":\"" + deviceCode + "\",";
    json += "\"level\":" + String(instantPct) + ",";
    json += "\"bits\":" + String(instantBits) + ",";
    json += "\"stableLevel\":" + String(confirmedPct) + ",";
    json += "\"stableBits\":" + String(sensorBits) + ",";
    json += "\"count\":" + String(SENSOR_COUNT) + ",";
    json += "\"flags\":" + String(flags) + ",";
    json += "\"error\":" + String(sensorError ? "true" : "false") + ",";
    json += "\"capL\":" + String(tankCapacityLitres) + ",";
    json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
    json += "\"wifi\":\"" + mvs.getWiFiStatus() + "\",";
    json += "\"firebase\":" + String(firebaseReady ? "true" : "false") + ",";
    json += "\"apMode\":" + String(apMode) + ",";
    json += "\"apLeft\":" + String(apLeft) + ",";
    // v20.0.0: expose sensor type + last raw distance so installer can
    // verify which US sensor was detected without a serial cable.
    #if USE_ULTRASONIC
      json += "\"usType\":" + String(usSensorType) + ",";
      json += "\"usBaud\":" + String(usDypBaud) + ",";
      json += "\"usRawCm\":" + String(usRawDistance, 1) + ",";
      // 21.1.5: sensorConnected=false when >=15 consecutive empty bursts.
      // AP page shows red banner. Auto-clears on next successful ping.
      json += "\"sensorConnected\":" + String(usSensorOffline ? "false" : "true") + ",";
      json += "\"failBursts\":" + String(usConsecFailBursts) + ",";
      json += "\"usFilteredCm\":" + String(usFilteredDistance, 1);
    #else
      json += "\"usType\":0,\"usBaud\":0,\"usRawCm\":0,\"usFilteredCm\":0";
    #endif
    json += "}";
    srv->send(200, "application/json", json);
  });

  // Tank capacity (litres) — saved to NVS so litres can be shown locally
  mvs.addEndpoint("/setcapacity", []() {
    WebServer* srv = mvs.getServer();
    uint32_t c = srv->arg("c").toInt();
    if (c <= 1000000) {  // sanity cap at 1,000,000 L
      tankCapacityLitres = c;
      Preferences capPrefs;
      capPrefs.begin("senseflow", false);
      capPrefs.putUInt("capL", c);
      capPrefs.end();
      Serial.println("Tank capacity set to: " + String(c) + " L");
    }
    srv->sendHeader("Location", "/"); srv->send(302);
  });

  // User-assigned physical-identity label. Installer types this on the
  // AP page during setup to match this specific hardware to its physical
  // location ("3F West Flush" etc.). Saved to NVS + mirrored to Firebase
  // /info if cloud is up. Max 32 chars, plain text only.
  mvs.addEndpoint("/setusername", []() {
    WebServer* srv = mvs.getServer();
    String name = srv->arg("n");
    name.trim();
    if (name.length() > 32) name = name.substring(0, 32);
    userAssignedName = name;
    Preferences np;
    np.begin("senseflow", false);
    np.putString("userName", name);
    np.end();
    Serial.println("[NAME] User-assigned name set to: '" + name + "'");
    // Push to Firebase /info immediately so admin sees it without waiting
    // for the next heartbeat. Silently skipped when cloud unavailable —
    // next heartbeat will sync via updateDeviceInfo().
    if (firebaseHealthy) updateDeviceInfo(true);
    srv->sendHeader("Location", "/"); srv->send(302);
  });

  // AP mode selector: 0 = always on, 1 = 10-min auto-off
  mvs.addEndpoint("/setapmode", []() {
    WebServer* srv = mvs.getServer();
    uint8_t mode = (uint8_t)srv->arg("mode").toInt();
    if (mode > 1) mode = 1;
    apMode = mode;
    Preferences apPrefs;
    apPrefs.begin("senseflow", false);
    apPrefs.putUChar("apMode", apMode);
    apPrefs.end();
    Serial.printf("AP mode set to: %u (0=always, 1=10min)\n", apMode);

    if (apMode == 0) {
      // Always-on: bring AP back if it was off
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP(apName.c_str(), AP_PASSWORD);
      apTimerStart = 0;
      apTimerDeadline = 0;
      apTimerEnded = false;
      Serial.println("AP forced ON (always mode)");
    } else {
      // 10-min mode: start countdown only if STA already connected
      if (WiFi.status() == WL_CONNECTED && apTimerStart == 0) {
        apTimerStart = millis();
        apTimerDeadline = apTimerStart + AP_AUTO_OFF_MS;
        apTimerEnded = false;
        Serial.println("AP 10-min timer started");
      }
    }
    srv->sendHeader("Location", "/"); srv->send(302);
  });

  // Manual extend AP timer by 5 min (button on UI)
  mvs.addEndpoint("/extendap", []() {
    WebServer* srv = mvs.getServer();
    if (apMode == 1 && apTimerStart != 0 && !apTimerEnded) {
      apTimerDeadline += AP_EXTEND_MS;
      Serial.println("AP timer extended +5 min");
    }
    srv->sendHeader("Location", "/"); srv->send(302);
  });

  // Debug dump — read the credentials directly out of the same NVS
  // namespace MvsConnect uses so we can see EXACTLY what's stored.
  // BENCH DEBUG: password is printed in cleartext on purpose so we can
  // spot character-encoding / trailing-whitespace / locale issues. REPLACE
  // this with `<hidden, len=N>` before flashing production devices.
  {
    Preferences dbg;
    dbg.begin("mvswifi", true);   // read-only
    String dbgSsid = dbg.getString("ssid", "");
    String dbgPass = dbg.getString("password", "");
    bool dbgValid  = dbg.getBool("valid", false);
    dbg.end();
    Serial.printf("[WIFI-DBG] NVS namespace 'mvswifi' contents:\n");
    Serial.printf("[WIFI-DBG]   ssid     = '%s' (len=%u)\n", dbgSsid.c_str(), dbgSsid.length());
    Serial.printf("[WIFI-DBG]   password = '%s' (len=%u)\n", dbgPass.c_str(), dbgPass.length());
    Serial.printf("[WIFI-DBG]   valid    = %s\n", dbgValid ? "true" : "false");
    // Byte-level hex dump of both fields. If something is being stored
    // with stray UTF-8 BOM, NUL terminator, or unicode look-alike chars,
    // this will show it. Each byte in 2-digit hex with " " separator.
    Serial.printf("[WIFI-DBG]   ssid hex   ="); for (size_t i = 0; i < dbgSsid.length(); i++) Serial.printf(" %02X", (uint8_t)dbgSsid[i]); Serial.println();
    Serial.printf("[WIFI-DBG]   pass hex   ="); for (size_t i = 0; i < dbgPass.length(); i++) Serial.printf(" %02X", (uint8_t)dbgPass[i]); Serial.println();
    Serial.printf("[WIFI-DBG] WiFi.macAddress() = %s\n", WiFi.macAddress().c_str());
  }

  // Scan visible networks once at boot so we can see whether the saved
  // SSID is even in range, and what auth/RSSI it advertises. Short scan
  // (~2-3 sec) — only happens once per boot, safe to add.
  {
    Serial.println("[WIFI-DBG] Scanning visible networks...");
    int n = WiFi.scanNetworks(false, false, false, 200);
    Serial.printf("[WIFI-DBG] Found %d networks:\n", n);
    for (int i = 0; i < n; i++) {
      Serial.printf("[WIFI-DBG]   %2d: %-32s  RSSI=%d  ch=%d  enc=%d  bssid=%s\n",
                    i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i),
                    WiFi.channel(i), (int)WiFi.encryptionType(i),
                    WiFi.BSSIDstr(i).c_str());
    }
    WiFi.scanDelete();
  }

  // Non-blocking WiFi kick-off. AP is already up (WIFI_AP_STA mode +
  // softAP called earlier in setup). We fire WiFi.begin() and return
  // — the main loop polls status every 30 sec and either progresses
  // to Firebase init, or retries begin() if the SSID isn't reachable
  // (moved device / router changed).
  //
  // Setup MUST exit quickly. The old 90-sec blocking wait wedged the
  // whole device when the previous SSID was gone — serial went silent,
  // AP disappeared, only a power cycle recovered. See feedback memory
  // and this thread's discussion of that bug.
  //
  // NEVER call WiFi.disconnect(true) — that erases ESP-IDF WiFi config
  // AND drops the AP. Use disconnect(false) or omit the disconnect
  // entirely and let WiFi.begin() handle it internally.
  {
    Preferences wp;
    wp.begin("mvswifi", true);
    String savedSsid = wp.getString("ssid", "");
    String savedPass = wp.getString("password", "");
    bool   savedVal  = wp.getBool("valid", false);
    wp.end();

    if (savedSsid.length() > 0 && savedVal) {
      // v21.0.10: NO scan gate at boot. The MvsConnect library already
      // scans internally (see [WIFI-DBG] boot output). Doing another
      // scan here pushed total blocking time past 11 sec → Task WDT
      // fired → crash loop → device unusable. Field-verified in v21.0.9.
      //
      // Just kick begin() and let the main-loop reconnect logic handle
      // the "SSID missing" case (it does its OWN scan-gated retry every
      // 60 sec, safely, because by then WDT context is normal).
      Serial.printf("[BOOT] Kicking WiFi.begin('%s') non-blocking — main loop will monitor\n",
                    savedSsid.c_str());
      setLED(0, 0, 255);
      WiFi.begin(savedSsid.c_str(), savedPass.c_str());
    } else {
      Serial.println("[BOOT] No saved WiFi — AP mode ready for setup");
      setLED(255, 255, 255);
    }
  }

  // MvsOTA
  mvsota.begin(DEVICE_NAME, FIRMWARE_VERSION, FIRMWARE_CODE);

#if ENABLE_LOCAL_MQTT
  mqttClient.setBufferSize(512);
  mqttClient.setCallback(mqttCommandCallback);
#endif

  ledCycleStart = millis();
}

// ══════════════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // Feed watchdog — proves main loop is alive
  esp_task_wdt_reset();

  // Heap monitor — if we ever drop below 30 KB, restart cleanly before crash
  static unsigned long lastHeapCheck = 0;
  if (now - lastHeapCheck >= 300000) {   // 5 min
    lastHeapCheck = now;
    uint32_t freeHeap = ESP.getFreeHeap();
    Serial.printf("[HEAP] free=%u bytes\n", freeHeap);
    if (freeHeap < 30000) {
      Serial.println("[HEAP] Below 30 KB — restarting cleanly");
      delay(500);
      safeRestart();
    }
  }

  // Diagnostics — persist current uptime to NVS so the NEXT boot knows
  // how long this session ran before dying. GATED on diagnosticsLoggingActive
  // so devices with diagnostics OFF never write to NVS for diagnostics
  // (zero wear for free customers). Adaptive cadence when ON: every 60s
  // in the first 10 min (catches early-boot brownouts), then every 10 min.
  static unsigned long lastUptimeSave = 0;
  const unsigned long UPTIME_SAVE_EARLY  = 60UL  * 1000UL;
  const unsigned long UPTIME_SAVE_NORMAL = 600UL * 1000UL;
  unsigned long uptimeSaveInterval = (now < 10UL * 60UL * 1000UL)
                                     ? UPTIME_SAVE_EARLY
                                     : UPTIME_SAVE_NORMAL;
  if (diagnosticsLoggingActive && now - lastUptimeSave >= uptimeSaveInterval) {
    lastUptimeSave = now;
    persistCurrentUptime();
  }

  // Scheduled auto-restart (industrial practice): reboot after 7 days uptime
  // to clear any TLS/heap fragmentation. Skip if OTA in progress or a push
  // happened in the last 2 minutes (avoid interrupting active transfer).
  const unsigned long REBOOT_AFTER_MS = 7UL * 24UL * 60UL * 60UL * 1000UL;  // 7 days
  if (now >= REBOOT_AFTER_MS && !mvsota.isUpdating() &&
      (lastSuccessfulPush == 0 || (now - lastSuccessfulPush) > 120000)) {
    Serial.println("[REBOOT] Scheduled 7-day restart");
    if (firebaseReady) updateDeviceInfo(false);
    delay(500);
    safeRestart();
  }

  // ── Cloud-stale self-heal watchdog ──────────────────────────────────
  // Field devices have gone "half-alive" — SSID still broadcasting, LED
  // still cycling, main loop still feeding Task WDT — but AP page won't
  // load and cloud shows offline. Only a manual power cycle recovered.
  //
  // Root cause is almost certainly an internal Firebase library stall
  // (TLS retry loop or heap fragmentation preventing allocation) that
  // keeps calling delay() often enough to feed Task WDT but never makes
  // progress. Task WDT can't catch this because we ARE resetting it.
  //
  // Detection: if network is confirmed up but we haven't successfully
  // pushed to /live in > 20 min (4x the 5-min heartbeat), the Firebase
  // layer is stuck. Force a clean safeRestart() to recover — that's the
  // exact recipe that fixed the 2 stuck devices Vishal reported.
  //
  // Guard: only fires AFTER Firebase has been healthy at least once
  // (lastSuccessfulPush != 0). Never fires during OTA or the first
  // few minutes on a fresh boot.
  // 21.1.6: this 20-min stale-cloud safeRestart() watchdog is now
  // SUBSUMED by the 3-rung recovery ladder in driveRecoveryLadder(),
  // which fires on the same signal (Trigger B: no heartbeat in 30 min)
  // but tries softer recovery rungs BEFORE resorting to a full restart.
  // Kept here in commented-out form as historical reference — remove in
  // v21.2 once the ladder proves reliable in the field.
  //
  // const unsigned long CLOUD_STALE_MS = 20UL * 60UL * 1000UL;   // 20 min
  // static unsigned long lastCloudCheck = 0;
  // if (!mvsota.isUpdating() && now - lastCloudCheck >= 60000UL) {
  //   lastCloudCheck = now;
  //   if (lastSuccessfulPush != 0 &&
  //       WiFi.status() == WL_CONNECTED &&
  //       internetAvailable &&
  //       now - lastSuccessfulPush > CLOUD_STALE_MS) {
  //     unsigned long staleMin = (now - lastSuccessfulPush) / 60000UL;
  //     Serial.printf("[WATCHDOG] Cloud stale %lu min — self-restart\n", staleMin);
  //     delay(500);
  //     safeRestart();
  //   }
  // }
  //
  // Drive the new recovery ladder (cheap when not armed).
  driveRecoveryLadder();

  // MvsConnect always runs (AP mode web server)
  mvs.handle();
  if (!mvsota.isUpdating()) mvsota.handle();

  // AP auto-off REMOVED in 19.0.1 per requirement: AP page must be
  // permanently accessible so installer/user can always diagnose,
  // regardless of WiFi state. The apMode toggle in the UI is kept for
  // backward-compat but treated as always-on. WiFi.softAPdisconnect
  // and WiFi.mode(WIFI_STA) calls are gone from the runtime path.

  // Local MQTT — discovery, connect, publish (Phase 1)
#if ENABLE_LOCAL_MQTT
  mqttLoop();
#endif

  // Read sensors continuously
  #if USE_ULTRASONIC
    processUltrasonic();
  #else
    processDipSensors();
  #endif

  // Handle LED state machine
  handleLED();

  // mDNS handled by MvsConnect library (<deviceName>-mvstech.local)

  // Layered network state: WiFi → internet (DNS chain) → firebase
  // Each layer is checked every 30s. Failure in one stops the next from
  // wasting time (especially the slow TLS handshake on dead internet).
  bool wifiUp = (WiFi.status() == WL_CONNECTED);

  if (!wifiUp) {
    internetAvailable = false;
    firebaseHealthy   = false;
  } else if (now - lastInternetCheck > 30000) {
    lastInternetCheck = now;
    bool prevInternet = internetAvailable;
    internetAvailable = checkInternet();    // DNS chain, ~1-3s worst case
    if (internetAvailable && !prevInternet) {
      Serial.println("[NET] Internet restored");
      // Recover Firebase only if it was previously unhealthy
      if (!firebaseHealthy) {
        Serial.println("[FB] Internet back — attempting recovery");
        if (!firebaseReady) {
          initFirebase();
        } else {
          firebaseHealthy = true;           // give it another chance
          consecutiveFailCount = 0;
        }
      }
    } else if (!internetAvailable && prevInternet) {
      Serial.println("[NET] Internet lost — gating Firebase");
      firebaseHealthy = false;
    }
  }

  // Firebase operations — only when WiFi + internet are confirmed up
  if (wifiUp && internetAvailable) {
    checkFirebaseReady();

    if (firebaseReady && firebaseHealthy) {
      // Change-driven data push (only fires after confirmation gates)
      if (hasDataChanged()) {
#if USE_ULTRASONIC
        // Layer 3 rate limit — cap /live to 1 push per US_LIVE_PUSH_MIN_GAP_MS.
        // Real level changes take minutes; this only bites when Layers 1+2
        // have (rarely) let noise through, so we don't spam cloud.
        // First push (usLastLivePushMs == 0) always passes.
        bool liveRateOk = (usLastLivePushMs == 0) ||
                          (now - usLastLivePushMs >= US_LIVE_PUSH_MIN_GAP_MS);
        if (!liveRateOk) {
          Serial.printf("[US] /live push RATE-LIMITED (%lu ms since last, need %u)\n",
                        now - usLastLivePushMs, (unsigned)US_LIVE_PUSH_MIN_GAP_MS);
        } else {
          Serial.printf("Data changed: bits=%d pct=%d flags=%d → pushing\n",
                        sensorBits, confirmedPct, flags);
          if (pushLiveData()) {
            usLastLivePushMs = now;
            updateDeviceInfo(true);
            writeHistory();
            pushNotifyTrigger(confirmedPct, flags);
          }
          handleLED();
        }
#else
        Serial.printf("Data changed: bits=%d pct=%d flags=%d → pushing\n", sensorBits, confirmedPct, flags);
        if (pushLiveData()) {
          updateDeviceInfo(true);
          writeHistory();
          pushNotifyTrigger(confirmedPct, flags);
        }
        handleLED();
#endif
      }

      // Idle 1-hour fallback — if no /history entry for 1 hour, push the
      // last-confirmed value so analytics chart isn't blank.
      writeHistoryIdleIfDue();
      handleLED();

      // NTP sync (first boot + every 24h)
      syncNTPIfDue();

      // OTA trigger poll (every 30 s, NTP-time-gated)
      if (now - lastOtaCheckAt >= OTA_CHECK_INTERVAL_MS) {
        lastOtaCheckAt = now;
        checkOtaTrigger();
        handleLED();
      }

      // 5-minute heartbeat — pushes last-confirmed-stable value, never the
      // possibly-glitchy current reading.
      if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        lastHeartbeat = now;
        Serial.println("Heartbeat — pushing last-confirmed value");
        // 21.1.6: capture heartbeat success as our proof-of-life for
        // Trigger B in the recovery ladder. If this returns true, the
        // /live push landed → cloud sees us → 30-min silent timer resets.
        if (pushLiveDataHeartbeat()) {
          lastHeartbeatSuccessMs = millis();
        }
        handleLED();
        updateDeviceInfo(true);
        handleLED();
      }

      // Check commands every 5 seconds
      if (now - lastCommandCheck >= COMMAND_CHECK_INTERVAL) {
        lastCommandCheck = now;
        checkCommands();
        handleLED();
      }

      // Check config every 60 seconds (analyticsOn / notifyOn / diagnosticsOn
      // flags). Was 30s — bumped to 60s to halve RTDB config polling
      // bandwidth. These flags change once in a blue moon (admin toggles),
      // 60s response time is fine. Combined with the command-poll bump,
      // baseline RTDB downloads drop ~75% per device.
      static unsigned long lastConfigCheck = 0;
      if (now - lastConfigCheck >= 60000) {
        lastConfigCheck = now;
        checkConfig();
      }
    }
  }

  if (!wifiUp) {
    // Clear manual WiFi flag after 30s
    if (manualWiFiInProgress && (now - manualWiFiStart > 30000)) {
      manualWiFiInProgress = false;
    }

    // Status-gated reconnect (19.0.2 fix for AP flapping).
    //
    // Previous 19.0.1 code fired disconnect(false)+begin() every 30 sec
    // regardless of what ESP-IDF was doing. In the field this produced:
    //   E (nnn) wifi:sta is connecting, return error
    // AND briefly flapped the AP, so MvsConnect app couldn't stay
    // connected. Root cause: STA state machine was still mid-association
    // from the previous begin(), and the fresh begin() forced the radio
    // to reconfigure — kicking the AP.
    //
    // Fix: only call begin() when ESP-IDF has DEFINITIVELY given up
    // (status is CONNECT_FAILED / NO_SSID_AVAIL / CONNECTION_LOST). If
    // it's still mid-association (IDLE_STATUS / DISCONNECTED), skip
    // this cycle. Also drop the pre-disconnect entirely — WiFi.begin()
    // handles cleanup internally when status is a definitive fail. And
    // widen the interval from 30→60 sec so we don't hammer.
    static unsigned long lastReconnect = 0;
    const unsigned long RECONNECT_INTERVAL_MS = 60000UL;
    if (!manualWiFiInProgress && (now - lastReconnect > RECONNECT_INTERVAL_MS)) {
      lastReconnect = now;

      wl_status_t s = WiFi.status();
      bool safeToBegin = (s == WL_CONNECT_FAILED  ||
                          s == WL_NO_SSID_AVAIL   ||
                          s == WL_CONNECTION_LOST ||
                          s == WL_DISCONNECTED);
      if (!safeToBegin) {
        Serial.printf("[RECONNECT] skip: WiFi.status=%d still in-progress\n", (int)s);
      } else {
        Preferences wp;
        wp.begin("mvswifi", true);
        String savedSsid = wp.getString("ssid", "");
        String savedPass = wp.getString("password", "");
        bool   savedVal  = wp.getBool("valid", false);
        wp.end();
        if (savedSsid.length() > 0 && savedVal) {
          // v21.0.8: SCAN GATE. Don't waste a WiFi.begin() call (which
          // silences the AP for 30+ sec) if the saved SSID isn't even
          // visible. Scan first — only fire begin() when router is
          // actually in range. Scan itself is ~2 sec.
          if (isSsidVisible(savedSsid)) {
            Serial.printf("[RECONNECT] status=%d, SSID visible — retrying '%s'\n",
                          (int)s, savedSsid.c_str());
            WiFi.begin(savedSsid.c_str(), savedPass.c_str());
          } else {
            Serial.printf("[RECONNECT] status=%d, SSID '%s' not visible — staying AP-only\n",
                          (int)s, savedSsid.c_str());
          }
        }
      }
    }
  }

  // Serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleSerialCommand(cmd);
  }
}

// ══════════════════════════════════════════════════
//  SERIAL COMMANDS
// ══════════════════════════════════════════════════

void handleSerialCommand(String cmd) {
  cmd.toUpperCase();

  if (cmd == "STATUS" || cmd == "S") {
    Serial.println("\n--- Device Status ---");
    Serial.println("Code:       " + deviceCode);
    Serial.println("WiFi:       " + mvs.getWiFiStatus());
    Serial.println("IP:         " + WiFi.localIP().toString());
    Serial.println("RSSI:       " + String(WiFi.RSSI()) + " dBm");
    Serial.println("Firebase:   " + String(firebaseReady ? "Ready" : "Not ready"));
    Serial.println("Level:      " + String(confirmedPct) + "%");
    Serial.println("SensorBits: " + String(sensorBits, BIN));
    Serial.println("Flags:      0x" + String(flags, HEX));
    Serial.println("Error:      " + String(sensorError ? "YES" : "No"));
    Serial.println("Uptime:     " + String(millis() / 1000) + "s");
    Serial.println("Free Heap:  " + String(ESP.getFreeHeap()));
    Serial.println("Firmware:   " + String(FIRMWARE_VERSION));
    Serial.printf ("NTP synced: %s (epoch=%u)\n", ntpSynced ? "yes" : "no", nowEpoch());
    Serial.printf ("firstBootAt:   %u\n", firstBootAt);
    Serial.printf ("lastUpdatedAt: %u\n", lastUpdatedAt);
    Serial.println("--------------------\n");
  }
  else if (cmd == "ADMIN") {
    printRegistrationInfo();
  }
  else if (cmd == "FIREBASE" || cmd == "FB") {
    Serial.println("\n--- Firebase Debug ---");
    Serial.println("WiFi:       " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Not connected"));
    Serial.println("Ready:      " + String(Firebase.ready() ? "Yes" : "No"));
    Serial.println("firebaseReady var: " + String(firebaseReady ? "Yes" : "No"));
    Serial.println("Attempting initFirebase now...");
    initFirebase();
    Serial.println("After init, ready: " + String(Firebase.ready() ? "Yes" : "No"));
    Serial.println("----------------------\n");
  }
  else if (cmd == "RESTART" || cmd == "RESET") {
    Serial.println("Restarting...");
    delay(500);
    safeRestart();
  }
  else if (cmd == "RESET_WIFI") {
    Serial.println("Clearing WiFi credentials...");
    mvs.clearSavedWiFi();
    delay(500);
    safeRestart();
  }
  else if (cmd == "AP_ON") {
    Serial.println("Re-enabling AP (clears 10-min timer)");
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(apName.c_str(), AP_PASSWORD);
    if (apMode == 1) {
      apTimerStart = millis();
      apTimerDeadline = apTimerStart + AP_AUTO_OFF_MS;
    }
    apTimerEnded = false;
    Serial.println("AP re-enabled");
  }
  else if (cmd.startsWith("WIFI ")) {
    // Format: WIFI ssid password
    // Example: WIFI MyNetwork MyPassword123
    // Example: WIFI MyNetwork (no password for open networks)
    String params = cmd.substring(5);
    int spaceIdx = params.indexOf(' ');
    String ssid, pass;
    if (spaceIdx > 0) {
      ssid = params.substring(0, spaceIdx);
      pass = params.substring(spaceIdx + 1);
    } else {
      ssid = params;
      pass = "";
    }
    ssid.trim(); pass.trim();
    if (ssid.length() == 0) {
      Serial.println("Usage: WIFI <ssid> <password>");
      return;
    }
    Serial.println("Setting WiFi: " + ssid);
    Preferences wifiPrefs;
    wifiPrefs.begin("mvswifi", false);
    wifiPrefs.putString("ssid", ssid);
    wifiPrefs.putString("password", pass);
    wifiPrefs.putBool("valid", true);
    wifiPrefs.end();
    Serial.println("Credentials saved. Restarting...");
    delay(500);
    safeRestart();
  }
  else if (cmd == "BOOTLOG") {
    dumpBootLogToSerial();
  }
  else if (cmd == "DIAG") {
    Serial.println("\n--- LIVE DIAGNOSTICS ---");
    Serial.printf("Uptime:              %us\n", (uint32_t)(millis() / 1000));
    Serial.printf("Free heap:           %u bytes\n", ESP.getFreeHeap());
    Serial.printf("RSSI:                %d dBm\n", WiFi.RSSI());
    Serial.printf("Internet available:  %s\n", internetAvailable ? "yes" : "no");
    Serial.printf("Firebase healthy:    %s\n", firebaseHealthy ? "yes" : "no");
    Serial.printf("Push fails (cons):   %d\n", consecutiveFailCount);
    Serial.printf("NTP synced:          %s (last sync %lu ms ago)\n",
                  ntpSynced ? "yes" : "no",
                  ntpSynced ? (millis() - lastNtpSyncAt) : 0);
    Serial.printf("notifyOn flag:       %s\n", notifyOn ? "ON" : "OFF");
    Serial.printf("diagnosticsOn flag:  %s\n", diagnosticsOn ? "ON" : "OFF");
    Serial.println("------------------------\n");
  }
  else if (cmd == "DIAG CLEAR") {
    clearBootLog();
    Serial.println("Boot log cleared (local NVS only — RTDB not touched).");
  }
  else if (cmd == "WIFI_DBG" || cmd == "WIFIDBG") {
    Preferences dbg;
    dbg.begin("mvswifi", true);
    String dbgSsid = dbg.getString("ssid", "");
    String dbgPass = dbg.getString("password", "");
    bool dbgValid  = dbg.getBool("valid", false);
    dbg.end();
    Serial.println("\n--- WIFI NVS DUMP ---");
    Serial.printf("ssid     = '%s' (len=%u)\n", dbgSsid.c_str(), dbgSsid.length());
    Serial.printf("password = '%s' (len=%u)\n", dbgPass.c_str(), dbgPass.length());
    Serial.printf("valid    = %s\n", dbgValid ? "true" : "false");
    Serial.printf("MAC      = %s\n", WiFi.macAddress().c_str());
    Serial.printf("ssid hex ="); for (size_t i = 0; i < dbgSsid.length(); i++) Serial.printf(" %02X", (uint8_t)dbgSsid[i]); Serial.println();
    Serial.printf("pass hex ="); for (size_t i = 0; i < dbgPass.length(); i++) Serial.printf(" %02X", (uint8_t)dbgPass[i]); Serial.println();
    Serial.printf("Status   = %d (3=connected)\n", WiFi.status());
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("SSID     = '%s'\n", WiFi.SSID().c_str());
      Serial.printf("IP       = %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("RSSI     = %d dBm\n", WiFi.RSSI());
    }
    Serial.println("Scanning visible networks...");
    int n = WiFi.scanNetworks(false, false, false, 200);
    Serial.printf("Found %d networks:\n", n);
    for (int i = 0; i < n; i++) {
      Serial.printf("  %2d: %-32s  RSSI=%d  ch=%d  enc=%d\n",
                    i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i),
                    WiFi.channel(i), (int)WiFi.encryptionType(i));
    }
    WiFi.scanDelete();
    Serial.println("---------------------\n");
  }
  else if (cmd == "HELP") {
    Serial.println("\nCommands: STATUS, ADMIN, FIREBASE, RESTART, RESET_WIFI, WIFI <ssid> <pass>, BOOTLOG, DIAG, DIAG CLEAR, WIFI_DBG, HELP\n");
  }
}
