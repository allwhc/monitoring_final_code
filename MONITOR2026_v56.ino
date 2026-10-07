/*
 * SenseFlow Tank Monitor - single tank, single sensor
 * ====================================================
 * One firmware for every connection type (selected in Settings, saved in EEPROM):
 *   Wired          - ultrasonic sensor wired straight to this monitor
 *   Long Range     - LoRa radio (up to several km)
 *   NearbyConnect  - direct wireless link to a nearby sensor (~200 m, no router)
 *   Home Network   - sensor and monitor on the same home WiFi (monitor polls the sensor)
 *   Auto           - listen on every wireless link this hardware has
 *
 * Every link delivers the same micro-payload:
 *   {"i":"US3847","s":51,"c":102,"d":143}   data      (d = distance in cm)
 *   {"i":"US3847","s":51,"c":103,"h":1}     heartbeat (sensor alive, reading failed)
 * i = node ID, s = session (new on every sensor reboot), c = packet counter.
 *
 * The display keeps showing the last good reading when only heartbeats arrive,
 * and logs a 24h history (sequence numbered, survives reboots) for the chart.
 */

#include <ArduinoJson.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <LoRa.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <time.h>
#include <math.h>
#include <mvsota_esp32.h>

#define FIRMWARE_VERSION "56.0.0"
#define FIRMWARE_CODE "SF-MN-2026-9867"

// Debug mode: 1 prints internal link names (LoRa / ESP-NOW / mDNS) on Serial
#define DEBUG_MODE 0

// Ultrasonic sensor pins (Wired connection type)
#define TRIG_PIN 25
#define ECHO_PIN 34  // Input-only pin: fit a 10k pull-down so an unplugged sensor reads "no echo"

// LoRa pins for ESP32 (Long Range connection type)
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 26

// Hardware pins (FIXED - No conflicts)
#define BUZZER_PIN 4   // Changed back to 4 - safer pin, no boot strapping issues
#define BUTTON_PIN 33  // Changed from 19 to 33
#define SDA_PIN 21
#define SCL_PIN 22

// OLED I2C address - auto-detected at boot, this one is tried first
#define OLED_I2C_ADDRESS 0x3C      // Most common
#define OLED_I2C_ADDRESS_ALT 0x3D  // Tried if the first address does not answer

// LoRa Configuration Constants
// CRITICAL: These values MUST match transmitter configuration
#define LORA_FREQUENCY 433E6
#define LORA_SPREADING_FACTOR 10
#define LORA_BANDWIDTH 125E3
#define LORA_CODING_RATE 8
#define LORA_PREAMBLE 8
#define LORA_SYNC_WORD 0x12

// Ultrasonic sampling / MAD filtering constants
#define SAMPLE_COUNT 15
#define MIN_VALID_SAMPLES 10
#define READING_DELAY 50
#define MAD_MULTIPLIER 2.5
#define SENSOR_READ_INTERVAL_MS 2000
#define NO_ECHO_ABORT_SAMPLES 3        // Stop a read cycle early if the first pulses all time out
#define WIRED_OFFLINE_CYCLES 3         // Cycles with no echo at all before the wired sensor shows OFFLINE

// Link supervision
#define WIRELESS_SILENT_MS 60000UL     // No packet at all for this long -> display shows OFFLINE
#define HN_POLL_INTERVAL_MS 5000UL     // Home Network: poll the sensor every 5s
#define HN_POLL_BACKOFF_MS 15000UL     // ...slower while the sensor is not answering
#define HN_HTTP_TIMEOUT_MS 1500
#define HN_FAILS_BEFORE_RESOLVE 3      // Look the sensor up on the network again after this many failed polls
#define LORA_REINIT_SILENT_MS 120000UL // Re-initialise LoRa if nothing arrives for 2 minutes

// Connection types (stored in EEPROM)
enum LinkType : uint8_t {
  LINK_WIRED = 0,
  LINK_LONG_RANGE = 1,   // LoRa
  LINK_NEARBY = 2,       // ESP-NOW
  LINK_HOME_NETWORK = 3, // mDNS discovery + HTTP polling (also accepts NearbyConnect as backup)
  LINK_AUTO = 4
};
#define LINK_TYPE_COUNT 5

// Which link actually delivered a message
enum LinkVia : uint8_t {
  VIA_NONE = 0,
  VIA_WIRED = 1,
  VIA_LORA = 2,
  VIA_NEARBY = 3,
  VIA_HOME_NET = 4
};

// Display state of the sensor link
enum LinkState : uint8_t {
  LS_WAITING = 0,   // Nothing heard since boot
  LS_LIVE = 1,      // Fresh readings arriving
  LS_HEARTBEAT = 2, // Sensor alive but its reading failed - showing last known value
  LS_OFFLINE = 3    // Sensor silent - showing last known value
};

// History logging
#define HISTORY_FILE "/history.bin"
#define HISTORY_SLOTS 96
#define TREND_SAMPLE_MS 300000UL  // Trend ring sample every 5 min
#define TREND_SLOTS 13            // 13 samples = last hour
#define TREND_DEADBAND 1.0        // +/-1% shows a flat arrow
#define SCREEN_ROTATE_MS 5000UL
#define CHART_POINTS 12

// OLED using U8g2 library for SH1106 (1.3" display)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /*reset=*/ U8X8_PIN_NONE, /*clock=*/ 22, /*data=*/ 21);

// WebServer - Port 7689 to match app expectations
WebServer server(7689);

// App Access Security - User-Agent check
#define USER_AGENT_CHECK "MVStech7689"
IPAddress apIP(192, 168, 4, 1);

// MVS OTA instance
MvsOTA mvsota;

// Configuration constants
#define SENSOR_BLIND_ZONE_CM 21
#define SENSOR_MAX_RANGE_CM 450
#define SENSOR_OFFLINE_TIMEOUT_MINUTES 5
#define DEFAULT_SNOOZE_MINUTES 5
#define MIN_PERCENTAGE_GAP 5

// EEPROM addresses
#define EEPROM_SIZE 512
#define EEPROM_CONFIGURED_FLAG 0
#define EEPROM_DEVICE_NAME 1         // 32 bytes
#define EEPROM_TANK_CONFIG_START 33  // Single tank config (25 bytes)
#define EEPROM_GENERAL_SETTINGS 119  // 10 bytes
#define EEPROM_ALARM_ACK_START 129   // Alarm acknowledgment data
#define EEPROM_WIFI_CONFIGURED 149   // WiFi configuration (1 byte flag)
#define EEPROM_WIFI_SSID 150         // WiFi SSID (32 bytes)
#define EEPROM_WIFI_PASSWORD 182     // WiFi password (64 bytes)
#define EEPROM_OTA_CONFIGURED 246    // OTA configuration (1 byte flag)
#define EEPROM_OTA_ENABLED 247       // OTA enabled flag (1 byte)
#define EEPROM_OTA_HOSTNAME 248      // OTA hostname (32 bytes)
#define EEPROM_GLOBAL_MUTE 280       // Global buzzer mute flag (1 byte)
// Tank node ID lives in bytes 0-7 of the tank config block (same layout as the old 2-tank wireless monitor)
#define EEPROM_LINK_MAGIC 300        // Link settings valid flag (1 byte)
#define EEPROM_LINK_TYPE 301         // Connection type (1 byte)
#define EEPROM_LOG_THRESHOLD 302     // History: log on % change (5 or 10)
#define EEPROM_LOG_INTERVAL 303      // History: periodic log interval in minutes (30 or 60)
#define MAGIC_NUMBER 0xAB
#define LINK_MAGIC 0xA7

// Function prototypes
String getStringInput(const String& prompt, int maxLength, bool allowEmpty = false);
int getIntInput(const String& prompt, int minVal, int maxVal);
bool validatePercentages(uint8_t lowPercent, uint8_t highPercent);

// Screen management system
struct ScreenManager {
  int currentScreen;
  int totalScreens;
  unsigned long lastUpdate;
  bool manualMode;
  unsigned long manualTimeout;
  bool showSystemInfo;
  unsigned long systemInfoTimeout;
};
ScreenManager screenMgr = {0, 0, 0, false, 0, false, 0};

// Alarm acknowledgment tracking structure
struct AlarmAckData {
  bool tankLowThresholdAck;
  bool tankHighThresholdAck;
  bool tankOfflineAck;
  bool isValid;
};

// Tank configuration structure
struct TankConfig {
  char nodeID[9];           // Paired wireless sensor (e.g. "US3847"), empty for Wired
  char customName[9];
  uint16_t height;
  uint32_t volume;          // Tank volume in litres (0-99999)
  uint8_t highAlarmPercent;
  uint8_t lowAlarmPercent;
  bool enabled;
};

// General settings structure
struct GeneralSettings {
  uint8_t acknowledgeTimeout;
  uint8_t offlineTimeoutMinutes;
  bool buzzerEnabled;
};

// Runtime sensor data structure
struct SensorData {
  float currentLevel;
  float currentPercent;
  unsigned long lastUpdate;    // Last sign of life (data OR heartbeat) - drives the offline alarm
  bool isOnline;               // Heard within the offline alarm timeout
  bool hasData;
  float minLevel;
  float maxLevel;
  float minPercent;
  float maxPercent;
  unsigned long lastDataTime;  // Last valid reading
  float lastDistance;          // Last valid sensor distance (cm)
  bool lastWasHeartbeat;       // Most recent message was a heartbeat
  bool everHeard;
  int rssi;                    // Signal of the last message (dBm, 0 = n/a)
  uint8_t via;                 // LinkVia of the last message
  uint32_t lastSession;
  uint32_t lastCounter;
  uint32_t packets;
  uint32_t duplicates;
  uint32_t heartbeats;
};

// History record (12 bytes) - one slot per sequence number in a circular LittleFS file
struct HistRec {
  uint32_t seq;      // Monotonic sequence number (0 = empty slot); survives reboots
  uint32_t epoch;    // Unix time if NTP was available, else 0
  uint16_t pctX10;   // Percent x10
  uint16_t levelX10; // Water level cm x10
};

// One decoded sensor message, whichever link it came from
struct SensorMsg {
  char id[12];
  uint32_t session;
  uint32_t counter;
  bool hasDistance;
  float distance;
  int rssi;
};

// Scan results (sensor discovery)
struct ScanEntry {
  char id[12];
  int16_t rssi;
  uint8_t via;
  IPAddress ip;
};
#define MAX_SCAN_RESULTS 8

// Global configuration variables
String deviceName = "monitor";
String fullDeviceName = "monitor_mvstech";
TankConfig tankConfig;
GeneralSettings generalSettings = {5, SENSOR_OFFLINE_TIMEOUT_MINUTES, true};
AlarmAckData alarmAck;

// WiFi variables
String wifiSSID = "";
String wifiPassword = "";
bool wifiConfigured = false;
bool wifiStationMode = false;

// OTA variables
bool otaEnabled = true;  // OTA enabled by default
String otaHostname = "";  // Will be auto-generated from device name if empty

// Global mute variable (persistent in EEPROM)
bool globalBuzzerMute = false;  // When true, buzzer is silenced but visual alarms remain

// OLED display availability flag (allows system to work without display)
bool oledAvailable = false;  // Set to true only if OLED is detected and initialized

// Runtime data
SensorData tankData;

// Connection settings
LinkType linkType = LINK_AUTO;
bool loraPresent = false;        // Long Range module detected at boot
bool loraActive = false;
bool nearbyActive = false;
unsigned long lastLoRaPacket = 0;
int wiredNoEchoCycles = 0;

// Home Network polling
IPAddress hnSensorIP;
bool hnSensorIPValid = false;
int hnFailCount = 0;
unsigned long hnLastPoll = 0;
unsigned long hnLastResolve = 0;

// NearbyConnect receive queue (filled in the WiFi task, drained in loop)
#define NEARBY_QUEUE_LEN 4
#define NEARBY_MAX_LEN 120
struct NearbyMsg {
  uint8_t len;
  int8_t rssi;
  char data[NEARBY_MAX_LEN];
};
NearbyMsg nearbyQueue[NEARBY_QUEUE_LEN];
volatile uint8_t nearbyHead = 0;
volatile uint8_t nearbyTail = 0;
const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Sensor scan
ScanEntry scanResults[MAX_SCAN_RESULTS];
uint8_t scanCount = 0;
bool scanActive = false;
unsigned long scanStartTime = 0;
#define SCAN_DURATION_MS 20000

// History logging
uint8_t logThresholdPercent = 10;  // Log when the level moves this much (5 or 10)
uint8_t logIntervalMinutes = 60;   // ...or at least this often (30 or 60)
HistRec histSlots[HISTORY_SLOTS];
uint32_t histNextSeq = 1;
bool historyReady = false;
bool hasLoggedThisBoot = false;
float lastLoggedPercent = -1;
unsigned long lastLogTime = 0;

// Trend (last hour, RAM only)
float trendRing[TREND_SLOTS];
uint8_t trendCount = 0;
uint8_t trendHead = 0;
unsigned long lastTrendSample = 0;

// NTP
bool ntpStarted = false;

// Alarm system variables
bool alarmActive = false;
bool allAlarmsAcknowledged = false;
unsigned long alarmStartTime = 0;
unsigned long lastBuzzerToggle = 0;
bool buzzerState = false;

// Button handling variables
bool lastButtonState = HIGH;
unsigned long buttonPressTime = 0;
bool longPressProcessed = false;
int buttonPressCount = 0;
unsigned long lastButtonPress = 0;
#define BUTTON_DEBOUNCE 50
#define LONG_PRESS_TIME 3000
#define DOUBLE_PRESS_WINDOW 500
#define TRIPLE_PRESS_COUNT 3

// System state
bool isConfigured = false;
bool apModeActive = false;

// ===========================================
// WIFI CONFIGURATION FUNCTIONS (SIMPLIFIED)
// ===========================================

void loadWiFiCredentials() {
  if (EEPROM.read(EEPROM_WIFI_CONFIGURED) == MAGIC_NUMBER) {
    wifiConfigured = true;

    wifiSSID = "";
    for (int i = 0; i < 32; i++) {
      char c = EEPROM.read(EEPROM_WIFI_SSID + i);
      if (c == 0) break;
      wifiSSID += c;
    }

    wifiPassword = "";
    for (int i = 0; i < 64; i++) {
      char c = EEPROM.read(EEPROM_WIFI_PASSWORD + i);
      if (c == 0) break;
      wifiPassword += c;
    }

    Serial.println("WiFi credentials loaded from EEPROM");
    Serial.print("SSID: ");
    Serial.println(wifiSSID);
  } else {
    wifiConfigured = false;
    Serial.println("No WiFi credentials in EEPROM");
  }
}

void saveWiFiCredentials() {
  EEPROM.write(EEPROM_WIFI_CONFIGURED, MAGIC_NUMBER);

  for (int i = 0; i < 32; i++) {
    if (i < wifiSSID.length()) {
      EEPROM.write(EEPROM_WIFI_SSID + i, wifiSSID[i]);
    } else {
      EEPROM.write(EEPROM_WIFI_SSID + i, 0);
    }
  }

  for (int i = 0; i < 64; i++) {
    if (i < wifiPassword.length()) {
      EEPROM.write(EEPROM_WIFI_PASSWORD + i, wifiPassword[i]);
    } else {
      EEPROM.write(EEPROM_WIFI_PASSWORD + i, 0);
    }
  }

  EEPROM.commit();
  wifiConfigured = true;
  Serial.println("WiFi credentials saved to EEPROM");
}

// ===========================================
// OTA CONFIGURATION FUNCTIONS
// ===========================================

void loadOTAConfig() {
  if (EEPROM.read(EEPROM_OTA_CONFIGURED) == MAGIC_NUMBER) {
    otaEnabled = EEPROM.read(EEPROM_OTA_ENABLED);

    otaHostname = "";
    for (int i = 0; i < 32; i++) {
      char c = EEPROM.read(EEPROM_OTA_HOSTNAME + i);
      if (c == 0) break;
      otaHostname += c;
    }

    Serial.println("OTA configuration loaded from EEPROM");
    Serial.print("OTA Enabled: ");
    Serial.println(otaEnabled ? "Yes" : "No");
    Serial.print("OTA Hostname: ");
    Serial.println(otaHostname.length() > 0 ? otaHostname : "(auto-generated)");
  } else {
    otaEnabled = true;  // Default enabled
    otaHostname = "";   // Auto-generate from device name
    Serial.println("No OTA configuration in EEPROM, using defaults");
  }
}

void saveOTAConfig() {
  EEPROM.write(EEPROM_OTA_CONFIGURED, MAGIC_NUMBER);
  EEPROM.write(EEPROM_OTA_ENABLED, otaEnabled);

  for (int i = 0; i < 32; i++) {
    if (i < otaHostname.length()) {
      EEPROM.write(EEPROM_OTA_HOSTNAME + i, otaHostname[i]);
    } else {
      EEPROM.write(EEPROM_OTA_HOSTNAME + i, 0);
    }
  }

  EEPROM.commit();
  Serial.println("OTA configuration saved to EEPROM");
}

// ===========================================
// GLOBAL MUTE CONFIGURATION FUNCTIONS
// ===========================================

void loadGlobalMuteConfig() {
  globalBuzzerMute = EEPROM.read(EEPROM_GLOBAL_MUTE);

  Serial.print("Global Buzzer Mute: ");
  Serial.println(globalBuzzerMute ? "ENABLED (Buzzer Silenced)" : "DISABLED (Buzzer Active)");
}

void saveGlobalMuteConfig() {
  EEPROM.write(EEPROM_GLOBAL_MUTE, globalBuzzerMute ? 1 : 0);
  EEPROM.commit();

  Serial.print("Global mute saved: ");
  Serial.println(globalBuzzerMute ? "MUTED" : "ACTIVE");
}

void toggleGlobalMute() {
  globalBuzzerMute = !globalBuzzerMute;
  saveGlobalMuteConfig();

  Serial.println("================================");
  Serial.print("BUZZER MUTE: ");
  Serial.println(globalBuzzerMute ? "ENABLED" : "DISABLED");
  Serial.println("================================");

  // Show feedback on OLED (only if available)
  if (oledAvailable) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_10x20_tr);
    if (globalBuzzerMute) {
      u8g2.drawStr(10, 30, "BUZZER");
      u8g2.drawStr(20, 55, "MUTED");
    } else {
      u8g2.drawStr(10, 30, "BUZZER");
      u8g2.drawStr(15, 55, "ACTIVE");
    }
    u8g2.sendBuffer();
    delay(2000);
  }
}

// SIMPLIFIED WiFi configuration function
void configureWiFiTerminal() {
  Serial.println("\n=== WIFI CONFIGURATION ===");
  Serial.print("Current Status: ");
  Serial.println(wifiStationMode ? ("Connected to " + wifiSSID) : "Not connected");
  Serial.println("(Type 'CANCEL' to exit configuration)");

  String newSSID = getStringInput("Enter WiFi SSID: ", 31, false);
  if (newSSID.equalsIgnoreCase("CANCEL")) {
    Serial.println("WiFi configuration cancelled.");
    return;
  }

  String newPassword = getStringInput("Enter WiFi Password (leave empty for open): ", 63, true);
  if (newPassword.equalsIgnoreCase("CANCEL")) {
    Serial.println("WiFi configuration cancelled.");
    return;
  }

  wifiSSID = newSSID;
  wifiPassword = newPassword;
  saveWiFiCredentials();

  Serial.println("Testing connection...");
  if (connectToWiFi()) {
    Serial.println("WiFi configuration successful!");
  } else {
    Serial.println("Connection failed. Credentials saved but not connected.");
  }
}

bool connectToWiFi() {
  if (!wifiConfigured || wifiSSID.length() == 0) {
    Serial.println("No WiFi credentials configured");
    return false;
  }

  Serial.print("Connecting to WiFi: ");
  Serial.println(wifiSSID);

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiStationMode = true;
    Serial.println("WiFi connected successfully!");
    Serial.print("Station IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    return true;
  } else {
    Serial.println("WiFi connection failed");
    wifiStationMode = false;
    WiFi.mode(WIFI_AP);
    return false;
  }
}

// ===========================================
// OTA (Over-The-Air) UPDATE FUNCTIONS
// ===========================================

void setupOTA() {
  // Check if OTA is enabled
  if (!otaEnabled) {
    Serial.println("OTA: Disabled by user configuration");
    return;
  }

  if (!wifiStationMode) {
    Serial.println("OTA: WiFi station mode not available");
    return;
  }

  // Set OTA hostname - use custom or auto-generate
  String hostname;
  if (otaHostname.length() > 0) {
    hostname = otaHostname;
  } else {
    // Auto-generate from device name
    hostname = deviceName + "_mvstech";
    hostname.replace(" ", "-");
    hostname.replace("_", "-");
    hostname.replace(".", "-");
    hostname.toLowerCase();
  }
  ArduinoOTA.setHostname(hostname.c_str());
  Serial.print("OTA: Hostname set to: ");
  Serial.println(hostname);

  // Set OTA password (optional - use device name for security)
  ArduinoOTA.setPassword("mvstech9867");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);

    // Show OTA progress on OLED (only if available)
    if (oledAvailable) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0, 15, "OTA Update");
      u8g2.drawStr(0, 30, "Starting...");
      u8g2.sendBuffer();
    }
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    if (oledAvailable) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0, 15, "OTA Update");
      u8g2.drawStr(0, 30, "Complete!");
      u8g2.drawStr(0, 45, "Restarting...");
      u8g2.sendBuffer();
    }
    delay(1000);
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));

    // Show progress on OLED (only if available)
    if (oledAvailable) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0, 15, "OTA Update");

      unsigned int percent = (progress / (total / 100));
      char progressStr[20];
      sprintf(progressStr, "Progress: %u%%", percent);
      u8g2.drawStr(0, 30, progressStr);

      // Simple progress bar
      int barWidth = (128 * percent) / 100;
      u8g2.drawFrame(0, 40, 128, 10);
      u8g2.drawBox(2, 42, barWidth - 4, 6);

      u8g2.sendBuffer();
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    String errorMsg = "";
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
      errorMsg = "Auth Failed";
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
      errorMsg = "Begin Failed";
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
      errorMsg = "Connect Failed";
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
      errorMsg = "Receive Failed";
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
      errorMsg = "End Failed";
    }

    // Show error on OLED (only if available)
    if (oledAvailable) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0, 15, "OTA Error:");
      u8g2.drawStr(0, 30, errorMsg.c_str());
      u8g2.sendBuffer();
    }
    delay(3000);
  });

  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// ===========================================
// DEVICE NAME MANAGEMENT
// ===========================================

void loadDeviceName() {
  if (EEPROM.read(EEPROM_DEVICE_NAME) == MAGIC_NUMBER) {
    deviceName = "";
    for (int i = 1; i < 32; i++) {
      char c = EEPROM.read(EEPROM_DEVICE_NAME + i);
      if (c == 0) break;
      deviceName += c;
    }

    if (deviceName.length() == 0 || deviceName.length() > 20) {
      deviceName = "monitor";
    }
  } else {
    deviceName = "monitor";
  }

  updateDynamicNames();
}

void saveDeviceName() {
  EEPROM.write(EEPROM_DEVICE_NAME, MAGIC_NUMBER);

  for (int i = 0; i < 31; i++) {
    if (i < deviceName.length()) {
      EEPROM.write(EEPROM_DEVICE_NAME + 1 + i, deviceName[i]);
    } else {
      EEPROM.write(EEPROM_DEVICE_NAME + 1 + i, 0);
    }
  }
  EEPROM.commit();
}

void updateDynamicNames() {
  String cleanName = deviceName;
  cleanName.trim();
  cleanName.toLowerCase();
  cleanName.replace(" ", "_");

  String validName = "";
  for (int i = 0; i < cleanName.length(); i++) {
    char c = cleanName[i];
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-') {
      validName += c;
    }
  }

  if (validName.length() > 20) {
    validName = validName.substring(0, 20);
  }

  fullDeviceName = validName + "_mvstech";
}

// ===========================================
// CONFIGURATION MANAGEMENT
// ===========================================

void loadConfiguration() {
  if (EEPROM.read(EEPROM_CONFIGURED_FLAG) == MAGIC_NUMBER) {
    isConfigured = true;

    // Load tank configuration
    int baseAddr = EEPROM_TANK_CONFIG_START;

    for (int j = 0; j < 8; j++) {
      tankConfig.nodeID[j] = EEPROM.read(baseAddr + j);
    }
    tankConfig.nodeID[8] = '\0';
    if (!isValidNodeID(String(tankConfig.nodeID))) {
      tankConfig.nodeID[0] = '\0';  // Wired-only monitors never stored a node ID
    }

    for (int j = 0; j < 8; j++) {
      tankConfig.customName[j] = EEPROM.read(baseAddr + 8 + j);
    }
    tankConfig.customName[8] = '\0';

    tankConfig.height = (EEPROM.read(baseAddr + 16) << 8) | EEPROM.read(baseAddr + 17);

    tankConfig.volume = ((uint32_t)EEPROM.read(baseAddr + 18) << 24) |
                        ((uint32_t)EEPROM.read(baseAddr + 19) << 16) |
                        ((uint32_t)EEPROM.read(baseAddr + 20) << 8) |
                        (uint32_t)EEPROM.read(baseAddr + 21);

    tankConfig.highAlarmPercent = EEPROM.read(baseAddr + 22);
    tankConfig.lowAlarmPercent = EEPROM.read(baseAddr + 23);
    tankConfig.enabled = (EEPROM.read(baseAddr + 24) == MAGIC_NUMBER);

    if (tankConfig.enabled) {
      tankData.minLevel = 9999;
      tankData.maxLevel = 0;
      tankData.minPercent = 100;
      tankData.maxPercent = 0;
    }

    // Load general settings
    generalSettings.acknowledgeTimeout = EEPROM.read(EEPROM_GENERAL_SETTINGS);
    generalSettings.offlineTimeoutMinutes = EEPROM.read(EEPROM_GENERAL_SETTINGS + 1);
    generalSettings.buzzerEnabled = (EEPROM.read(EEPROM_GENERAL_SETTINGS + 2) == MAGIC_NUMBER);

    if (generalSettings.acknowledgeTimeout == 0 || generalSettings.acknowledgeTimeout > 60) {
      generalSettings.acknowledgeTimeout = 5;
    }
    if (generalSettings.offlineTimeoutMinutes == 0 || generalSettings.offlineTimeoutMinutes > 60) {
      generalSettings.offlineTimeoutMinutes = SENSOR_OFFLINE_TIMEOUT_MINUTES;
    }

    loadAlarmAckData();
    Serial.println("Configuration loaded from EEPROM");
  } else {
    Serial.println("No configuration found - device needs setup");
    isConfigured = false;

    tankConfig.enabled = false;
    tankConfig.nodeID[0] = '\0';
    tankData.isOnline = false;
    tankData.hasData = false;

    initializeAlarmAckData();
  }

  loadLinkSettings();
}

// Connection type + history logging settings
void loadLinkSettings() {
  if (EEPROM.read(EEPROM_LINK_MAGIC) == LINK_MAGIC) {
    uint8_t t = EEPROM.read(EEPROM_LINK_TYPE);
    linkType = (t < LINK_TYPE_COUNT) ? (LinkType)t : LINK_AUTO;
    logThresholdPercent = EEPROM.read(EEPROM_LOG_THRESHOLD);
    logIntervalMinutes = EEPROM.read(EEPROM_LOG_INTERVAL);
  } else {
    // First boot on this firmware: a monitor that already knows a wireless node ID
    // keeps listening for it, everything else starts as Wired (old wired firmware).
    linkType = (tankConfig.nodeID[0] != '\0') ? LINK_AUTO : LINK_WIRED;
    logThresholdPercent = 10;
    logIntervalMinutes = 60;
  }
  if (logThresholdPercent != 5 && logThresholdPercent != 10) logThresholdPercent = 10;
  if (logIntervalMinutes != 30 && logIntervalMinutes != 60) logIntervalMinutes = 60;
}

void saveLinkSettings() {
  EEPROM.write(EEPROM_LINK_MAGIC, LINK_MAGIC);
  EEPROM.write(EEPROM_LINK_TYPE, (uint8_t)linkType);
  EEPROM.write(EEPROM_LOG_THRESHOLD, logThresholdPercent);
  EEPROM.write(EEPROM_LOG_INTERVAL, logIntervalMinutes);
  EEPROM.commit();
}

// Save just the paired node ID (bytes 0-7 of the tank block)
void saveNodeID() {
  int baseAddr = EEPROM_TANK_CONFIG_START;
  for (int j = 0; j < 8; j++) {
    EEPROM.write(baseAddr + j, tankConfig.nodeID[j]);
  }
  EEPROM.commit();
}

// Valid node ID: "US" followed by 4 digits
bool isValidNodeID(const String& id) {
  if (id.length() != 6 || !id.startsWith("US")) return false;
  for (int i = 2; i < 6; i++) {
    if (!isDigit(id[i])) return false;
  }
  return true;
}

// Pair with a wireless sensor - resets duplicate tracking and the Home Network address
void pairSensor(const String& id) {
  memset(tankConfig.nodeID, 0, sizeof(tankConfig.nodeID));
  strncpy(tankConfig.nodeID, id.c_str(), 8);
  saveNodeID();
  tankData.lastSession = 0;
  tankData.lastCounter = 0;
  hnSensorIPValid = false;
  hnFailCount = 0;
  for (int i = 0; i < scanCount; i++) {
    if (id == scanResults[i].id && (uint32_t)scanResults[i].ip != 0) {
      hnSensorIP = scanResults[i].ip;
      hnSensorIPValid = true;
    }
  }
  Serial.print("Paired with sensor: ");
  Serial.println(tankConfig.nodeID);
}

// Customer-facing names - internal technology names never appear in the UI
const char* linkTypeName(LinkType t) {
  switch (t) {
    case LINK_WIRED: return "Wired";
    case LINK_LONG_RANGE: return "Long Range";
    case LINK_NEARBY: return "NearbyConnect";
    case LINK_HOME_NETWORK: return "Home Network";
    default: return "Auto";
  }
}

const char* linkViaName(uint8_t via) {
  switch (via) {
    case VIA_WIRED: return "Wired";
    case VIA_LORA: return "Long Range";
    case VIA_NEARBY: return "NearbyConnect";
    case VIA_HOME_NET: return "Home Network";
    default: return "--";
  }
}

bool linkIsWireless() {
  return linkType != LINK_WIRED;
}

bool linkUsesLoRa() {
  return linkType == LINK_LONG_RANGE || linkType == LINK_AUTO;
}

bool linkUsesNearby() {
  return linkType == LINK_NEARBY || linkType == LINK_HOME_NETWORK || linkType == LINK_AUTO;
}

bool linkUsesHomeNetwork() {
  return linkType == LINK_HOME_NETWORK || linkType == LINK_AUTO;
}

void initializeAlarmAckData() {
  memset(&alarmAck, 0, sizeof(alarmAck));

  alarmAck.tankLowThresholdAck = false;
  alarmAck.tankHighThresholdAck = false;
  alarmAck.tankOfflineAck = false;

  alarmAck.isValid = true;
  saveAlarmAckData();
}

void loadAlarmAckData() {
  if (EEPROM.read(EEPROM_ALARM_ACK_START) == MAGIC_NUMBER) {
    int addr = EEPROM_ALARM_ACK_START + 1;

    alarmAck.tankLowThresholdAck = (EEPROM.read(addr++) == MAGIC_NUMBER);
    alarmAck.tankHighThresholdAck = (EEPROM.read(addr++) == MAGIC_NUMBER);
    alarmAck.tankOfflineAck = (EEPROM.read(addr++) == MAGIC_NUMBER);

    alarmAck.isValid = true;
    Serial.println("Alarm acknowledgment data loaded from EEPROM");
  } else {
    Serial.println("No alarm acknowledgment data found - initializing");
    initializeAlarmAckData();
  }
}

void saveAlarmAckData() {
  EEPROM.write(EEPROM_ALARM_ACK_START, MAGIC_NUMBER);
  int addr = EEPROM_ALARM_ACK_START + 1;

  EEPROM.write(addr++, alarmAck.tankLowThresholdAck ? MAGIC_NUMBER : 0);
  EEPROM.write(addr++, alarmAck.tankHighThresholdAck ? MAGIC_NUMBER : 0);
  EEPROM.write(addr++, alarmAck.tankOfflineAck ? MAGIC_NUMBER : 0);

  EEPROM.commit();
  Serial.println("Alarm acknowledgment data saved to EEPROM");
}

void saveConfiguration() {
  EEPROM.write(EEPROM_CONFIGURED_FLAG, MAGIC_NUMBER);

  // Save tank configuration
  int baseAddr = EEPROM_TANK_CONFIG_START;

  for (int j = 0; j < 8; j++) {
    EEPROM.write(baseAddr + j, tankConfig.nodeID[j]);
  }

  for (int j = 0; j < 8; j++) {
    EEPROM.write(baseAddr + 8 + j, tankConfig.customName[j]);
  }

  EEPROM.write(baseAddr + 16, tankConfig.height >> 8);
  EEPROM.write(baseAddr + 17, tankConfig.height & 0xFF);

  EEPROM.write(baseAddr + 18, (tankConfig.volume >> 24) & 0xFF);
  EEPROM.write(baseAddr + 19, (tankConfig.volume >> 16) & 0xFF);
  EEPROM.write(baseAddr + 20, (tankConfig.volume >> 8) & 0xFF);
  EEPROM.write(baseAddr + 21, tankConfig.volume & 0xFF);

  EEPROM.write(baseAddr + 22, tankConfig.highAlarmPercent);
  EEPROM.write(baseAddr + 23, tankConfig.lowAlarmPercent);
  EEPROM.write(baseAddr + 24, tankConfig.enabled ? MAGIC_NUMBER : 0);

  // Save general settings
  EEPROM.write(EEPROM_GENERAL_SETTINGS, generalSettings.acknowledgeTimeout);
  EEPROM.write(EEPROM_GENERAL_SETTINGS + 1, generalSettings.offlineTimeoutMinutes);
  EEPROM.write(EEPROM_GENERAL_SETTINGS + 2, generalSettings.buzzerEnabled ? MAGIC_NUMBER : 0);

  EEPROM.commit();
  isConfigured = true;
  Serial.println("Configuration saved to EEPROM");
}

void resetConfiguration() {
  EEPROM.write(EEPROM_CONFIGURED_FLAG, 0);
  EEPROM.commit();

  tankConfig.enabled = false;
  memset(tankConfig.nodeID, 0, sizeof(tankConfig.nodeID));
  saveNodeID();
  memset(tankConfig.customName, 0, sizeof(tankConfig.customName));
  tankConfig.height = 0;
  tankConfig.volume = 0;
  tankConfig.highAlarmPercent = 0;
  tankConfig.lowAlarmPercent = 0;

  generalSettings.acknowledgeTimeout = 5;
  generalSettings.offlineTimeoutMinutes = SENSOR_OFFLINE_TIMEOUT_MINUTES;
  generalSettings.buzzerEnabled = true;

  isConfigured = false;
  Serial.println("Configuration reset to factory defaults");
}

// ===========================================
// VALIDATION FUNCTIONS
// ===========================================

bool validateCustomName(const String& name) {
  if (name.length() == 0 || name.length() > 8) {
    return false;
  }

  for (int i = 0; i < name.length(); i++) {
    char c = name[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
      return false;
    }
  }

  return true;
}

bool validatePercentages(uint8_t lowPercent, uint8_t highPercent) {
  if (lowPercent == 0 && highPercent == 0) {
    return true;
  }

  if (lowPercent > 0 && highPercent > 0) {
    return highPercent >= (lowPercent + MIN_PERCENTAGE_GAP);
  }

  return true;
}

// ===========================================
// SERIAL CONFIGURATION FUNCTIONS
// ===========================================

String getStringInput(const String& prompt, int maxLength, bool allowEmpty) {
  while (true) {
    Serial.println(prompt);

    while (Serial.available() == 0) {
      delay(100);
    }

    String input = Serial.readString();
    input.trim();

    if (!allowEmpty && input.length() == 0) {
      Serial.println("Input cannot be empty. Please try again.");
      continue;
    }

    if (input.length() > maxLength) {
      Serial.print("Input too long (max ");
      Serial.print(maxLength);
      Serial.println(" characters). Please try again.");
      continue;
    }

    return input;
  }
}

int getIntInput(const String& prompt, int minVal, int maxVal) {
  while (true) {
    Serial.println(prompt);

    while (Serial.available() == 0) {
      delay(100);
    }

    String input = Serial.readString();
    input.trim();

    if (input.length() == 0) {
      Serial.println("Please enter a number.");
      continue;
    }

    int value = input.toInt();
    if (value == 0 && input != "0") {
      Serial.println("Invalid number format. Please try again.");
      continue;
    }

    if (value < minVal || value > maxVal) {
      Serial.print("Value must be between ");
      Serial.print(minVal);
      Serial.print(" and ");
      Serial.print(maxVal);
      Serial.println(". Please try again.");
      continue;
    }

    return value;
  }
}

void showMainMenu() {
  Serial.println("\n=== TANK MONITOR CONFIGURATION ===");
  Serial.println("1. Configure Tank Sensor");
  Serial.println("2. General Settings");
  Serial.println("3. WiFi Configuration");
  Serial.println("4. View Current Configuration");
  Serial.println("5. Save & Exit");
  Serial.println("6. Reset All Configuration");
  Serial.println("7. Sensor Connection & Pairing");
  Serial.println("8. History Logging");
  Serial.print("Select option (1-8): ");
}

void configureConnection() {
  Serial.println("\n=== SENSOR CONNECTION ===");
  Serial.print("Current: ");
  Serial.println(linkTypeName(linkType));
  if (linkIsWireless()) {
    Serial.print("Paired sensor: ");
    Serial.println(tankConfig.nodeID[0] ? tankConfig.nodeID : "(none)");
  }
  Serial.println("\n0. Wired         - sensor cable connected to this display");
  Serial.println("1. Long Range    - wireless up to several km (Long Range module needed)");
  Serial.println("2. NearbyConnect - wireless up to ~200 m, no router needed");
  Serial.println("3. Home Network  - sensor and display on the same WiFi network");
  Serial.println("4. Auto          - use whatever wireless link the sensor has");
  Serial.println("5. Scan & pair a wireless sensor");
  Serial.println("6. Enter sensor ID manually");
  Serial.println("7. Back");

  int choice = getIntInput("Select option (0-7): ", 0, 7);
  if (choice <= 4) {
    linkType = (LinkType)choice;
    saveLinkSettings();
    Serial.println("Connection type saved. Restarting to apply...");
    delay(500);
    ESP.restart();
  } else if (choice == 5) {
    if (!linkIsWireless()) {
      Serial.println("Select a wireless connection type first.");
      return;
    }
    startSensorScan();
    Serial.println("Scanning for 20 seconds...");
    while (scanActive) {
      serviceLinks();
      delay(20);
    }
    if (scanCount == 0) {
      Serial.println("No sensors found. Make sure the sensor is powered on and uses the same connection type.");
      return;
    }
    for (int i = 0; i < scanCount; i++) {
      Serial.printf("%d. %s  (%s, %d dBm)\n", i + 1, scanResults[i].id, linkViaName(scanResults[i].via), scanResults[i].rssi);
    }
    int pick = getIntInput("Select sensor to pair (0 = cancel): ", 0, scanCount);
    if (pick > 0) pairSensor(String(scanResults[pick - 1].id));
  } else if (choice == 6) {
    String id = getStringInput("Enter sensor ID (e.g. US3847): ", 6);
    id.toUpperCase();
    if (isValidNodeID(id)) {
      pairSensor(id);
    } else {
      Serial.println("Invalid ID. Format: US followed by 4 digits.");
    }
  }
}

void configureLogging() {
  Serial.println("\n=== HISTORY LOGGING ===");
  Serial.printf("Log on level change of: %d%%\n", logThresholdPercent);
  Serial.printf("Log at least every: %d minutes\n", logIntervalMinutes);
  Serial.printf("Stored readings: %d (next #%lu)\n", historyCount(), (unsigned long)histNextSeq);
  Serial.println("\n1. Change threshold (5% / 10%)");
  Serial.println("2. Change interval (30 / 60 min)");
  Serial.println("3. Clear history");
  Serial.println("4. Back");
  int choice = getIntInput("Select option (1-4): ", 1, 4);
  if (choice == 1) {
    int t = getIntInput("Threshold percent (5 or 10): ", 5, 10);
    logThresholdPercent = (t < 8) ? 5 : 10;
    saveLinkSettings();
  } else if (choice == 2) {
    int m = getIntInput("Interval minutes (30 or 60): ", 30, 60);
    logIntervalMinutes = (m < 45) ? 30 : 60;
    saveLinkSettings();
  } else if (choice == 3) {
    String confirm = getStringInput("Clear all history? (Y/N): ", 1);
    if (confirm.equalsIgnoreCase("Y")) clearHistory();
  }
}

void configureTankSensor() {
  Serial.println("\n=== TANK SENSOR CONFIGURATION ===");

  TankConfig newConfig = tankConfig;

  // Custom Name
  while (true) {
    String customName = getStringInput("Enter Custom Tank Name (max 8 chars, e.g., Terrace): ", 8);

    if (!validateCustomName(customName)) {
      Serial.println("Invalid format. Use letters and numbers only.");
      continue;
    }

    strcpy(newConfig.customName, customName.c_str());
    break;
  }

  // Tank Height
  newConfig.height = getIntInput("Enter Tank Height (cm, 50-450): ", 50, 450);

  // Tank Volume
  newConfig.volume = getIntInput("Enter Tank Volume in Litres (0 to skip, 1-99999): ", 0, 99999);

  // Low Alarm Percentage
  newConfig.lowAlarmPercent = getIntInput("Enter Low Level Alarm Percentage (0=disable, 1-94): ", 0, 94);

  // High Alarm Percentage
  String highPrompt = "Enter High Level Alarm Percentage (0=disable";
  if (newConfig.lowAlarmPercent > 0) {
    highPrompt += ", min " + String(newConfig.lowAlarmPercent + MIN_PERCENTAGE_GAP);
  }
  highPrompt += ", max 99): ";

  while (true) {
    newConfig.highAlarmPercent = getIntInput(highPrompt, 0, 99);

    if (validatePercentages(newConfig.lowAlarmPercent, newConfig.highAlarmPercent)) {
      break;
    }

    Serial.print("High percentage must be at least ");
    Serial.print(MIN_PERCENTAGE_GAP);
    Serial.println("% higher than low percentage.");
  }

  // Confirm and save
  Serial.println("\nConfiguration Summary:");
  Serial.print("Custom Name: ");
  Serial.println(newConfig.customName);
  Serial.print("Tank Height: ");
  Serial.print(newConfig.height);
  Serial.println(" cm");
  Serial.print("Tank Volume: ");
  if (newConfig.volume > 0) {
    Serial.print(newConfig.volume);
    Serial.println(" litres");
  } else {
    Serial.println("Not configured");
  }
  Serial.print("Low Alarm: ");
  Serial.print(newConfig.lowAlarmPercent);
  Serial.println("%");
  Serial.print("High Alarm: ");
  Serial.print(newConfig.highAlarmPercent);
  Serial.println("%");

  String confirm = getStringInput("Save this configuration? (Y/N): ", 1);
  if (confirm.equalsIgnoreCase("Y")) {
    newConfig.enabled = true;
    tankConfig = newConfig;

    tankData.minLevel = 9999;
    tankData.maxLevel = 0;
    tankData.minPercent = 100;
    tankData.maxPercent = 0;
    tankData.isOnline = false;
    tankData.hasData = false;

    saveConfiguration();
    Serial.println("Tank sensor configuration saved!");
  } else {
    Serial.println("Configuration cancelled.");
  }
}

void configureGeneralSettings() {
  Serial.println("\n=== GENERAL SETTINGS ===");

  Serial.print("Current Device Name: ");
  Serial.println(deviceName);
  Serial.print("Current Acknowledge Timeout: ");
  Serial.print(generalSettings.acknowledgeTimeout);
  Serial.println(" minutes (not used in acknowledge mode)");
  Serial.print("Current Offline Timeout: ");
  Serial.print(generalSettings.offlineTimeoutMinutes);
  Serial.println(" minutes");
  Serial.print("Buzzer: ");
  Serial.println(generalSettings.buzzerEnabled ? "Enabled" : "Disabled");

  Serial.println("\n1. Change Device Name");
  Serial.println("2. Change Acknowledge Timeout (legacy setting)");
  Serial.println("3. Change Offline Timeout");
  Serial.println("4. Enable/Disable Buzzer");
  Serial.println("5. Test Buzzer");
  Serial.println("6. Back to Main Menu");

  int choice = getIntInput("Select option (1-6): ", 1, 6);

  switch (choice) {
    case 1: {
      String newName = getStringInput("Enter new device name (max 20 chars): ", 20);
      deviceName = newName;
      updateDynamicNames();
      saveDeviceName();
      Serial.println("Device name updated!");
      break;
    }
    case 2: {
      Serial.println("Note: This setting is not used in acknowledge mode.");
      generalSettings.acknowledgeTimeout = getIntInput("Enter acknowledge timeout in minutes (1-60): ", 1, 60);
      saveConfiguration();
      Serial.println("Acknowledge timeout updated (legacy setting)!");
      break;
    }
    case 3: {
      generalSettings.offlineTimeoutMinutes = getIntInput("Enter offline timeout in minutes (1-60): ", 1, 60);
      saveConfiguration();
      Serial.println("Offline timeout updated!");
      break;
    }
    case 4: {
      int buzzerChoice = getIntInput("Enable buzzer? (1=Yes, 0=No): ", 0, 1);
      generalSettings.buzzerEnabled = (buzzerChoice == 1);
      saveConfiguration();
      Serial.println("Buzzer setting updated!");
      break;
    }
    case 5: {
      testAlarm();
      break;
    }
    case 6:
      return;
  }
}

void viewCurrentConfiguration() {
  Serial.println("\n=== CURRENT CONFIGURATION ===");
  Serial.print("Device Name: ");
  Serial.println(fullDeviceName);
  Serial.print("Configuration Status: ");
  Serial.println(isConfigured ? "Configured" : "Not Configured");

  Serial.println("\nWiFi Configuration:");
  Serial.print("Configured: ");
  Serial.println(wifiConfigured ? "Yes" : "No");
  if (wifiConfigured) {
    Serial.print("SSID: ");
    Serial.println(wifiSSID);
    Serial.print("Connected: ");
    Serial.println(wifiStationMode ? "Yes" : "No");
    if (wifiStationMode) {
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    }
  }

  Serial.println("\nSensor Connection:");
  Serial.print("Type: ");
  Serial.println(linkTypeName(linkType));
  if (linkIsWireless()) {
    Serial.print("Paired sensor: ");
    Serial.println(tankConfig.nodeID[0] ? tankConfig.nodeID : "(none)");
  }
  Serial.printf("History: log on %d%% change or every %d min\n", logThresholdPercent, logIntervalMinutes);

  Serial.println("\nTank Sensor:");
  if (tankConfig.enabled) {
    Serial.print(tankConfig.customName);
    Serial.print(" - Height: ");
    Serial.print(tankConfig.height);
    Serial.print("cm, Volume: ");
    if (tankConfig.volume > 0) {
      Serial.print(tankConfig.volume);
      Serial.print("L");
    } else {
      Serial.print("Not set");
    }
    Serial.print(", Low: ");
    Serial.print(tankConfig.lowAlarmPercent);
    Serial.print("%, High: ");
    Serial.print(tankConfig.highAlarmPercent);
    Serial.println("%");
  } else {
    Serial.println("Not configured");
  }

  Serial.println("\nGeneral Settings:");
  Serial.print("Acknowledge Timeout: ");
  Serial.print(generalSettings.acknowledgeTimeout);
  Serial.println(" minutes (legacy setting)");
  Serial.print("Offline Timeout: ");
  Serial.print(generalSettings.offlineTimeoutMinutes);
  Serial.println(" minutes");
  Serial.print("Buzzer: ");
  Serial.println(generalSettings.buzzerEnabled ? "Enabled" : "Disabled");
}

void runConfiguration() {
  Serial.println("\nEntering configuration mode...");

  while (true) {
    showMainMenu();

    while (Serial.available() == 0) {
      delay(100);
    }

    String input = Serial.readString();
    input.trim();
    int choice = input.toInt();

    switch (choice) {
      case 1:
        configureTankSensor();
        break;
      case 2:
        configureGeneralSettings();
        break;
      case 3:
        configureWiFiTerminal();
        break;
      case 4:
        viewCurrentConfiguration();
        break;
      case 5:
        Serial.println("Configuration saved and exiting...");
        return;
      case 6: {
        String confirm = getStringInput("Reset all configuration? This cannot be undone! (Y/N): ", 1);
        if (confirm.equalsIgnoreCase("Y")) {
          resetConfiguration();
        }
        break;
      }
      case 7:
        configureConnection();
        break;
      case 8:
        configureLogging();
        break;
      default:
        Serial.println("Invalid option. Please select 1-8.");
        break;
    }
  }
}

// ===========================================
// ULTRASONIC SENSOR (WIRED, MAD FILTERING)
// ===========================================

// Simple bubble sort helper for MAD filtering
void sortReadings(float arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

// Fire one trig pulse and return the echo duration in microseconds (0 = no echo)
long readEchoDuration() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(TRIG_PIN, LOW);

  return pulseIn(ECHO_PIN, HIGH, 60000);  // 60ms timeout
}

// Takes multiple samples and returns a MAD-filtered median distance in cm.
// Returns -1.0 if not enough valid samples were captured.
// echoCount reports how many pulses got ANY echo (even out of range), so a sensor
// that answers but cannot measure is told apart from one that is disconnected.
float readDistanceCM(int* echoCount) {
  float readings[SAMPLE_COUNT];
  int count = 0;
  int echoes = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    long duration = readEchoDuration();
    float distance = duration * 0.0343 / 2;  // Convert to cm

    if (duration > 0) {
      echoes++;
    }
    if (duration > 0 && distance > SENSOR_BLIND_ZONE_CM && distance <= SENSOR_MAX_RANGE_CM) {
      readings[count++] = distance;
    }

    // Nothing answering at all - stop early instead of blocking the loop for ~1.6s
    if (i + 1 == NO_ECHO_ABORT_SAMPLES && echoes == 0) {
      break;
    }

    delay(READING_DELAY);
  }

  if (echoCount) {
    *echoCount = echoes;
  }

  if (count < MIN_VALID_SAMPLES) {
    #if DEBUG_MODE
    Serial.printf("Ultrasonic read failed: %d valid of %d echoes\n", count, echoes);
    #endif
    return -1.0;
  }

  sortReadings(readings, count);

  float median;
  if (count % 2 == 1) {
    median = readings[count / 2];
  } else {
    median = (readings[count / 2 - 1] + readings[count / 2]) / 2.0;
  }

  // Median Absolute Deviation - reject outlier readings
  float deviations[SAMPLE_COUNT];
  for (int i = 0; i < count; i++) {
    deviations[i] = fabs(readings[i] - median);
  }
  sortReadings(deviations, count);

  float mad;
  if (count % 2 == 1) {
    mad = deviations[count / 2];
  } else {
    mad = (deviations[count / 2 - 1] + deviations[count / 2]) / 2.0;
  }

  float cleanReadings[SAMPLE_COUNT];
  int cleanCount = 0;
  float threshold = MAD_MULTIPLIER * mad;
  if (threshold < 1.0) threshold = 15.0;  // All readings identical - use backup threshold

  for (int i = 0; i < count; i++) {
    if (fabs(readings[i] - median) <= threshold) {
      cleanReadings[cleanCount++] = readings[i];
    }
  }

  if (cleanCount >= (MIN_VALID_SAMPLES / 2)) {
    sortReadings(cleanReadings, cleanCount);
    if (cleanCount % 2 == 1) {
      median = cleanReadings[cleanCount / 2];
    } else {
      median = (cleanReadings[cleanCount / 2 - 1] + cleanReadings[cleanCount / 2]) / 2.0;
    }
  }

  return median;
}

void processWaterLevelData(float ultrasonicReading) {
  if (!tankConfig.enabled) {
    return;
  }

  tankData.lastDistance = ultrasonicReading;

  if (ultrasonicReading > tankConfig.height) {
    ultrasonicReading = tankConfig.height;
  } else if (ultrasonicReading < SENSOR_BLIND_ZONE_CM) {
    ultrasonicReading = SENSOR_BLIND_ZONE_CM;
  }

  float usableTankDepth = tankConfig.height - SENSOR_BLIND_ZONE_CM;
  float waterLevel = tankConfig.height - ultrasonicReading;
  float waterPercent = (waterLevel / usableTankDepth) * 100.0;

  if (waterPercent < 0) waterPercent = 0;
  if (waterPercent > 100) waterPercent = 100;

  tankData.currentLevel = waterLevel;
  tankData.currentPercent = waterPercent;
  tankData.lastUpdate = millis();
  tankData.lastDataTime = tankData.lastUpdate;
  tankData.isOnline = true;
  tankData.hasData = true;

  if (waterLevel < tankData.minLevel) {
    tankData.minLevel = waterLevel;
  }
  if (waterLevel > tankData.maxLevel) {
    tankData.maxLevel = waterLevel;
  }
  if (waterPercent < tankData.minPercent) {
    tankData.minPercent = waterPercent;
  }
  if (waterPercent > tankData.maxPercent) {
    tankData.maxPercent = waterPercent;
  }

  updateTrend();
  historyOnReading(waterPercent, waterLevel);
}

// Any sign of life from the sensor (data, heartbeat or a repeated packet)
void markSensorAlive(uint8_t via, int rssi) {
  tankData.lastUpdate = millis();
  tankData.everHeard = true;
  tankData.via = via;
  if (rssi != 0) {
    tankData.rssi = rssi;
  }
}

// Wired: poll the sensor on this monitor's own pins
void pollWiredSensor() {
  if (!tankConfig.enabled) return;

  int echoes = 0;
  float distance = readDistanceCM(&echoes);

  if (distance >= 0) {
    wiredNoEchoCycles = 0;
    markSensorAlive(VIA_WIRED, 0);
    tankData.lastWasHeartbeat = false;
    tankData.packets++;
    processWaterLevelData(distance);

    #if DEBUG_MODE
    Serial.printf("Tank (%s): %.1fcm -> %.1f%%\n", tankConfig.customName, distance, tankData.currentPercent);
    #endif
    return;
  }

  // Water right under the sensor (tank full) can silence it too - treat as alive, not disconnected
  bool nearFull = tankData.hasData && tankData.lastDistance > 0 &&
                  tankData.lastDistance < SENSOR_BLIND_ZONE_CM + 10;

  if (echoes > 0 || nearFull) {
    // Sensor answers but could not measure - heartbeat, keep last known value
    wiredNoEchoCycles = 0;
    markSensorAlive(VIA_WIRED, 0);
    tankData.lastWasHeartbeat = true;
    tankData.heartbeats++;
  } else {
    if (wiredNoEchoCycles < 255) wiredNoEchoCycles++;
    if (wiredNoEchoCycles == WIRED_OFFLINE_CYCLES) {
      Serial.println("Wired sensor: no echo - check the sensor cable");
    }
  }
}

// What the display should show for the sensor link right now
LinkState getLinkState() {
  if (linkType == LINK_WIRED && wiredNoEchoCycles >= WIRED_OFFLINE_CYCLES) {
    return LS_OFFLINE;
  }
  if (!tankData.everHeard) {
    return LS_WAITING;
  }
  if (linkType != LINK_WIRED && (millis() - tankData.lastUpdate) > WIRELESS_SILENT_MS) {
    return LS_OFFLINE;
  }
  return tankData.lastWasHeartbeat ? LS_HEARTBEAT : LS_LIVE;
}

const char* linkStateKey(LinkState s) {
  switch (s) {
    case LS_LIVE: return "live";
    case LS_HEARTBEAT: return "heartbeat";
    case LS_OFFLINE: return "offline";
    default: return "waiting";
  }
}

// Signal bars 0-3 for the link that delivered the last message
int signalBars() {
  if (tankData.via == VIA_WIRED || tankData.rssi == 0) return 0;
  int rssi = tankData.rssi;
  if (tankData.via == VIA_LORA) {
    if (rssi >= -85) return 3;
    if (rssi >= -105) return 2;
    return 1;
  }
  if (rssi >= -60) return 3;
  if (rssi >= -75) return 2;
  return 1;
}

// ===========================================
// WIRELESS LINKS (shared message handling)
// ===========================================

// Parse the micro-payload (or the older full payload) into a SensorMsg
bool parseSensorPayload(const char* json, size_t len, SensorMsg* msg) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, json, len)) {
    return false;
  }

  memset(msg, 0, sizeof(SensorMsg));

  if (doc.containsKey("i")) {
    // Micro-payload: {"i":"US3847","s":51,"c":102,"d":143} or {...,"h":1}
    strncpy(msg->id, doc["i"] | "", sizeof(msg->id) - 1);
    msg->session = doc["s"] | 0UL;
    msg->counter = doc["c"] | 0UL;
    int hb = doc["h"] | 0;
    if (!hb && doc.containsKey("d")) {
      msg->distance = doc["d"].as<float>();
      msg->hasDistance = msg->distance > 0;
    }
    msg->rssi = doc["r"] | 0;  // Home Network: sensor's own WiFi signal
  } else if (doc.containsKey("node_id")) {
    // Older transmitter firmware: {"node_id":"US3847","msg_id":"US3847_<session>_<counter>","data":"143"}
    String nodeId = doc["node_id"].as<String>();
    int dash = nodeId.indexOf('-');
    if (dash >= 0) nodeId = nodeId.substring(dash + 1);
    strncpy(msg->id, nodeId.c_str(), sizeof(msg->id) - 1);

    String msgId = doc["msg_id"] | "";
    int u1 = msgId.indexOf('_');
    int u2 = (u1 >= 0) ? msgId.indexOf('_', u1 + 1) : -1;
    if (u1 >= 0 && u2 > u1) {
      msg->session = strtoul(msgId.substring(u1 + 1, u2).c_str(), NULL, 10);
      msg->counter = strtoul(msgId.substring(u2 + 1).c_str(), NULL, 10);
    }
    msg->distance = String(doc["data"] | "0").toFloat();
    msg->hasDistance = msg->distance > 0;
    if (msg->counter == 0) msg->counter = 1;  // Old format without counter - always treat as new
  } else {
    return false;
  }

  return msg->id[0] != '\0';
}

bool viaAllowed(uint8_t via) {
  switch (linkType) {
    case LINK_LONG_RANGE: return via == VIA_LORA;
    case LINK_NEARBY: return via == VIA_NEARBY;
    case LINK_HOME_NETWORK: return via == VIA_HOME_NET || via == VIA_NEARBY;
    case LINK_AUTO: return via != VIA_WIRED;
    default: return false;
  }
}

void addToScanResults(const char* id, int rssi, uint8_t via, IPAddress ip) {
  if (!scanActive || !isValidNodeID(String(id))) return;

  for (int i = 0; i < scanCount; i++) {
    if (strcmp(scanResults[i].id, id) == 0) {
      if (rssi != 0 && (scanResults[i].rssi == 0 || rssi > scanResults[i].rssi)) {
        scanResults[i].rssi = rssi;
      }
      if (via == VIA_HOME_NET) {
        scanResults[i].ip = ip;
      }
      return;
    }
  }

  if (scanCount < MAX_SCAN_RESULTS) {
    strncpy(scanResults[scanCount].id, id, sizeof(scanResults[scanCount].id) - 1);
    scanResults[scanCount].id[sizeof(scanResults[scanCount].id) - 1] = '\0';
    scanResults[scanCount].rssi = rssi;
    scanResults[scanCount].via = via;
    scanResults[scanCount].ip = ip;
    scanCount++;
  }
}

// One handler for every link. Returns true if the message came from the paired sensor.
bool handleSensorMessage(const SensorMsg& msg, uint8_t via) {
  if (scanActive) {
    addToScanResults(msg.id, msg.rssi, via, IPAddress());
  }

  if (!linkIsWireless() || !tankConfig.enabled || tankConfig.nodeID[0] == '\0') return false;
  if (strcmp(msg.id, tankConfig.nodeID) != 0) return false;
  if (!viaAllowed(via)) return false;

  markSensorAlive(via, msg.rssi);

  // Sensor has no reading yet (Home Network poll right after it booted)
  if (msg.counter == 0) {
    tankData.lastWasHeartbeat = true;
    return true;
  }

  // Same session and an old counter: a repeat (relay, second link or a poll between readings).
  // Still proof the sensor is alive, but the value is not processed twice.
  if (msg.session == tankData.lastSession && msg.counter <= tankData.lastCounter) {
    tankData.duplicates++;
    return true;
  }

  tankData.lastSession = msg.session;
  tankData.lastCounter = msg.counter;
  tankData.packets++;

  if (msg.hasDistance) {
    tankData.lastWasHeartbeat = false;
    processWaterLevelData(msg.distance);
    #if DEBUG_MODE
    Serial.printf("[%s] %s #%lu: %.1fcm -> %.1f%% (%d dBm)\n", linkViaName(via), msg.id,
                  (unsigned long)msg.counter, msg.distance, tankData.currentPercent, msg.rssi);
    #endif
  } else {
    tankData.lastWasHeartbeat = true;
    tankData.heartbeats++;
    #if DEBUG_MODE
    Serial.printf("[%s] %s #%lu: heartbeat\n", linkViaName(via), msg.id, (unsigned long)msg.counter);
    #endif
  }
  return true;
}

// ---------- Long Range (LoRa) ----------

bool startLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    return false;  // No module answering - begin() checks the chip version
  }
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setPreambleLength(LORA_PREAMBLE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.receive();
  return true;
}

void serviceLoRa() {
  if (!loraActive) return;

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    char buf[200];
    int n = 0;
    while (LoRa.available()) {
      char c = (char)LoRa.read();
      if (n < (int)sizeof(buf) - 1) buf[n++] = c;
    }
    buf[n] = '\0';
    int rssi = LoRa.packetRssi();
    lastLoRaPacket = millis();

    SensorMsg msg;
    if (parseSensorPayload(buf, n, &msg)) {
      msg.rssi = rssi;
      handleSensorMessage(msg, VIA_LORA);
    }
    LoRa.receive();  // Back to continuous receive
  }

  // Health check - re-initialise the radio if a sensor that was heard has gone silent
  if (lastLoRaPacket > 0 && millis() - lastLoRaPacket > LORA_REINIT_SILENT_MS) {
    Serial.println("Long Range: no packets for 2 minutes - reinitialising");
    LoRa.end();
    delay(100);
    loraActive = startLoRa();
    lastLoRaPacket = millis();
  }
}

// ---------- NearbyConnect (ESP-NOW) ----------

// Runs in the WiFi task: copy the packet into the queue only, loop() does the work
void queueNearbyPacket(const uint8_t* data, int len, int rssi) {
  if (len <= 0 || len >= NEARBY_MAX_LEN) return;
  uint8_t next = (nearbyHead + 1) % NEARBY_QUEUE_LEN;
  if (next == nearbyTail) return;  // Queue full - drop
  NearbyMsg& m = nearbyQueue[nearbyHead];
  memcpy(m.data, data, len);
  m.data[len] = '\0';
  m.len = len;
  m.rssi = (int8_t)rssi;
  __sync_synchronize();
  nearbyHead = next;
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onNearbyReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  int rssi = (info && info->rx_ctrl) ? info->rx_ctrl->rssi : 0;
  queueNearbyPacket(data, len, rssi);
}
#else
void onNearbyReceive(const uint8_t* mac, const uint8_t* data, int len) {
  queueNearbyPacket(data, len, 0);
}
#endif

bool startNearby() {
  if (esp_now_init() != ESP_OK) {
    return false;
  }
  esp_now_register_recv_cb(onNearbyReceive);

  // Broadcast peer for acknowledgements (ESP-NOW follows the current WiFi channel)
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = 0;
  peer.ifidx = WIFI_IF_AP;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  return true;
}

// Tell the sensor we got it, so it stays locked on this channel
void sendNearbyAck(uint32_t counter) {
  char buf[48];
  int n = snprintf(buf, sizeof(buf), "{\"a\":\"%s\",\"c\":%lu}", tankConfig.nodeID, (unsigned long)counter);
  esp_now_send(BROADCAST_MAC, (const uint8_t*)buf, n);
}

void serviceNearby() {
  if (!nearbyActive) return;

  while (nearbyTail != nearbyHead) {
    NearbyMsg& m = nearbyQueue[nearbyTail];
    SensorMsg msg;
    if (parseSensorPayload(m.data, m.len, &msg)) {
      if (msg.rssi == 0) msg.rssi = m.rssi;
      if (handleSensorMessage(msg, VIA_NEARBY)) {
        sendNearbyAck(msg.counter);
      }
    }
    __sync_synchronize();
    nearbyTail = (nearbyTail + 1) % NEARBY_QUEUE_LEN;
  }
}

// ---------- Home Network (mDNS discovery + HTTP polling) ----------

IPAddress mdnsResultIP(int i) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return MDNS.address(i);
#else
  return MDNS.IP(i);
#endif
}

// Node ID advertised by result i of an mDNS "_mvstech._tcp" query (TXT "device", else the hostname)
String mdnsResultNodeID(int i) {
  String id = MDNS.hasTxt(i, "device") ? MDNS.txt(i, "device") : String("");
  if (!isValidNodeID(id)) {
    id = MDNS.hostname(i).substring(0, 6);
  }
  id.toUpperCase();
  return id;
}

// Find the paired sensor on the home network. Blocks for up to ~3s (mDNS query).
bool resolveHomeNetworkSensor() {
  hnLastResolve = millis();
  if (!wifiStationMode || tankConfig.nodeID[0] == '\0') return false;

  int n = MDNS.queryService("mvstech", "tcp");
  for (int i = 0; i < n; i++) {
    if (mdnsResultNodeID(i) == String(tankConfig.nodeID)) {
      hnSensorIP = mdnsResultIP(i);
      hnSensorIPValid = true;
      hnFailCount = 0;
      #if DEBUG_MODE
      Serial.print("Home Network: sensor found at ");
      Serial.println(hnSensorIP);
      #endif
      return true;
    }
  }

  // Fallback: the sensor's own hostname
  IPAddress ip = MDNS.queryHost(String(tankConfig.nodeID) + "_mvstechsensors", 2000);
  if (ip != IPAddress(0, 0, 0, 0)) {
    hnSensorIP = ip;
    hnSensorIPValid = true;
    hnFailCount = 0;
    return true;
  }
  return false;
}

// One HTTP poll of the sensor's /api/r endpoint
bool pollHomeNetworkSensor() {
  WiFiClient client;
  HTTPClient http;
  http.setConnectTimeout(1000);
  http.setTimeout(HN_HTTP_TIMEOUT_MS);

  String url = "http://" + hnSensorIP.toString() + ":7689/api/r";
  if (!http.begin(client, url)) {
    return false;
  }
  http.setUserAgent(USER_AGENT_CHECK);

  bool ok = false;
  int code = http.GET();
  if (code == 200) {
    String body = http.getString();
    SensorMsg msg;
    if (parseSensorPayload(body.c_str(), body.length(), &msg) &&
        strcmp(msg.id, tankConfig.nodeID) == 0) {
      handleSensorMessage(msg, VIA_HOME_NET);
      ok = true;
    }
  }
  http.end();
  return ok;
}

void serviceHomeNetwork() {
  if (!linkUsesHomeNetwork() || !tankConfig.enabled || tankConfig.nodeID[0] == '\0') return;
  if (!wifiStationMode || WiFi.status() != WL_CONNECTED) return;

  unsigned long now = millis();

  // Auto: another link is already delivering - don't spend time on network lookups
  if (linkType == LINK_AUTO && tankData.everHeard && tankData.via != VIA_HOME_NET &&
      now - tankData.lastUpdate < WIRELESS_SILENT_MS) {
    return;
  }

  unsigned long interval = (hnFailCount >= HN_FAILS_BEFORE_RESOLVE) ? HN_POLL_BACKOFF_MS : HN_POLL_INTERVAL_MS;
  if (now - hnLastPoll < interval) return;
  hnLastPoll = now;

  if (!hnSensorIPValid) {
    unsigned long resolveEvery = (linkType == LINK_AUTO) ? 300000UL : 30000UL;
    if (hnLastResolve != 0 && now - hnLastResolve < resolveEvery) return;
    if (!resolveHomeNetworkSensor()) return;
  }

  if (pollHomeNetworkSensor()) {
    hnFailCount = 0;
  } else {
    hnFailCount++;
    // The router may have given the sensor a new address - look it up again
    if (hnFailCount >= HN_FAILS_BEFORE_RESOLVE && now - hnLastResolve > 60000UL) {
      hnSensorIPValid = false;
      hnLastResolve = 0;
    }
  }
}

// ---------- Sensor scan (pairing) ----------

void scanHomeNetwork() {
  if (!wifiStationMode) return;
  int n = MDNS.queryService("mvstech", "tcp");
  for (int i = 0; i < n; i++) {
    String id = mdnsResultNodeID(i);
    if (isValidNodeID(id)) {
      addToScanResults(id.c_str(), 0, VIA_HOME_NET, mdnsResultIP(i));
    }
  }
}

void startSensorScan() {
  scanCount = 0;
  for (int i = 0; i < MAX_SCAN_RESULTS; i++) {
    scanResults[i] = ScanEntry();  // IPAddress is a class - no memset
  }
  scanActive = true;
  scanStartTime = millis();
  if (linkUsesHomeNetwork()) {
    scanHomeNetwork();  // Network lookup is one-shot; radio links fill in over the next 20s
  }
}

// ---------- Link setup ----------

void startLinks() {
  if (linkType == LINK_WIRED) {
    Serial.println("Connection: Wired");
    return;
  }

  if (linkUsesLoRa()) {
    SPI.begin();
    loraPresent = startLoRa();
    loraActive = loraPresent;
    #if DEBUG_MODE
    Serial.println(loraPresent ? "LoRa receiver ready" : "LoRa module not detected");
    #endif
  }

  if (linkUsesNearby()) {
    nearbyActive = startNearby();
    #if DEBUG_MODE
    Serial.println(nearbyActive ? "ESP-NOW receiver ready" : "ESP-NOW init failed");
    #endif
  }

  Serial.print("Connection: ");
  Serial.print(linkTypeName(linkType));
  if (linkType == LINK_LONG_RANGE && !loraPresent) {
    Serial.print(" - WARNING: Long Range module not detected");
  }
  Serial.println();
  Serial.print("Paired sensor: ");
  Serial.println(tankConfig.nodeID[0] ? tankConfig.nodeID : "(none - use Settings > Sensor Connection)");
}

void serviceLinks() {
  serviceLoRa();
  serviceNearby();
  serviceHomeNetwork();

  if (scanActive && millis() - scanStartTime >= SCAN_DURATION_MS) {
    scanActive = false;
  }
}

// ===========================================
// HISTORY (24h chart) & TREND
// ===========================================

uint32_t currentEpoch() {
  time_t now = time(nullptr);
  return (now > 1700000000) ? (uint32_t)now : 0;  // 0 = no NTP time yet
}

void startNTPIfConnected() {
  if (!ntpStarted && wifiStationMode && WiFi.status() == WL_CONNECTED) {
    configTime(0, 0, "pool.ntp.org", "time.google.com");
    ntpStarted = true;
  }
}

void writeAllHistory() {
  File f = LittleFS.open(HISTORY_FILE, "w");
  if (!f) {
    Serial.println("History: cannot write file");
    return;
  }
  f.write((const uint8_t*)histSlots, sizeof(histSlots));
  f.close();
}

// Load the circular history file and continue the sequence after the highest number found
void initHistory() {
  memset(histSlots, 0, sizeof(histSlots));

  bool loaded = false;
  File f = LittleFS.open(HISTORY_FILE, "r");
  if (f) {
    if (f.size() == sizeof(histSlots)) {
      loaded = (f.read((uint8_t*)histSlots, sizeof(histSlots)) == sizeof(histSlots));
    }
    f.close();
  }
  if (!loaded) {
    memset(histSlots, 0, sizeof(histSlots));
    writeAllHistory();
  }

  uint32_t maxSeq = 0;
  for (int i = 0; i < HISTORY_SLOTS; i++) {
    if (histSlots[i].seq % HISTORY_SLOTS != (uint32_t)i) {
      memset(&histSlots[i], 0, sizeof(HistRec));  // Slot doesn't match its sequence - ignore it
    } else if (histSlots[i].seq > maxSeq) {
      maxSeq = histSlots[i].seq;
    }
  }
  histNextSeq = maxSeq + 1;
  historyReady = true;

  Serial.printf("History: %d readings stored, next #%lu\n", historyCount(), (unsigned long)histNextSeq);
}

void appendHistory(float percent, float level) {
  HistRec r;
  r.seq = histNextSeq++;
  r.epoch = currentEpoch();
  r.pctX10 = (uint16_t)lroundf(constrain(percent, 0, 100) * 10);
  r.levelX10 = (uint16_t)lroundf(max(level, 0.0f) * 10);

  uint32_t slot = r.seq % HISTORY_SLOTS;
  histSlots[slot] = r;

  File f = LittleFS.open(HISTORY_FILE, "r+");
  if (f) {
    f.seek(slot * sizeof(HistRec));
    f.write((const uint8_t*)&r, sizeof(HistRec));
    f.close();
  } else {
    writeAllHistory();
  }

  lastLoggedPercent = percent;
  lastLogTime = millis();
  hasLoggedThisBoot = true;

  #if DEBUG_MODE
  Serial.printf("History #%lu: %.1f%%\n", (unsigned long)r.seq, percent);
  #endif
}

// Hybrid logging: big level change OR periodic interval (first reading after boot always logs)
void historyOnReading(float percent, float level) {
  if (!historyReady) return;

  bool shouldLog = !hasLoggedThisBoot ||
                   fabs(percent - lastLoggedPercent) >= logThresholdPercent ||
                   (millis() - lastLogTime) >= (unsigned long)logIntervalMinutes * 60000UL;
  if (shouldLog) {
    appendHistory(percent, level);
  }
}

int historyCount() {
  int n = 0;
  for (int i = 0; i < HISTORY_SLOTS; i++) {
    if (histSlots[i].seq != 0) n++;
  }
  return n;
}

// Newest maxN records, oldest first (ordered by sequence number, never by time)
int getHistorySorted(HistRec* out, int maxN) {
  int n = 0;
  uint32_t s = histNextSeq;
  for (int k = 0; k < HISTORY_SLOTS && s > 1 && n < maxN; k++) {
    s--;
    const HistRec& r = histSlots[s % HISTORY_SLOTS];
    if (r.seq == s) {
      out[n++] = r;
    }
  }
  for (int i = 0; i < n / 2; i++) {
    HistRec t = out[i];
    out[i] = out[n - 1 - i];
    out[n - 1 - i] = t;
  }
  return n;
}

void clearHistory() {
  memset(histSlots, 0, sizeof(histSlots));
  writeAllHistory();
  hasLoggedThisBoot = false;
  lastLoggedPercent = -1;
  Serial.println("History cleared");
}

void updateTrend() {
  if (!tankData.hasData) return;
  if (trendCount > 0 && millis() - lastTrendSample < TREND_SAMPLE_MS) return;
  lastTrendSample = millis();
  trendRing[trendHead] = tankData.currentPercent;
  trendHead = (trendHead + 1) % TREND_SLOTS;
  if (trendCount < TREND_SLOTS) trendCount++;
}

// Change over (up to) the last hour; false until at least 10 minutes of data exist
bool getTrend(float* delta) {
  if (trendCount < 3) return false;
  uint8_t oldest = (trendHead + TREND_SLOTS - trendCount) % TREND_SLOTS;
  *delta = tankData.currentPercent - trendRing[oldest];
  return true;
}

String formatFeetInches(float cm) {
  if (cm < 0) cm = 0;
  int totalIn = (int)lroundf(cm / 2.54f);
  return String(totalIn / 12) + "ft " + String(totalIn % 12) + "in";
}

uint32_t currentLitres() {
  if (tankConfig.volume == 0 || !tankData.hasData) return 0;
  return (uint32_t)lroundf(tankData.currentPercent / 100.0f * tankConfig.volume);
}

// ===========================================
// ALARM SYSTEM
// ===========================================

bool checkAlarmConditions() {
  bool alarmTriggered = false;
  bool hasUnacknowledgedAlarm = false;

  if (tankConfig.enabled && tankData.hasData) {
    unsigned long offlineThreshold = generalSettings.offlineTimeoutMinutes * 60000UL;
    if ((millis() - tankData.lastUpdate) > offlineThreshold) {
      tankData.isOnline = false;

      if (!alarmAck.tankOfflineAck) {
        alarmTriggered = true;
        hasUnacknowledgedAlarm = true;
      }
    } else {
      if (!tankData.isOnline && alarmAck.tankOfflineAck) {
        alarmAck.tankOfflineAck = false;
        saveAlarmAckData();
        Serial.print("Tank ");
        Serial.print(tankConfig.customName);
        Serial.println(" came back online - offline acknowledgment cleared");
      }
      tankData.isOnline = true;

      if (tankConfig.lowAlarmPercent > 0 &&
          tankData.currentPercent < tankConfig.lowAlarmPercent) {
        alarmTriggered = true;

        if (!alarmAck.tankLowThresholdAck) {
          hasUnacknowledgedAlarm = true;
        }
      } else {
        if (alarmAck.tankLowThresholdAck) {
          alarmAck.tankLowThresholdAck = false;
          saveAlarmAckData();
          Serial.print("Tank ");
          Serial.print(tankConfig.customName);
          Serial.println(" level cleared low threshold - acknowledgment reset");
        }
      }

      if (tankConfig.highAlarmPercent > 0 &&
          tankData.currentPercent > tankConfig.highAlarmPercent) {
        alarmTriggered = true;

        if (!alarmAck.tankHighThresholdAck) {
          hasUnacknowledgedAlarm = true;
        }
      } else {
        if (alarmAck.tankHighThresholdAck) {
          alarmAck.tankHighThresholdAck = false;
          saveAlarmAckData();
          Serial.print("Tank ");
          Serial.print(tankConfig.customName);
          Serial.println(" level cleared high threshold - acknowledgment reset");
        }
      }
    }
  }

  allAlarmsAcknowledged = alarmTriggered && !hasUnacknowledgedAlarm;

  return hasUnacknowledgedAlarm;
}

void updateAlarmSystem() {
  bool shouldBuzz = checkAlarmConditions();

  if (shouldBuzz && !alarmActive) {
    alarmActive = true;
    alarmStartTime = millis();
    lastBuzzerToggle = millis();  // Initialize toggle timer when alarm starts
    buzzerState = false;  // Start with buzzer OFF, will toggle to ON immediately
    Serial.println("ALARM TRIGGERED - New unacknowledged conditions detected!");
    if (globalBuzzerMute) {
      Serial.println("WARNING: BUZZER MUTED - Visual alarm only");
    }
  } else if (!shouldBuzz && alarmActive) {
    alarmActive = false;
    buzzerState = false;
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Alarm stopped - All conditions cleared or acknowledged");
  }

  // BUZZER CONTROL - Respects both buzzerEnabled setting AND globalBuzzerMute
  if (alarmActive && generalSettings.buzzerEnabled && !globalBuzzerMute) {
    unsigned long buzzerInterval = 1000;

    if (millis() - lastBuzzerToggle >= buzzerInterval) {
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
      lastBuzzerToggle = millis();
    }
  } else {
    buzzerState = false;
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void acknowledgeAllAlarms() {
  if (!alarmActive) {
    Serial.println("No active alarms to acknowledge");
    return;
  }

  bool acknowledgedSomething = false;

  if (tankConfig.enabled && tankData.hasData) {
    if (!tankData.isOnline && !alarmAck.tankOfflineAck) {
      alarmAck.tankOfflineAck = true;
      acknowledgedSomething = true;
      Serial.print("Acknowledged Tank ");
      Serial.print(tankConfig.customName);
      Serial.println(" offline condition");
    }

    if (tankData.isOnline) {
      if (tankConfig.lowAlarmPercent > 0 &&
          tankData.currentPercent < tankConfig.lowAlarmPercent &&
          !alarmAck.tankLowThresholdAck) {
        alarmAck.tankLowThresholdAck = true;
        acknowledgedSomething = true;
        Serial.print("Acknowledged Tank ");
        Serial.print(tankConfig.customName);
        Serial.print(" low threshold condition (current: ");
        Serial.print(tankData.currentPercent, 1);
        Serial.println("%)");
      }

      if (tankConfig.highAlarmPercent > 0 &&
          tankData.currentPercent > tankConfig.highAlarmPercent &&
          !alarmAck.tankHighThresholdAck) {
        alarmAck.tankHighThresholdAck = true;
        acknowledgedSomething = true;
        Serial.print("Acknowledged Tank ");
        Serial.print(tankConfig.customName);
        Serial.print(" high threshold condition (current: ");
        Serial.print(tankData.currentPercent, 1);
        Serial.println("%)");
      }
    }
  }

  if (acknowledgedSomething) {
    saveAlarmAckData();
    Serial.println("All current alarm conditions acknowledged");
    Serial.println("Alarms will not sound again until conditions clear and return");

    alarmActive = false;
    buzzerState = false;
    digitalWrite(BUZZER_PIN, LOW);
  } else {
    Serial.println("No new conditions to acknowledge");
  }
}

void testAlarm() {
  Serial.println("Testing alarm system...");

  if (!generalSettings.buzzerEnabled) {
    Serial.println("Buzzer is disabled in settings");
    return;
  }

  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }

  Serial.println("Alarm test completed");
}

// ===========================================
// BUTTON HANDLING
// ===========================================

void handleButton() {
  bool currentButtonState = digitalRead(BUTTON_PIN);

  if (currentButtonState != lastButtonState) {
    delay(BUTTON_DEBOUNCE);
    currentButtonState = digitalRead(BUTTON_PIN);

    if (currentButtonState != lastButtonState) {
      if (currentButtonState == LOW) {
        buttonPressTime = millis();
        longPressProcessed = false;
      } else {
        if (!longPressProcessed) {
          unsigned long pressDuration = millis() - buttonPressTime;

          if (pressDuration < LONG_PRESS_TIME) {
            unsigned long timeSinceLastPress = millis() - lastButtonPress;

            if (timeSinceLastPress < DOUBLE_PRESS_WINDOW) {
              buttonPressCount++;

              // Triple press - toggle mute
              if (buttonPressCount >= TRIPLE_PRESS_COUNT) {
                Serial.println("Triple press detected - toggling global mute");
                toggleGlobalMute();
                buttonPressCount = 0;
              }
              // Double press - show system info
              else if (buttonPressCount >= 2) {
                Serial.println("Double press detected - showing system info on OLED");
                screenMgr.showSystemInfo = true;
                screenMgr.systemInfoTimeout = millis() + 10000; // System info 5s, then sensor tech 5s
                // Don't reset count yet, wait for potential triple press
              }
            } else {
              // Single press - acknowledge alarms
              buttonPressCount = 1;
              acknowledgeAllAlarms();
            }

            lastButtonPress = millis();
          }
        }
      }

      lastButtonState = currentButtonState;
    }
  }

  // Long press behavior for manual screen control
  if (currentButtonState == LOW && !longPressProcessed) {
    if (millis() - buttonPressTime > LONG_PRESS_TIME) {
      longPressProcessed = true;

      if (!screenMgr.manualMode) {
        screenMgr.manualMode = true;
        screenMgr.manualTimeout = millis() + 30000; // 30 second timeout
        Serial.println("Manual screen control ON");
      } else {
        screenMgr.currentScreen = (screenMgr.currentScreen + 1) % screenMgr.totalScreens;
        screenMgr.manualTimeout = millis() + 30000; // Reset timeout
        Serial.println("Next screen");
      }
    }
  }

  if (millis() - lastButtonPress > DOUBLE_PRESS_WINDOW && buttonPressCount > 0) {
    buttonPressCount = 0;
  }
}

void showIndividualSensorReadings() {
  Serial.println("\n=== SENSOR READING ===");

  if (tankConfig.enabled) {
    Serial.print(tankConfig.customName);
    Serial.print(": ");

    if (tankData.hasData) {
      Serial.print(tankData.currentPercent, 1);
      Serial.print("% (");
      Serial.print(tankData.currentLevel, 1);
      Serial.print("cm) | ");
      Serial.print(linkStateKey(getLinkState()));
      Serial.print(" via ");
      Serial.print(linkViaName(tankData.via));
      if (tankData.rssi != 0) {
        Serial.printf(" (%d dBm)", tankData.rssi);
      }
      Serial.print(" | Min: ");
      Serial.print(tankData.minPercent, 1);
      Serial.print("% | Max: ");
      Serial.print(tankData.maxPercent, 1);
      Serial.println("%");
    } else {
      Serial.println("No data yet");
    }
  } else {
    Serial.println("Not configured");
  }
  Serial.println("===================================\n");
}

// ===========================================
// DISPLAY SYSTEM (AUTO-CYCLING)
// ===========================================

// Test I2C connection to OLED
bool testOLEDConnection(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

// Gracefully handles missing OLED - tries 0x3C then 0x3D
void initializeDisplay() {
  uint8_t address = 0;
  if (testOLEDConnection(OLED_I2C_ADDRESS)) {
    address = OLED_I2C_ADDRESS;
  } else if (testOLEDConnection(OLED_I2C_ADDRESS_ALT)) {
    address = OLED_I2C_ADDRESS_ALT;
  }

  if (address == 0) {
    Serial.println("WARNING: OLED not found at 0x3C or 0x3D!");
    Serial.println("System will continue WITHOUT display - WebUI still available");
    oledAvailable = false;
    return;  // Exit early - no display to initialize
  }

  Serial.print("OLED detected at 0x");
  Serial.println(address, HEX);

  // U8g2 hardware I2C expects address << 1
  u8g2.setI2CAddress(address << 1);

  // Initialize the display
  if (!u8g2.begin()) {
    Serial.println("ERROR: OLED initialization failed!");
    Serial.println("System will continue WITHOUT display - WebUI still available");
    oledAvailable = false;
    return;  // Exit early - display init failed
  }

  // OLED successfully initialized
  oledAvailable = true;
  Serial.println("OLED initialized successfully");

  // Show startup screen with proper positioning
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_8x13_tr);  // Use smaller font
  u8g2.drawStr(8, 20, "SenseFlow.in");  // Moved left and fits

  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(25, 40, "Tank Monitor");
  u8g2.drawStr(25, 55, "Starting...");
  u8g2.sendBuffer();

  delay(2000); // Show branding for 2 seconds
  Serial.println("OLED Display initialized with startup branding");
}

// Screen management functions
// Screen 0 = live level, screen 1 = trend chart (only once 3+ readings are logged)
void buildScreenList() {
  screenMgr.totalScreens = 1;
  if (tankConfig.enabled && historyCount() >= 3) {
    screenMgr.totalScreens = 2;
  }
  if (screenMgr.currentScreen >= screenMgr.totalScreens) {
    screenMgr.currentScreen = 0;
  }
}

void handleScreenTiming() {
  // Handle system info timeout - return to normal cycling
  if (screenMgr.showSystemInfo && millis() > screenMgr.systemInfoTimeout) {
    screenMgr.showSystemInfo = false;
    Serial.println("System info timeout - resuming normal cycling");
  }

  // Manual mode timeout
  if (screenMgr.manualMode && millis() > screenMgr.manualTimeout) {
    screenMgr.manualMode = false;
    Serial.println("Manual mode timeout - resuming auto-cycling");
  }

  // Auto-rotate between live and chart screens
  if (!screenMgr.manualMode && millis() - screenMgr.lastUpdate >= SCREEN_ROTATE_MS) {
    screenMgr.currentScreen = (screenMgr.currentScreen + 1) % screenMgr.totalScreens;
    screenMgr.lastUpdate = millis();
  }
}

void showCurrentScreen() {
  if (!isConfigured) {
    showUnconfiguredScreen();
    return;
  }

  // PRIORITY: Lock to alarm screen when alarm is active (until acknowledged)
  if (alarmActive) {
    showAlarmScreen();
    return;
  }

  // Double press: system info for 5s, then sensor tech details for 5s
  if (screenMgr.showSystemInfo) {
    if (millis() + 5000 < screenMgr.systemInfoTimeout) {
      showSystemScreen();
    } else {
      showSensorTechScreen();
    }
    return;
  }

  if (tankConfig.enabled) {
    if (screenMgr.currentScreen == 1) {
      showChartScreen();
    } else {
      showTankScreen();
    }
    return;
  }

  showUnconfiguredScreen();
}

// Skips if OLED not available
void updateDisplay() {
  if (!oledAvailable) return;  // Skip display updates if no OLED
  buildScreenList();
  handleScreenTiming();
  showCurrentScreen();
}

void showUnconfiguredScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 15, "Not Configured");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 30, "Use CONFIG command");
  u8g2.drawStr(0, 45, "to setup sensor");
  u8g2.sendBuffer();
}

// Heart icon (13x11 at full size). Filled + pulsing while the sensor is alive.
void drawHeart(int x, int y, bool alive) {
  if (!alive) {
    u8g2.drawCircle(x + 3, y + 3, 3);
    u8g2.drawCircle(x + 9, y + 3, 3);
    u8g2.drawLine(x, y + 4, x + 6, y + 10);
    u8g2.drawLine(x + 12, y + 4, x + 6, y + 10);
    return;
  }

  // Lub-dub: two quick beats every second
  unsigned long phase = millis() % 1000;
  bool big = (phase < 150) || (phase >= 250 && phase < 400);
  if (big) {
    u8g2.drawDisc(x + 3, y + 3, 3);
    u8g2.drawDisc(x + 9, y + 3, 3);
    u8g2.drawTriangle(x, y + 4, x + 13, y + 4, x + 6, y + 11);
  } else {
    u8g2.drawDisc(x + 4, y + 4, 2);
    u8g2.drawDisc(x + 8, y + 4, 2);
    u8g2.drawTriangle(x + 2, y + 5, x + 11, y + 5, x + 6, y + 9);
  }
}

// Three signal bars, bottom-left at (x, y)
void drawSignalBars(int x, int y, int bars) {
  for (int i = 0; i < 3; i++) {
    int h = 3 + i * 2;
    if (i < bars) {
      u8g2.drawBox(x + i * 4, y - h + 1, 3, h);
    } else {
      u8g2.drawHLine(x + i * 4, y, 3);
    }
  }
}

// Short age text: "12s", "5m", "3h"
String formatAge(unsigned long ms) {
  unsigned long s = ms / 1000;
  if (s < 60) return String(s) + "s";
  if (s < 3600) return String(s / 60) + "m";
  return String(s / 3600) + "h";
}

// Up/down/flat arrow (7x7) with top-left at (x, y)
void drawTrendArrow(int x, int y, float delta) {
  if (delta > TREND_DEADBAND) {
    u8g2.drawTriangle(x, y + 6, x + 7, y + 6, x + 3, y);
  } else if (delta < -TREND_DEADBAND) {
    u8g2.drawTriangle(x, y, x + 7, y, x + 3, y + 6);
  } else {
    u8g2.drawHLine(x, y + 3, 6);
    u8g2.drawTriangle(x + 5, y, x + 5, y + 7, x + 8, y + 3);
  }
}

// Screen 1: live level - big %, trend, depth (cm + ft/in), litres, heart, link
void showTankScreen() {
  LinkState state = getLinkState();
  bool alive = (state == LS_LIVE || state == LS_HEARTBEAT);

  u8g2.clearBuffer();

  // Narrow vertical tank on the left
  u8g2.drawFrame(0, 10, 12, 54);
  if (tankData.hasData) {
    int fillHeight = (int)(52 * tankData.currentPercent / 100.0);
    if (fillHeight > 0) {
      u8g2.drawBox(1, 63 - fillHeight, 10, fillHeight);
    }
  }

  // Title row: name + state tag, heart top-right
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(16, 8, tankConfig.customName);
  const char* tag = "";
  if (state == LS_HEARTBEAT) tag = "[HB]";
  else if (state == LS_OFFLINE) tag = "OFFLINE";
  else if (state == LS_WAITING) tag = "WAIT";
  u8g2.drawStr(70, 8, tag);
  drawHeart(114, 0, alive);

  if (!tankData.hasData) {
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(16, 32, state == LS_OFFLINE ? "Sensor offline" : "Waiting for");
    u8g2.drawStr(16, 44, state == LS_OFFLINE ? "Check sensor" : "sensor data...");
  } else {
    // Big percentage
    u8g2.setFont(u8g2_font_logisoso22_tr);
    String pct = String((int)round(tankData.currentPercent)) + "%";
    u8g2.drawStr(16, 36, pct.c_str());

    // Trend vs. up to an hour ago
    float delta;
    if (getTrend(&delta)) {
      drawTrendArrow(84, 20, delta);
      u8g2.setFont(u8g2_font_6x10_tr);
      String d = (delta >= 0 ? "+" : "") + String((int)round(delta)) + "%";
      u8g2.drawStr(95, 27, d.c_str());
    }

    // Depth in cm + ft/in
    u8g2.setFont(u8g2_font_6x10_tr);
    String depth = String((int)round(tankData.currentLevel)) + "cm " + formatFeetInches(tankData.currentLevel);
    u8g2.drawStr(16, 47, depth.c_str());

    // Volume
    if (tankConfig.volume > 0) {
      String vol = String(currentLitres()) + "/" + String(tankConfig.volume) + " L";
      u8g2.drawStr(16, 56, vol.c_str());
    }
  }

  // Bottom row: link in use + signal, or how long ago we last heard the sensor
  u8g2.setFont(u8g2_font_5x7_tr);
  if (state == LS_OFFLINE || state == LS_HEARTBEAT) {
    String ago = String(state == LS_OFFLINE ? "Last seen " : "Last read ");
    unsigned long since = (state == LS_OFFLINE) ? tankData.lastUpdate : tankData.lastDataTime;
    ago += (tankData.everHeard && since > 0) ? formatAge(millis() - since) + " ago" : String("--");
    u8g2.drawStr(16, 64, ago.c_str());
  } else {
    uint8_t via = (linkType == LINK_WIRED) ? (uint8_t)VIA_WIRED : tankData.via;
    String link = (via == VIA_NONE) ? String(linkTypeName(linkType)) : String(linkViaName(via));
    if (globalBuzzerMute) link += " MUTE";
    u8g2.drawStr(16, 64, link.c_str());
    if (via != VIA_WIRED && via != VIA_NONE) {
      drawSignalBars(116, 63, signalBars());
    }
  }

  u8g2.sendBuffer();
}

// Screen 2: line chart of the last 12 logged readings (ordered by sequence number)
void showChartScreen() {
  HistRec pts[CHART_POINTS];
  int n = getHistorySorted(pts, CHART_POINTS);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(0, 7, "Level trend");
  String count = String(n) + " readings";
  u8g2.drawStr(128 - u8g2.getStrWidth(count.c_str()), 7, count.c_str());

  const int x0 = 18, x1 = 127, yTop = 11, yBottom = 55;
  u8g2.drawStr(0, yTop + 6, "100");
  u8g2.drawStr(10, yBottom, "0");
  u8g2.drawVLine(x0 - 2, yTop, yBottom - yTop + 1);
  for (int x = x0; x <= x1; x += 4) {
    u8g2.drawPixel(x, (yTop + yBottom) / 2);  // dotted 50% line
  }

  if (n >= 2) {
    int prevX = 0, prevY = 0;
    for (int i = 0; i < n; i++) {
      int x = x0 + (long)i * (x1 - x0) / (n - 1);
      int y = yBottom - (long)pts[i].pctX10 * (yBottom - yTop) / 1000;
      u8g2.drawDisc(x, y, 1);
      if (i > 0) u8g2.drawLine(prevX, prevY, x, y);
      prevX = x;
      prevY = y;
    }
    String first = "#" + String(pts[0].seq);
    String last = "#" + String(pts[n - 1].seq);
    u8g2.drawStr(x0, 64, first.c_str());
    u8g2.drawStr(128 - u8g2.getStrWidth(last.c_str()), 64, last.c_str());
  }

  u8g2.sendBuffer();
}

void showAlarmScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(25, 12, "ALARM ACTIVE!");

  int yPos = 25;
  u8g2.setFont(u8g2_font_6x10_tr);

  // Display shows LIVE values, matches main display
  if (tankConfig.enabled && tankData.hasData) {
    String alarmText = "";
    if (!tankData.isOnline) {
      alarmText = String(tankConfig.customName) + ": OFFLINE";
    } else {
      if (tankConfig.lowAlarmPercent > 0 && tankData.currentPercent < tankConfig.lowAlarmPercent) {
        alarmText = String(tankConfig.customName) + ": LOW (" + String((int)round(tankData.currentPercent)) + "%)";
      } else if (tankConfig.highAlarmPercent > 0 && tankData.currentPercent > tankConfig.highAlarmPercent) {
        alarmText = String(tankConfig.customName) + ": HIGH (" + String((int)round(tankData.currentPercent)) + "%)";
      }
    }

    if (alarmText.length() > 0 && yPos < 50) {
      u8g2.drawStr(0, yPos, alarmText.c_str());
      yPos += 10;
    }
  }

  // Instructions
  u8g2.drawStr(0, yPos + 5, "Press button to ACK");

  // Alarm duration
  if (alarmActive) {
    unsigned long alarmDuration = (millis() - alarmStartTime) / 1000;
    int minutes = alarmDuration / 60;
    int seconds = alarmDuration % 60;
    String durationStr = "Buzzing: " + String(minutes) + "m " + String(seconds) + "s";
    u8g2.drawStr(0, yPos + 15, durationStr.c_str());
  }

  u8g2.sendBuffer();
}

void showSystemScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 12, deviceName.c_str());

  u8g2.setFont(u8g2_font_6x10_tr);

  // WiFi info
  if (wifiStationMode) {
    String wifiStr = "WiFi: " + wifiSSID;
    if (wifiStr.length() > 20) wifiStr = wifiStr.substring(0, 20);
    u8g2.drawStr(0, 25, wifiStr.c_str());
    u8g2.drawStr(0, 35, ("IP: " + WiFi.localIP().toString()).c_str());
    u8g2.drawStr(0, 45, "OTA: Ready");
  } else {
    u8g2.drawStr(0, 25, "WiFi: AP Mode Only");
    u8g2.drawStr(0, 35, "OTA: Unavailable");
  }

  u8g2.drawStr(0, 55, ("AP: " + WiFi.softAPIP().toString()).c_str());

  String fwStr = "FW v" + String(FIRMWARE_VERSION);
  u8g2.drawStr(0, 64, fwStr.c_str());

  u8g2.sendBuffer();
}

// Sensor technical details (second half of the double-press info)
void showSensorTechScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 12, "Sensor Link");

  u8g2.setFont(u8g2_font_6x10_tr);
  String line = String("Type: ") + linkTypeName(linkType);
  u8g2.drawStr(0, 24, line.c_str());

  if (linkType == LINK_WIRED) {
    line = "Echo: " + String(wiredNoEchoCycles >= WIRED_OFFLINE_CYCLES ? "NONE" : "OK");
  } else {
    line = "ID: " + String(tankConfig.nodeID[0] ? tankConfig.nodeID : "not paired");
  }
  u8g2.drawStr(0, 34, line.c_str());

  if (linkType != LINK_WIRED && tankData.rssi != 0) {
    line = "Signal: " + String(tankData.rssi) + " dBm";
  } else if (linkType == LINK_LONG_RANGE && !loraPresent) {
    line = "Long Range: no module";
  } else {
    line = String("Via: ") + linkViaName(tankData.via);
  }
  u8g2.drawStr(0, 44, line.c_str());

  line = "Last: " + (tankData.everHeard ? formatAge(millis() - tankData.lastUpdate) + " ago" : String("--"));
  u8g2.drawStr(0, 54, line.c_str());

  line = "Rx:" + String(tankData.packets) + " HB:" + String(tankData.heartbeats) + " Dup:" + String(tankData.duplicates);
  u8g2.drawStr(0, 64, line.c_str());

  u8g2.sendBuffer();
}

// ===========================================
// SERIAL COMMANDS
// ===========================================

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toUpperCase();

    if (command == "CONFIG") {
      runConfiguration();
    } else if (command == "STATUS") {
      showSystemStatus();
    } else if (command == "ALARM") {
      testAlarm();
    } else if (command == "WIFI") {
      configureWiFiTerminal();
    } else if (command == "RESET") {
      String confirm = getStringInput("Reset all configuration? This cannot be undone! (Y/N): ", 1);
      if (confirm.equalsIgnoreCase("Y")) {
        resetConfiguration();
        ESP.restart();
      }
    } else if (command == "RESTART") {
      Serial.println("Restarting system...");
      ESP.restart();
    } else if (command == "MUTE") {
      if (!globalBuzzerMute) {
        globalBuzzerMute = true;
        saveGlobalMuteConfig();
        Serial.println("BUZZER MUTED");
        Serial.println("Visual alarms will remain active");
      } else {
        Serial.println("Buzzer is already muted");
      }
    } else if (command == "UNMUTE") {
      if (globalBuzzerMute) {
        globalBuzzerMute = false;
        saveGlobalMuteConfig();
        Serial.println("BUZZER UNMUTED");
        Serial.println("Alarm buzzer is now active");
      } else {
        Serial.println("Buzzer is already unmuted");
      }
    } else if (command == "MUTESTATUS") {
      Serial.println("\n=== BUZZER MUTE STATUS ===");
      Serial.print("Global Mute: ");
      Serial.println(globalBuzzerMute ? "ENABLED (Muted)" : "DISABLED (Active)");
      Serial.print("Buzzer Enabled in Settings: ");
      Serial.println(generalSettings.buzzerEnabled ? "Yes" : "No");
      Serial.println("\nCommands:");
      Serial.println("  MUTE    - Enable global buzzer mute");
      Serial.println("  UNMUTE  - Disable global buzzer mute");
    } else if (command == "HISTORY") {
      HistRec recs[HISTORY_SLOTS];
      int n = getHistorySorted(recs, HISTORY_SLOTS);
      Serial.printf("\n=== HISTORY (%d readings) ===\n", n);
      for (int i = 0; i < n; i++) {
        Serial.printf("#%lu  %5.1f%%  %6.1fcm", (unsigned long)recs[i].seq, recs[i].pctX10 / 10.0, recs[i].levelX10 / 10.0);
        if (recs[i].epoch) {
          time_t t = recs[i].epoch;
          struct tm tmv;
          gmtime_r(&t, &tmv);
          Serial.printf("  %04d-%02d-%02d %02d:%02d UTC", tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min);
        }
        Serial.println();
      }
    } else if (command == "LINK") {
      configureConnection();
    } else {
      Serial.println("Available commands: CONFIG, STATUS, ALARM, WIFI, LINK, HISTORY, RESET, RESTART, MUTE, UNMUTE, MUTESTATUS");
    }
  }
}

void showSystemStatus() {
  Serial.println("\n=== SYSTEM STATUS ===");
  Serial.print("Device: ");
  Serial.println(fullDeviceName);
  Serial.print("Configuration: ");
  Serial.println(isConfigured ? "Complete" : "Incomplete");
  Serial.print("Alarm System: ");
  Serial.println(generalSettings.buzzerEnabled ? "Enabled" : "Disabled");
  Serial.print("Display Mode: ");
  if (screenMgr.manualMode) {
    Serial.println("Manual Override Active");
  } else {
    Serial.println("Auto-Cycling");
  }

  Serial.println("\nWiFi Status:");
  Serial.print("Configured: ");
  Serial.println(wifiConfigured ? "Yes" : "No");
  if (wifiConfigured) {
    Serial.print("SSID: ");
    Serial.println(wifiSSID);
    Serial.print("Connected: ");
    Serial.println(wifiStationMode ? "Yes" : "No");
    if (wifiStationMode) {
      Serial.print("Station IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("Signal: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
    }
  }
  // Clean device name first, then add _mvstech suffix
  String cleanDeviceName = deviceName;
  cleanDeviceName.replace(" ", "-");
  cleanDeviceName.replace("_", "-");
  cleanDeviceName.replace(".", "-");
  cleanDeviceName.toLowerCase();
  String apName = cleanDeviceName + "_mvstech";
  Serial.print("AP SSID: ");
  Serial.println(apName);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  if (alarmActive) {
    Serial.print("Alarm Status: ACTIVE");
    if (allAlarmsAcknowledged) {
      Serial.print(" (All conditions acknowledged)");
    }
    Serial.println();
  } else {
    Serial.println("Alarm Status: Inactive");
  }

  showIndividualSensorReadings();
}

// ===========================================
// WEB SERVER FUNCTIONS
// ===========================================

void setupAccessPoint() {
  // Clean device name first (replace spaces, underscores, dots)
  String cleanDeviceName = deviceName;
  cleanDeviceName.replace(" ", "-");
  cleanDeviceName.replace("_", "-");
  cleanDeviceName.replace(".", "-");
  cleanDeviceName.toLowerCase();

  // Add _mvstech suffix AFTER cleaning
  String apName = cleanDeviceName + "_mvstech";

  // IMPORTANT: Set WiFi mode to AP before starting soft AP
  WiFi.mode(WIFI_AP);
  delay(100);  // Small delay to ensure mode is set

  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(apName.c_str(), "mvstech9867");

  Serial.println("Access Point Started:");
  Serial.print("SSID: ");
  Serial.println(apName);
  Serial.println("Password: mvstech9867");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  apModeActive = true;
}

void setupWebServer() {
  // Enable User-Agent header collection for app verification
  const char* headerKeys[] = {"User-Agent"};
  server.collectHeaders(headerKeys, 1);

  server.on("/", handleWebRoot);
  server.on("/wifi", HTTP_GET, handleWiFiPage);
  server.on("/wifi", HTTP_POST, handleWiFiPost);
  server.on("/config", handleConfigPage);
  server.on("/api/config", HTTP_GET, handleConfigAPI);
  server.on("/api/tank", HTTP_PUT, handleUpdateTank);
  server.on("/alarm/acknowledge", HTTP_POST, handleAlarmAcknowledge);
  server.on("/api/status", handleAPIStatus);
  server.on("/api/device-name", HTTP_POST, handleDeviceNameUpdate);
  server.on("/api/mute/toggle", HTTP_POST, handleMuteToggle);
  server.on("/api/mute/status", HTTP_GET, handleMuteStatus);
  server.on("/api/history", HTTP_GET, handleAPIHistory);
  server.on("/api/history/clear", HTTP_POST, handleHistoryClear);
  server.on("/api/link", HTTP_POST, handleLinkUpdate);
  server.on("/api/logging", HTTP_POST, handleLoggingUpdate);
  server.on("/api/scan", handleAPIScan);
  server.on("/api/pair", HTTP_POST, handlePair);

  server.begin();
  Serial.println("Web UI started on port 7689");
  Serial.print("Access WebUI at: http://");
  Serial.print(WiFi.softAPIP());
  Serial.println(":7689");

  // Setup mDNS with device name + "-mvstech" suffix
  String mdnsName = deviceName + "_mvstech";
  if (MDNS.begin(mdnsName.c_str())) {
    Serial.print("mDNS responder started: ");
    Serial.print(mdnsName);
    Serial.println(".local");
    MDNS.addService("http", "tcp", 7689);  // Fixed: Use correct port 7689
  } else {
    Serial.println("Error setting up mDNS responder!");
  }
}

// Check if request is from SenseFlow app (User-Agent verification)
bool isAuthorizedApp() {
  String userAgent = server.header("User-Agent");
  return (userAgent.indexOf(USER_AGENT_CHECK) >= 0);
}

void sendUnauthorizedResponse() {
  server.send(403, "text/html", "<!DOCTYPE html><html><body style='font-family:Arial;text-align:center;padding:50px'><h2>Access Restricted</h2><p>This device is accessible only through the <strong>SenseFlow</strong> app.</p><p>Download the app to monitor your sensor.</p></body></html>");
}

void handleWebRoot() {
  if (!isAuthorizedApp()) {
    Serial.println("Main page blocked - unauthorized User-Agent");
    sendUnauthorizedResponse();
    return;
  }
  Serial.println("Main page requested");
  String html = getMainPageHTML();
  Serial.print("Main page HTML length: ");
  Serial.println(html.length());
  server.send(200, "text/html", html);
  Serial.println("Main page sent");
}

void handleWiFiPage() {
  String html = getWiFiPageHTML();
  server.send(200, "text/html", html);
}

void handleWiFiPost() {
  // Process WiFi credentials
  if (server.hasArg("ssid") && server.hasArg("password")) {
    String newSSID = server.arg("ssid");
    String newPassword = server.arg("password");

    newSSID.trim();
    newPassword.trim();

    if (newSSID.length() > 0) {
      wifiSSID = newSSID;
      wifiPassword = newPassword;
      saveWiFiCredentials();

      Serial.print("WiFi config received: ");
      Serial.println(wifiSSID);

      connectToWiFi();

      // Return JSON response for AJAX
      if (wifiStationMode) {
        server.send(200, "application/json", "{\"success\":true,\"message\":\"Connected to WiFi\"}");
      } else {
        server.send(200, "application/json", "{\"success\":false,\"message\":\"Could not connect. Check credentials.\"}");
      }
      return;
    }
  }

  server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid WiFi credentials\"}");
}

void handleAlarmAcknowledge() {
  if (!isAuthorizedApp()) { server.send(403, "application/json", "{\"error\":\"Unauthorized\"}"); return; }
  acknowledgeAllAlarms();
  server.send(200, "text/plain", "Alarms acknowledged");
}

// ===========================================
// MUTE API HANDLERS
// ===========================================

void handleMuteToggle() {
  if (!isAuthorizedApp()) { server.send(403, "application/json", "{\"error\":\"Unauthorized\"}"); return; }
  toggleGlobalMute();

  DynamicJsonDocument doc(200);
  doc["success"] = true;
  doc["muted"] = globalBuzzerMute;
  doc["message"] = globalBuzzerMute ? "Buzzer muted" : "Buzzer active";

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleMuteStatus() {
  if (!isAuthorizedApp()) { server.send(403, "application/json", "{\"error\":\"Unauthorized\"}"); return; }
  DynamicJsonDocument doc(200);
  doc["muted"] = globalBuzzerMute;
  doc["buzzer_enabled"] = generalSettings.buzzerEnabled;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleAPIStatus() {
  if (!isAuthorizedApp()) { server.send(403, "application/json", "{\"error\":\"Unauthorized\"}"); return; }
  Serial.println("API Status requested");

  // Use ArduinoJson library for proper JSON generation
  DynamicJsonDocument doc(2048);

  doc["configured"] = isConfigured;
  doc["alarm_active"] = alarmActive;
  doc["all_acknowledged"] = allAlarmsAcknowledged;
  doc["wifi_station"] = wifiStationMode;

  // Add detailed alarm information
  JsonArray activeAlarms = doc.createNestedArray("active_alarms");
  if (alarmActive && tankConfig.enabled) {
    if (tankData.isOnline) {
      if (!isnan(tankData.currentPercent) && tankData.currentPercent >= 0 && tankData.currentPercent <= 100) {
        if (tankConfig.lowAlarmPercent > 0 && tankData.currentPercent <= tankConfig.lowAlarmPercent && !alarmAck.tankLowThresholdAck) {
          JsonObject alarm = activeAlarms.createNestedObject();
          tankConfig.customName[8] = '\0'; // Ensure null termination
          alarm["type"] = "Tank Low Level";
          alarm["sensor"] = String(tankConfig.customName);
          alarm["message"] = "Tank level is " + String((int)round(tankData.currentPercent)) + "% (below " + String(tankConfig.lowAlarmPercent) + "% threshold)";
        }
        if (tankConfig.highAlarmPercent > 0 && tankData.currentPercent >= tankConfig.highAlarmPercent && !alarmAck.tankHighThresholdAck) {
          JsonObject alarm = activeAlarms.createNestedObject();
          tankConfig.customName[8] = '\0'; // Ensure null termination
          alarm["type"] = "Tank High Level";
          alarm["sensor"] = String(tankConfig.customName);
          alarm["message"] = "Tank level is " + String((int)round(tankData.currentPercent)) + "% (above " + String(tankConfig.highAlarmPercent) + "% threshold)";
        }
      }
    }
    if (!tankData.isOnline && !alarmAck.tankOfflineAck) {
      JsonObject alarm = activeAlarms.createNestedObject();
      tankConfig.customName[8] = '\0'; // Ensure null termination
      alarm["type"] = "Tank Offline";
      alarm["sensor"] = String(tankConfig.customName);
      alarm["message"] = "Tank sensor is offline (no reading received)";
    }
  }

  // Tank data
  JsonObject tank = doc.createNestedObject("tank");
  tank["enabled"] = tankConfig.enabled;

  if (tankConfig.enabled) {
    tankConfig.customName[8] = '\0'; // Ensure null termination

    tank["name"] = String(tankConfig.customName);
    tank["online"] = tankData.isOnline;
    tank["percent"] = tankData.currentPercent;
    tank["level"] = tankData.currentLevel;
    tank["lastUpdate"] = tankData.everHeard ? millis() - tankData.lastUpdate : 0;

    // Heartbeat / link details
    LinkState state = getLinkState();
    tank["hasData"] = tankData.hasData;
    tank["state"] = linkStateKey(state);
    tank["lastData"] = tankData.hasData ? millis() - tankData.lastDataTime : 0;
    tank["levelFtIn"] = formatFeetInches(tankData.currentLevel);
    tank["litres"] = currentLitres();
    tank["volume"] = tankConfig.volume;
    tank["height"] = tankConfig.height;
    float delta;
    if (tankData.hasData && getTrend(&delta)) {
      tank["trend"] = round(delta * 10) / 10.0;
    }

    JsonObject link = doc.createNestedObject("link");
    link["type"] = linkTypeName(linkType);
    uint8_t via = (linkType == LINK_WIRED) ? (uint8_t)VIA_WIRED : tankData.via;
    link["via"] = (via == VIA_NONE) ? linkTypeName(linkType) : linkViaName(via);
    link["sensor"] = tankConfig.nodeID;
    link["rssi"] = tankData.rssi;
    link["bars"] = signalBars();
  }

  String json;
  size_t jsonSize = serializeJson(doc, json);

  if (jsonSize == 0) {
    Serial.println("ERROR: JSON serialization failed!");
    server.send(500, "application/json", "{\"error\":\"Serialization failed\"}");
    return;
  }

  Serial.print("JSON response length: ");
  Serial.println(json.length());

  server.send(200, "application/json", json);
}

// ===========================================
// CONFIGURATION API HANDLERS
// ===========================================

void handleConfigPage() {
  Serial.println("Settings page requested");
  String html = getConfigPageHTML();
  server.send(200, "text/html", html);
}

void handleConfigAPI() {
  Serial.println("Configuration API requested");

  DynamicJsonDocument doc(1024);

  JsonObject tank = doc.createNestedObject("tank");
  tank["enabled"] = tankConfig.enabled;
  if (tankConfig.enabled) {
    tankConfig.customName[8] = '\0';
    tank["name"] = String(tankConfig.customName);
    tank["height"] = tankConfig.height;
    tank["volume"] = tankConfig.volume;
    tank["lowAlarm"] = tankConfig.lowAlarmPercent;
    tank["highAlarm"] = tankConfig.highAlarmPercent;
    tank["online"] = tankData.isOnline;
  }

  JsonObject link = doc.createNestedObject("link");
  link["type"] = (int)linkType;
  link["sensor"] = tankConfig.nodeID;
  link["longRangeModule"] = loraPresent;
  link["longRangeChecked"] = linkUsesLoRa();
  link["wifiConnected"] = wifiStationMode;

  JsonObject logging = doc.createNestedObject("logging");
  logging["threshold"] = logThresholdPercent;
  logging["interval"] = logIntervalMinutes;
  logging["count"] = historyCount();

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

// History for the web chart: oldest first, ordered by sequence number
void handleAPIHistory() {
  if (!isAuthorizedApp()) { server.send(403, "application/json", "{\"error\":\"Unauthorized\"}"); return; }

  HistRec recs[HISTORY_SLOTS];
  int n = getHistorySorted(recs, HISTORY_SLOTS);

  String json;
  json.reserve(60 + n * 48);
  json = "{\"now\":" + String(currentEpoch()) + ",\"points\":[";
  for (int i = 0; i < n; i++) {
    if (i > 0) json += ",";
    json += "{\"q\":" + String(recs[i].seq) +
            ",\"t\":" + String(recs[i].epoch) +
            ",\"p\":" + String(recs[i].pctX10 / 10.0, 1) +
            ",\"l\":" + String(recs[i].levelX10 / 10.0, 1) + "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleHistoryClear() {
  clearHistory();
  server.send(200, "application/json", "{\"success\":true}");
}

// Change connection type - restarts so the radios start cleanly
void handleLinkUpdate() {
  if (!server.hasArg("type")) {
    server.send(400, "application/json", "{\"error\":\"Missing type\"}");
    return;
  }
  int t = server.arg("type").toInt();
  if (t < 0 || t >= LINK_TYPE_COUNT) {
    server.send(400, "application/json", "{\"error\":\"Invalid connection type\"}");
    return;
  }

  bool changed = (t != (int)linkType);
  linkType = (LinkType)t;
  saveLinkSettings();
  server.send(200, "application/json", changed ? "{\"success\":true,\"restart\":true}" : "{\"success\":true}");
  if (changed) {
    Serial.print("Connection type changed to ");
    Serial.print(linkTypeName(linkType));
    Serial.println(" - restarting");
    delay(500);
    ESP.restart();
  }
}

void handleLoggingUpdate() {
  if (server.hasArg("threshold")) {
    logThresholdPercent = (server.arg("threshold").toInt() <= 5) ? 5 : 10;
  }
  if (server.hasArg("interval")) {
    logIntervalMinutes = (server.arg("interval").toInt() <= 30) ? 30 : 60;
  }
  saveLinkSettings();
  server.send(200, "application/json", "{\"success\":true}");
}

// Sensor discovery: ?action=start | stop, otherwise returns results
void handleAPIScan() {
  String action = server.hasArg("action") ? server.arg("action") : "status";

  if (action == "start") {
    if (!linkIsWireless()) {
      server.send(400, "application/json", "{\"error\":\"Wired connection does not need pairing\"}");
      return;
    }
    startSensorScan();
    server.send(200, "application/json", "{\"status\":\"scanning\"}");
    return;
  }
  if (action == "stop") {
    scanActive = false;
    server.send(200, "application/json", "{\"status\":\"stopped\"}");
    return;
  }

  DynamicJsonDocument doc(1024);
  doc["scanning"] = scanActive;
  doc["remaining"] = scanActive ? (SCAN_DURATION_MS - (millis() - scanStartTime)) / 1000 : 0;
  JsonArray devices = doc.createNestedArray("devices");
  for (int i = 0; i < scanCount; i++) {
    JsonObject d = devices.createNestedObject();
    d["id"] = scanResults[i].id;
    d["via"] = linkViaName(scanResults[i].via);
    d["rssi"] = scanResults[i].rssi;
    d["paired"] = (strcmp(scanResults[i].id, tankConfig.nodeID) == 0);
  }
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handlePair() {
  String id = server.hasArg("id") ? server.arg("id") : "";
  id.trim();
  id.toUpperCase();
  if (!isValidNodeID(id)) {
    server.send(400, "application/json", "{\"error\":\"Sensor ID must be US followed by 4 digits\"}");
    return;
  }
  pairSensor(id);
  server.send(200, "application/json", "{\"success\":true}");
}

void handleUpdateTank() {
  // Update sensor
  if (server.hasArg("name")) {
    String name = server.arg("name");
    if (name.length() > 0 && name.length() <= 8) {
      strncpy(tankConfig.customName, name.c_str(), 8);
    }
  }
  if (server.hasArg("height")) {
    tankConfig.height = server.arg("height").toInt();
  }
  if (server.hasArg("volume")) {
    tankConfig.volume = server.arg("volume").toInt();
  }
  if (server.hasArg("lowAlarm")) {
    tankConfig.lowAlarmPercent = server.arg("lowAlarm").toInt();
  }
  if (server.hasArg("highAlarm")) {
    tankConfig.highAlarmPercent = server.arg("highAlarm").toInt();
  }

  tankConfig.enabled = true;
  saveConfiguration();
  server.send(200, "application/json", "{\"success\":true}");
}

void handleDeviceNameUpdate() {
  if (!server.hasArg("name")) {
    server.send(400, "application/json", "{\"error\":\"Missing name parameter\"}");
    return;
  }

  String newName = server.arg("name");
  newName.trim();

  // Validate device name
  if (newName.length() == 0 || newName.length() > 20) {
    server.send(400, "application/json", "{\"error\":\"Device name must be 1-20 characters\"}");
    return;
  }

  // Check for invalid characters
  for (int i = 0; i < newName.length(); i++) {
    char c = newName[i];
    if (!isAlphaNumeric(c) && c != ' ' && c != '-' && c != '_') {
      server.send(400, "application/json", "{\"error\":\"Device name contains invalid characters. Use only letters, numbers, spaces, hyphens, and underscores.\"}");
      return;
    }
  }

  // Update device name
  deviceName = newName;
  saveDeviceName();
  updateDynamicNames();

  Serial.print("Device name updated to: ");
  Serial.println(deviceName);

  server.send(200, "application/json", "{\"success\":true,\"message\":\"Device name updated successfully\"}");
}

// Dashboard: live level with heartbeat status, link, trend and the 24h history chart
static const char MAIN_PAGE_HEAD[] = R"rawliteral(<!DOCTYPE html><html><head>
<meta charset='UTF-8'><title>Tank Monitor</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
body{font-family:Arial;margin:20px;background:#f0f8ff;color:#2c3e50}
.container{max-width:600px;margin:0 auto;background:white;padding:20px;border-radius:10px}
.header{text-align:center;color:#2c3e50;border-bottom:2px solid #3498db;padding-bottom:10px;margin-bottom:20px}
.status{background:#f8f9fa;padding:15px;border-radius:5px;margin:10px 0}
.alarm{background:#ffe6e6;border:2px solid #ff4444;padding:15px;border-radius:5px;margin:10px 0}
.alarm-details{background:#ffcccc;border:1px solid #ff6666;padding:10px;margin:10px 0;border-radius:5px}
.alarm-item{background:#fff2f2;border-left:4px solid #ff4444;padding:8px;margin:5px 0;border-radius:3px}
.alarm-message{font-size:0.9em;color:#666;font-style:italic}
.btn{background:#3498db;color:white;padding:10px 20px;border:none;border-radius:5px;margin:5px;cursor:pointer;text-decoration:none;display:inline-block}
.btn-alarm{background:#e74c3c}.btn-mute{background:#f39c12}.btn-unmute{background:#27ae60}.btn:hover{opacity:0.8}
.mute-control{background:#fff3cd;border:2px solid #f39c12;padding:15px;border-radius:5px;margin:10px 0;text-align:center}
.mute-status{font-weight:bold;font-size:1.1em;margin:10px 0}
.tank-row{display:flex;align-items:center;gap:15px}
.mini-tank{flex-shrink:0;width:40px}
.mt-cap{width:25px;height:3px;background:#34495e;border-radius:50% 50% 0 0;margin:0 auto}
.mt-top{width:36px;height:10px;background:linear-gradient(to bottom,#e8eef1,#d5dbdb);clip-path:polygon(15% 0,85% 0,100% 100%,0 100%);margin:0 auto}
.mt-body{width:36px;height:70px;background:linear-gradient(to bottom,#ecf0f1,#d5dbdb);border:2px solid #34495e;border-top:none;border-radius:0 0 6px 6px;position:relative;overflow:hidden;margin:0 auto}
.mt-water{position:absolute;bottom:0;width:100%;background:linear-gradient(to top,#1565c0,#42a5f5);transition:height 0.5s}
.mt-water.stale{background:linear-gradient(to top,#7f8c8d,#bdc3c7)}
.tank-details{flex:1;min-width:0}
.big{font-size:2.4em;font-weight:bold;line-height:1.1}
.trend{font-size:0.9em;color:#555;margin-left:8px}
.row{margin:3px 0;font-size:0.95em}
.ago{font-size:0.8em;color:#888}
.gear{text-decoration:none;font-size:1.4em;color:#2980b9;float:right;cursor:pointer;font-weight:bold}
.pill{display:inline-flex;align-items:center;gap:6px;padding:3px 10px;border-radius:12px;font-size:0.85em;font-weight:bold;margin:4px 0}
.st-live{background:#e8f8ef;color:#1e8449}.st-heartbeat{background:#fef5e7;color:#9a6700}
.st-offline{background:#fdecea;color:#c0392b}.st-waiting{background:#eef2f5;color:#566573}
.heart{display:inline-block;color:#e74c3c;font-size:1.1em}
.heart.beat{animation:beat 1s infinite}
@keyframes beat{0%,40%,100%{transform:scale(1)}10%,30%{transform:scale(1.35)}}
.bars{display:inline-flex;align-items:flex-end;gap:2px;height:12px;margin-left:4px;vertical-align:middle}
.bars i{display:block;width:4px;background:#d5dbdb;border-radius:1px}
.bars i.on{background:#2c3e50}
.chart-wrap{position:relative}
#chart{width:100%;height:200px;display:block;touch-action:none}
.tip{position:absolute;pointer-events:none;background:#fff;border:1px solid #d5dbdb;border-radius:4px;padding:4px 8px;font-size:0.8em;color:#2c3e50;box-shadow:0 2px 6px rgba(0,0,0,0.15);display:none;white-space:nowrap}
table.hist{width:100%;border-collapse:collapse;font-size:0.85em;margin-top:8px}
table.hist td,table.hist th{padding:3px 6px;border-bottom:1px solid #eee;text-align:left}
</style>
<script>
function timeAgo(ms){if(!ms||ms<=0)return '';const s=Math.round(ms/1000);
if(s<60)return s+'s ago';if(s<3600)return Math.floor(s/60)+'m ago';return Math.floor(s/3600)+'h ago';}
function esc(s){return String(s==null?'':s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}
const STATE_TEXT={live:'Live',heartbeat:'Sensor alive - showing last reading',offline:'Offline - showing last reading',waiting:'Waiting for sensor'};
function bars(n){let h='<span class="bars">';for(let i=0;i<3;i++){h+='<i class="'+(i<n?'on':'')+'" style="height:'+(4+i*4)+'px"></i>';}return h+'</span>';}
function buildStatusHTML(data){
try{
if(!data){return '<div class="status">No data available</div>';}
let html='';
if(data.alarm_active){
html+='<div class="alarm"><h3>ALARM ACTIVE!</h3>';
if(data.active_alarms&&data.active_alarms.length>0){
html+='<div class="alarm-details"><h4>Active Alarms:</h4>';
for(const a of data.active_alarms){html+='<div class="alarm-item"><strong>'+esc(a.type||'Unknown')+'</strong> - '+esc(a.sensor||'')+'<br><span class="alarm-message">'+esc(a.message||'')+'</span></div>';}
html+='</div>';}
html+='<button class="btn btn-alarm" onclick="acknowledgeAlarms()">Acknowledge All Alarms</button>';
if(data.all_acknowledged){html+='<p>All current conditions acknowledged. Buzzer will restart if conditions worsen.</p>';}
html+='</div>';}
const tank=data.tank,link=data.link||{};
if(tank&&tank.enabled){
const st=tank.state||'waiting',has=!!tank.hasData,alive=(st==='live'||st==='heartbeat');
const pct=has&&tank.percent!=null?Math.round(tank.percent):0;
html+='<div class="status"><div class="tank-row">';
html+='<div class="mini-tank"><div class="mt-cap"></div><div class="mt-top"></div><div class="mt-body">';
html+='<div class="mt-water'+(st==='live'?'':' stale')+'" style="height:'+(has?pct:0)+'%"></div></div></div>';
html+='<div class="tank-details"><a class="gear" href="/config" title="Settings">&#x2699;</a>';
html+='<strong>'+esc(tank.name||'Tank')+'</strong><br>';
html+='<span class="pill st-'+st+'"><span class="heart'+(alive?' beat':'')+'">'+(alive?'&#9829;':'&#9825;')+'</span>'+STATE_TEXT[st]+'</span>';
if(tank.lastUpdate){html+=' <span class="ago">'+(st==='live'?'updated ':'last seen ')+timeAgo(tank.lastUpdate)+'</span>';}
if(has){
html+='<div><span class="big">'+pct+'%</span>';
if(tank.trend!=null){const t=tank.trend;const arrow=t>1?'&#9650;':(t<-1?'&#9660;':'&#9654;');
html+='<span class="trend">'+arrow+' '+(t>0?'+':'')+t.toFixed(0)+'% in last hour</span>';}
html+='</div>';
html+='<div class="row">Water height: <strong>'+tank.level.toFixed(0)+' cm</strong> ('+esc(tank.levelFtIn)+')</div>';
if(tank.volume>0){html+='<div class="row">Volume: <strong>'+tank.litres+' L</strong> / '+tank.volume+' L</div>';}
if(st==='heartbeat'&&tank.lastData){html+='<div class="row ago">Last good reading '+timeAgo(tank.lastData)+'</div>';}
}else{html+='<div class="row">No reading yet</div>';}
html+='<div class="row ago">Connection: '+esc(link.via||link.type||'')+(link.bars>0?bars(link.bars):'')+(link.sensor?' &middot; Sensor '+esc(link.sensor):'')+'</div>';
html+='</div></div></div>';
}else{html+='<div class="status">No tank sensor configured</div>';}
return html;
}catch(e){console.error('Error in buildStatusHTML:',e);return '<div class="status">Error displaying data</div>';}}
function updateStatus(){
fetch('/api/status').then(r=>{if(!r.ok){throw new Error('HTTP '+r.status);}return r.json();})
.then(data=>{document.getElementById('status').innerHTML=buildStatusHTML(data);})
.catch(e=>{console.error('Update failed:',e);document.getElementById('status').innerHTML='<div class="status">Error loading data: '+esc(e.message)+'</div>';});}
function acknowledgeAlarms(){
fetch('/alarm/acknowledge',{'method':'POST'}).then(r=>{if(!r.ok){throw new Error('HTTP '+r.status);}updateStatus();})
.catch(e=>{alert('Failed to acknowledge alarms: '+e.message);});}
function toggleMute(){
fetch('/api/mute/toggle',{'method':'POST'}).then(r=>{if(!r.ok){throw new Error('HTTP '+r.status);}return r.json();})
.then(()=>{updateMuteStatus();}).catch(e=>{alert('Failed to toggle mute: '+e.message);});}
function updateMuteStatus(){
fetch('/api/mute/status').then(r=>{if(!r.ok){throw new Error('HTTP '+r.status);}return r.json();}).then(data=>{
const d=document.getElementById('mute-control');if(!d)return;
let h='<h3>Alarm Buzzer Control</h3><div class="mute-status">';
h+=data.muted?'Status: <span style="color:#f39c12">MUTED</span><br><small>Visual alarms remain active</small>':'Status: <span style="color:#27ae60">ACTIVE</span>';
h+='</div><button class="btn '+(data.muted?'btn-unmute':'btn-mute')+'" onclick="toggleMute()">'+(data.muted?'Unmute':'Mute')+' Buzzer</button>';
d.innerHTML=h;}).catch(e=>{console.error('Failed to get mute status:',e);});}

// ---- History chart (single series, ordered by reading number) ----
let chartPts=[],chartHover=-1;
function ptLabel(p,full){
if(p.t>0){const d=new Date(p.t*1000);const hm=d.toLocaleTimeString([],{hour:'2-digit',minute:'2-digit'});
return full?d.toLocaleDateString([],{day:'numeric',month:'short'})+' '+hm:hm;}
return 'Reading #'+p.q;}
function loadHistory(){
fetch('/api/history').then(r=>r.json()).then(d=>{
let pts=d.points||[];
// With real time available, keep the last 24 hours; otherwise the latest readings
if(d.now>0){const cut=d.now-86400;const recent=pts.filter(p=>p.t===0||p.t>=cut);if(recent.length>=2)pts=recent;}
chartPts=pts;drawChart();buildTable();})
.catch(e=>{document.getElementById('chart-note').textContent='History unavailable';});}
function chartGeom(c){const w=c.clientWidth,h=c.clientHeight;return {w:w,h:h,l:38,r:w-10,t:10,b:h-26};}
function drawChart(){
const c=document.getElementById('chart');if(!c)return;
const note=document.getElementById('chart-note');
const dpr=window.devicePixelRatio||1;c.width=c.clientWidth*dpr;c.height=c.clientHeight*dpr;
const x=c.getContext('2d');x.setTransform(dpr,0,0,dpr,0,0);
const g=chartGeom(c);x.clearRect(0,0,g.w,g.h);
x.font='11px Arial';x.textBaseline='middle';
// Recessive grid + y axis labels
for(const v of [0,50,100]){const y=g.b-(g.b-g.t)*v/100;
x.strokeStyle='#e5e8e8';x.lineWidth=1;x.beginPath();x.moveTo(g.l,y);x.lineTo(g.r,y);x.stroke();
x.fillStyle='#7f8c8d';x.textAlign='right';x.fillText(v+'%',g.l-6,y);}
const n=chartPts.length;
if(n<2){note.textContent=n===0?'No readings logged yet.':'Waiting for more readings to draw the chart.';return;}
note.textContent=n+' readings'+(chartPts[0].t>0?'':' (connect WiFi to see clock times)');
const px=i=>g.l+(g.r-g.l)*i/(n-1),py=p=>g.b-(g.b-g.t)*p/100;
// X labels: first, middle, last
x.fillStyle='#7f8c8d';x.textBaseline='top';
[[0,'left'],[Math.floor((n-1)/2),'center'],[n-1,'right']].forEach(([i,a])=>{x.textAlign=a;x.fillText(ptLabel(chartPts[i],false),px(i),g.b+8);});
// Line
x.strokeStyle='#2471a3';x.lineWidth=2;x.lineJoin='round';x.beginPath();
chartPts.forEach((p,i)=>{const X=px(i),Y=py(p.p);if(i===0)x.moveTo(X,Y);else x.lineTo(X,Y);});x.stroke();
// Latest point marker
const lx=px(n-1),ly=py(chartPts[n-1].p);
x.fillStyle='#2471a3';x.strokeStyle='#fff';x.lineWidth=2;x.beginPath();x.arc(lx,ly,4,0,7);x.fill();x.stroke();
// Hover crosshair
if(chartHover>=0&&chartHover<n){const X=px(chartHover),Y=py(chartPts[chartHover].p);
x.strokeStyle='#aab7b8';x.lineWidth=1;x.beginPath();x.moveTo(X,g.t);x.lineTo(X,g.b);x.stroke();
x.fillStyle='#2471a3';x.strokeStyle='#fff';x.lineWidth=2;x.beginPath();x.arc(X,Y,5,0,7);x.fill();x.stroke();}}
function chartPointer(ev){
const c=document.getElementById('chart'),tip=document.getElementById('chart-tip');const n=chartPts.length;if(n<2)return;
const rect=c.getBoundingClientRect();const cx=(ev.touches?ev.touches[0].clientX:ev.clientX)-rect.left;
const g=chartGeom(c);let i=Math.round((cx-g.l)/(g.r-g.l)*(n-1));i=Math.max(0,Math.min(n-1,i));
chartHover=i;drawChart();const p=chartPts[i];
tip.innerHTML='<strong>'+p.p.toFixed(1)+'%</strong> &middot; '+p.l.toFixed(0)+' cm<br>'+esc(ptLabel(p,true))+(p.t>0?' &middot; #'+p.q:'');
tip.style.display='block';const X=g.l+(g.r-g.l)*i/(n-1);
tip.style.left=Math.max(0,Math.min(rect.width-tip.offsetWidth,X-tip.offsetWidth/2))+'px';tip.style.top='0px';}
function chartLeave(){chartHover=-1;document.getElementById('chart-tip').style.display='none';drawChart();}
function buildTable(){
const t=document.getElementById('hist-table');if(!t)return;
let h='<tr><th>Reading</th><th>Time</th><th>Level</th><th>Height</th></tr>';
for(let i=chartPts.length-1;i>=0;i--){const p=chartPts[i];
h+='<tr><td>#'+p.q+'</td><td>'+(p.t>0?esc(ptLabel(p,true)):'-')+'</td><td>'+p.p.toFixed(1)+'%</td><td>'+p.l.toFixed(0)+' cm</td></tr>';}
t.innerHTML=h;}
window.addEventListener('resize',drawChart);
setInterval(updateStatus,5000);
setInterval(updateMuteStatus,5000);
setInterval(loadHistory,60000);
</script></head><body>
<div class='container'>
<div class='header'><h1>Tank Monitor System</h1></div>
<div id='status'>Loading sensor data...</div>
<div class='status'><h3 style='margin-top:0'>Level history</h3>
<div class='chart-wrap'><canvas id='chart' onmousemove='chartPointer(event)' ontouchstart='chartPointer(event)' ontouchmove='chartPointer(event)' onmouseleave='chartLeave()' ontouchend='chartLeave()'></canvas>
<div id='chart-tip' class='tip'></div></div>
<div id='chart-note' class='ago'></div>
<details><summary class='ago' style='cursor:pointer'>Show as table</summary><table class='hist' id='hist-table'></table></details>
</div>
<div id='mute-control' class='mute-control'>Loading mute status...</div>
)rawliteral";

static const char MAIN_PAGE_TAIL[] = R"rawliteral(
<div style='text-align:center;margin-top:20px'>
<a href='/wifi' class='btn'>WiFi Settings</a>
<a href='/config' class='btn'>Settings</a>
<button class='btn' onclick='updateStatus();loadHistory()'>Refresh</button>
</div>
</div>
<script>updateStatus();updateMuteStatus();loadHistory();</script>
</body></html>)rawliteral";

String getMainPageHTML() {
  String html;
  html.reserve(sizeof(MAIN_PAGE_HEAD) + sizeof(MAIN_PAGE_TAIL) + 512);
  html += MAIN_PAGE_HEAD;

  html += "<div class='status'>";
  html += "<h3>System Info:</h3>";
  html += "Firmware Version: <span style='color:#27ae60;font-weight:bold'>" + String(FIRMWARE_VERSION) + "</span><br>";
  html += "<small style='color:#666'>To update firmware, open the Menu and select Firmware Update</small><br><br>";
  html += "Sensor connection: <strong>" + String(linkTypeName(linkType)) + "</strong><br>";
  if (wifiStationMode) {
    // Sanitize WiFi SSID for HTML display
    String safeSSID = wifiSSID;
    safeSSID.replace("&", "&amp;");
    safeSSID.replace("<", "&lt;");
    safeSSID.replace(">", "&gt;");
    safeSSID.replace("\"", "&quot;");
    html += "WiFi: <span style='color:#27ae60;font-weight:bold'>Connected</span> (" + safeSSID + ")<br>";
  } else {
    html += "WiFi: <span style='color:#f39c12'>Not Connected</span><br>";
  }
  html += "</div>";

  html += MAIN_PAGE_TAIL;
  return html;
}

String getWiFiPageHTML() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>WiFi Settings</title>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>";
  html += "<style>";
  html += "*{margin:0;padding:0;box-sizing:border-box}";
  html += "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Arial,sans-serif;background:#f0f8ff;color:#333;padding:20px}";
  html += ".wrap{max-width:500px;margin:0 auto;background:#fff;border-radius:12px;box-shadow:0 4px 20px rgba(0,0,0,0.1);overflow:hidden}";
  html += ".hdr{background:#2980b9;color:#fff;padding:20px;display:flex;align-items:center;gap:15px}";
  html += ".hdr a{color:#fff;text-decoration:none;font-size:1.4em}";
  html += ".hdr h1{font-size:1.2em}";
  html += ".content{padding:20px}";

  // Status card
  html += ".status{display:flex;align-items:center;gap:15px;padding:18px;background:#f8f9fa;border-radius:10px;margin-bottom:20px}";
  html += ".dot{width:14px;height:14px;border-radius:50%;flex-shrink:0}";
  html += ".dot.on{background:#27ae60;box-shadow:0 0 8px rgba(39,174,96,0.5)}";
  html += ".dot.off{background:#e74c3c}";
  html += ".status-info h3{font-size:1em;margin-bottom:2px}";
  html += ".status-info p{font-size:0.85em;color:#888;margin:0}";

  // Form
  html += ".fg{margin-bottom:18px}";
  html += ".fg label{display:block;font-weight:600;color:#555;margin-bottom:8px;font-size:0.95em}";
  html += ".fg input{width:100%;padding:14px;border:2px solid #ddd;border-radius:8px;font-size:1em}";
  html += ".fg input:focus{outline:none;border-color:#3498db}";
  html += ".pw-wrap{position:relative}";
  html += ".pw-wrap input{padding-right:50px}";
  html += ".eye{position:absolute;right:14px;top:50%;transform:translateY(-50%);cursor:pointer;font-size:1.2em;color:#888;user-select:none}";
  html += ".eye:hover{color:#555}";

  // Buttons
  html += ".btn{display:block;width:100%;padding:16px;border:none;border-radius:8px;font-size:1.05em;font-weight:600;cursor:pointer;text-align:center;text-decoration:none;margin-bottom:12px}";
  html += ".btn-p{background:#27ae60;color:#fff}";
  html += ".btn-p:hover{background:#229954}";
  html += ".btn-s{background:#95a5a6;color:#fff}";
  html += ".btn-s:hover{background:#7f8c8d}";
  html += ".btn:disabled{background:#bdc3c7;cursor:not-allowed}";

  // Result message
  html += ".result{padding:15px;border-radius:8px;margin-bottom:15px;display:none;text-align:center;font-weight:600}";
  html += ".result.ok{display:block;background:#d4edda;color:#155724}";
  html += ".result.err{display:block;background:#f8d7da;color:#721c24}";

  html += "</style></head><body>";

  html += "<div class='wrap'>";

  // Header
  html += "<div class='hdr'><a href='/'>&#60;</a><h1>WiFi Settings</h1></div>";

  html += "<div class='content'>";

  // Status card - simple connected/not connected
  html += "<div class='status'>";
  html += "<div class='dot " + String(wifiStationMode ? "on" : "off") + "'></div>";
  html += "<div class='status-info'>";
  if (wifiStationMode) {
    // Sanitize SSID
    String safeSSID = wifiSSID;
    safeSSID.replace("&", "&amp;");
    safeSSID.replace("<", "&lt;");
    safeSSID.replace(">", "&gt;");
    html += "<h3>Connected</h3>";
    html += "<p>" + safeSSID + "</p>";
  } else if (wifiConfigured) {
    String safeSSID = wifiSSID;
    safeSSID.replace("&", "&amp;");
    safeSSID.replace("<", "&lt;");
    safeSSID.replace(">", "&gt;");
    html += "<h3>Not Connected</h3>";
    html += "<p>" + safeSSID + "</p>";
  } else {
    html += "<h3>Not Configured</h3>";
    html += "<p>Enter your WiFi details below</p>";
  }
  html += "</div></div>";

  // Result message placeholder
  html += "<div id='result' class='result'></div>";

  // Form - simple WiFi name and password only
  html += "<form id='wf' method='POST'>";
  html += "<div class='fg'><label>WiFi Network Name</label>";

  // Sanitize SSID for input value
  String safeSsidAttr = wifiSSID;
  safeSsidAttr.replace("&", "&amp;");
  safeSsidAttr.replace("\"", "&quot;");
  safeSsidAttr.replace("'", "&#39;");
  html += "<input type='text' name='ssid' id='ssid' maxlength='31' value='" + safeSsidAttr + "' placeholder='Your WiFi name' required></div>";

  html += "<div class='fg'><label>WiFi Password</label>";
  html += "<div class='pw-wrap'><input type='password' name='password' id='pw' maxlength='63' placeholder='Your WiFi password'>";
  html += "<span class='eye' onclick='togglePw()'>Show</span></div></div>";

  html += "<button type='submit' class='btn btn-p' id='sbtn'>Save & Connect</button>";
  html += "</form>";

  html += "<a href='/' class='btn btn-s'>Back to Dashboard</a>";

  html += "</div></div>";

  // JavaScript
  html += "<script>";

  // Password toggle
  html += "function togglePw(){const p=document.getElementById('pw');const e=document.querySelector('.eye');";
  html += "if(p.type==='password'){p.type='text';e.textContent='Hide';}else{p.type='password';e.textContent='Show';}}";

  // Form submit with AJAX
  html += "document.getElementById('wf').onsubmit=function(e){";
  html += "e.preventDefault();";
  html += "const btn=document.getElementById('sbtn');";
  html += "const res=document.getElementById('result');";
  html += "btn.disabled=true;btn.textContent='Connecting...';";
  html += "res.className='result';res.style.display='none';";
  html += "const fd=new FormData(this);";
  html += "fetch('/wifi',{method:'POST',body:fd})";
  html += ".then(r=>r.text())";
  html += ".then(d=>{";
  html += "if(d.includes('success')||d.includes('Connected')||d.includes('Attempting')){";
  html += "res.className='result ok';res.textContent='Saved! Connecting to WiFi...';res.style.display='block';";
  html += "setTimeout(()=>location.reload(),4000);";
  html += "}else{";
  html += "res.className='result err';res.textContent='Connection failed. Check password.';res.style.display='block';";
  html += "btn.disabled=false;btn.textContent='Save & Connect';";
  html += "}";
  html += "}).catch(err=>{";
  html += "res.className='result err';res.textContent='Error: '+err;res.style.display='block';";
  html += "btn.disabled=false;btn.textContent='Save & Connect';";
  html += "});";
  html += "return false;";
  html += "};";

  html += "</script></body></html>";

  return html;
}

String getWiFiResultHTML() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>WiFi Connection Result</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='5;url=/'>";
  html += "<style>";
  html += "body{font-family:Arial;margin:20px;background:#f0f8ff;text-align:center}";
  html += ".container{max-width:400px;margin:50px auto;background:white;padding:30px;border-radius:10px}";
  html += ".success{color:#27ae60;font-size:1.2em}";
  html += ".error{color:#e74c3c;font-size:1.2em}";
  html += "</style></head><body>";

  html += "<div class='container'>";
  // Sanitize SSID for HTML display
  String safeSSID = wifiSSID;
  safeSSID.replace("&", "&amp;");
  safeSSID.replace("<", "&lt;");
  safeSSID.replace(">", "&gt;");
  safeSSID.replace("\"", "&quot;");

  if (wifiStationMode) {
    html += "<div class='success'>WiFi Connected!</div>";
    html += "<p>Successfully connected to: " + safeSSID + "</p>";
    html += "<p>Device is now accessible on your WiFi network.</p>";
  } else {
    html += "<div class='error'>Connection Failed</div>";
    html += "<p>Could not connect to: " + safeSSID + "</p>";
    html += "<p>Please check credentials and try again.</p>";
    html += "<p>Device remains in AP mode.</p>";
  }
  html += "<p>Redirecting to status page...</p>";
  html += "<a href='/'>View Status Now</a>";
  html += "</div></body></html>";

  return html;
}

// Settings page: connection type (customer-facing names only) + sensor pairing
static const char CONFIG_LINK_SECTION[] = R"rawliteral(
<style>
.fg label.lk-opt{display:flex;gap:12px;align-items:flex-start;padding:12px;margin:6px 0;background:#fff;border:2px solid #ddd;border-radius:8px;cursor:pointer;font-weight:normal;color:#333}
.fg label.lk-opt.sel{border-color:#3498db;background:#eef6fc}
.lk-opt input{margin-top:4px;width:18px;height:18px;flex-shrink:0}
.lk-title{font-weight:600}.lk-desc{font-size:0.85em;color:#666}
.lk-warn{background:#fff3cd;border-radius:6px;padding:8px;font-size:0.85em;margin:8px 0;display:none}
.scan-item{background:#fff;border:1px solid #ddd;padding:10px;margin:4px 0;border-radius:6px;cursor:pointer;display:flex;justify-content:space-between}
.scan-item:hover{background:#e8f4fc;border-color:#3498db}
</style>
<details id='link-section'>
<summary>Sensor Connection</summary>
<div class='sec-content'>
<div class='fg'><label>How is the tank sensor connected?</label>
<label class='lk-opt' data-v='0'><input type='radio' name='lk' value='0'><div><div class='lk-title'>Wired</div><div class='lk-desc'>The sensor cable is plugged straight into this display.</div></div></label>
<label class='lk-opt' data-v='4'><input type='radio' name='lk' value='4'><div><div class='lk-title'>Auto (recommended for wireless)</div><div class='lk-desc'>The display listens on every wireless connection it has and uses whichever the sensor sends on.</div></div></label>
<label class='lk-opt' data-v='1'><input type='radio' name='lk' value='1'><div><div class='lk-title'>Long Range</div><div class='lk-desc'>For tanks far away - up to several kilometres in open areas. Needs the Long Range module in both the sensor and this display.</div></div></label>
<label class='lk-opt' data-v='2'><input type='radio' name='lk' value='2'><div><div class='lk-title'>NearbyConnect</div><div class='lk-desc'>Sensor and display talk directly to each other, up to about 200 m in open space (less through thick walls). No router or internet needed.</div></div></label>
<label class='lk-opt' data-v='3'><input type='radio' name='lk' value='3'><div><div class='lk-title'>Home Network</div><div class='lk-desc'>Sensor and display both join your home WiFi. <strong>Both must be on the same WiFi network.</strong> Works anywhere your WiFi reaches. NearbyConnect is used as a backup if the router is off.</div></div></label>
</div>
<div class='lk-warn' id='lk-warn'></div>
<button class='btn btn-p btn-full' onclick='saveLink()'>Save Connection Type</button>
<small style='color:#888'>The display restarts after the connection type changes.</small>
<div id='pair-box' style='margin-top:20px;display:none'>
<h4 style='margin-bottom:6px'>Paired Sensor: <span id='pair-current'>none</span></h4>
<button class='btn btn-w btn-full' id='scan-btn' onclick='startScan()'>Scan for Sensors</button>
<div id='scan-status' style='font-size:0.85em;color:#666'></div>
<div id='scan-results'></div>
<div class='fg' style='margin-top:12px'><label>Or enter the sensor ID</label>
<input type='text' id='pair-id' maxlength='6' placeholder='e.g. US3847'><small>Printed on the sensor and shown on its setup page</small></div>
<button class='btn btn-s btn-full' onclick='pairSensor(document.getElementById("pair-id").value)'>Pair Sensor</button>
</div>
</div></details>
)rawliteral";

static const char CONFIG_LOGGING_SECTION[] = R"rawliteral(
<details id='log-section'>
<summary>History Logging</summary>
<div class='sec-content'>
<div class='fg'><label>Save a reading when the level changes by</label>
<select id='log-threshold'><option value='5'>5%</option><option value='10'>10% (default)</option></select></div>
<div class='fg'><label>...and at least every</label>
<select id='log-interval'><option value='30'>30 minutes</option><option value='60'>1 hour (default)</option></select>
<small>The chart keeps the latest 96 readings.</small></div>
<div id='log-count' style='font-size:0.85em;color:#666;margin-bottom:10px'></div>
<button class='btn btn-p btn-full' onclick='saveLogging()'>Save Logging Settings</button>
<button class='btn btn-d btn-full' onclick='clearHistory()'>Clear History</button>
</div></details>
)rawliteral";

static const char CONFIG_LINK_SCRIPT[] = R"rawliteral(
let linkCfg={};
function selLink(v){document.querySelectorAll('.lk-opt').forEach(o=>{const on=o.dataset.v==String(v);o.classList.toggle('sel',on);o.querySelector('input').checked=on;});
const w=document.getElementById('lk-warn');let msg='';
if(v==1&&linkCfg.longRangeChecked&&!linkCfg.longRangeModule)msg='Long Range module was not detected in this display.';
if(v==3&&!linkCfg.wifiConnected)msg='This display is not connected to WiFi. Enter your WiFi details on both the sensor and this display.';
w.textContent=msg;w.style.display=msg?'block':'none';
document.getElementById('pair-box').style.display=(v==0)?'none':'block';}
function renderLink(l){linkCfg=l;selLink(l.type!=null?l.type:4);
document.getElementById('pair-current').textContent=l.sensor||'none';
document.querySelectorAll('.lk-opt').forEach(o=>{o.onclick=()=>selLink(o.dataset.v);});}
function saveLink(){const s=document.querySelector('input[name=lk]:checked');if(!s)return;
const fd=new FormData();fd.append('type',s.value);
fetch('/api/link',{method:'POST',body:fd}).then(r=>r.json()).then(d=>{
if(d.error)showToast(d.error,true);else if(d.restart){showToast('Saved. Restarting...');setTimeout(()=>location.reload(),8000);}else showToast('Saved!');
}).catch(()=>{showToast('Saved. Restarting...');setTimeout(()=>location.reload(),8000);});}
let scanTimer=null;
function startScan(){const b=document.getElementById('scan-btn');b.disabled=true;b.textContent='Scanning...';
document.getElementById('scan-results').innerHTML='';document.getElementById('scan-status').textContent='Looking for sensors...';
fetch('/api/scan?action=start').then(r=>r.json()).then(d=>{if(d.error){scanDone(d.error);return;}
scanTimer=setInterval(pollScan,2000);pollScan();}).catch(()=>scanDone('Scan failed. Try again.'));}
function scanDone(msg){clearInterval(scanTimer);scanTimer=null;const b=document.getElementById('scan-btn');b.disabled=false;b.textContent='Scan for Sensors';
if(msg)document.getElementById('scan-status').textContent=msg;}
function pollScan(){fetch('/api/scan').then(r=>r.json()).then(d=>{
let h='';for(const s of d.devices){h+='<div class="scan-item" onclick="pairSensor(\''+s.id+'\')"><strong>'+s.id+'</strong><span>'+s.via+(s.rssi?' &middot; '+s.rssi+' dBm':'')+(s.paired?' &middot; paired':'')+'</span></div>';}
document.getElementById('scan-results').innerHTML=h;
if(d.scanning){document.getElementById('scan-status').textContent='Scanning... '+d.remaining+'s remaining';}
else{scanDone(d.devices.length?'Tap a sensor to pair it.':'No sensors found. Make sure the sensor is powered on and set to the same connection type.');}});}
function pairSensor(id){id=(id||'').trim().toUpperCase();
if(!/^US[0-9]{4}$/.test(id)){showToast('Sensor ID must be US followed by 4 digits',true);return;}
const fd=new FormData();fd.append('id',id);
fetch('/api/pair',{method:'POST',body:fd}).then(r=>r.json()).then(d=>{
if(d.error)showToast(d.error,true);else{showToast('Paired with '+id);document.getElementById('pair-current').textContent=id;}});}
function renderLogging(g){if(g.threshold)document.getElementById('log-threshold').value=g.threshold;
if(g.interval)document.getElementById('log-interval').value=g.interval;
document.getElementById('log-count').textContent=(g.count||0)+' readings stored';}
function saveLogging(){const fd=new FormData();
fd.append('threshold',document.getElementById('log-threshold').value);fd.append('interval',document.getElementById('log-interval').value);
fetch('/api/logging',{method:'POST',body:fd}).then(r=>r.json()).then(()=>showToast('Logging settings saved'));}
function clearHistory(){if(!confirm('Clear all saved readings?'))return;
fetch('/api/history/clear',{method:'POST'}).then(r=>r.json()).then(()=>{showToast('History cleared');loadConfigData();});}
)rawliteral";

String getConfigPageHTML() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>Settings - " + deviceName + "</title>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";

  // Base styles - Mobile First
  html += "*{margin:0;padding:0;box-sizing:border-box}";
  html += "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Arial,sans-serif;background:#f0f8ff;color:#333;line-height:1.6}";
  html += ".container{max-width:800px;margin:0 auto;background:#fff;min-height:100vh;box-shadow:0 0 20px rgba(0,0,0,0.1);padding-bottom:20px}";

  // Header
  html += ".header{background:#2980b9;color:#fff;padding:15px 20px;display:flex;align-items:center;gap:15px;min-height:60px;position:sticky;top:0;z-index:100;max-width:800px;margin:0 auto}";
  html += ".header h1{font-size:1.3em;font-weight:bold;flex:1;text-align:center}";
  html += ".back-btn,.refresh-btn{background:rgba(255,255,255,0.25);color:#fff;border:2px solid rgba(255,255,255,0.3);padding:8px 12px;border-radius:6px;text-decoration:none;font-size:0.9em;transition:all 0.2s;cursor:pointer}";
  html += ".back-btn:hover,.refresh-btn:hover{background:rgba(255,255,255,0.4)}";
  html += ".refresh-btn{font-size:1.2em;line-height:1;border:none;background:rgba(255,255,255,0.15)}";

  // Toast notification
  html += ".toast{display:none;position:fixed;top:70px;left:50%;transform:translateX(-50%);background:#27ae60;color:#fff;padding:12px 24px;border-radius:6px;z-index:101;font-weight:600;box-shadow:0 4px 12px rgba(0,0,0,0.2)}";
  html += ".toast.error{background:#e74c3c}";
  html += ".toast.show{display:block;animation:fadeOut 3s forwards}";
  html += "@keyframes fadeOut{0%,80%{opacity:1}100%{opacity:0}}";

  // Collapsible sections
  html += "details{border-bottom:1px solid #dee2e6}";
  html += "summary{padding:18px 20px;font-weight:600;font-size:1em;cursor:pointer;display:flex;align-items:center;justify-content:flex-start;gap:10px;user-select:none;list-style:none;background:#fff}";
  html += "summary::-webkit-details-marker{display:none}";
  html += "summary::after{content:'\\203A';margin-left:auto;font-size:1.3em;color:#999;transition:transform 0.2s}";
  html += "details[open] summary::after{transform:rotate(90deg)}";
  html += "summary:hover{background:#f8f9fa}";
  html += ".sec-content{padding:15px 20px 20px;background:#f8f9fa}";

  // Form elements
  html += ".fg{margin-bottom:15px}";
  html += ".fg label{display:block;font-weight:600;color:#555;margin-bottom:6px;font-size:0.9em}";
  html += ".fg input[type=text],.fg input[type=number],.fg select{width:100%;padding:12px;border:2px solid #ddd;border-radius:6px;font-size:1em}";
  html += ".fg input:focus,.fg select:focus{outline:none;border-color:#3498db}";
  html += ".fg small{color:#888;font-size:0.8em;display:block;margin-top:4px}";

  // Buttons
  html += ".btn{display:inline-block;padding:12px 20px;border:none;border-radius:6px;font-size:1em;font-weight:600;cursor:pointer;text-align:center;text-decoration:none;transition:all 0.2s}";
  html += ".btn-p{background:#3498db;color:#fff}";
  html += ".btn-s{background:#27ae60;color:#fff}";
  html += ".btn-d{background:#e74c3c;color:#fff}";
  html += ".btn-w{background:#f39c12;color:#fff}";
  html += ".btn-g{background:#95a5a6;color:#fff}";
  html += ".btn:hover{opacity:0.9}";
  html += ".btn:disabled{background:#bdc3c7;cursor:not-allowed}";
  html += ".btn-full{width:100%;display:block;margin-bottom:10px}";

  // Tank visualization CSS
  html += ".tank-viz{display:flex;gap:20px;align-items:flex-start;margin:15px 0;flex-wrap:wrap}";
  html += ".tank-graphic{flex-shrink:0}";
  html += ".tank-cap{width:50px;height:6px;background:linear-gradient(135deg,#34495e,#2c3e50);border-radius:50% 50% 0 0;margin:0 auto}";
  html += ".tank-top{width:80px;height:20px;background:linear-gradient(to bottom,#e8eef1,#d5dbdb);clip-path:polygon(15% 0,85% 0,100% 100%,0 100%);margin:0 auto;border-left:2px solid #34495e;border-right:2px solid #34495e}";
  html += ".tank-body{width:80px;height:120px;background:linear-gradient(to bottom,#ecf0f1,#d5dbdb);border:2px solid #34495e;border-top:none;border-radius:0 0 10px 10px;position:relative;overflow:hidden;margin:0 auto}";
  html += ".tank-water{position:absolute;bottom:0;width:100%;background:linear-gradient(to top,#1565c0,#42a5f5);transition:height 0.3s}";
  html += ".tank-line{position:absolute;width:100%;height:3px;left:0;transition:bottom 0.2s}";
  html += ".tank-line-low{background:#27ae60;box-shadow:0 0 6px rgba(39,174,96,0.7)}";
  html += ".tank-line-high{background:#e74c3c;box-shadow:0 0 6px rgba(231,76,60,0.7)}";
  html += ".tank-line-lbl{position:absolute;right:-45px;top:-8px;font-size:0.7em;font-weight:bold;white-space:nowrap;padding:2px 4px;border-radius:3px;background:rgba(255,255,255,0.9)}";
  html += ".tank-line-low .tank-line-lbl{color:#27ae60}";
  html += ".tank-line-high .tank-line-lbl{color:#e74c3c}";

  // Alarm sliders
  html += ".alarm-section{flex:1;min-width:0;width:100%}";
  html += ".alarm-row{margin:10px 0;padding:12px;background:#f8f9fa;border-radius:6px}";
  html += ".alarm-row.disabled{opacity:0.5}";
  html += ".alarm-row-top{display:flex;align-items:center;gap:10px;margin-bottom:8px}";
  html += ".alarm-row-top input[type=checkbox]{width:18px;height:18px;flex-shrink:0}";
  html += ".alarm-row-top label{font-weight:600;font-size:0.9em;flex:1}";
  html += ".alarm-desc{font-size:0.8em;color:#888;margin-bottom:8px}";
  html += ".alarm-slider-row{display:flex;align-items:center;gap:10px}";
  html += ".alarm-slider-row input[type=range]{flex:1;height:6px;border-radius:3px;-webkit-appearance:none;background:#ddd}";
  html += ".alarm-slider-row input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;border-radius:50%;background:#3498db;cursor:pointer}";
  html += ".alarm-slider-row input[type=range]:disabled{background:#eee}";
  html += ".alarm-slider-row input[type=range]:disabled::-webkit-slider-thumb{background:#bdc3c7;cursor:not-allowed}";
  html += ".alarm-val{font-weight:bold;font-size:1.2em;color:#2980b9;min-width:45px;text-align:right}";
  html += ".alarm-val.disabled{color:#bdc3c7}";
  html += ".alarm-note{font-size:0.8em;color:#888;margin-top:10px;padding:8px;background:#fff3cd;border-radius:4px}";

  // Card action buttons
  html += ".card-actions{display:flex;gap:8px;margin-top:15px;padding-top:12px;border-top:1px solid #eee}";
  html += ".card-actions .btn{flex:1;padding:10px}";

  html += ".status-badge{padding:4px 10px;border-radius:12px;font-size:0.75em;font-weight:bold;display:inline-block}";
  html += ".st-online{background:#27ae60;color:#fff}";
  html += ".st-offline{background:#e74c3c;color:#fff}";

  // Responsive
  html += "@media(max-width:600px){";
  html += ".tank-viz{flex-direction:column;align-items:center}";
  html += ".alarm-section{min-width:100%}";
  html += "}";

  html += "</style></head><body>";

  // Toast notification
  html += "<div id='toast' class='toast'></div>";

  // Header (outside container for sticky to work on all browsers)
  html += "<div class='header'><a id='header-back' class='back-btn' onclick='handleBack(event)'>&lt; Back</a><h1>SETTINGS</h1><button class='refresh-btn' onclick='refreshData()' title='Refresh'>&#x21bb;</button></div>";

  // Container
  html += "<div class='container'>";

  // SECTION 1: Device Settings
  html += "<details open>";
  html += "<summary>Device Settings</summary>";
  html += "<div class='sec-content'>";
  html += "<div class='fg'>";
  html += "<label>Device Name</label>";
  html += "<input type='text' id='device-name' value='" + deviceName + "' maxlength='20'>";
  html += "<small>Device will be accessible as: <strong>" + deviceName + "_mvstech.local</strong></small>";
  html += "</div>";
  html += "<button class='btn btn-p' onclick='updateDeviceName()'>Update Device Name</button>";
  html += "</div></details>";

  // SECTION 2: Tank Sensor Settings
  html += "<details open id='tank-section'>";
  html += "<summary>Tank Sensor</summary>";
  html += "<div class='sec-content' id='tank-content'><p style='color:#888'>Loading...</p></div>";
  html += "</details>";

  // SECTION 3: Sensor Connection (connection type + pairing)
  html += CONFIG_LINK_SECTION;

  // SECTION 4: History Logging
  html += CONFIG_LOGGING_SECTION;

  html += "</div>"; // container

  // JavaScript
  html += "<script>";

  // Global variables
  html += "let hasUnsavedChanges=false;";

  // Toast notification
  html += "function showToast(msg,isError){";
  html += "const t=document.getElementById('toast');";
  html += "t.textContent=msg;";
  html += "t.className='toast'+(isError?' error':'')+' show';";
  html += "setTimeout(()=>t.classList.remove('show'),3000);";
  html += "}";

  // Load configuration data
  html += "function loadConfigData(){";
  html += "fetch('/api/config').then(r=>r.json()).then(data=>{";
  html += "renderTankSettings(data.tank||{enabled:false});";
  html += "renderLink(data.link||{});renderLogging(data.logging||{});";
  html += "}).catch(e=>showToast('Error loading: '+e.message,true));";
  html += "}";

  // Render tank sensor settings (single sensor, always slot 0)
  html += "function renderTankSettings(t){";
  html += "const box=document.getElementById('tank-content');";
  html += "if(!t.enabled){";
  html += "box.innerHTML=`<div class='fg'><label>Sensor Name</label><input type='text' id='tank-name' placeholder='e.g., MainTank' maxlength='8'><small>Max 8 characters, letters and numbers only</small></div>";
  html += "<div class='fg'><label>Height (cm)</label><input type='number' id='tank-height' min='50' max='450' placeholder='e.g., 150'><small>Measure from the sensor to the base of the tank</small></div>";
  html += "<div class='fg'><label>Volume (Litres)</label><input type='number' id='tank-volume' min='0' max='99999' placeholder='e.g., 1000 (optional)'></div>";
  html += "<div class='fg'><label><input type='checkbox' id='tank-low-en' checked> Low Level Alarm</label>";
  html += "<input type='range' id='tank-low-slider' min='1' max='94' value='20' oninput='tankVizUpdate()'><span class='alarm-val' id='tank-low-val'>20%</span></div>";
  html += "<div class='fg'><label><input type='checkbox' id='tank-high-en' checked> High Level Alarm</label>";
  html += "<input type='range' id='tank-high-slider' min='25' max='99' value='90' oninput='tankVizUpdate()'><span class='alarm-val' id='tank-high-val'>90%</span></div>";
  html += "<button class='btn btn-s btn-full' onclick='saveTank()'>Add Tank Sensor</button>`;";
  html += "return;";
  html += "}";
  html += "const stCls=t.online?'st-online':'st-offline';const stTxt=t.online?'Online':'Offline';";
  html += "const lowEn=t.lowAlarm>0;const highEn=t.highAlarm>0;";
  html += "const lowVal=lowEn?t.lowAlarm:20;const highVal=highEn?t.highAlarm:90;";
  html += "let html=`<div style=\"margin-bottom:15px\"><span class='status-badge ${stCls}'>${stTxt}</span></div>`;";
  html += "html+=`<div class='fg'><label>Tank Name</label><input type='text' id='tank-name' value='${t.name}' maxlength='8'><small>Max 8 characters, letters and numbers only</small></div>`;";
  html += "html+=`<div class='fg'><label>Height (cm)</label><input type='number' id='tank-height' value='${t.height}'></div>`;";
  html += "html+=`<div class='fg'><label>Volume (Litres)</label><input type='number' id='tank-volume' value='${t.volume}'></div>`;";
  html += "html+=`<div class='tank-viz'>`;";
  html += "html+=`<div class='tank-graphic'><div class='tank-cap'></div><div class='tank-top'></div>`;";
  html += "html+=`<div class='tank-body'><div class='tank-water' style='height:50%'></div>`;";
  html += "html+=`<div class='tank-line tank-line-low' id='tank-line-low' style='bottom:${lowVal}%;display:${lowEn?'block':'none'}'><span class='tank-line-lbl'>LOW</span></div>`;";
  html += "html+=`<div class='tank-line tank-line-high' id='tank-line-high' style='bottom:${highVal}%;display:${highEn?'block':'none'}'><span class='tank-line-lbl'>HIGH</span></div>`;";
  html += "html+=`</div></div>`;";
  html += "html+=`<div class='alarm-section'>`;";
  html += "html+=`<div class='alarm-row ${lowEn?'':'disabled'}' id='tank-low-row'>`;";
  html += "html+=`<div class='alarm-row-top'><input type='checkbox' id='tank-low-en' ${lowEn?'checked':''} onchange='tankToggleSlider(\"low\")'><label>Low Alarm</label></div>`;";
  html += "html+=`<div class='alarm-desc'>Alarm when water drops below this level</div>`;";
  html += "html+=`<div class='alarm-slider-row'><input type='range' id='tank-low-slider' min='1' max='94' value='${lowVal}' ${lowEn?'':'disabled'} oninput='tankVizUpdate()'><span class='alarm-val ${lowEn?'':'disabled'}' id='tank-low-val'>${lowVal}%</span></div>`;";
  html += "html+=`</div>`;";
  html += "html+=`<div class='alarm-row ${highEn?'':'disabled'}' id='tank-high-row'>`;";
  html += "html+=`<div class='alarm-row-top'><input type='checkbox' id='tank-high-en' ${highEn?'checked':''} onchange='tankToggleSlider(\"high\")'><label>High Alarm</label></div>`;";
  html += "html+=`<div class='alarm-desc'>Alarm when water rises above this level</div>`;";
  html += "html+=`<div class='alarm-slider-row'><input type='range' id='tank-high-slider' min='${lowEn?lowVal+5:1}' max='99' value='${highVal}' ${highEn?'':'disabled'} oninput='tankVizUpdate()'><span class='alarm-val ${highEn?'':'disabled'}' id='tank-high-val'>${highVal}%</span></div>`;";
  html += "html+=`</div>`;";
  html += "html+=`<div class='alarm-note'>You can set both alarms, or either one individually. Minimum 5% gap when both are enabled</div>`;";
  html += "html+=`</div></div>`;";
  html += "html+=`<button class='btn btn-s btn-full' id='save-tank-btn' onclick='saveTank()'>Save</button>`;";
  html += "box.innerHTML=html;";
  html += "}";

  // Mark unsaved
  html += "function markChanged(){";
  html += "hasUnsavedChanges=true;";
  html += "const btn=document.getElementById('save-tank-btn');";
  html += "if(btn){btn.style.background='#f39c12';btn.textContent='Save *';}";
  html += "}";

  // Toggle alarm slider (works for both add and edit forms)
  html += "function tankToggleSlider(type){";
  html += "const chk=document.getElementById('tank-'+type+'-en');";
  html += "const slider=document.getElementById('tank-'+type+'-slider');";
  html += "const val=document.getElementById('tank-'+type+'-val');";
  html += "const row=document.getElementById('tank-'+type+'-row');";
  html += "const line=document.getElementById('tank-line-'+type);";
  html += "if(chk.checked){";
  html += "slider.disabled=false;if(row)row.classList.remove('disabled');if(val)val.classList.remove('disabled');if(line)line.style.display='block';";
  html += "tankVizUpdate();";
  html += "}else{";
  html += "slider.disabled=true;if(row)row.classList.add('disabled');if(val)val.classList.add('disabled');if(line)line.style.display='none';";
  html += "}";
  html += "markChanged();";
  html += "}";

  // Update tank visualization (works for both add and edit forms)
  html += "function tankVizUpdate(){";
  html += "const lowEnEl=document.getElementById('tank-low-en');const highEnEl=document.getElementById('tank-high-en');";
  html += "const lowEn=lowEnEl?lowEnEl.checked:true;const highEn=highEnEl?highEnEl.checked:true;";
  html += "const lowSlider=document.getElementById('tank-low-slider');const highSlider=document.getElementById('tank-high-slider');";
  html += "let low=parseInt(lowSlider.value);let high=parseInt(highSlider.value);";
  html += "if(lowEn&&highEn){if(high<low+5){high=low+5;highSlider.value=high;}highSlider.min=low+5;}else{highSlider.min=1;}";
  html += "const lv=document.getElementById('tank-low-val');const hv=document.getElementById('tank-high-val');";
  html += "if(lv)lv.textContent=low+'%';if(hv)hv.textContent=high+'%';";
  html += "const ll=document.getElementById('tank-line-low');const hl=document.getElementById('tank-line-high');";
  html += "if(ll)ll.style.bottom=low+'%';if(hl)hl.style.bottom=high+'%';";
  html += "markChanged();";
  html += "}";

  // Save tank settings
  html += "function saveTank(){";
  html += "const n=document.getElementById('tank-name').value.trim();";
  html += "if(!n||n.length>8||!/^[a-zA-Z0-9]+$/.test(n)){showToast('Invalid name: max 8 chars, letters and numbers only',true);return;}";
  html += "const h=document.getElementById('tank-height').value;";
  html += "if(!h||h<50||h>450){showToast('Height must be between 50 and 450 cm',true);return;}";
  html += "const v=document.getElementById('tank-volume').value||0;";
  html += "const lowEnEl=document.getElementById('tank-low-en');const highEnEl=document.getElementById('tank-high-en');";
  html += "const lowEn=lowEnEl?lowEnEl.checked:false;const highEn=highEnEl?highEnEl.checked:false;";
  html += "const lowVal=lowEn?document.getElementById('tank-low-slider').value:0;";
  html += "const highVal=highEn?document.getElementById('tank-high-slider').value:0;";
  html += "const fd=new FormData();";
  html += "fd.append('name',n);fd.append('height',h);fd.append('volume',v);fd.append('lowAlarm',lowVal);fd.append('highAlarm',highVal);";
  html += "fetch('/api/tank',{method:'PUT',body:fd}).then(r=>r.json()).then(d=>{";
  html += "if(d.error)showToast(d.error,true);";
  html += "else{showToast('Saved!');hasUnsavedChanges=false;loadConfigData();}";
  html += "}).catch(e=>showToast('Error: '+e.message,true));";
  html += "}";

  // Update device name
  html += "function updateDeviceName(){";
  html += "const name=document.getElementById('device-name').value.trim();";
  html += "if(!name||name.length>20){showToast('Invalid device name',true);return;}";
  html += "const fd=new FormData();fd.append('name',name);";
  html += "fetch('/api/device-name',{method:'POST',body:fd}).then(r=>r.json()).then(d=>{";
  html += "if(d.error)showToast(d.error,true);";
  html += "else{showToast('Device name updated! Restart to apply.');setTimeout(()=>location.reload(),2000);}";
  html += "}).catch(e=>showToast('Error: '+e.message,true));";
  html += "}";

  // Refresh
  html += "function refreshData(){hasUnsavedChanges=false;loadConfigData();showToast('Data refreshed');}";

  // Handle back with unsaved changes warning
  html += "function handleBack(e){";
  html += "if(e)e.preventDefault();";
  html += "if(hasUnsavedChanges){";
  html += "if(confirm('You have unsaved changes. Leave without saving?')){window.location.href='/';}";
  html += "}else{window.location.href='/';}}";

  // Browser beforeunload warning
  html += "window.addEventListener('beforeunload',function(e){";
  html += "if(hasUnsavedChanges){e.preventDefault();e.returnValue='';return '';}});";

  html += CONFIG_LINK_SCRIPT;

  // Initialize
  html += "window.onload=function(){loadConfigData();};";

  html += "</script></body></html>";
  return html;
}

// ===========================================
// MAIN SETUP AND LOOP
// ===========================================

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("=== TANK MONITOR v" FIRMWARE_VERSION " ===");
  Serial.println("FEATURES:");
  Serial.println("- Wired, Long Range, NearbyConnect or Home Network sensor");
  Serial.println("- Heartbeat: last reading stays on screen if the sensor cannot measure");
  Serial.println("- 24h level history chart (OLED + Web UI)");
  Serial.println("- OLED display optional - system works without it");
  Serial.println("- Web UI always available at http://192.168.4.1:7689");
  Serial.println("- Auto-cycling display with manual control");
  Serial.println("=====================================");

  // Initialize hardware with FIXED pins
  pinMode(BUZZER_PIN, OUTPUT);  // Pin 4 (safer than Pin 2, no boot strapping issues)
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Pin 33

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.print("Ultrasonic sensor: TRIG=GPIO");
  Serial.print(TRIG_PIN);
  Serial.print(", ECHO=GPIO");
  Serial.println(ECHO_PIN);

  // Initialize I2C and OLED (optional - system works without it)
  Wire.begin(SDA_PIN, SCL_PIN);
  initializeDisplay();  // Detects OLED and shows branding if available

  // Report OLED status
  Serial.print("OLED Display: ");
  Serial.println(oledAvailable ? "CONNECTED" : "NOT CONNECTED (WebUI still works)");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Load device name and configuration
  loadDeviceName();
  loadConfiguration();
  loadWiFiCredentials();
  loadOTAConfig();
  loadGlobalMuteConfig();

  // Setup WiFi and web server
  setupAccessPoint();

  // Initialize MVS OTA
    mvsota.begin(deviceName, FIRMWARE_VERSION,FIRMWARE_CODE);

  mvsota.onStart([]() {
      // Monitor has no motor, just acknowledge OTA start
  });

  setupWebServer();

  // Try WiFi station connection if configured
  if (wifiConfigured) {
    Serial.println("Attempting to connect to configured WiFi...");
    connectToWiFi();

    // Setup OTA if WiFi is connected
    if (wifiStationMode) {
      setupOTA();
    }
  }

  // Initialize LittleFS (formats on first use) and load the reading history
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed - history disabled");
  } else {
    Serial.println("LittleFS initialized");
    initHistory();
  }

  // Start the sensor connection (radios start after WiFi so they follow its channel)
  startLinks();
  startNTPIfConnected();

  Serial.println("System Ready!");
  Serial.println("=====================================");
  // Clean device name first, then add _mvstech suffix
  String cleanDeviceName = deviceName;
  cleanDeviceName.replace(" ", "-");
  cleanDeviceName.replace("_", "-");
  cleanDeviceName.replace(".", "-");
  cleanDeviceName.toLowerCase();
  String apName = cleanDeviceName + "_mvstech";
  Serial.print("Web Interface: http://");
  Serial.println(WiFi.softAPIP());
  Serial.print("AP SSID: ");
  Serial.println(apName);
  Serial.println("AP Password: mvstech9867");

  if (wifiStationMode) {
    Serial.print("Also available at: http://");
    Serial.println(WiFi.localIP());
  }

  if (!isConfigured) {
    Serial.println(">>> Device not configured. Use 'CONFIG' command <<<");
    Serial.println(">>> Use 'WIFI' command for simplified WiFi setup <<<");
  } else {
    Serial.println(">>> Configuration loaded. Web UI and OLED display active <<<");
    Serial.print(">>> Monitoring: ");
    Serial.println(tankConfig.enabled ? "Tank sensor active" : "No tank configured");
  }

  // Initialize screen management
  screenMgr.lastUpdate = millis();
  updateDisplay();

  Serial.println("\nFEATURES CONFIRMED WORKING:");
  Serial.println("- Web UI shows tank status with acknowledge button");
  Serial.println("- OLED auto-cycling display (4s per screen)");
  Serial.println("- Buzzer test available in CONFIG > General Settings > Test Buzzer");
  Serial.println("- Button functions: Single=ACK, Double=System info (5s), Long=Manual screen");
  Serial.println("- API endpoint working: /api/status");
  Serial.println("- All pin assignments fixed and verified");
  Serial.println("\nTERMINAL COMMANDS:");
  Serial.println("- CONFIG: Complete sensor setup menu");
  Serial.println("- WIFI: Direct WiFi SSID/password setup");
  Serial.println("- STATUS: Show all system and sensor status");
  Serial.println("- ALARM: Test buzzer system");
  Serial.println("=====================================");
}

void loop() {
  // Handle serial commands
  handleSerialCommands();

  // Handle web server
  server.handleClient();

  // Handle OTA updates
  if (wifiStationMode) {
    ArduinoOTA.handle();
  }

  // Handle MVS OTA updates
  if (!mvsota.isUpdating()) {
    mvsota.handle();
  }

  // Handle mDNS - ESP32 mDNS runs automatically, no update() needed

  // Handle button presses
  handleButton();

  // Sensor input: wired pins, or the wireless links
  if (linkType == LINK_WIRED) {
    static unsigned long lastSensorRead = 0;
    if (millis() - lastSensorRead > SENSOR_READ_INTERVAL_MS) {
      pollWiredSensor();
      lastSensorRead = millis();
    }
  } else {
    serviceLinks();
  }

  // Trend ring keeps sampling while readings arrive; NTP once WiFi is up
  updateTrend();
  startNTPIfConnected();

  // Update alarm system
  updateAlarmSystem();

  // Update display with auto-cycling system
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 100) { // Check more frequently for smooth transitions
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  // Periodic sensor online check every 30 seconds
  static unsigned long lastSensorCheck = 0;
  if (millis() - lastSensorCheck > 30000) {
    checkSensorOnlineStatus();
    lastSensorCheck = millis();
  }

  delay(50);
}

void checkSensorOnlineStatus() {
  unsigned long currentTime = millis();
  unsigned long offlineThreshold = generalSettings.offlineTimeoutMinutes * 60000UL;

  if (tankConfig.enabled && tankData.hasData) {
    bool wasOnline = tankData.isOnline;
    tankData.isOnline = (currentTime - tankData.lastUpdate) <= offlineThreshold;

    if (wasOnline && !tankData.isOnline) {
      Serial.print("Tank (");
      Serial.print(tankConfig.customName);
      Serial.println(") went OFFLINE");
    } else if (!wasOnline && tankData.isOnline) {
      // Clear stale alarm acknowledgments so alarms can re-trigger fresh after recovery
      if (alarmAck.tankLowThresholdAck) {
        alarmAck.tankLowThresholdAck = false;
        Serial.print("Tank (");
        Serial.print(tankConfig.customName);
        Serial.println(") back online - low threshold acknowledgment cleared");
      }
      if (alarmAck.tankHighThresholdAck) {
        alarmAck.tankHighThresholdAck = false;
        Serial.print("Tank (");
        Serial.print(tankConfig.customName);
        Serial.println(") back online - high threshold acknowledgment cleared");
      }
      saveAlarmAckData();
    }
  }
}
