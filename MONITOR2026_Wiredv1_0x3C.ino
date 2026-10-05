#include <ArduinoJson.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <math.h>
#include <mvsota_esp32.h>

#define FIRMWARE_VERSION "10.0.0"
#define FIRMWARE_CODE "SF-MON-20266-9867"



// Ultrasonic sensor pins (wired directly - no LoRa)
#define TRIG_PIN 25
#define ECHO_PIN 34

// Hardware pins (FIXED - No conflicts)
#define BUZZER_PIN 4   // Changed back to 4 - safer pin, no boot strapping issues
#define BUTTON_PIN 33  // Changed from 19 to 33
#define SDA_PIN 21
#define SCL_PIN 22

// OLED I2C Address Configuration
// Change this to 0x3D if your OLED uses that address
#define OLED_I2C_ADDRESS 0x3C  // Most common: 0x3C, Alternative: 0x3D

// Ultrasonic sampling / MAD filtering constants
#define SAMPLE_COUNT 15
#define MIN_VALID_SAMPLES 10
#define READING_DELAY 50
#define MAD_MULTIPLIER 2.5
#define SENSOR_READ_INTERVAL_MS 2000

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
#define MAGIC_NUMBER 0xAB

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
  unsigned long lastUpdate;
  bool isOnline;
  bool hasData;
  float minLevel;
  float maxLevel;
  float minPercent;
  float maxPercent;
};

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
    tankData.isOnline = false;
    tankData.hasData = false;

    initializeAlarmAckData();
  }
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
  Serial.print("Select option (1-6): ");
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
      default:
        Serial.println("Invalid option. Please select 1-6.");
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

// Read a single trig/echo pulse and convert to cm
float readSingleUltrasonicPulse() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 60000);  // 60ms timeout
  float distance = duration * 0.0343 / 2;  // Convert to cm

  return distance;
}

// Takes multiple samples and returns a MAD-filtered median distance in cm.
// Returns -1.0 if not enough valid samples were captured.
float readDistanceCM() {
  float readings[SAMPLE_COUNT];
  int count = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    float distance = readSingleUltrasonicPulse();

    if (distance > SENSOR_BLIND_ZONE_CM && distance <= SENSOR_MAX_RANGE_CM) {
      readings[count++] = distance;
    }

    delay(READING_DELAY);
  }

  if (count < MIN_VALID_SAMPLES) {
    Serial.print("Ultrasonic read failed: only ");
    Serial.print(count);
    Serial.println(" valid samples");
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
}

// Poll the sensor and update tank data if a valid reading was captured
void pollUltrasonicSensor() {
  if (!tankConfig.enabled) return;

  float distance = readDistanceCM();
  if (distance < 0) {
    return;  // Not enough valid samples this cycle - keep last known reading
  }

  processWaterLevelData(distance);

  Serial.print("Tank (");
  Serial.print(tankConfig.customName);
  Serial.print("): ");
  Serial.print(distance, 1);
  Serial.print("cm -> ");
  Serial.print(tankData.currentPercent, 1);
  Serial.println("%");
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
                screenMgr.systemInfoTimeout = millis() + 5000; // Show system info for 5 seconds
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
      Serial.print(tankData.isOnline ? "ONLINE" : "OFFLINE");
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
bool testOLEDConnection() {
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  return (Wire.endTransmission() == 0);
}

// Gracefully handles missing OLED
void initializeDisplay() {
  Serial.print("Testing OLED connection at 0x");
  Serial.print(OLED_I2C_ADDRESS, HEX);
  Serial.println("...");

  // Test if OLED responds on I2C bus
  if (!testOLEDConnection()) {
    Serial.println("WARNING: OLED not found at configured address!");
    Serial.println("System will continue WITHOUT display - WebUI still available");
    oledAvailable = false;
    return;  // Exit early - no display to initialize
  }

  Serial.println("OLED detected on I2C bus");

  // Use configured OLED address (U8g2 hardware I2C expects address << 1)
  u8g2.setI2CAddress(OLED_I2C_ADDRESS << 1);

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
void buildScreenList() {
  screenMgr.totalScreens = tankConfig.enabled ? 1 : 0;

  // Ensure we have at least one screen (show unconfigured screen if nothing else)
  if (screenMgr.totalScreens == 0) screenMgr.totalScreens = 1;
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

  // Show system info if requested via double press
  if (screenMgr.showSystemInfo) {
    showSystemScreen();
    return;
  }

  if (tankConfig.enabled) {
    showTankScreen();
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

void showTankScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);

  // Tank name and alarm indicator
  String title = String(tankConfig.customName);
  bool hasAlarm = false;

  if (tankData.hasData) {
    hasAlarm = (tankConfig.lowAlarmPercent > 0 &&
                tankData.currentPercent < tankConfig.lowAlarmPercent) ||
               (tankConfig.highAlarmPercent > 0 &&
                tankData.currentPercent > tankConfig.highAlarmPercent) ||
               !tankData.isOnline;
  } else if (!tankData.isOnline) {
    hasAlarm = true;
  }

  // Add mute icon if global mute is enabled and alarm is present
  if (hasAlarm && globalBuzzerMute) {
    title += " MUTE";
  } else if (hasAlarm) {
    title += " ALARM";
  }

  u8g2.drawStr(0, 12, title.c_str());

  // Wider vertical tank (doubled width: 24px instead of 12px)
  u8g2.drawFrame(5, 18, 24, 30);
  if (tankData.hasData && tankData.isOnline) {
    // Fill tank graphic with LIVE percentage (immediate visual feedback)
    int fillHeight = (28 * tankData.currentPercent) / 100;
    if (fillHeight > 0) {
      u8g2.drawBox(6, 47-fillHeight, 22, fillHeight); // Wider fill area
    }
  }

  if (tankData.hasData && tankData.isOnline) {
    // Show LIVE percentage (immediate updates from every reading)
    String percentStr = String((int)round(tankData.currentPercent)) + "%";
    u8g2.setFont(u8g2_font_10x20_tr); // Large font for percentage
    u8g2.drawStr(35, 32, percentStr.c_str()); // Moved right and adjusted Y position

    // Volume in smaller font below percentage
    if (tankConfig.volume > 0) {
      float litres = (tankData.currentPercent / 100.0) * tankConfig.volume;
      String volumeStr = String((int)litres) + " Litres";
      u8g2.setFont(u8g2_font_6x10_tr);
      u8g2.drawStr(35, 45, volumeStr.c_str()); // Moved right and adjusted Y position
    }

    // Min/Max alarm percentages in very small font at bottom
    if (tankConfig.lowAlarmPercent > 0 || tankConfig.highAlarmPercent > 0) {
      String alarmStr = "Low: " + String(tankConfig.lowAlarmPercent) + "% High: " + String(tankConfig.highAlarmPercent) + "%";
      u8g2.setFont(u8g2_font_5x7_tr); // Very small font
      u8g2.drawStr(0, 63, alarmStr.c_str());
    }
  } else {
    // Offline/No Data status - moved to match new layout
    u8g2.setFont(u8g2_font_6x10_tr);
    String status = tankData.hasData ? "OFFLINE" : "NO DATA";
    u8g2.drawStr(35, 35, status.c_str()); // Moved right to match percentage position
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

  String monitorStr = tankConfig.enabled ? "Monitoring: Tank" : "Monitoring: None";
  u8g2.drawStr(0, 64, monitorStr.c_str());

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
    } else {
      Serial.println("Available commands: CONFIG, STATUS, ALARM, WIFI, RESET, RESTART, MUTE, UNMUTE, MUTESTATUS");
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
    tank["lastUpdate"] = tankData.hasData ? millis() - tankData.lastUpdate : 0;
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

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
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

// FIXED: Updated getMainPageHTML with corrected JavaScript
String getMainPageHTML() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>Tank Monitor</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body{font-family:Arial;margin:20px;background:#f0f8ff}";
  html += ".container{max-width:600px;margin:0 auto;background:white;padding:20px;border-radius:10px}";
  html += ".header{text-align:center;color:#2c3e50;border-bottom:2px solid #3498db;padding-bottom:10px;margin-bottom:20px}";
  html += ".status{background:#f8f9fa;padding:15px;border-radius:5px;margin:10px 0}";
  html += ".alarm{background:#ffe6e6;border:2px solid #ff4444;padding:15px;border-radius:5px;margin:10px 0}";
  html += ".alarm-details{background:#ffcccc;border:1px solid #ff6666;padding:10px;margin:10px 0;border-radius:5px}";
  html += ".alarm-item{background:#fff2f2;border-left:4px solid #ff4444;padding:8px;margin:5px 0;border-radius:3px}";
  html += ".alarm-message{font-size:0.9em;color:#666;font-style:italic}";
  html += ".btn{background:#3498db;color:white;padding:10px 20px;border:none;border-radius:5px;margin:5px;cursor:pointer;text-decoration:none;display:inline-block}";
  html += ".btn-alarm{background:#e74c3c}";
  html += ".btn-mute{background:#f39c12}";
  html += ".btn-unmute{background:#27ae60}";
  html += ".btn:hover{opacity:0.8}";
  html += ".online{color:#27ae60}";
  html += ".offline{color:#e74c3c}";
  html += ".mute-control{background:#fff3cd;border:2px solid #f39c12;padding:15px;border-radius:5px;margin:10px 0;text-align:center}";
  html += ".mute-status{font-weight:bold;font-size:1.1em;margin:10px 0}";
  // Mini tank graphic for dashboard
  html += ".tank-row{display:flex;align-items:center;gap:15px}";
  html += ".mini-tank{flex-shrink:0;width:40px}";
  html += ".mt-cap{width:25px;height:3px;background:#34495e;border-radius:50% 50% 0 0;margin:0 auto}";
  html += ".mt-top{width:36px;height:10px;background:linear-gradient(to bottom,#e8eef1,#d5dbdb);clip-path:polygon(15% 0,85% 0,100% 100%,0 100%);margin:0 auto}";
  html += ".mt-body{width:36px;height:50px;background:linear-gradient(to bottom,#ecf0f1,#d5dbdb);border:2px solid #34495e;border-top:none;border-radius:0 0 6px 6px;position:relative;overflow:hidden;margin:0 auto}";
  html += ".mt-water{position:absolute;bottom:0;width:100%;background:linear-gradient(to top,#1565c0,#42a5f5);transition:height 0.5s}";
  html += ".mt-pct{position:absolute;width:100%;text-align:center;top:50%;transform:translateY(-50%);font-size:0.65em;font-weight:bold;color:#fff;text-shadow:0 0 3px rgba(0,0,0,0.5)}";
  html += ".tank-details{flex:1}";
  html += ".ago{font-size:0.8em;color:#999}";
  html += ".gear{text-decoration:none;font-size:1.4em;color:#2980b9;float:right;cursor:pointer;font-weight:bold}";
  html += ".gear:hover{color:#3498db}";
  html += "</style>";
  html += "<script>";

  // Time ago helper
  html += "function timeAgo(elapsedMs){";
  html += "if(!elapsedMs||elapsedMs<=0)return '';";
  html += "const s=Math.round(elapsedMs/1000);";
  html += "if(s<60)return s+'s ago';";
  html += "if(s<3600)return Math.floor(s/60)+'m ago';";
  html += "return Math.floor(s/3600)+'h ago';}";

  // Corrected buildStatusHTML function
  html += "function buildStatusHTML(data){";
  html += "try{";
  html += "if(!data){return '<div class=\"status\">No data available</div>';}";
  html += "let html='';";
  html += "if(data.alarm_active){";
  html += "html+='<div class=\"alarm\"><h3>ALARM ACTIVE!</h3>';";
  html += "if(data.active_alarms && data.active_alarms.length > 0){";
  html += "html+='<div class=\"alarm-details\">';";
  html += "html+='<h4>Active Alarms:</h4>';";
  html += "for(let i=0; i<data.active_alarms.length; i++){";
  html += "let alarm=data.active_alarms[i];";
  html += "html+='<div class=\"alarm-item\">';";
  html += "html+='<strong>'+(alarm.type||'Unknown')+'</strong> - '+(alarm.sensor||'Unknown Sensor')+'<br>';";
  html += "html+='<span class=\"alarm-message\">'+(alarm.message||'No details available')+'</span>';";
  html += "html+='</div>';";
  html += "}";
  html += "html+='</div>';";
  html += "}";
  html += "html+='<button class=\"btn btn-alarm\" onclick=\"acknowledgeAlarms()\">Acknowledge All Alarms</button>';";
  html += "if(data.all_acknowledged){html+='<p>All current conditions acknowledged. Buzzer will restart if conditions worsen.</p>';}";
  html += "html+='</div>';}";
  html += "html+='<h3>Tank Sensor:</h3>';";
  html += "const tank=data.tank;";
  html += "if(tank && tank.enabled){";
  html += "const pct=tank.percent!=null?Math.round(tank.percent):0;";
  html += "const fillH=tank.online?pct:0;";
  html += "html+='<div class=\"status\"><div class=\"tank-row\">';";
  // Mini tank graphic
  html += "html+='<div class=\"mini-tank\">';";
  html += "html+='<div class=\"mt-cap\"></div><div class=\"mt-top\"></div>';";
  html += "html+='<div class=\"mt-body\"><div class=\"mt-water\" style=\"height:'+fillH+'%\"></div>';";
  html += "if(tank.online){html+='<div class=\"mt-pct\">'+pct+'%</div>';}";
  html += "html+='</div></div>';";
  // Tank details
  html += "html+='<div class=\"tank-details\">';";
  html += "html+='<a class=\"gear\" href=\"/config\" title=\"Settings\">&#x2699;</a>';";
  html += "html+='<strong>'+(tank.name||'Unknown')+'</strong><br>';";
  html += "html+='Status: <span class=\"'+(tank.online?'online':'offline')+'\">'+(tank.online?'ONLINE':'OFFLINE')+'</span>';";
  html += "if(tank.lastUpdate){html+=' <span class=\"ago\">'+timeAgo(tank.lastUpdate)+'</span>';}";
  html += "html+='<br>';";
  html += "if(tank.online){";
  html += "html+='Level: <strong style=\"font-size:1.2em\">'+pct+'%</strong> ('+(tank.level!=null?tank.level.toFixed(1):'--')+'cm)<br>';";
  html += "}";
  html += "html+='</div></div></div>';";
  html += "}else{html+='<div class=\"status\">No tank sensor configured</div>';}";
  html += "return html;";
  html += "}catch(e){console.error('Error in buildStatusHTML:',e);return '<div class=\"status\">Error displaying data</div>';}}";

  html += "function updateStatus(){";
  html += "fetch('/api/status').then(r=>{";
  html += "if(!r.ok){throw new Error('HTTP '+r.status);}";
  html += "return r.json();";
  html += "}).then(data=>{";
  html += "document.getElementById('status').innerHTML=buildStatusHTML(data);";
  html += "}).catch(e=>{";
  html += "console.error('Update failed:',e);";
  html += "document.getElementById('status').innerHTML='<div class=\"status\">Error loading data: '+e.message+'</div>';";
  html += "});}";
  html += "function acknowledgeAlarms(){";
  html += "fetch('/alarm/acknowledge',{'method':'POST'}).then(r=>{";
  html += "if(!r.ok){throw new Error('HTTP '+r.status);}";
  html += "updateStatus();";
  html += "}).catch(e=>{";
  html += "console.error('Failed to acknowledge alarms:',e);";
  html += "alert('Failed to acknowledge alarms: '+e.message);";
  html += "});}";

  // Mute control functions
  html += "function toggleMute(){";
  html += "fetch('/api/mute/toggle',{'method':'POST'}).then(r=>{";
  html += "if(!r.ok){throw new Error('HTTP '+r.status);}";
  html += "return r.json();";
  html += "}).then(data=>{";
  html += "updateMuteStatus();";
  html += "}).catch(e=>{";
  html += "console.error('Failed to toggle mute:',e);";
  html += "alert('Failed to toggle mute: '+e.message);";
  html += "});}";

  html += "function updateMuteStatus(){";
  html += "fetch('/api/mute/status').then(r=>{";
  html += "if(!r.ok){throw new Error('HTTP '+r.status);}";
  html += "return r.json();";
  html += "}).then(data=>{";
  html += "const muteDiv=document.getElementById('mute-control');";
  html += "if(muteDiv){";
  html += "let html='<h3>Alarm Buzzer Control</h3>';";
  html += "html+='<div class=\"mute-status\">';";
  html += "if(data.muted){";
  html += "html+='Status: <span style=\"color:#f39c12\">MUTED</span><br>';";
  html += "html+='<small>Visual alarms remain active</small>';";
  html += "}else{";
  html += "html+='Status: <span style=\"color:#27ae60\">ACTIVE</span>';";
  html += "}";
  html += "html+='</div>';";
  html += "html+='<button class=\"btn '+(data.muted?'btn-unmute':'btn-mute')+'\" onclick=\"toggleMute()\">';";
  html += "html+=(data.muted?'Unmute':'Mute')+' Buzzer</button>';";
  html += "muteDiv.innerHTML=html;";
  html += "}";
  html += "}).catch(e=>{";
  html += "console.error('Failed to get mute status:',e);";
  html += "});}";

  html += "setInterval(updateStatus,5000);";
  html += "setInterval(updateMuteStatus,5000);";
  html += "</script></head><body>";

  html += "<div class='container'>";
  html += "<div class='header'><h1>Tank Monitor System</h1></div>";

  // Always show the status div - let JavaScript populate it
  html += "<div id='status'>Loading sensor data...</div>";

  // Mute control section
  html += "<div id='mute-control' class='mute-control'>Loading mute status...</div>";

  html += "<div class='status'>";
  html += "<h3>System Info:</h3>";
  html += "Firmware Version: <span style='color:#27ae60;font-weight:bold'>" + String(FIRMWARE_VERSION) + "</span><br>";
  html += "<small style='color:#666'>To update firmware, open the Menu and select Firmware Update</small><br><br>";
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

  html += "<div style='text-align:center;margin-top:20px'>";
  html += "<a href='/wifi' class='btn'>WiFi Settings</a>";
  html += "<a href='/config' class='btn'>Settings</a>";
  html += "<button class='btn' onclick='updateStatus()'>Refresh</button>";
  html += "</div>";

  html += "</div>";
  html += "<script>updateStatus();updateMuteStatus();</script>";  // Initialize on page load
  html += "</body></html>";

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

  Serial.println("=== TANK MONITOR v5.0.1 ===");
  Serial.println("FEATURES:");
  Serial.println("- Wired ultrasonic sensor (direct trig/echo)");
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

  // Initialize LittleFS
  if (!LittleFS.begin(false)) {
    Serial.println("LittleFS mount failed");
  } else {
    Serial.println("LittleFS initialized");
  }

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

  // Poll the wired ultrasonic sensor on a fixed interval
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead > SENSOR_READ_INTERVAL_MS) {
    pollUltrasonicSensor();
    lastSensorRead = millis();
  }

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
