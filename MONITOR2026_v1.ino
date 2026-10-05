#include <SPI.h>
#include <LoRa.h>
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

#define FIRMWARE_VERSION "5.0.0"
#define FIRMWARE_CODE "SF-MON-2026-9867"



// LoRa pins for ESP32 (FIXED)
#define SS      5
#define RST     14
#define DIO0    26

// Hardware pins (FIXED - No conflicts)
#define BUZZER_PIN 4   // Changed back to 4 - safer pin, no boot strapping issues
#define BUTTON_PIN 33  // Changed from 19 to 33
#define SDA_PIN 21
#define SCL_PIN 22

// OLED I2C Address Configuration
// Change this to 0x3D if your OLED uses that address
#define OLED_I2C_ADDRESS 0x3D  // Most common: 0x3C, Alternative: 0x3D

// LoRa Configuration Constants
// CRITICAL: These values MUST match transmitter configuration
// Changing these will break communication with sensors
#define LORA_SPREADING_FACTOR 10
#define LORA_BANDWIDTH 125E3
#define LORA_CODING_RATE 8
#define LORA_PREAMBLE 8
#define LORA_SYNC_WORD 0x12

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
#define MAX_TANKS 2
#define MAX_WP_SENSORS 2
#define SENSOR_BLIND_ZONE_CM 21
#define SENSOR_OFFLINE_TIMEOUT_MINUTES 5
#define DEFAULT_SNOOZE_MINUTES 5
#define MIN_PERCENTAGE_GAP 5
#define DEFAULT_CONFIRMATIONS_REQUIRED 3
#define MIN_CONFIRMATIONS 1
#define MAX_CONFIRMATIONS 10

// EEPROM addresses (UPDATED for new volume field)
#define EEPROM_SIZE 512
#define EEPROM_CONFIGURED_FLAG 0
#define EEPROM_DEVICE_NAME 1         // 32 bytes
#define EEPROM_TANK_CONFIG_START 33  // 2 tanks × 25 bytes = 50 bytes (was 42)
#define EEPROM_WP_CONFIG_START 83    // 2 WP sensors × 18 bytes = 36 bytes (was 75)
#define EEPROM_GENERAL_SETTINGS 119  // 10 bytes (was 111)
#define EEPROM_ALARM_ACK_START 129   // Alarm acknowledgment data (20 bytes) (was 121)
#define EEPROM_WIFI_CONFIGURED 149   // WiFi configuration (1 byte flag) (was 141)
#define EEPROM_WIFI_SSID 150         // WiFi SSID (32 bytes) (was 142)
#define EEPROM_WIFI_PASSWORD 182     // WiFi password (64 bytes) (was 174)
#define EEPROM_OTA_CONFIGURED 246    // OTA configuration (1 byte flag)
#define EEPROM_OTA_ENABLED 247       // OTA enabled flag (1 byte)
#define EEPROM_OTA_HOSTNAME 248      // OTA hostname (32 bytes)
#define EEPROM_GLOBAL_MUTE 280       // Global buzzer mute flag (1 byte)
#define MAGIC_NUMBER 0xAB

// Function prototypes
String getStringInput(const String& prompt, int maxLength, bool allowEmpty = false);
int getIntInput(const String& prompt, int minVal, int maxVal);
bool validateNodeID(const String& nodeID);
bool validateCustomName(const String& name);
bool isNodeIDUnique(const String& nodeID, int skipIndex, bool isTank);
bool isCustomNameUnique(const String& customName, int skipIndex, bool isTank);
bool validatePercentages(uint8_t lowPercent, uint8_t highPercent);

// Screen management system (REPLACES DisplayMode)
struct ScreenManager {
  int currentScreen;
  int totalScreens;
  unsigned long lastUpdate;
  bool manualMode;
  unsigned long manualTimeout;
  bool showSystemInfo;
  unsigned long systemInfoTimeout;
  bool showSensorTech;
  unsigned long sensorTechTimeout;
};
ScreenManager screenMgr = {0, 0, 0, false, 0, false, 0, false, 0};

// Alarm acknowledgment tracking structure
struct AlarmAckData {
  bool tankLowThresholdAck[MAX_TANKS];
  bool tankHighThresholdAck[MAX_TANKS];
  bool tankOfflineAck[MAX_TANKS];
  bool wpAlarmAck[MAX_WP_SENSORS];
  bool wpOfflineAck[MAX_WP_SENSORS];
  bool isValid;
};

// Tank configuration structure (UPDATED with volume field)
struct TankConfig {
  char nodeID[9];
  char customName[9];
  uint16_t height;
  uint32_t volume;          // NEW: Tank volume in litres (0-99999)
  uint8_t highAlarmPercent;
  uint8_t lowAlarmPercent;
  bool enabled;
};

// Water Presence sensor configuration structure
struct WPConfig {
  char nodeID[9];
  char customName[9];
  uint8_t alarmMode;
  bool enabled;
};

// General settings structure
struct GeneralSettings {
  uint8_t acknowledgeTimeout;
  uint8_t offlineTimeoutMinutes;
  bool buzzerEnabled;
};

// Sensor confirmation settings (not saved to EEPROM)
uint8_t confirmationsRequired = DEFAULT_CONFIRMATIONS_REQUIRED;

// Runtime sensor data structure
struct SensorData {
  float currentLevel;
  float currentPercent;
  unsigned long lastUpdate;
  bool isOnline;
  int rssi;
  bool hasData;
  float minLevel;
  float maxLevel;
  float minPercent;
  float maxPercent;
};

struct WPSensorData {
  bool waterPresent;
  unsigned long lastUpdate;
  bool isOnline;
  int rssi;
  bool hasData;
};

// Sensor confirmation structure (prevents false alarms from single bad readings)
struct SensorConfirmation {
  int confirmCount;              // Current confirmation count (0-5)
  String lastConfirmMsgId;       // Last message ID processed
  unsigned long lastConfirmTime; // For 60s timeout tracking
  bool confirmationReady;        // True when 5 confirmations reached
  float confirmedPercent;        // Last confirmed water percentage
  float confirmedLevel;          // Last confirmed water level (cm)

  // Confirmation tracking for alarm clearing (prevents false resets)
  int clearConfirmCountLow;      // Confirmation count for clearing low alarm
  int clearConfirmCountHigh;     // Confirmation count for clearing high alarm
  bool lowClearConfirmed;        // True when low alarm clear is confirmed
  bool highClearConfirmed;       // True when high alarm clear is confirmed
};

// Global configuration variables
String deviceName = "monitor";
String fullDeviceName = "monitor_mvstech";
TankConfig tankConfigs[MAX_TANKS];
WPConfig wpConfigs[MAX_WP_SENSORS];
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
SensorData tankData[MAX_TANKS];
WPSensorData wpData[MAX_WP_SENSORS];

// Sensor confirmation arrays (one per tank sensor)
SensorConfirmation tankConfirm[MAX_TANKS];

// Fresh data protection flags (prevent acting on stale/cached readings)
bool needFreshData[MAX_TANKS] = {true, true};

// LoRa health check variables
unsigned long lastPacketTime = 0;
static unsigned int loRaReinitCount = 0;

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
String lastReceivedMsgId = "";

// Session ID and Counter tracking for duplicate packet rejection (per sensor)
// Tracks each configured sensor separately to reject relay duplicates
uint32_t lastUS_SessionID[MAX_TANKS] = {0, 0};      // US sensor 0, US sensor 1
uint32_t lastUS_Counter[MAX_TANKS] = {0, 0};
uint32_t lastWP_SessionID[MAX_WP_SENSORS] = {0, 0}; // WP sensor 0, WP sensor 1
uint32_t lastWP_Counter[MAX_WP_SENSORS] = {0, 0};

// LoRa device scan variables (RAM only - max 5 US + 5 WP)
struct ScanEntry { char id[12]; int8_t rssi; };
ScanEntry scannedUS[5];
ScanEntry scannedWP[5];
uint8_t scanUSCount = 0;
uint8_t scanWPCount = 0;
bool scanActive = false;
unsigned long scanStartTime = 0;
#define SCAN_DURATION_MS 20000

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
    
    // Load tank configurations (UPDATED for new volume field)
    for (int i = 0; i < MAX_TANKS; i++) {
      int baseAddr = EEPROM_TANK_CONFIG_START + (i * 25); // Was 21, now 25 bytes
      
      for (int j = 0; j < 8; j++) {
        tankConfigs[i].nodeID[j] = EEPROM.read(baseAddr + j);
      }
      tankConfigs[i].nodeID[8] = '\0';
      
      for (int j = 0; j < 8; j++) {
        tankConfigs[i].customName[j] = EEPROM.read(baseAddr + 8 + j);
      }
      tankConfigs[i].customName[8] = '\0';
      
      tankConfigs[i].height = (EEPROM.read(baseAddr + 16) << 8) | EEPROM.read(baseAddr + 17);
      
      // Load volume (NEW - 4 bytes)
      tankConfigs[i].volume = ((uint32_t)EEPROM.read(baseAddr + 18) << 24) |
                              ((uint32_t)EEPROM.read(baseAddr + 19) << 16) |
                              ((uint32_t)EEPROM.read(baseAddr + 20) << 8) |
                              (uint32_t)EEPROM.read(baseAddr + 21);
      
      tankConfigs[i].highAlarmPercent = EEPROM.read(baseAddr + 22);
      tankConfigs[i].lowAlarmPercent = EEPROM.read(baseAddr + 23);
      tankConfigs[i].enabled = (EEPROM.read(baseAddr + 24) == MAGIC_NUMBER);
      
      if (tankConfigs[i].enabled) {
        tankData[i].minLevel = 9999;
        tankData[i].maxLevel = 0;
        tankData[i].minPercent = 100;
        tankData[i].maxPercent = 0;
      }
    }
    
    // Load WP configurations
    for (int i = 0; i < MAX_WP_SENSORS; i++) {
      int baseAddr = EEPROM_WP_CONFIG_START + (i * 18);
      
      for (int j = 0; j < 8; j++) {
        wpConfigs[i].nodeID[j] = EEPROM.read(baseAddr + j);
      }
      wpConfigs[i].nodeID[8] = '\0';
      
      for (int j = 0; j < 8; j++) {
        wpConfigs[i].customName[j] = EEPROM.read(baseAddr + 8 + j);
      }
      wpConfigs[i].customName[8] = '\0';
      
      wpConfigs[i].alarmMode = EEPROM.read(baseAddr + 16);
      wpConfigs[i].enabled = (EEPROM.read(baseAddr + 17) == MAGIC_NUMBER);
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
    
    for (int i = 0; i < MAX_TANKS; i++) {
      tankConfigs[i].enabled = false;
      tankData[i].isOnline = false;
      tankData[i].hasData = false;
    }
    
    for (int i = 0; i < MAX_WP_SENSORS; i++) {
      wpConfigs[i].enabled = false;
      wpData[i].isOnline = false;
      wpData[i].hasData = false;
    }
    
    initializeAlarmAckData();
  }
}

void initializeAlarmAckData() {
  memset(&alarmAck, 0, sizeof(alarmAck));
  
  for (int i = 0; i < MAX_TANKS; i++) {
    alarmAck.tankLowThresholdAck[i] = false;
    alarmAck.tankHighThresholdAck[i] = false;
    alarmAck.tankOfflineAck[i] = false;
  }
  
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    alarmAck.wpAlarmAck[i] = false;
    alarmAck.wpOfflineAck[i] = false;
  }
  
  alarmAck.isValid = true;
  saveAlarmAckData();
}

void loadAlarmAckData() {
  if (EEPROM.read(EEPROM_ALARM_ACK_START) == MAGIC_NUMBER) {
    int addr = EEPROM_ALARM_ACK_START + 1;
    
    for (int i = 0; i < MAX_TANKS; i++) {
      alarmAck.tankLowThresholdAck[i] = (EEPROM.read(addr++) == MAGIC_NUMBER);
      alarmAck.tankHighThresholdAck[i] = (EEPROM.read(addr++) == MAGIC_NUMBER);
      alarmAck.tankOfflineAck[i] = (EEPROM.read(addr++) == MAGIC_NUMBER);
    }
    
    for (int i = 0; i < MAX_WP_SENSORS; i++) {
      alarmAck.wpAlarmAck[i] = (EEPROM.read(addr++) == MAGIC_NUMBER);
      alarmAck.wpOfflineAck[i] = (EEPROM.read(addr++) == MAGIC_NUMBER);
    }
    
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
  
  for (int i = 0; i < MAX_TANKS; i++) {
    EEPROM.write(addr++, alarmAck.tankLowThresholdAck[i] ? MAGIC_NUMBER : 0);
    EEPROM.write(addr++, alarmAck.tankHighThresholdAck[i] ? MAGIC_NUMBER : 0);
    EEPROM.write(addr++, alarmAck.tankOfflineAck[i] ? MAGIC_NUMBER : 0);
  }
  
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    EEPROM.write(addr++, alarmAck.wpAlarmAck[i] ? MAGIC_NUMBER : 0);
    EEPROM.write(addr++, alarmAck.wpOfflineAck[i] ? MAGIC_NUMBER : 0);
  }
  
  EEPROM.commit();
  Serial.println("Alarm acknowledgment data saved to EEPROM");
}

void saveConfiguration() {
  EEPROM.write(EEPROM_CONFIGURED_FLAG, MAGIC_NUMBER);
  
  // Save tank configurations (UPDATED for new volume field)
  for (int i = 0; i < MAX_TANKS; i++) {
    int baseAddr = EEPROM_TANK_CONFIG_START + (i * 25); // Now 25 bytes
    
    for (int j = 0; j < 8; j++) {
      EEPROM.write(baseAddr + j, tankConfigs[i].nodeID[j]);
    }
    
    for (int j = 0; j < 8; j++) {
      EEPROM.write(baseAddr + 8 + j, tankConfigs[i].customName[j]);
    }
    
    EEPROM.write(baseAddr + 16, tankConfigs[i].height >> 8);
    EEPROM.write(baseAddr + 17, tankConfigs[i].height & 0xFF);
    
    // Save volume (NEW - 4 bytes)
    EEPROM.write(baseAddr + 18, (tankConfigs[i].volume >> 24) & 0xFF);
    EEPROM.write(baseAddr + 19, (tankConfigs[i].volume >> 16) & 0xFF);
    EEPROM.write(baseAddr + 20, (tankConfigs[i].volume >> 8) & 0xFF);
    EEPROM.write(baseAddr + 21, tankConfigs[i].volume & 0xFF);
    
    EEPROM.write(baseAddr + 22, tankConfigs[i].highAlarmPercent);
    EEPROM.write(baseAddr + 23, tankConfigs[i].lowAlarmPercent);
    EEPROM.write(baseAddr + 24, tankConfigs[i].enabled ? MAGIC_NUMBER : 0);
  }
  
  // Save WP configurations
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    int baseAddr = EEPROM_WP_CONFIG_START + (i * 18);
    
    for (int j = 0; j < 8; j++) {
      EEPROM.write(baseAddr + j, wpConfigs[i].nodeID[j]);
    }
    
    for (int j = 0; j < 8; j++) {
      EEPROM.write(baseAddr + 8 + j, wpConfigs[i].customName[j]);
    }
    
    EEPROM.write(baseAddr + 16, wpConfigs[i].alarmMode);
    EEPROM.write(baseAddr + 17, wpConfigs[i].enabled ? MAGIC_NUMBER : 0);
  }
  
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
  
  for (int i = 0; i < MAX_TANKS; i++) {
    tankConfigs[i].enabled = false;
    memset(tankConfigs[i].nodeID, 0, sizeof(tankConfigs[i].nodeID));
    memset(tankConfigs[i].customName, 0, sizeof(tankConfigs[i].customName));
    tankConfigs[i].height = 0;
    tankConfigs[i].volume = 0;
    tankConfigs[i].highAlarmPercent = 0;
    tankConfigs[i].lowAlarmPercent = 0;
  }
  
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    wpConfigs[i].enabled = false;
    memset(wpConfigs[i].nodeID, 0, sizeof(wpConfigs[i].nodeID));
    memset(wpConfigs[i].customName, 0, sizeof(wpConfigs[i].customName));
    wpConfigs[i].alarmMode = 0;
  }
  
  generalSettings.acknowledgeTimeout = 5;
  generalSettings.offlineTimeoutMinutes = SENSOR_OFFLINE_TIMEOUT_MINUTES;
  generalSettings.buzzerEnabled = true;
  
  isConfigured = false;
  Serial.println("Configuration reset to factory defaults");
}

// ===========================================
// VALIDATION FUNCTIONS
// ===========================================

bool isNodeIDUnique(const String& nodeID, int skipIndex, bool isTank) {
  for (int i = 0; i < MAX_TANKS; i++) {
    if (tankConfigs[i].enabled && i != (isTank ? skipIndex : -1)) {
      if (String(tankConfigs[i].nodeID) == nodeID) {
        return false;
      }
    }
  }
  
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    if (wpConfigs[i].enabled && i != (!isTank ? skipIndex : -1)) {
      if (String(wpConfigs[i].nodeID) == nodeID) {
        return false;
      }
    }
  }
  
  return true;
}

bool isCustomNameUnique(const String& customName, int skipIndex, bool isTank) {
  for (int i = 0; i < MAX_TANKS; i++) {
    if (tankConfigs[i].enabled && i != (isTank ? skipIndex : -1)) {
      if (String(tankConfigs[i].customName) == customName) {
        return false;
      }
    }
  }
  
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    if (wpConfigs[i].enabled && i != (!isTank ? skipIndex : -1)) {
      if (String(wpConfigs[i].customName) == customName) {
        return false;
      }
    }
  }
  
  return true;
}

bool validateNodeID(const String& nodeID) {
  // Allow 2-8 characters for flexible node ID formats
  // Supports: US01, US0001, USxxxx (4-digit), WP01, WPxxxx, etc.
  if (nodeID.length() < 2 || nodeID.length() > 8) {
    return false;
  }

  for (int i = 0; i < nodeID.length(); i++) {
    char c = nodeID[i];
    // Allow uppercase letters and digits
    if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
      return false;
    }
  }

  return true;
}

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
  Serial.println("1. Add/Edit Tank Sensor (Max 2)");
  Serial.println("2. Add/Edit Water Presence Sensor (Max 2)");
  Serial.println("3. General Settings");
  Serial.println("4. WiFi Configuration");
  Serial.println("5. View Current Configuration");
  Serial.println("6. Scan for Available Sensors");
  Serial.println("7. Save & Exit");
  Serial.println("8. Reset All Configuration");
  Serial.print("Select option (1-8): ");
}

void scanForSensors() {
  Serial.println("\n=== SENSOR DISCOVERY ===");
  Serial.println("Scanning for available sensors for 30 seconds...");
  Serial.println("Press any key to stop scan early.");
  
  while (Serial.available()) {
    Serial.read();
  }
  
  unsigned long scanStart = millis();
  unsigned long scanDuration = 30000;
  String discoveredSensors[20];
  int discoveredCount = 0;
  
  while ((millis() - scanStart) < scanDuration && discoveredCount < 20) {
    if (Serial.available()) {
      Serial.println("\nScan interrupted by user.");
      break;
    }
    
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String receivedData = "";
      while (LoRa.available()) {
        receivedData += (char)LoRa.read();
      }
      int rssi = LoRa.packetRssi();
      
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, receivedData);

      if (!error && doc.containsKey("node_id")) {
        String nodeId = doc["node_id"].as<String>();
        String extractedSensorId = getSensorID(nodeId);

        bool alreadyFound = false;
        for (int i = 0; i < discoveredCount; i++) {
          if (discoveredSensors[i].indexOf(extractedSensorId) >= 0) {
            alreadyFound = true;
            break;
          }
        }

        if (!alreadyFound) {
          String sensorInfo = extractedSensorId + " (RSSI: " + String(rssi) + " dBm)";
          discoveredSensors[discoveredCount] = sensorInfo;
          discoveredCount++;

          Serial.print("Found: ");
          Serial.println(sensorInfo);
        }
      }
    }
    
    static unsigned long lastProgress = 0;
    if (millis() - lastProgress > 5000) {
      unsigned long elapsed = (millis() - scanStart) / 1000;
      unsigned long remaining = (scanDuration / 1000) - elapsed;
      Serial.print("Scanning... ");
      Serial.print(remaining);
      Serial.print("s remaining, ");
      Serial.print(discoveredCount);
      Serial.println(" sensors found");
      lastProgress = millis();
    }
    
    delay(50);
  }
  
  while (Serial.available()) {
    Serial.read();
  }
  
  Serial.println("\n=== SCAN RESULTS ===");
  if (discoveredCount == 0) {
    Serial.println("No sensors discovered.");
    Serial.println("Make sure sensors are transmitting and within range.");
  } else {
    Serial.print("Discovered ");
    Serial.print(discoveredCount);
    Serial.println(" sensor(s):");
    
    for (int i = 0; i < discoveredCount; i++) {
      Serial.print(i + 1);
      Serial.print(". ");
      Serial.println(discoveredSensors[i]);
    }
    
    Serial.println("\nNote: You can use these Node IDs in the configuration menu.");
    Serial.println("Tank sensors typically start with 'US' (e.g., US01, US03)");
    Serial.println("Water presence sensors typically start with 'WP' (e.g., WP01)");
  }
  
  Serial.println("\nPress Enter to return to main menu...");
  while (Serial.available() == 0) {
    delay(100);
  }
  Serial.readString();
}

void configureTankSensor() {
  Serial.println("\n=== TANK SENSOR CONFIGURATION ===");
  
  Serial.println("Available Tank Slots:");
  for (int i = 0; i < MAX_TANKS; i++) {
    Serial.print("Slot ");
    Serial.print(i + 1);
    Serial.print(": ");
    if (tankConfigs[i].enabled) {
      Serial.print(tankConfigs[i].nodeID);
      Serial.print(" (");
      Serial.print(tankConfigs[i].customName);
      Serial.println(")");
    } else {
      Serial.println("Empty");
    }
  }
  
  int slot = getIntInput("Select slot to configure (1-2): ", 1, 2) - 1;
  
  TankConfig newConfig = tankConfigs[slot];
  
  // Node ID
  while (true) {
    String nodeID = getStringInput("Enter Tank Sensor Node ID (4-6 chars, e.g., US03): ", 6);
    nodeID.toUpperCase();
    
    if (!validateNodeID(nodeID)) {
      Serial.println("Invalid format. Use 4-6 characters (A-Z, 0-9) only.");
      continue;
    }
    
    if (!isNodeIDUnique(nodeID, slot, true)) {
      Serial.println("Node ID already in use. Please choose a different one.");
      continue;
    }
    
    strcpy(newConfig.nodeID, nodeID.c_str());
    break;
  }
  
  // Custom Name
  while (true) {
    String customName = getStringInput("Enter Custom Tank Name (max 8 chars, e.g., Terrace): ", 8);
    
    if (!validateCustomName(customName)) {
      Serial.println("Invalid format. Use letters and numbers only.");
      continue;
    }
    
    if (!isCustomNameUnique(customName, slot, true)) {
      Serial.println("Custom name already in use. Please choose a different one.");
      continue;
    }
    
    strcpy(newConfig.customName, customName.c_str());
    break;
  }
  
  // Tank Height
  newConfig.height = getIntInput("Enter Tank Height (cm, 50-500): ", 50, 500);
  
  // Tank Volume (NEW)
  newConfig.volume = getIntInput("Enter Tank Volume in Litres (0 to skip, 1-99999): ", 0, 99999);
  
  // Low Alarm Percentage
  newConfig.lowAlarmPercent = getIntInput("Enter Low Level Alarm Percentage (0=disable, 1-94): ", 0, 94);
  
  // High Alarm Percentage
  int maxHigh = (newConfig.lowAlarmPercent > 0) ? 99 : 99;
  int minHigh = (newConfig.lowAlarmPercent > 0) ? (newConfig.lowAlarmPercent + MIN_PERCENTAGE_GAP) : 0;
  
  String highPrompt = "Enter High Level Alarm Percentage (0=disable";
  if (newConfig.lowAlarmPercent > 0) {
    highPrompt += ", min " + String(minHigh);
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
  Serial.print("Node ID: ");
  Serial.println(newConfig.nodeID);
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
    tankConfigs[slot] = newConfig;
    
    tankData[slot].minLevel = 9999;
    tankData[slot].maxLevel = 0;
    tankData[slot].minPercent = 100;
    tankData[slot].maxPercent = 0;
    tankData[slot].isOnline = false;
    tankData[slot].hasData = false;
    
    saveConfiguration();
    Serial.println("Tank sensor configuration saved!");
  } else {
    Serial.println("Configuration cancelled.");
  }
}

void configureWPSensor() {
  Serial.println("\n=== WATER PRESENCE SENSOR CONFIGURATION ===");
  
  Serial.println("Available WP Slots:");
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    Serial.print("Slot ");
    Serial.print(i + 1);
    Serial.print(": ");
    if (wpConfigs[i].enabled) {
      Serial.print(wpConfigs[i].nodeID);
      Serial.print(" (");
      Serial.print(wpConfigs[i].customName);
      Serial.println(")");
    } else {
      Serial.println("Empty");
    }
  }
  
  int slot = getIntInput("Select slot to configure (1-2): ", 1, 2) - 1;
  
  WPConfig newConfig = wpConfigs[slot];
  
  // Node ID
  while (true) {
    String nodeID = getStringInput("Enter WP Sensor Node ID (4-6 chars, e.g., WP01): ", 6);
    nodeID.toUpperCase();
    
    if (!validateNodeID(nodeID)) {
      Serial.println("Invalid format. Use 4-6 characters (A-Z, 0-9) only.");
      continue;
    }
    
    if (!isNodeIDUnique(nodeID, slot, false)) {
      Serial.println("Node ID already in use. Please choose a different one.");
      continue;
    }
    
    strcpy(newConfig.nodeID, nodeID.c_str());
    break;
  }
  
  // Custom Name
  while (true) {
    String customName = getStringInput("Enter Custom WP Name (max 8 chars, e.g., MainPipe): ", 8);
    
    if (!validateCustomName(customName)) {
      Serial.println("Invalid format. Use letters and numbers only.");
      continue;
    }
    
    if (!isCustomNameUnique(customName, slot, false)) {
      Serial.println("Custom name already in use. Please choose a different one.");
      continue;
    }
    
    strcpy(newConfig.customName, customName.c_str());
    break;
  }
  
  // Alarm Mode
  Serial.println("Alarm Condition:");
  Serial.println("1. Alarm when NO water (typical use)");
  Serial.println("2. Alarm when water PRESENT (leak detection)");
  Serial.println("3. No alarm (monitoring only)");
  newConfig.alarmMode = getIntInput("Choose alarm mode (1-3): ", 1, 3);
  
  // Confirm and save
  Serial.println("\nConfiguration Summary:");
  Serial.print("Node ID: ");
  Serial.println(newConfig.nodeID);
  Serial.print("Custom Name: ");
  Serial.println(newConfig.customName);
  Serial.print("Alarm Mode: ");
  switch (newConfig.alarmMode) {
    case 1: Serial.println("Alarm when NO water"); break;
    case 2: Serial.println("Alarm when water PRESENT"); break;
    case 3: Serial.println("No alarm (monitoring only)"); break;
  }
  
  String confirm = getStringInput("Save this configuration? (Y/N): ", 1);
  if (confirm.equalsIgnoreCase("Y")) {
    newConfig.enabled = true;
    wpConfigs[slot] = newConfig;
    
    wpData[slot].isOnline = false;
    wpData[slot].hasData = false;
    
    saveConfiguration();
    Serial.println("WP sensor configuration saved!");
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
  Serial.println("5. Test Buzzer");  // NEW: Added buzzer test option
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
  
  Serial.println("\nTank Sensors:");
  for (int i = 0; i < MAX_TANKS; i++) {
    Serial.print("Slot ");
    Serial.print(i + 1);
    Serial.print(": ");
    if (tankConfigs[i].enabled) {
      Serial.print(tankConfigs[i].nodeID);
      Serial.print(" (");
      Serial.print(tankConfigs[i].customName);
      Serial.print(") - Height: ");
      Serial.print(tankConfigs[i].height);
      Serial.print("cm, Volume: ");
      if (tankConfigs[i].volume > 0) {
        Serial.print(tankConfigs[i].volume);
        Serial.print("L");
      } else {
        Serial.print("Not set");
      }
      Serial.print(", Low: ");
      Serial.print(tankConfigs[i].lowAlarmPercent);
      Serial.print("%, High: ");
      Serial.print(tankConfigs[i].highAlarmPercent);
      Serial.println("%");
    } else {
      Serial.println("Not configured");
    }
  }
  
  Serial.println("\nWater Presence Sensors:");
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    Serial.print("Slot ");
    Serial.print(i + 1);
    Serial.print(": ");
    if (wpConfigs[i].enabled) {
      Serial.print(wpConfigs[i].nodeID);
      Serial.print(" (");
      Serial.print(wpConfigs[i].customName);
      Serial.print(") - Mode: ");
      switch (wpConfigs[i].alarmMode) {
        case 1: Serial.println("Alarm when NO water"); break;
        case 2: Serial.println("Alarm when water PRESENT"); break;
        case 3: Serial.println("No alarm"); break;
      }
    } else {
      Serial.println("Not configured");
    }
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
        configureWPSensor();
        break;
      case 3:
        configureGeneralSettings();
        break;
      case 4:
        configureWiFiTerminal();
        break;
      case 5:
        viewCurrentConfiguration();
        break;
      case 6:
        scanForSensors();
        break;
      case 7:
        Serial.println("Configuration saved and exiting...");
        return;
      case 8: {
        String confirm = getStringInput("Reset all configuration? This cannot be undone! (Y/N): ", 1);
        if (confirm.equalsIgnoreCase("Y")) {
          resetConfiguration();
        }
        break;
      }
      default:
        Serial.println("Invalid option. Please select 1-8.");
        break;
    }
  }
}

// ===========================================
// SENSOR DATA PROCESSING
// ===========================================

// Reset sensor confirmation tracking (called when sensor goes offline or at startup)
void resetSensorConfirmation(int tankIndex) {
  if (tankIndex < 0 || tankIndex >= MAX_TANKS) return;

  tankConfirm[tankIndex].confirmCount = 0;
  tankConfirm[tankIndex].lastConfirmMsgId = "";
  tankConfirm[tankIndex].lastConfirmTime = 0;
  tankConfirm[tankIndex].confirmationReady = false;
  tankConfirm[tankIndex].confirmedPercent = 0;
  tankConfirm[tankIndex].confirmedLevel = 0;
  tankConfirm[tankIndex].clearConfirmCountLow = 0;
  tankConfirm[tankIndex].clearConfirmCountHigh = 0;
  tankConfirm[tankIndex].lowClearConfirmed = false;
  tankConfirm[tankIndex].highClearConfirmed = false;
  needFreshData[tankIndex] = true;
}

// Process sensor confirmation (requires multiple consecutive readings before acting)
void processConfirmation(int tankIndex, String msgId, float reading) {
  if (tankIndex < 0 || tankIndex >= MAX_TANKS) return;

  // Check confirmation timeout (60s without new readings)
  if (tankConfirm[tankIndex].confirmCount > 0 &&
      (millis() - tankConfirm[tankIndex].lastConfirmTime) > 60000) {
    Serial.print("Tank ");
    Serial.print(tankIndex + 1);
    Serial.print(" (");
    Serial.print(tankConfigs[tankIndex].customName);
    Serial.println(") confirmation timeout - resetting counter");
    resetSensorConfirmation(tankIndex);
  }

  // Check for duplicate msg_id (prevent counting same packet twice)
  if (msgId == tankConfirm[tankIndex].lastConfirmMsgId && msgId.length() > 0) {
    return; // Duplicate packet, ignore
  }

  // New unique message - increment confirmation counter
  tankConfirm[tankIndex].confirmCount++;
  tankConfirm[tankIndex].lastConfirmMsgId = msgId;
  tankConfirm[tankIndex].lastConfirmTime = millis();

  Serial.print("Tank ");
  Serial.print(tankIndex + 1);
  Serial.print(" (");
  Serial.print(tankConfigs[tankIndex].customName);
  Serial.print(") confirmation: ");
  Serial.print(tankConfirm[tankIndex].confirmCount);
  Serial.print("/");
  Serial.println(confirmationsRequired);

  // Process reading temporarily (updates tankData for averaging/smoothing)
  processWaterLevelData(tankIndex, reading);

  // Check if confirmation requirement met
  if (tankConfirm[tankIndex].confirmCount >= confirmationsRequired) {
    tankConfirm[tankIndex].confirmationReady = true;
    tankConfirm[tankIndex].confirmedPercent = tankData[tankIndex].currentPercent;
    tankConfirm[tankIndex].confirmedLevel = tankData[tankIndex].currentLevel;

    Serial.print("Tank ");
    Serial.print(tankIndex + 1);
    Serial.print(" (");
    Serial.print(tankConfigs[tankIndex].customName);
    Serial.print(") CONFIRMED (");
    Serial.print(confirmationsRequired);
    Serial.println(" readings verified) - updating display/alarms");
  }

  // Process clear confirmations for low alarm (if alarm was triggered and now level is above threshold)
  if (tankConfigs[tankIndex].lowAlarmPercent > 0 && alarmAck.tankLowThresholdAck[tankIndex]) {
    if (tankData[tankIndex].currentPercent >= tankConfigs[tankIndex].lowAlarmPercent) {
      tankConfirm[tankIndex].clearConfirmCountLow++;
      if (tankConfirm[tankIndex].clearConfirmCountLow >= confirmationsRequired) {
        tankConfirm[tankIndex].lowClearConfirmed = true;
        Serial.print("Tank ");
        Serial.print(tankIndex);
        Serial.println(" LOW alarm clear CONFIRMED - ready to reset");
      }
    } else {
      // Level dropped back below threshold - reset clear counter
      if (tankConfirm[tankIndex].clearConfirmCountLow > 0) {
        tankConfirm[tankIndex].clearConfirmCountLow = 0;
        tankConfirm[tankIndex].lowClearConfirmed = false;
        Serial.print("Tank ");
        Serial.print(tankIndex);
        Serial.println(" LOW alarm clear reset (level dropped again)");
      }
    }
  }

  // Process clear confirmations for high alarm (if alarm was triggered and now level is below threshold)
  if (tankConfigs[tankIndex].highAlarmPercent > 0 && alarmAck.tankHighThresholdAck[tankIndex]) {
    if (tankData[tankIndex].currentPercent <= tankConfigs[tankIndex].highAlarmPercent) {
      tankConfirm[tankIndex].clearConfirmCountHigh++;
      if (tankConfirm[tankIndex].clearConfirmCountHigh >= confirmationsRequired) {
        tankConfirm[tankIndex].highClearConfirmed = true;
        Serial.print("Tank ");
        Serial.print(tankIndex);
        Serial.println(" HIGH alarm clear CONFIRMED - ready to reset");
      }
    } else {
      // Level rose back above threshold - reset clear counter
      if (tankConfirm[tankIndex].clearConfirmCountHigh > 0) {
        tankConfirm[tankIndex].clearConfirmCountHigh = 0;
        tankConfirm[tankIndex].highClearConfirmed = false;
        Serial.print("Tank ");
        Serial.print(tankIndex);
        Serial.println(" HIGH alarm clear reset (level rose again)");
      }
    }
  }
}

// CALIBRATION FUNCTION - CURRENTLY NOT USED
// Kept for reference - calibration disabled to match receiver readings exactly
// To re-enable: uncomment line in processWaterLevelData() that calls applyCalibration()
float applyCalibration(float rawReading) {
  if (rawReading > 21 && rawReading < 400) {
    return -0.456079 + (1.030190 * rawReading);
  } else {
    return rawReading;
  }
}

void processWaterLevelData(int tankIndex, float ultrasonicReading) {
  if (tankIndex < 0 || tankIndex >= MAX_TANKS || !tankConfigs[tankIndex].enabled) {
    return;
  }

  // No calibration - use raw reading directly (matches receiver behavior)
  if (ultrasonicReading > tankConfigs[tankIndex].height) {
    ultrasonicReading = tankConfigs[tankIndex].height;
  } else if (ultrasonicReading < SENSOR_BLIND_ZONE_CM) {
    ultrasonicReading = SENSOR_BLIND_ZONE_CM;
  }

  float usableTankDepth = tankConfigs[tankIndex].height - SENSOR_BLIND_ZONE_CM;
  float waterLevel = tankConfigs[tankIndex].height - ultrasonicReading;
  float waterPercent = (waterLevel / usableTankDepth) * 100.0;
  
  if (waterPercent < 0) waterPercent = 0;
  if (waterPercent > 100) waterPercent = 100;
  
  tankData[tankIndex].currentLevel = waterLevel;
  tankData[tankIndex].currentPercent = waterPercent;
  tankData[tankIndex].lastUpdate = millis();
  tankData[tankIndex].isOnline = true;
  tankData[tankIndex].hasData = true;
  
  if (waterLevel < tankData[tankIndex].minLevel) {
    tankData[tankIndex].minLevel = waterLevel;
  }
  if (waterLevel > tankData[tankIndex].maxLevel) {
    tankData[tankIndex].maxLevel = waterLevel;
  }
  if (waterPercent < tankData[tankIndex].minPercent) {
    tankData[tankIndex].minPercent = waterPercent;
  }
  if (waterPercent > tankData[tankIndex].maxPercent) {
    tankData[tankIndex].maxPercent = waterPercent;
  }
}

String getSensorID(String nodeName) {
  int dashIndex = nodeName.indexOf('-');
  if (dashIndex != -1) {
    return nodeName.substring(dashIndex + 1);
  } else {
    return nodeName;
  }
}

// Get RSSI signal strength label for scan results
String getRSSILabel(int rssi) {
  if (rssi >= -60) return "Excellent";
  if (rssi >= -99) return "Good";
  return "Weak";
}

// Add device to scan results if scanning is active
void addToScanResults(const char* nodeId, int rssi) {
  if (!scanActive) return;

  // Auto-stop scan after duration
  if (millis() - scanStartTime >= SCAN_DURATION_MS) {
    scanActive = false;
    return;
  }

  // Determine type based on sensor ID
  bool isUS = strstr(nodeId, "US") != NULL;
  bool isWP = strstr(nodeId, "WP") != NULL;
  if (!isUS && !isWP) return;

  ScanEntry* arr = isUS ? scannedUS : scannedWP;
  uint8_t* cnt = isUS ? &scanUSCount : &scanWPCount;

  // Check if already exists (update RSSI if better signal)
  for (int i = 0; i < *cnt; i++) {
    if (strcmp(arr[i].id, nodeId) == 0) {
      if (rssi > arr[i].rssi) arr[i].rssi = (int8_t)rssi;
      return;
    }
  }

  // Add new entry if space available
  if (*cnt < 5) {
    strncpy(arr[*cnt].id, nodeId, 11);
    arr[*cnt].id[11] = '\0';
    arr[*cnt].rssi = (int8_t)rssi;
    (*cnt)++;
  }
}

// Check if packet is duplicate based on session ID and counter
// Returns true if duplicate (should be rejected), false if new packet
bool isDuplicatePacket(String msgId, int sensorIndex, bool isWP) {
  // Parse msg_id format: "US03_1234567_5" or "WP01_1234567_5"
  int firstUnderscore = msgId.indexOf('_');
  if (firstUnderscore == -1) {
    // Old format without session/counter - allow through (backward compatible)
    return false;
  }

  int secondUnderscore = msgId.indexOf('_', firstUnderscore + 1);
  if (secondUnderscore == -1) {
    // Invalid format - allow through
    return false;
  }

  // Extract session ID and counter
  uint32_t sessionId = msgId.substring(firstUnderscore + 1, secondUnderscore).toInt();
  uint32_t counter = msgId.substring(secondUnderscore + 1).toInt();

  // Get references to correct tracking arrays based on sensor type
  uint32_t& lastSession = isWP ? lastWP_SessionID[sensorIndex] : lastUS_SessionID[sensorIndex];
  uint32_t& lastCounter = isWP ? lastWP_Counter[sensorIndex] : lastUS_Counter[sensorIndex];

  // Check for duplicate: same session AND counter <= last seen counter
  if (sessionId == lastSession && counter <= lastCounter) {
    return true;  // DUPLICATE - reject this packet
  }

  // New packet - update tracking
  lastSession = sessionId;
  lastCounter = counter;
  return false;  // Not duplicate - process this packet
}

void processPacket(String jsonData, int rssi) {
  // Update global packet time for LoRa health check
  lastPacketTime = millis();

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, jsonData);

  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (!doc.containsKey("node_id") || !doc.containsKey("msg_id") ||
      !doc.containsKey("data")) {
    return;
  }

  String nodeId = doc["node_id"].as<String>();
  String msgId = doc["msg_id"].as<String>();
  String data = doc["data"].as<String>();

  String extractedSensorId = getSensorID(nodeId);

  // If scanning, add to scan results (before filtering by configured sensors)
  if (scanActive) {
    addToScanResults(extractedSensorId.c_str(), rssi);
  }

  // Check if this is a configured tank sensor
  for (int i = 0; i < MAX_TANKS; i++) {
    if (tankConfigs[i].enabled && String(tankConfigs[i].nodeID) == extractedSensorId) {
      // Check for duplicate packet from relay
      if (isDuplicatePacket(msgId, i, false)) {
        Serial.print("Duplicate US packet for ");
        Serial.print(tankConfigs[i].customName);
        Serial.println(" - ignored");
        return;
      }

      float ultrasonicReading = data.toFloat();

      // Handle fresh data requirement
      if (needFreshData[i]) {
        needFreshData[i] = false;
        Serial.print("Fresh data received for tank ");
        Serial.print(tankConfigs[i].customName);
        Serial.println(" after recovery");
      }

      // Update RSSI
      tankData[i].rssi = rssi;

      // Process confirmation (requires multiple consecutive readings)
      processConfirmation(i, msgId, ultrasonicReading);

      Serial.print("Tank ");
      Serial.print(i + 1);
      Serial.print(" (");
      Serial.print(tankConfigs[i].customName);
      Serial.print("): ");
      Serial.print(ultrasonicReading, 1);
      Serial.print("cm | RSSI: ");
      Serial.println(rssi);
      return;
    }
  }
  
  // Check if this is a configured WP sensor
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    if (wpConfigs[i].enabled && String(wpConfigs[i].nodeID) == extractedSensorId) {
      // Check for duplicate packet from relay
      if (isDuplicatePacket(msgId, i, true)) {
        Serial.print("Duplicate WP packet for ");
        Serial.print(wpConfigs[i].customName);
        Serial.println(" - ignored");
        return;
      }

      wpData[i].waterPresent = (data == "1");
      wpData[i].lastUpdate = millis();
      wpData[i].isOnline = true;
      wpData[i].hasData = true;
      wpData[i].rssi = rssi;

      Serial.print("WP ");
      Serial.print(i + 1);
      Serial.print(" (");
      Serial.print(wpConfigs[i].customName);
      Serial.print("): ");
      Serial.print(wpData[i].waterPresent ? "Water Present" : "No Water");
      Serial.print(" | RSSI: ");
      Serial.println(rssi);
      return;
    }
  }
}

// ===========================================
// ALARM SYSTEM
// ===========================================

bool checkAlarmConditions() {
  bool alarmTriggered = false;
  bool hasUnacknowledgedAlarm = false;
  
  for (int i = 0; i < MAX_TANKS; i++) {
    if (!tankConfigs[i].enabled || !tankData[i].hasData) continue;
    
    unsigned long offlineThreshold = generalSettings.offlineTimeoutMinutes * 60000UL;
    if ((millis() - tankData[i].lastUpdate) > offlineThreshold) {
      tankData[i].isOnline = false;
      
      if (!alarmAck.tankOfflineAck[i]) {
        alarmTriggered = true;
        hasUnacknowledgedAlarm = true;
      }
      continue;
    } else {
      if (!tankData[i].isOnline && alarmAck.tankOfflineAck[i]) {
        alarmAck.tankOfflineAck[i] = false;
        saveAlarmAckData();
        Serial.print("Tank ");
        Serial.print(tankConfigs[i].customName);
        Serial.println(" came back online - offline acknowledgment cleared");
      }
      tankData[i].isOnline = true;
    }
    
    if (!tankData[i].isOnline) continue;

    // Only trigger alarms on CONFIRMED readings (prevents false alarms)
    if (tankConfirm[i].confirmationReady) {
      if (tankConfigs[i].lowAlarmPercent > 0 &&
          tankConfirm[i].confirmedPercent < tankConfigs[i].lowAlarmPercent) {
        alarmTriggered = true;

        if (!alarmAck.tankLowThresholdAck[i]) {
          hasUnacknowledgedAlarm = true;
        }
      } else {
        // Only reset acknowledgment if clear condition is CONFIRMED (prevents false resets from brief level changes)
        if (alarmAck.tankLowThresholdAck[i] && tankConfirm[i].lowClearConfirmed) {
          alarmAck.tankLowThresholdAck[i] = false;
          saveAlarmAckData();
          tankConfirm[i].clearConfirmCountLow = 0;
          tankConfirm[i].lowClearConfirmed = false;
          Serial.print("Tank ");
          Serial.print(tankConfigs[i].customName);
          Serial.println(" level cleared low threshold - acknowledgment reset (CONFIRMED)");
        }
      }

      if (tankConfigs[i].highAlarmPercent > 0 &&
          tankConfirm[i].confirmedPercent > tankConfigs[i].highAlarmPercent) {
        alarmTriggered = true;

        if (!alarmAck.tankHighThresholdAck[i]) {
          hasUnacknowledgedAlarm = true;
        }
      } else {
        // Only reset acknowledgment if clear condition is CONFIRMED (prevents false resets from brief level changes)
        if (alarmAck.tankHighThresholdAck[i] && tankConfirm[i].highClearConfirmed) {
          alarmAck.tankHighThresholdAck[i] = false;
          saveAlarmAckData();
          tankConfirm[i].clearConfirmCountHigh = 0;
          tankConfirm[i].highClearConfirmed = false;
          Serial.print("Tank ");
          Serial.print(tankConfigs[i].customName);
          Serial.println(" level cleared high threshold - acknowledgment reset (CONFIRMED)");
        }
      }
    }
  }
  
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    if (!wpConfigs[i].enabled || !wpData[i].hasData || wpConfigs[i].alarmMode == 3) continue;
    
    unsigned long offlineThreshold = generalSettings.offlineTimeoutMinutes * 60000UL;
    if ((millis() - wpData[i].lastUpdate) > offlineThreshold) {
      wpData[i].isOnline = false;
      
      if (!alarmAck.wpOfflineAck[i]) {
        alarmTriggered = true;
        hasUnacknowledgedAlarm = true;
      }
      continue;
    } else {
      if (!wpData[i].isOnline && alarmAck.wpOfflineAck[i]) {
        alarmAck.wpOfflineAck[i] = false;
        saveAlarmAckData();
        Serial.print("WP ");
        Serial.print(wpConfigs[i].customName);
        Serial.println(" came back online - offline acknowledgment cleared");
      }
      wpData[i].isOnline = true;
    }
    
    if (!wpData[i].isOnline) continue;
    
    bool wpAlarmCondition = false;
    if (wpConfigs[i].alarmMode == 1 && !wpData[i].waterPresent) {
      wpAlarmCondition = true;
    } else if (wpConfigs[i].alarmMode == 2 && wpData[i].waterPresent) {
      wpAlarmCondition = true;
    }
    
    if (wpAlarmCondition) {
      alarmTriggered = true;
      
      if (!alarmAck.wpAlarmAck[i]) {
        hasUnacknowledgedAlarm = true;
      }
    } else {
      if (alarmAck.wpAlarmAck[i]) {
        alarmAck.wpAlarmAck[i] = false;
        saveAlarmAckData();
        Serial.print("WP ");
        Serial.print(wpConfigs[i].customName);
        Serial.println(" condition cleared - acknowledgment reset");
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
  
  for (int i = 0; i < MAX_TANKS; i++) {
    if (!tankConfigs[i].enabled || !tankData[i].hasData) continue;
    
    if (!tankData[i].isOnline && !alarmAck.tankOfflineAck[i]) {
      alarmAck.tankOfflineAck[i] = true;
      acknowledgedSomething = true;
      Serial.print("Acknowledged Tank ");
      Serial.print(tankConfigs[i].customName);
      Serial.println(" offline condition");
    }
    
    if (!tankData[i].isOnline) continue;
    
    if (tankConfigs[i].lowAlarmPercent > 0 && 
        tankData[i].currentPercent < tankConfigs[i].lowAlarmPercent &&
        !alarmAck.tankLowThresholdAck[i]) {
      alarmAck.tankLowThresholdAck[i] = true;
      acknowledgedSomething = true;
      Serial.print("Acknowledged Tank ");
      Serial.print(tankConfigs[i].customName);
      Serial.print(" low threshold condition (current: ");
      Serial.print(tankData[i].currentPercent, 1);
      Serial.println("%)");
    }
    
    if (tankConfigs[i].highAlarmPercent > 0 && 
        tankData[i].currentPercent > tankConfigs[i].highAlarmPercent &&
        !alarmAck.tankHighThresholdAck[i]) {
      alarmAck.tankHighThresholdAck[i] = true;
      acknowledgedSomething = true;
      Serial.print("Acknowledged Tank ");
      Serial.print(tankConfigs[i].customName);
      Serial.print(" high threshold condition (current: ");
      Serial.print(tankData[i].currentPercent, 1);
      Serial.println("%)");
    }
  }
  
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    if (!wpConfigs[i].enabled || wpConfigs[i].alarmMode == 3) continue;
    
    if (!wpData[i].isOnline && !alarmAck.wpOfflineAck[i]) {
      alarmAck.wpOfflineAck[i] = true;
      acknowledgedSomething = true;
      Serial.print("Acknowledged WP ");
      Serial.print(wpConfigs[i].customName);
      Serial.println(" offline condition");
    }
    
    if (!wpData[i].hasData || !wpData[i].isOnline) continue;
    
    bool wpAlarmCondition = false;
    if (wpConfigs[i].alarmMode == 1 && !wpData[i].waterPresent) {
      wpAlarmCondition = true;
    } else if (wpConfigs[i].alarmMode == 2 && wpData[i].waterPresent) {
      wpAlarmCondition = true;
    }
    
    if (wpAlarmCondition && !alarmAck.wpAlarmAck[i]) {
      alarmAck.wpAlarmAck[i] = true;
      acknowledgedSomething = true;
      Serial.print("Acknowledged WP ");
      Serial.print(wpConfigs[i].customName);
      Serial.print(" alarm condition: ");
      Serial.println(wpData[i].waterPresent ? "Water Present" : "No Water");
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
// BUTTON HANDLING (UPDATED)
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
                Serial.println("Double press detected - showing system info + sensor tech on OLED");
                screenMgr.showSystemInfo = true;
                screenMgr.systemInfoTimeout = millis() + 5000; // Show system info for 5 seconds
                screenMgr.showSensorTech = false; // Reset sensor tech display
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
  
  // UPDATED: New long press behavior for manual screen control
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
  Serial.println("\n=== INDIVIDUAL SENSOR READINGS ===");
  Serial.print("Confirmation System: ");
  Serial.print(confirmationsRequired);
  Serial.println(" readings required");

  Serial.println("\nTank Sensors:");
  for (int i = 0; i < MAX_TANKS; i++) {
    if (tankConfigs[i].enabled) {
      Serial.print(tankConfigs[i].customName);
      Serial.print(" (");
      Serial.print(tankConfigs[i].nodeID);
      Serial.print("): ");

      if (tankData[i].hasData) {
        // Show confirmation status
        Serial.print("[");
        if (tankConfirm[i].confirmationReady) {
          Serial.print("CONFIRMED: ");
          Serial.print(tankConfirm[i].confirmedPercent, 1);
          Serial.print("%");
        } else {
          Serial.print("CONFIRMING ");
          Serial.print(tankConfirm[i].confirmCount);
          Serial.print("/");
          Serial.print(confirmationsRequired);
        }
        Serial.print("] | ");

        Serial.print(tankData[i].currentPercent, 1);
        Serial.print("% (");
        Serial.print(tankData[i].currentLevel, 1);
        Serial.print("cm) | ");
        Serial.print(tankData[i].isOnline ? "ONLINE" : "OFFLINE");
        Serial.print(" | RSSI: ");
        Serial.print(tankData[i].rssi);
        Serial.print(" | Min: ");
        Serial.print(tankData[i].minPercent, 1);
        Serial.print("% | Max: ");
        Serial.print(tankData[i].maxPercent, 1);
        Serial.println("%");
      } else {
        Serial.println("No data received");
      }
    }
  }
  
  Serial.println("\nWater Presence Sensors:");
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    if (wpConfigs[i].enabled) {
      Serial.print(wpConfigs[i].customName);
      Serial.print(" (");
      Serial.print(wpConfigs[i].nodeID);
      Serial.print("): ");
      
      if (wpData[i].hasData) {
        Serial.print(wpData[i].waterPresent ? "Water Present" : "No Water");
        Serial.print(" | ");
        Serial.print(wpData[i].isOnline ? "ONLINE" : "OFFLINE");
        Serial.print(" | RSSI: ");
        Serial.println(wpData[i].rssi);
      } else {
        Serial.println("No data received");
      }
    }
  }
  Serial.println("===================================\n");
}

// ===========================================
// NEW DISPLAY SYSTEM (AUTO-CYCLING) - FIXED STARTUP
// ===========================================

// Test I2C connection to OLED
bool testOLEDConnection() {
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  return (Wire.endTransmission() == 0);
}

// UPDATED: Fixed startup screen - gracefully handles missing OLED
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

// NEW: Screen management functions
void buildScreenList() {
  screenMgr.totalScreens = 0;

  // Count configured tanks
  for(int i = 0; i < MAX_TANKS; i++) {
    if(tankConfigs[i].enabled) screenMgr.totalScreens++;
  }

  // Count configured WP sensors
  for(int i = 0; i < MAX_WP_SENSORS; i++) {
    if(wpConfigs[i].enabled) screenMgr.totalScreens++;
  }

  // NOTE: Alarm screen is now LOCKED (not part of rotation)
  // When alarmActive is true, showCurrentScreen() shows alarm screen exclusively

  // Ensure we have at least one screen (show unconfigured screen if nothing else)
  if(screenMgr.totalScreens == 0) screenMgr.totalScreens = 1;
}

void handleScreenTiming() {
  // Handle system info timeout - transition to sensor tech details
  if(screenMgr.showSystemInfo && millis() > screenMgr.systemInfoTimeout) {
    screenMgr.showSystemInfo = false;
    screenMgr.showSensorTech = true;
    screenMgr.sensorTechTimeout = millis() + 5000; // Show sensor tech for 5 seconds
    Serial.println("System info timeout - showing sensor technical details");
  }

  // Handle sensor tech timeout - return to normal cycling
  if(screenMgr.showSensorTech && millis() > screenMgr.sensorTechTimeout) {
    screenMgr.showSensorTech = false;
    Serial.println("Sensor tech timeout - resuming normal cycling");
  }

  // Normal auto-cycling (only if not showing any special screens and not in manual mode)
  if(!screenMgr.manualMode && !screenMgr.showSystemInfo && !screenMgr.showSensorTech && millis() - screenMgr.lastUpdate > 4000) {
    screenMgr.currentScreen = (screenMgr.currentScreen + 1) % screenMgr.totalScreens;
    screenMgr.lastUpdate = millis();
  }

  // Manual mode timeout
  if(screenMgr.manualMode && millis() > screenMgr.manualTimeout) {
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
  if(screenMgr.showSystemInfo) {
    showSystemScreen();
    return;
  }

  // Show sensor technical details if requested via double press (second phase)
  if(screenMgr.showSensorTech) {
    showSensorTechScreen();
    return;
  }

  int screenIndex = 0;

  // Show tanks first
  for(int i = 0; i < MAX_TANKS; i++) {
    if(tankConfigs[i].enabled) {
      if(screenIndex == screenMgr.currentScreen) {
        showTankScreen(i);
        return;
      }
      screenIndex++;
    }
  }

  // Then WP sensors
  for(int i = 0; i < MAX_WP_SENSORS; i++) {
    if(wpConfigs[i].enabled) {
      if(screenIndex == screenMgr.currentScreen) {
        showWPScreen(i);
        return;
      }
      screenIndex++;
    }
  }

  // Fallback - cycle back to first screen if we've gone past all sensor screens
  screenMgr.currentScreen = 0;
}

// NEW: Updated display function - skips if OLED not available
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
  u8g2.drawStr(0, 45, "to setup sensors");
  u8g2.sendBuffer();
}

void showTankScreen(int tankIndex) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);

  // Tank name and alarm indicator
  String title = String(tankConfigs[tankIndex].customName);
  bool hasAlarm = false;

  if (tankData[tankIndex].hasData) {
    hasAlarm = (tankConfigs[tankIndex].lowAlarmPercent > 0 &&
                tankData[tankIndex].currentPercent < tankConfigs[tankIndex].lowAlarmPercent) ||
               (tankConfigs[tankIndex].highAlarmPercent > 0 &&
                tankData[tankIndex].currentPercent > tankConfigs[tankIndex].highAlarmPercent) ||
               !tankData[tankIndex].isOnline;
  } else if (!tankData[tankIndex].isOnline) {
    hasAlarm = true;
  }

  // Add mute icon if global mute is enabled and alarm is present
  if(hasAlarm && globalBuzzerMute) {
    title += " MUTE";
  } else if(hasAlarm) {
    title += " ALARM";
  }

  u8g2.drawStr(0, 12, title.c_str());

  // Wider vertical tank (doubled width: 24px instead of 12px)
  u8g2.drawFrame(5, 18, 24, 30);
  if(tankData[tankIndex].hasData && tankData[tankIndex].isOnline) {
    // Fill tank graphic with LIVE percentage (immediate visual feedback)
    int fillHeight = (28 * tankData[tankIndex].currentPercent) / 100;
    if(fillHeight > 0) {
      u8g2.drawBox(6, 47-fillHeight, 22, fillHeight); // Wider fill area
    }
  }

  if(tankData[tankIndex].hasData && tankData[tankIndex].isOnline) {
    // Show LIVE percentage (immediate updates from every packet)
    // Alarms still use confirmed values in background
    String percentStr = String((int)round(tankData[tankIndex].currentPercent)) + "%";
    u8g2.setFont(u8g2_font_10x20_tr); // Large font for percentage
    u8g2.drawStr(35, 32, percentStr.c_str()); // Moved right and adjusted Y position

    // Volume in smaller font below percentage - changed to "Litres"
    if(tankConfigs[tankIndex].volume > 0) {
      float litres = (tankData[tankIndex].currentPercent / 100.0) * tankConfigs[tankIndex].volume;
      String volumeStr = String((int)litres) + " Litres";
      u8g2.setFont(u8g2_font_6x10_tr);
      u8g2.drawStr(35, 45, volumeStr.c_str()); // Moved right and adjusted Y position
    }

    // Min/Max alarm percentages in very small font at bottom
    if (tankConfigs[tankIndex].lowAlarmPercent > 0 || tankConfigs[tankIndex].highAlarmPercent > 0) {
      String alarmStr = "Low: " + String(tankConfigs[tankIndex].lowAlarmPercent) + "% High: " + String(tankConfigs[tankIndex].highAlarmPercent) + "%";
      u8g2.setFont(u8g2_font_5x7_tr); // Very small font
      u8g2.drawStr(0, 63, alarmStr.c_str());
    }
  } else {
    // Offline/No Data status - moved to match new layout
    u8g2.setFont(u8g2_font_6x10_tr);
    String status = tankData[tankIndex].hasData ? "OFFLINE" : "NO DATA";
    u8g2.drawStr(35, 35, status.c_str()); // Moved right to match percentage position
  }

  u8g2.sendBuffer();
}

void showWPScreen(int wpIndex) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);

  // WP name and alarm indicator
  String title = String(wpConfigs[wpIndex].customName);
  bool hasAlarm = false;

  if(wpData[wpIndex].hasData && wpData[wpIndex].isOnline) {
    hasAlarm = (wpConfigs[wpIndex].alarmMode == 1 && !wpData[wpIndex].waterPresent) ||
               (wpConfigs[wpIndex].alarmMode == 2 && wpData[wpIndex].waterPresent);
  } else if(!wpData[wpIndex].isOnline && wpData[wpIndex].hasData) {
    hasAlarm = true;
  }

  // Add mute indicator if global mute is enabled and alarm is present
  if(hasAlarm && globalBuzzerMute) {
    title += " MUTE";
  } else if(hasAlarm) {
    title += " ALARM";
  }

  u8g2.drawStr(0, 12, title.c_str());

  // Water status in BIG font for easy reading from distance
  String status = "NO DATA";
  if(wpData[wpIndex].hasData) {
    if(wpData[wpIndex].isOnline) {
      status = wpData[wpIndex].waterPresent ? "Water Present" : "No Water";
    } else {
      status = "OFFLINE";
    }
  }

  u8g2.setFont(u8g2_font_10x20_tr); // Large font for water status
  u8g2.drawStr(5, 35, status.c_str());

  // Alarm mode explanation in smaller font
  String modeText = "";
  switch(wpConfigs[wpIndex].alarmMode) {
    case 0: modeText = "Alert when water absent"; break;
    case 1: modeText = "Alert when water present"; break;
    default: modeText = "Monitor mode"; break;
  }

  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 55, modeText.c_str());

  u8g2.sendBuffer();
}

void showAlarmScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(25, 12, "ALARM ACTIVE!");
  
  int yPos = 25;
  u8g2.setFont(u8g2_font_6x10_tr);
  
  // List alarm conditions (combined on one screen)
  // Display shows LIVE values, but alarms only trigger on CONFIRMED values
  for(int i = 0; i < MAX_TANKS; i++) {
    if(!tankConfigs[i].enabled || !tankData[i].hasData) continue;

    String alarmText = "";
    if(!tankData[i].isOnline) {
      alarmText = String(tankConfigs[i].customName) + ": OFFLINE";
    } else {
      // Show LIVE percentage on alarm screen (matches main display)
      if(tankConfigs[i].lowAlarmPercent > 0 && tankData[i].currentPercent < tankConfigs[i].lowAlarmPercent) {
        alarmText = String(tankConfigs[i].customName) + ": LOW (" + String((int)round(tankData[i].currentPercent)) + "%)";
      } else if(tankConfigs[i].highAlarmPercent > 0 && tankData[i].currentPercent > tankConfigs[i].highAlarmPercent) {
        alarmText = String(tankConfigs[i].customName) + ": HIGH (" + String((int)round(tankData[i].currentPercent)) + "%)";
      }
    }

    if(alarmText.length() > 0 && yPos < 50) {
      u8g2.drawStr(0, yPos, alarmText.c_str());
      yPos += 10;
    }
  }
  
  // WP alarms
  for(int i = 0; i < MAX_WP_SENSORS; i++) {
    if(!wpConfigs[i].enabled || !wpData[i].hasData || wpConfigs[i].alarmMode == 3) continue;
    
    String alarmText = "";
    if(!wpData[i].isOnline) {
      alarmText = String(wpConfigs[i].customName) + ": OFFLINE";
    } else {
      bool alarmCondition = (wpConfigs[i].alarmMode == 1 && !wpData[i].waterPresent) ||
                           (wpConfigs[i].alarmMode == 2 && wpData[i].waterPresent);
      if(alarmCondition) {
        alarmText = String(wpConfigs[i].customName) + ": " + (wpData[i].waterPresent ? "WATER" : "NO WATER");
      }
    }
    
    if(alarmText.length() > 0 && yPos < 50) {
      u8g2.drawStr(0, yPos, alarmText.c_str());
      yPos += 10;
    }
  }
  
  // Instructions
  u8g2.drawStr(0, yPos + 5, "Press button to ACK");
  
  // Alarm duration
  if(alarmActive) {
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
  if(wifiStationMode) {
    String wifiStr = "WiFi: " + wifiSSID;
    if(wifiStr.length() > 20) wifiStr = wifiStr.substring(0, 20);
    u8g2.drawStr(0, 25, wifiStr.c_str());
    u8g2.drawStr(0, 35, ("IP: " + WiFi.localIP().toString()).c_str());
    u8g2.drawStr(0, 45, "OTA: Ready");
  } else {
    u8g2.drawStr(0, 25, "WiFi: AP Mode Only");
    u8g2.drawStr(0, 35, "OTA: Unavailable");
  }

  u8g2.drawStr(0, 55, ("AP: " + WiFi.softAPIP().toString()).c_str());
  
  // Sensor count
  int tanks = 0, wps = 0;
  for(int i = 0; i < MAX_TANKS; i++) if(tankConfigs[i].enabled) tanks++;
  for(int i = 0; i < MAX_WP_SENSORS; i++) if(wpConfigs[i].enabled) wps++;
  
  String monitorStr = "Monitoring: " + String(tanks) + "T + " + String(wps) + "WP";
  u8g2.drawStr(0, 64, monitorStr.c_str());
  
  u8g2.sendBuffer();
}

void showSensorTechScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 12, "Sensor Tech Info");

  u8g2.setFont(u8g2_font_6x10_tr);

  int yPos = 25;

  // Show tank sensor technical details
  for(int i = 0; i < MAX_TANKS; i++) {
    if(tankConfigs[i].enabled && yPos < 60) {
      String tankTech = String(tankConfigs[i].customName) + " (" + String(tankConfigs[i].nodeID) + ")";
      u8g2.drawStr(0, yPos, tankTech.c_str());
      yPos += 10;

      if(tankData[i].hasData && yPos < 60) {
        String signalInfo = "Signal: " + String(tankData[i].rssi) + " dBm";
        u8g2.drawStr(10, yPos, signalInfo.c_str());
        yPos += 10;
      }
    }
  }

  // Show WP sensor technical details
  for(int i = 0; i < MAX_WP_SENSORS; i++) {
    if(wpConfigs[i].enabled && yPos < 60) {
      String wpTech = String(wpConfigs[i].customName) + " (" + String(wpConfigs[i].nodeID) + ")";
      u8g2.drawStr(0, yPos, wpTech.c_str());
      yPos += 10;

      if(wpData[i].hasData && yPos < 60) {
        String signalInfo = "Signal: " + String(wpData[i].rssi) + " dBm";
        u8g2.drawStr(10, yPos, signalInfo.c_str());
        yPos += 10;
      }
    }
  }

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
    } else if (command.startsWith("CONFIRM:")) {
      // CONFIRM:5 sets confirmationsRequired to 5
      String valueStr = command.substring(8);
      int value = valueStr.toInt();
      if (value >= MIN_CONFIRMATIONS && value <= MAX_CONFIRMATIONS) {
        confirmationsRequired = value;
        Serial.print("Confirmations required set to: ");
        Serial.println(confirmationsRequired);
        Serial.println("Note: This setting is not saved to EEPROM and will reset on reboot");
      } else {
        Serial.print("ERROR: Invalid value. Must be between ");
        Serial.print(MIN_CONFIRMATIONS);
        Serial.print(" and ");
        Serial.println(MAX_CONFIRMATIONS);
      }
    } else if (command == "CONFIRM") {
      Serial.print("Current confirmations required: ");
      Serial.println(confirmationsRequired);
      Serial.print("Valid range: ");
      Serial.print(MIN_CONFIRMATIONS);
      Serial.print(" to ");
      Serial.println(MAX_CONFIRMATIONS);
      Serial.println("Usage: CONFIRM:5 (sets to 5 confirmations)");
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
      Serial.println("Available commands: CONFIG, STATUS, ALARM, WIFI, RESET, RESTART, CONFIRM, MUTE, UNMUTE, MUTESTATUS");
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
  if(screenMgr.manualMode) {
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
// WEB SERVER FUNCTIONS - FIXED
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
  server.on("/api/tank", HTTP_POST, handleAddTank);
  server.on("/api/tank", HTTP_PUT, handleUpdateTank);
  server.on("/api/tank", HTTP_DELETE, handleDeleteTank);
  server.on("/api/wp", HTTP_POST, handleAddWP);
  server.on("/api/wp", HTTP_PUT, handleUpdateWP);
  server.on("/api/wp", HTTP_DELETE, handleDeleteWP);
  server.on("/alarm/acknowledge", HTTP_POST, handleAlarmAcknowledge);
  server.on("/api/status", handleAPIStatus);
  server.on("/api/device-name", HTTP_POST, handleDeviceNameUpdate);
  server.on("/api/mute/toggle", HTTP_POST, handleMuteToggle);
  server.on("/api/mute/status", HTTP_GET, handleMuteStatus);
  server.on("/api/scan", HTTP_GET, handleAPIScan);

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
  server.send(403, "text/html", "<!DOCTYPE html><html><body style='font-family:Arial;text-align:center;padding:50px'><h2>Access Restricted</h2><p>This device is accessible only through the <strong>SenseFlow</strong> app.</p><p>Download the app to monitor your sensors.</p></body></html>");
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

// ===========================================
// SCAN API HANDLER
// ===========================================

void handleAPIScan() {
  String action = server.hasArg("action") ? server.arg("action") : "status";
  String type = server.hasArg("type") ? server.arg("type") : "US";

  if (action == "start") {
    // Clear previous results
    scanUSCount = 0;
    scanWPCount = 0;
    for (int i = 0; i < 5; i++) {
      scannedUS[i].id[0] = '\0';
      scannedUS[i].rssi = -127;
      scannedWP[i].id[0] = '\0';
      scannedWP[i].rssi = -127;
    }
    scanActive = true;
    scanStartTime = millis();
    server.send(200, "application/json", "{\"status\":\"scanning\"}");
    return;
  }

  if (action == "stop") {
    scanActive = false;
    server.send(200, "application/json", "{\"status\":\"stopped\"}");
    return;
  }

  // Default: return results
  // Auto-stop scan after duration
  if (scanActive && (millis() - scanStartTime >= SCAN_DURATION_MS)) {
    scanActive = false;
  }

  String json = "{\"scanning\":";
  json += scanActive ? "true" : "false";
  json += ",\"remaining\":";
  json += scanActive ? String((SCAN_DURATION_MS - (millis() - scanStartTime)) / 1000) : "0";
  json += ",\"devices\":[";

  uint8_t count = (type == "WP") ? scanWPCount : scanUSCount;
  ScanEntry* arr = (type == "WP") ? scannedWP : scannedUS;

  for (int i = 0; i < count; i++) {
    if (i > 0) json += ",";
    json += "{\"id\":\"";
    json += arr[i].id;
    json += "\",\"rssi\":";
    json += String((int)arr[i].rssi);
    json += ",\"signal\":\"" + getRSSILabel(arr[i].rssi) + "\"}";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

// JSON escaping function removed - now using ArduinoJson library

void handleAPIStatus() {
  if (!isAuthorizedApp()) { server.send(403, "application/json", "{\"error\":\"Unauthorized\"}"); return; }
  Serial.println("API Status requested");

  // Use ArduinoJson library for proper JSON generation
  DynamicJsonDocument doc(4096);

  doc["configured"] = isConfigured;
  doc["alarm_active"] = alarmActive;
  doc["all_acknowledged"] = allAlarmsAcknowledged;
  doc["wifi_station"] = wifiStationMode;

  // Add detailed alarm information
  JsonArray activeAlarms = doc.createNestedArray("active_alarms");
  if (alarmActive) {
    for (int i = 0; i < MAX_TANKS; i++) {
      if (tankConfigs[i].enabled && tankData[i].isOnline) {
        // Tank low threshold alarm - add safety checks
        if (!isnan(tankData[i].currentPercent) && tankData[i].currentPercent >= 0 && tankData[i].currentPercent <= 100) {
          if (tankData[i].currentPercent <= tankConfigs[i].lowAlarmPercent && !alarmAck.tankLowThresholdAck[i]) {
            JsonObject alarm = activeAlarms.createNestedObject();
            tankConfigs[i].customName[8] = '\0'; // Ensure null termination
            alarm["type"] = "Tank Low Level";
            alarm["sensor"] = String(tankConfigs[i].customName);
            alarm["message"] = "Tank level is " + String((int)round(tankData[i].currentPercent)) + "% (below " + String(tankConfigs[i].lowAlarmPercent) + "% threshold)";
          }
          // Tank high threshold alarm - add safety checks
          if (tankData[i].currentPercent >= tankConfigs[i].highAlarmPercent && !alarmAck.tankHighThresholdAck[i]) {
            JsonObject alarm = activeAlarms.createNestedObject();
            tankConfigs[i].customName[8] = '\0'; // Ensure null termination
            alarm["type"] = "Tank High Level";
            alarm["sensor"] = String(tankConfigs[i].customName);
            alarm["message"] = "Tank level is " + String((int)round(tankData[i].currentPercent)) + "% (above " + String(tankConfigs[i].highAlarmPercent) + "% threshold)";
          }
        }
      }
      // Tank offline alarm
      if (tankConfigs[i].enabled && !tankData[i].isOnline && !alarmAck.tankOfflineAck[i]) {
        JsonObject alarm = activeAlarms.createNestedObject();
        tankConfigs[i].customName[8] = '\0'; // Ensure null termination
        alarm["type"] = "Tank Offline";
        alarm["sensor"] = String(tankConfigs[i].customName);
        alarm["message"] = "Tank sensor is offline (no signal received)";
      }
    }

    for (int i = 0; i < MAX_WP_SENSORS; i++) {
      if (wpConfigs[i].enabled && wpData[i].isOnline) {
        // Water presence alarm
        bool alarmCondition = (wpConfigs[i].alarmMode == 1) ? wpData[i].waterPresent : !wpData[i].waterPresent;
        if (alarmCondition && !alarmAck.wpAlarmAck[i]) {
          JsonObject alarm = activeAlarms.createNestedObject();
          wpConfigs[i].customName[8] = '\0'; // Ensure null termination
          alarm["type"] = "Water Presence Alert";
          alarm["sensor"] = String(wpConfigs[i].customName);
          if (wpConfigs[i].alarmMode == 1) {
            // Alarm when water IS present - currently detected
            alarm["message"] = "Water detected (configured to alarm when water is present)";
          } else {
            // Alarm when water is NOT present - currently not detected
            alarm["message"] = "Water not detected (configured to alarm when water is absent)";
          }
        }
      }
      // WP sensor offline alarm
      if (wpConfigs[i].enabled && !wpData[i].isOnline && !alarmAck.wpOfflineAck[i]) {
        JsonObject alarm = activeAlarms.createNestedObject();
        wpConfigs[i].customName[8] = '\0'; // Ensure null termination
        alarm["type"] = "Water Presence Sensor Offline";
        alarm["sensor"] = String(wpConfigs[i].customName);
        alarm["message"] = "Water presence sensor is offline (no signal received)";
      }
    }
  }

  // Tank data
  JsonArray tanks = doc.createNestedArray("tanks");
  for (int i = 0; i < MAX_TANKS; i++) {
    JsonObject tank = tanks.createNestedObject();
    tank["enabled"] = tankConfigs[i].enabled;

    if (tankConfigs[i].enabled) {
      // Ensure null termination and copy to safe strings
      tankConfigs[i].customName[8] = '\0';
      tankConfigs[i].nodeID[8] = '\0';

      tank["name"] = String(tankConfigs[i].customName);
      tank["node"] = String(tankConfigs[i].nodeID);
      tank["online"] = tankData[i].isOnline;
      tank["percent"] = tankData[i].currentPercent;
      tank["level"] = tankData[i].currentLevel;
      tank["rssi"] = tankData[i].rssi;
      tank["lastUpdate"] = tankData[i].hasData ? millis() - tankData[i].lastUpdate : 0;
    }
  }

  // WP data
  JsonArray wpSensors = doc.createNestedArray("wp_sensors");
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    JsonObject wp = wpSensors.createNestedObject();
    wp["enabled"] = wpConfigs[i].enabled;

    if (wpConfigs[i].enabled) {
      // Ensure null termination and copy to safe strings
      wpConfigs[i].customName[8] = '\0';
      wpConfigs[i].nodeID[8] = '\0';

      wp["name"] = String(wpConfigs[i].customName);
      wp["node"] = String(wpConfigs[i].nodeID);
      wp["online"] = wpData[i].isOnline;
      wp["water_present"] = wpData[i].waterPresent;
      wp["rssi"] = wpData[i].rssi;
      wp["lastUpdate"] = wpData[i].hasData ? millis() - wpData[i].lastUpdate : 0;
    }
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

  if (json.length() > 300) {
    Serial.println("JSON content (first 300 chars):");
    Serial.println(json.substring(0, 300));
  } else {
    Serial.println("JSON content:");
    Serial.println(json);
  }

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

  DynamicJsonDocument doc(4096);

  // Tank sensors
  JsonArray tanks = doc.createNestedArray("tanks");
  for (int i = 0; i < MAX_TANKS; i++) {
    JsonObject tank = tanks.createNestedObject();
    tank["index"] = i;
    tank["enabled"] = tankConfigs[i].enabled;
    if (tankConfigs[i].enabled) {
      tankConfigs[i].customName[8] = '\0';
      tankConfigs[i].nodeID[8] = '\0';
      tank["nodeId"] = String(tankConfigs[i].nodeID);
      tank["name"] = String(tankConfigs[i].customName);
      tank["height"] = tankConfigs[i].height;
      tank["volume"] = tankConfigs[i].volume;
      tank["lowAlarm"] = tankConfigs[i].lowAlarmPercent;
      tank["highAlarm"] = tankConfigs[i].highAlarmPercent;
      tank["online"] = tankData[i].isOnline;
    }
  }

  // WP sensors
  JsonArray wpSensors = doc.createNestedArray("wpSensors");
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    JsonObject wp = wpSensors.createNestedObject();
    wp["index"] = i;
    wp["enabled"] = wpConfigs[i].enabled;
    if (wpConfigs[i].enabled) {
      wpConfigs[i].customName[8] = '\0';
      wpConfigs[i].nodeID[8] = '\0';
      wp["nodeId"] = String(wpConfigs[i].nodeID);
      wp["name"] = String(wpConfigs[i].customName);
      wp["alarmMode"] = wpConfigs[i].alarmMode;
      wp["online"] = wpData[i].isOnline;
    }
  }

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleAddTank() {
  if (!server.hasArg("nodeId") || !server.hasArg("name")) {
    server.send(400, "application/json", "{\"error\":\"Missing required fields\"}");
    return;
  }

  String nodeId = server.arg("nodeId");
  String name = server.arg("name");
  int height = server.arg("height").toInt();
  long volume = server.arg("volume").toInt();
  int lowAlarm = server.arg("lowAlarm").toInt();
  int highAlarm = server.arg("highAlarm").toInt();

  // Validation
  if (nodeId.length() == 0 || nodeId.length() > 8) {
    server.send(400, "application/json", "{\"error\":\"Invalid Node ID length\"}");
    return;
  }
  if (name.length() == 0 || name.length() > 8) {
    server.send(400, "application/json", "{\"error\":\"Invalid name length\"}");
    return;
  }

  // UPDATED VALIDATION: Allow all alarm combinations
  // 0,0 = No alarms (monitoring only)
  // 0,X = Only high alarm
  // X,0 = Only low alarm
  // X,Y = Both alarms (X must be < Y)
  if (lowAlarm > 0 && highAlarm > 0 && lowAlarm >= highAlarm) {
    server.send(400, "application/json", "{\"error\":\"Low alarm must be less than high alarm when both are enabled\"}");
    return;
  }
  if (lowAlarm < 0 || lowAlarm > 100 || highAlarm < 0 || highAlarm > 100) {
    server.send(400, "application/json", "{\"error\":\"Alarm percentages must be between 0-100\"}");
    return;
  }

  // Check for duplicates
  for (int i = 0; i < MAX_TANKS; i++) {
    if (tankConfigs[i].enabled) {
      if (strcmp(tankConfigs[i].nodeID, nodeId.c_str()) == 0) {
        server.send(400, "application/json", "{\"error\":\"Node ID already exists\"}");
        return;
      }
      if (strcmp(tankConfigs[i].customName, name.c_str()) == 0) {
        server.send(400, "application/json", "{\"error\":\"Name already exists\"}");
        return;
      }
    }
  }

  // Find empty slot
  int slot = -1;
  for (int i = 0; i < MAX_TANKS; i++) {
    if (!tankConfigs[i].enabled) {
      slot = i;
      break;
    }
  }

  if (slot == -1) {
    server.send(400, "application/json", "{\"error\":\"Maximum tank sensors reached\"}");
    return;
  }

  // Add tank
  strncpy(tankConfigs[slot].nodeID, nodeId.c_str(), 8);
  strncpy(tankConfigs[slot].customName, name.c_str(), 8);
  tankConfigs[slot].height = height;
  tankConfigs[slot].volume = volume;
  tankConfigs[slot].lowAlarmPercent = lowAlarm;
  tankConfigs[slot].highAlarmPercent = highAlarm;
  tankConfigs[slot].enabled = true;

  saveConfiguration();
  server.send(200, "application/json", "{\"success\":true}");
}

void handleUpdateTank() {
  if (!server.hasArg("index")) {
    server.send(400, "application/json", "{\"error\":\"Missing index\"}");
    return;
  }

  int index = server.arg("index").toInt();
  if (index < 0 || index >= MAX_TANKS) {
    server.send(400, "application/json", "{\"error\":\"Invalid index\"}");
    return;
  }

  bool enabled = server.hasArg("enabled") ? (server.arg("enabled") == "true") : tankConfigs[index].enabled;

  if (!enabled) {
    // Disable sensor
    tankConfigs[index].enabled = false;
    saveConfiguration();
    server.send(200, "application/json", "{\"success\":true}");
    return;
  }

  // Update sensor
  if (server.hasArg("nodeId")) {
    String nodeId = server.arg("nodeId");
    if (nodeId.length() > 0 && nodeId.length() <= 8) {
      strncpy(tankConfigs[index].nodeID, nodeId.c_str(), 8);
    }
  }
  if (server.hasArg("name")) {
    String name = server.arg("name");
    if (name.length() > 0 && name.length() <= 8) {
      strncpy(tankConfigs[index].customName, name.c_str(), 8);
    }
  }
  if (server.hasArg("height")) {
    tankConfigs[index].height = server.arg("height").toInt();
  }
  if (server.hasArg("volume")) {
    tankConfigs[index].volume = server.arg("volume").toInt();
  }
  if (server.hasArg("lowAlarm")) {
    tankConfigs[index].lowAlarmPercent = server.arg("lowAlarm").toInt();
  }
  if (server.hasArg("highAlarm")) {
    tankConfigs[index].highAlarmPercent = server.arg("highAlarm").toInt();
  }

  tankConfigs[index].enabled = true;
  saveConfiguration();
  server.send(200, "application/json", "{\"success\":true}");
}

void handleDeleteTank() {
  if (!server.hasArg("index")) {
    server.send(400, "application/json", "{\"error\":\"Missing index\"}");
    return;
  }

  int index = server.arg("index").toInt();
  if (index < 0 || index >= MAX_TANKS) {
    server.send(400, "application/json", "{\"error\":\"Invalid index\"}");
    return;
  }

  tankConfigs[index].enabled = false;
  memset(&tankConfigs[index], 0, sizeof(TankConfig));
  tankData[index].isOnline = false;
  tankData[index].hasData = false;
  saveConfiguration();
  server.send(200, "application/json", "{\"success\":true}");
}

void handleAddWP() {
  if (!server.hasArg("nodeId") || !server.hasArg("name")) {
    server.send(400, "application/json", "{\"error\":\"Missing required fields\"}");
    return;
  }

  String nodeId = server.arg("nodeId");
  String name = server.arg("name");
  int alarmMode = server.arg("alarmMode").toInt();

  // Validation
  if (nodeId.length() == 0 || nodeId.length() > 8) {
    server.send(400, "application/json", "{\"error\":\"Invalid Node ID length\"}");
    return;
  }
  if (name.length() == 0 || name.length() > 8) {
    server.send(400, "application/json", "{\"error\":\"Invalid name length\"}");
    return;
  }

  // Check for duplicates
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    if (wpConfigs[i].enabled) {
      if (strcmp(wpConfigs[i].nodeID, nodeId.c_str()) == 0) {
        server.send(400, "application/json", "{\"error\":\"Node ID already exists\"}");
        return;
      }
      if (strcmp(wpConfigs[i].customName, name.c_str()) == 0) {
        server.send(400, "application/json", "{\"error\":\"Name already exists\"}");
        return;
      }
    }
  }

  // Find empty slot
  int slot = -1;
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    if (!wpConfigs[i].enabled) {
      slot = i;
      break;
    }
  }

  if (slot == -1) {
    server.send(400, "application/json", "{\"error\":\"Maximum WP sensors reached\"}");
    return;
  }

  // Add WP sensor
  strncpy(wpConfigs[slot].nodeID, nodeId.c_str(), 8);
  strncpy(wpConfigs[slot].customName, name.c_str(), 8);
  wpConfigs[slot].alarmMode = alarmMode;
  wpConfigs[slot].enabled = true;

  saveConfiguration();
  server.send(200, "application/json", "{\"success\":true}");
}

void handleUpdateWP() {
  if (!server.hasArg("index")) {
    server.send(400, "application/json", "{\"error\":\"Missing index\"}");
    return;
  }

  int index = server.arg("index").toInt();
  if (index < 0 || index >= MAX_WP_SENSORS) {
    server.send(400, "application/json", "{\"error\":\"Invalid index\"}");
    return;
  }

  bool enabled = server.hasArg("enabled") ? (server.arg("enabled") == "true") : wpConfigs[index].enabled;

  if (!enabled) {
    // Disable sensor
    wpConfigs[index].enabled = false;
    saveConfiguration();
    server.send(200, "application/json", "{\"success\":true}");
    return;
  }

  // Update sensor
  if (server.hasArg("nodeId")) {
    String nodeId = server.arg("nodeId");
    if (nodeId.length() > 0 && nodeId.length() <= 8) {
      strncpy(wpConfigs[index].nodeID, nodeId.c_str(), 8);
    }
  }
  if (server.hasArg("name")) {
    String name = server.arg("name");
    if (name.length() > 0 && name.length() <= 8) {
      strncpy(wpConfigs[index].customName, name.c_str(), 8);
    }
  }
  if (server.hasArg("alarmMode")) {
    wpConfigs[index].alarmMode = server.arg("alarmMode").toInt();
  }

  wpConfigs[index].enabled = true;
  saveConfiguration();
  server.send(200, "application/json", "{\"success\":true}");
}

void handleDeleteWP() {
  if (!server.hasArg("index")) {
    server.send(400, "application/json", "{\"error\":\"Missing index\"}");
    return;
  }

  int index = server.arg("index").toInt();
  if (index < 0 || index >= MAX_WP_SENSORS) {
    server.send(400, "application/json", "{\"error\":\"Invalid index\"}");
    return;
  }

  wpConfigs[index].enabled = false;
  memset(&wpConfigs[index], 0, sizeof(WPConfig));
  wpData[index].isOnline = false;
  wpData[index].hasData = false;
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
  html += ".sig-excellent{color:#27ae60;font-weight:bold}";
  html += ".sig-good{color:#3498db;font-weight:bold}";
  html += ".sig-weak{color:#e74c3c;font-weight:bold}";
  html += ".ago{font-size:0.8em;color:#999}";
  html += ".gear{text-decoration:none;font-size:1.4em;color:#2980b9;float:right;cursor:pointer;font-weight:bold}";
  html += ".gear:hover{color:#3498db}";
  html += "</style>";
  html += "<script>";

  // Signal strength helper
  html += "function sigLabel(rssi){";
  html += "if(rssi>=-60)return '<span class=\"sig-excellent\">Excellent</span>';";
  html += "if(rssi>=-99)return '<span class=\"sig-good\">Good</span>';";
  html += "return '<span class=\"sig-weak\">Weak</span>';}";

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
  html += "html+='<h3>Tank Sensors:</h3>';";
  html += "let tankCount=0;";
  html += "for(let i=0;i<(data.tanks?data.tanks.length:0);i++){";
  html += "let tank=data.tanks[i];";
  html += "if(tank.enabled){";
  html += "tankCount++;";
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
  html += "html+='<a class=\"gear\" href=\"/config?open=tank-'+i+'\" title=\"Settings\">&#x2699;</a>';";
  html += "html+='<strong>'+(tank.name||'Unknown')+'</strong><br>';";
  html += "html+='Status: <span class=\"'+(tank.online?'online':'offline')+'\">'+(tank.online?'ONLINE':'OFFLINE')+'</span>';";
  html += "if(tank.lastUpdate){html+=' <span class=\"ago\">'+timeAgo(tank.lastUpdate)+'</span>';}";
  html += "html+='<br>';";
  html += "if(tank.online){";
  html += "html+='Level: <strong style=\"font-size:1.2em\">'+pct+'%</strong> ('+(tank.level!=null?tank.level.toFixed(1):'--')+'cm)<br>';";
  html += "html+='Signal: '+sigLabel(tank.rssi);}";
  html += "html+='</div></div></div>';}}";
  html += "if(tankCount===0){html+='<div class=\"status\">No tank sensors configured</div>';}";
  html += "html+='<h3>Water Presence:</h3>';";
  html += "let wpCount=0;";
  html += "for(let i=0;i<(data.wp_sensors?data.wp_sensors.length:0);i++){";
  html += "let wp=data.wp_sensors[i];";
  html += "if(wp.enabled){";
  html += "wpCount++;";
  html += "html+='<div class=\"status\">';";
  html += "html+='<a class=\"gear\" href=\"/config?open=wp-'+i+'\" title=\"Settings\">&#x2699;</a>';";
  html += "html+='<strong>'+(wp.name||'Unknown')+'</strong><br>';";
  html += "html+='Status: <span class=\"'+(wp.online?'online':'offline')+'\">'+(wp.online?'ONLINE':'OFFLINE')+'</span>';";
  html += "if(wp.lastUpdate){html+=' <span class=\"ago\">'+timeAgo(wp.lastUpdate)+'</span>';}";
  html += "html+='<br>';";
  html += "if(wp.online){";
  html += "html+='Water: <strong>'+(wp.water_present?'Present':'Not Present')+'</strong><br>';";
  html += "html+='Signal: '+sigLabel(wp.rssi);}";
  html += "html+='</div>';}}";
  html += "if(wpCount===0){html+='<div class=\"status\">No water presence sensors configured</div>';}";
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

  // Scan functionality
  html += ".btn-scan{background:#9b59b6;color:#fff;padding:12px 20px;border-radius:6px;width:100%;font-size:1em;font-weight:600;border:none;cursor:pointer}";
  html += ".btn-scan:hover{background:#8e44ad}";
  html += ".scan-status{font-size:0.85em;color:#666;margin-top:6px}";
  html += ".scan-results{margin-top:8px;max-height:200px;overflow-y:auto}";
  html += ".scan-item{background:#fff;border:1px solid #ddd;padding:12px;margin:4px 0;border-radius:6px;cursor:pointer;display:flex;justify-content:space-between;align-items:center;transition:all 0.2s}";
  html += ".scan-item:hover{background:#e8f4fc;border-color:#3498db}";
  html += ".scan-item.configured{opacity:0.5;cursor:default;background:#f0f0f0}";
  html += ".scan-item.configured:hover{background:#f0f0f0;border-color:#ddd}";
  html += ".scan-configured-tag{font-size:0.75em;color:#999;font-style:italic}";
  html += ".scan-item-id{font-weight:bold;color:#2c3e50}";
  html += ".scan-item-signal{font-size:0.85em}";
  html += ".sig-strong{color:#27ae60}";
  html += ".sig-good{color:#f39c12}";
  html += ".sig-weak{color:#e74c3c}";

  // Sensor cards - compact collapsed style
  html += ".sensor-grid{display:grid;gap:10px;margin:10px 0}";
  html += ".sensor-card{background:#fff;border:2px solid #e0e0e0;border-radius:8px;overflow:hidden;transition:border-color 0.2s}";
  html += ".sensor-card.changed{border-color:#f39c12}";
  html += ".sensor-summary{display:flex;justify-content:space-between;align-items:center;padding:14px 15px;cursor:pointer;user-select:none}";
  html += ".sensor-summary:hover{background:#f8f9fa}";
  html += ".sensor-info{display:flex;align-items:center;gap:10px;flex:1;min-width:0}";
  html += ".sensor-name-id{font-weight:600;color:#2c3e50;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}";
  html += ".sensor-name-sub{font-size:0.8em;color:#7f8c8d}";
  html += ".sensor-status{padding:4px 10px;border-radius:12px;font-size:0.75em;font-weight:bold;flex-shrink:0}";
  html += ".st-online{background:#27ae60;color:#fff}";
  html += ".st-offline{background:#e74c3c;color:#fff}";
  html += ".sensor-expand{color:#999;font-size:1.2em;margin-left:8px;transition:transform 0.2s;flex-shrink:0}";
  html += ".sensor-card.open .sensor-expand{transform:rotate(90deg)}";
  html += ".sensor-detail{display:none;padding:0 15px 15px;border-top:1px solid #eee}";
  html += ".sensor-card.open .sensor-detail{display:block}";

  // Sensor edit fields
  html += ".s-field{margin-bottom:12px}";
  html += ".s-field label{display:block;font-weight:600;color:#555;margin-bottom:4px;font-size:0.85em}";
  html += ".s-field input,.s-field select{width:100%;padding:10px;border:1px solid #ddd;border-radius:4px;font-size:1em}";

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

  // Alarm sliders - FIXED: full width layout for mobile
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

  // Wizard steps
  html += ".wizard-step{display:none;padding:15px 0}";
  html += ".wizard-step.active{display:block}";
  html += ".wizard-nav{display:flex;gap:10px;margin-top:15px}";
  html += ".wizard-nav .btn{flex:1}";
  html += ".helper-text{font-size:0.85em;color:#666;margin:8px 0 15px;padding:10px;background:#e8f4fc;border-radius:6px;border-left:3px solid #3498db}";
  html += ".info-note{font-size:0.8em;color:#888;padding:8px 12px;background:#f8f9fa;border-radius:4px;margin:10px 0}";

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

  // SECTION 2: Configured Devices (BEFORE add sensor)
  html += "<details open id='configured-devices-section'>";
  html += "<summary>Configured Devices</summary>";
  html += "<div class='sec-content'>";
  html += "<div class='info-note'>If a sensor shows offline, check that it is powered on and within range</div>";
  html += "<div class='sensor-grid' id='configured-devices'><p style='color:#888'>Loading...</p></div>";
  html += "</div></details>";

  // SECTION 3: Add Sensor (wizard-based)
  html += "<details id='add-sensor-section'>";
  html += "<summary>Add Sensor</summary>";
  html += "<div class='sec-content'>";

  // Step 1: Scan
  html += "<div class='wizard-step active' id='wiz-step-1'>";
  html += "<h4 style='margin-bottom:5px;color:#2c3e50'>Scan for Sensors</h4>";
  html += "<div class='helper-text'>Scan for nearby SenseFlow sensors. Ensure the sensor is powered on before scanning.</div>";
  html += "<button class='btn-scan' id='add-scan-btn' onclick='wizStartScan()'>Scan for Sensors</button>";
  html += "<div class='scan-status' id='add-scan-status'></div>";
  html += "<div class='scan-results' id='add-scan-results'></div>";
  html += "</div>";

  // Step 2: Name
  html += "<div class='wizard-step' id='wiz-step-2'>";
  html += "<h4 style='margin-bottom:5px;color:#2c3e50'>Sensor Name</h4>";
  html += "<div class='helper-text'>Give this sensor a friendly name (max 8 characters, letters and numbers only)</div>";
  html += "<div class='fg'>";
  html += "<label>Selected Sensor: <span id='wiz-selected-id' style='color:#2980b9'></span></label>";
  html += "</div>";
  html += "<div class='fg'>";
  html += "<label>Custom Name</label>";
  html += "<input type='text' id='wiz-name' placeholder='e.g., MainTank' maxlength='8' oninput='wizCheckName()'>";
  html += "</div>";
  html += "<div class='wizard-nav'>";
  html += "<button class='btn btn-g' onclick='wizGoTo(1)'>Back</button>";
  html += "<button class='btn btn-p' id='wiz-name-next' disabled onclick='wizGoTo(3)'>Next</button>";
  html += "</div>";
  html += "</div>";

  // Step 3: Tank Height (US only)
  html += "<div class='wizard-step' id='wiz-step-3'>";
  html += "<h4 style='margin-bottom:5px;color:#2c3e50'>Tank Height</h4>";
  html += "<div class='helper-text'>Measure from the ultrasonic sensor to the base of the tank (in cm)</div>";
  html += "<div class='fg'>";
  html += "<label>Height (cm)</label>";
  html += "<input type='number' id='wiz-height' min='31' max='450' placeholder='e.g., 150' oninput='wizCheckHeight()'>";
  html += "</div>";
  html += "<div class='wizard-nav'>";
  html += "<button class='btn btn-g' onclick='wizGoTo(2)'>Back</button>";
  html += "<button class='btn btn-p' id='wiz-height-next' disabled onclick='wizGoTo(4)'>Next</button>";
  html += "</div>";
  html += "</div>";

  // Step 4: Volume (US only, optional)
  html += "<div class='wizard-step' id='wiz-step-4'>";
  html += "<h4 style='margin-bottom:5px;color:#2c3e50'>Tank Volume</h4>";
  html += "<div class='helper-text'>Optional - enter total tank capacity for litre display on dashboard</div>";
  html += "<div class='fg'>";
  html += "<label>Volume (Litres)</label>";
  html += "<input type='number' id='wiz-volume' min='0' max='99999' placeholder='e.g., 1000'>";
  html += "</div>";
  html += "<div class='wizard-nav'>";
  html += "<button class='btn btn-g' onclick='wizGoTo(3)'>Back</button>";
  html += "<button class='btn btn-p' onclick='wizGoTo(5)'>Next</button>";
  html += "</div>";
  html += "</div>";

  // Step 5: Alarms with tank graphic (US only)
  html += "<div class='wizard-step' id='wiz-step-5'>";
  html += "<h4 style='margin-bottom:5px;color:#2c3e50'>Alarm Thresholds</h4>";
  html += "<div class='helper-text'>Set water level thresholds to trigger buzzer alerts</div>";
  html += "<div class='tank-viz'>";
  // Tank graphic
  html += "<div class='tank-graphic'>";
  html += "<div class='tank-cap'></div>";
  html += "<div class='tank-top'></div>";
  html += "<div class='tank-body'>";
  html += "<div class='tank-water' id='new-tank-water' style='height:50%'></div>";
  html += "<div class='tank-line tank-line-low' id='new-tank-line-low' style='bottom:20%'><span class='tank-line-lbl'>LOW</span></div>";
  html += "<div class='tank-line tank-line-high' id='new-tank-line-high' style='bottom:90%'><span class='tank-line-lbl'>HIGH</span></div>";
  html += "</div>";
  html += "</div>";
  // Alarm sliders
  html += "<div class='alarm-section'>";
  // Low alarm
  html += "<div class='alarm-row' id='new-low-row'>";
  html += "<div class='alarm-row-top'>";
  html += "<input type='checkbox' id='wiz-low-enable' checked onchange='wizToggleSlider(\"low\")'>";
  html += "<label>Low Level Alarm</label>";
  html += "</div>";
  html += "<div class='alarm-desc'>Alarm when water drops below this level</div>";
  html += "<div class='alarm-slider-row'>";
  html += "<input type='range' id='wiz-low-slider' min='1' max='94' value='20' oninput='wizUpdateViz()'>";
  html += "<span class='alarm-val' id='wiz-low-val'>20%</span>";
  html += "</div>";
  html += "</div>";
  // High alarm
  html += "<div class='alarm-row' id='new-high-row'>";
  html += "<div class='alarm-row-top'>";
  html += "<input type='checkbox' id='wiz-high-enable' checked onchange='wizToggleSlider(\"high\")'>";
  html += "<label>High Level Alarm</label>";
  html += "</div>";
  html += "<div class='alarm-desc'>Alarm when water rises above this level</div>";
  html += "<div class='alarm-slider-row'>";
  html += "<input type='range' id='wiz-high-slider' min='25' max='99' value='90' oninput='wizUpdateViz()'>";
  html += "<span class='alarm-val' id='wiz-high-val'>90%</span>";
  html += "</div>";
  html += "</div>";
  html += "<div class='alarm-note'>You can set both alarms, or either one individually. Minimum 5% gap when both are enabled</div>";
  html += "</div>";
  html += "</div>";
  html += "<div class='wizard-nav'>";
  html += "<button class='btn btn-g' onclick='wizGoTo(4)'>Back</button>";
  html += "<button class='btn btn-s' onclick='wizAddTank()'>Add Sensor</button>";
  html += "</div>";
  html += "</div>";

  // Step 3-WP: Alarm Mode (WP only)
  html += "<div class='wizard-step' id='wiz-step-3wp'>";
  html += "<h4 style='margin-bottom:5px;color:#2c3e50'>Alarm Mode</h4>";
  html += "<div class='helper-text'>Choose when this sensor should trigger an alarm</div>";
  html += "<div class='fg'>";
  html += "<label>Alarm Mode</label>";
  html += "<select id='wiz-wp-alarm-mode'>";
  html += "<option value='1'>Alarm when water NOT present (pipe monitoring)</option>";
  html += "<option value='2'>Alarm when water IS present (leak detection)</option>";
  html += "<option value='3'>No alarm (monitoring only)</option>";
  html += "</select>";
  html += "</div>";
  html += "<div class='wizard-nav'>";
  html += "<button class='btn btn-g' onclick='wizGoTo(2)'>Back</button>";
  html += "<button class='btn btn-s' onclick='wizAddWP()'>Add Sensor</button>";
  html += "</div>";
  html += "</div>";

  html += "<div style='margin-top:20px;padding:14px 16px;background:#e8f4fc;border:1px solid #3498db;border-radius:8px;font-size:0.95em;color:#2c3e50;text-align:center'>To monitor additional tanks or water presence points, purchase SenseFlow sensors at <strong style='color:#2980b9'>senseflow.in</strong></div>";
  html += "</div></details>";

  html += "</div>"; // container

  // JavaScript
  html += "<script>";

  // Global variables
  html += "let configData={tanks:[],wpSensors:[]};";
  html += "let scanTimer=null;";
  html += "let wizSensorType='';";
  html += "let wizSelectedId='';";
  html += "let hasUnsavedCards=false;";

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
  html += "configData=data;";
  html += "renderConfiguredDevices();";
  html += "}).catch(e=>showToast('Error loading: '+e.message,true));";
  html += "}";

  // Render ALL configured devices (tanks + WP) as compact expandable cards
  html += "function renderConfiguredDevices(){";
  html += "const grid=document.getElementById('configured-devices');";
  html += "let html='';";
  html += "let hasDevices=false;";

  // Tanks
  html += "if(configData.tanks){configData.tanks.forEach(t=>{if(!t.enabled)return;";
  html += "hasDevices=true;";
  html += "const stCls=t.online?'st-online':'st-offline';";
  html += "const stTxt=t.online?'Online':'Offline';";
  html += "const lowEn=t.lowAlarm>0;const highEn=t.highAlarm>0;";
  html += "const lowVal=lowEn?t.lowAlarm:20;const highVal=highEn?t.highAlarm:90;";
  html += "html+=`<div class='sensor-card' id='tank-card-${t.index}'>`;";
  // Compact summary row
  html += "html+=`<div class='sensor-summary' onclick='toggleCard(\"tank-card-${t.index}\")'>`;";
  html += "html+=`<div class='sensor-info'><div><span class='sensor-name-id'>${t.name}</span> <span class='sensor-name-sub'>${t.nodeId}</span></div></div>`;";
  html += "html+=`<span class='sensor-status ${stCls}'>${stTxt}</span>`;";
  html += "html+=`<span class='sensor-expand'>\\u203A</span>`;";
  html += "html+=`</div>`;";
  // Expandable detail
  html += "html+=`<div class='sensor-detail'>`;";
  html += "html+=`<div class='s-field'><label>Tank Name</label><input type='text' id='tank-${t.index}-name' value='${t.name}' maxlength='8' onchange='markCardChanged(\"tank-card-${t.index}\")'><small style='color:#888;font-size:0.75em'>Max 8 characters, letters and numbers only</small></div>`;";
  html += "html+=`<div class='s-field'><label>Height (cm)</label><input type='number' id='tank-${t.index}-height' value='${t.height}' onchange='markCardChanged(\"tank-card-${t.index}\")'></div>`;";
  html += "html+=`<div class='s-field'><label>Volume (Litres)</label><input type='number' id='tank-${t.index}-volume' value='${t.volume}' onchange='markCardChanged(\"tank-card-${t.index}\")'></div>`;";
  // Tank viz + alarms
  html += "html+=`<div class='tank-viz'>`;";
  html += "html+=`<div class='tank-graphic'><div class='tank-cap'></div><div class='tank-top'></div>`;";
  html += "html+=`<div class='tank-body'><div class='tank-water' style='height:50%'></div>`;";
  html += "html+=`<div class='tank-line tank-line-low' id='tank-${t.index}-line-low' style='bottom:${lowVal}%;display:${lowEn?'block':'none'}'><span class='tank-line-lbl'>LOW</span></div>`;";
  html += "html+=`<div class='tank-line tank-line-high' id='tank-${t.index}-line-high' style='bottom:${highVal}%;display:${highEn?'block':'none'}'><span class='tank-line-lbl'>HIGH</span></div>`;";
  html += "html+=`</div></div>`;";
  html += "html+=`<div class='alarm-section'>`;";
  // Low alarm
  html += "html+=`<div class='alarm-row ${lowEn?'':'disabled'}' id='tank-${t.index}-low-row'>`;";
  html += "html+=`<div class='alarm-row-top'><input type='checkbox' id='tank-${t.index}-low-en' ${lowEn?'checked':''} onchange='toggleExistingSlider(${t.index},\"low\")'><label>Low Alarm</label></div>`;";
  html += "html+=`<div class='alarm-desc'>Alarm when water drops below this level</div>`;";
  html += "html+=`<div class='alarm-slider-row'><input type='range' id='tank-${t.index}-low-slider' min='1' max='94' value='${lowVal}' ${lowEn?'':'disabled'} oninput='updateExistingTankViz(${t.index})'><span class='alarm-val ${lowEn?'':'disabled'}' id='tank-${t.index}-low-val'>${lowVal}%</span></div>`;";
  html += "html+=`</div>`;";
  // High alarm
  html += "html+=`<div class='alarm-row ${highEn?'':'disabled'}' id='tank-${t.index}-high-row'>`;";
  html += "html+=`<div class='alarm-row-top'><input type='checkbox' id='tank-${t.index}-high-en' ${highEn?'checked':''} onchange='toggleExistingSlider(${t.index},\"high\")'><label>High Alarm</label></div>`;";
  html += "html+=`<div class='alarm-desc'>Alarm when water rises above this level</div>`;";
  html += "html+=`<div class='alarm-slider-row'><input type='range' id='tank-${t.index}-high-slider' min='${lowEn?lowVal+5:1}' max='99' value='${highVal}' ${highEn?'':'disabled'} oninput='updateExistingTankViz(${t.index})'><span class='alarm-val ${highEn?'':'disabled'}' id='tank-${t.index}-high-val'>${highVal}%</span></div>`;";
  html += "html+=`</div>`;";
  html += "html+=`<div class='alarm-note'>You can set both alarms, or either one individually. Minimum 5% gap when both are enabled</div>`;";
  html += "html+=`</div></div>`;";
  // Action buttons
  html += "html+=`<div class='card-actions'>`;";
  html += "html+=`<button class='btn btn-s' id='save-tank-${t.index}' onclick='saveTank(${t.index})'>Save</button>`;";
  html += "html+=`<button class='btn btn-d' onclick='deleteTank(${t.index})'>Delete</button>`;";
  html += "html+=`</div></div></div>`;";
  html += "});}";

  // WP sensors
  html += "if(configData.wpSensors){configData.wpSensors.forEach(w=>{if(!w.enabled)return;";
  html += "hasDevices=true;";
  html += "const stCls=w.online?'st-online':'st-offline';";
  html += "const stTxt=w.online?'Online':'Offline';";
  html += "html+=`<div class='sensor-card' id='wp-card-${w.index}'>`;";
  html += "html+=`<div class='sensor-summary' onclick='toggleCard(\"wp-card-${w.index}\")'>`;";
  html += "html+=`<div class='sensor-info'><div><span class='sensor-name-id'>${w.name}</span> <span class='sensor-name-sub'>${w.nodeId}</span></div></div>`;";
  html += "html+=`<span class='sensor-status ${stCls}'>${stTxt}</span>`;";
  html += "html+=`<span class='sensor-expand'>\\u203A</span>`;";
  html += "html+=`</div>`;";
  html += "html+=`<div class='sensor-detail'>`;";
  html += "html+=`<div class='s-field'><label>Sensor Name</label><input type='text' id='wp-${w.index}-name' value='${w.name}' maxlength='8' onchange='markCardChanged(\"wp-card-${w.index}\")'><small style='color:#888;font-size:0.75em'>Max 8 characters, letters and numbers only</small></div>`;";
  html += "html+=`<div class='s-field'><label>Alarm Mode</label><select id='wp-${w.index}-alarmMode' onchange='markCardChanged(\"wp-card-${w.index}\")'>`;";
  html += "html+=`<option value='1' ${w.alarmMode==1?'selected':''}>Alarm when water NOT present</option>`;";
  html += "html+=`<option value='2' ${w.alarmMode==2?'selected':''}>Alarm when water IS present</option>`;";
  html += "html+=`<option value='3' ${w.alarmMode==3?'selected':''}>Monitor only (no alarm)</option>`;";
  html += "html+=`</select></div>`;";
  html += "html+=`<div class='card-actions'>`;";
  html += "html+=`<button class='btn btn-s' id='save-wp-${w.index}' onclick='saveWP(${w.index})'>Save</button>`;";
  html += "html+=`<button class='btn btn-d' onclick='deleteWP(${w.index})'>Delete</button>`;";
  html += "html+=`</div></div></div>`;";
  html += "});}";

  html += "if(!hasDevices){";
  html += "document.getElementById('configured-devices-section').style.display='none';";
  html += "document.getElementById('add-sensor-section').open=true;";
  html += "}else{";
  html += "document.getElementById('configured-devices-section').style.display='';";
  html += "}";
  html += "grid.innerHTML=html;";
  html += "}";

  // Toggle card expand/collapse
  html += "function toggleCard(id){";
  html += "const card=document.getElementById(id);";
  html += "if(card)card.classList.toggle('open');";
  html += "}";

  // Mark card as changed (orange border + save button turns orange)
  html += "function markCardChanged(id){";
  html += "hasUnsavedCards=true;";
  html += "const card=document.getElementById(id);";
  html += "if(card){card.classList.add('changed');";
  html += "const saveBtn=card.querySelector('.btn-s');";
  html += "if(saveBtn){saveBtn.style.background='#f39c12';saveBtn.textContent='Save *';}";
  html += "}}";

  // Save individual tank
  html += "function saveTank(idx){";
  html += "const n=document.getElementById('tank-'+idx+'-name').value.trim();";
  html += "if(!n||n.length>8||!/^[a-zA-Z0-9]+$/.test(n)){showToast('Invalid name: max 8 chars, letters and numbers only',true);return;}";
  html += "const h=document.getElementById('tank-'+idx+'-height').value;";
  html += "const v=document.getElementById('tank-'+idx+'-volume').value;";
  html += "const lowEn=document.getElementById('tank-'+idx+'-low-en').checked;";
  html += "const highEn=document.getElementById('tank-'+idx+'-high-en').checked;";
  html += "const lowVal=lowEn?document.getElementById('tank-'+idx+'-low-slider').value:0;";
  html += "const highVal=highEn?document.getElementById('tank-'+idx+'-high-slider').value:0;";
  html += "const fd=new FormData();fd.append('index',idx);";
  html += "fd.append('name',n);fd.append('height',h);fd.append('volume',v);fd.append('lowAlarm',lowVal);fd.append('highAlarm',highVal);";
  html += "fetch('/api/tank',{method:'PUT',body:fd}).then(r=>r.json()).then(d=>{";
  html += "if(d.error)showToast(d.error,true);";
  html += "else{showToast('Saved!');hasUnsavedCards=false;";
  html += "const card=document.getElementById('tank-card-'+idx);";
  html += "if(card){card.classList.remove('changed');";
  html += "const sb=card.querySelector('.btn-s');if(sb){sb.style.background='#27ae60';sb.textContent='Save';}}";
  html += "loadConfigData();}";
  html += "}).catch(e=>showToast('Error: '+e.message,true));";
  html += "}";

  // Save individual WP sensor
  html += "function saveWP(idx){";
  html += "const n=document.getElementById('wp-'+idx+'-name').value.trim();";
  html += "if(!n||n.length>8||!/^[a-zA-Z0-9]+$/.test(n)){showToast('Invalid name: max 8 chars, letters and numbers only',true);return;}";
  html += "const am=document.getElementById('wp-'+idx+'-alarmMode').value;";
  html += "const fd=new FormData();fd.append('index',idx);fd.append('name',n);fd.append('alarmMode',am);";
  html += "fetch('/api/wp',{method:'PUT',body:fd}).then(r=>r.json()).then(d=>{";
  html += "if(d.error)showToast(d.error,true);";
  html += "else{showToast('Saved!');hasUnsavedCards=false;";
  html += "const card=document.getElementById('wp-card-'+idx);";
  html += "if(card){card.classList.remove('changed');";
  html += "const sb=card.querySelector('.btn-s');if(sb){sb.style.background='#27ae60';sb.textContent='Save';}}";
  html += "loadConfigData();}";
  html += "}).catch(e=>showToast('Error: '+e.message,true));";
  html += "}";

  // Toggle existing tank slider
  html += "function toggleExistingSlider(idx,type){";
  html += "const chk=document.getElementById('tank-'+idx+'-'+type+'-en');";
  html += "const slider=document.getElementById('tank-'+idx+'-'+type+'-slider');";
  html += "const val=document.getElementById('tank-'+idx+'-'+type+'-val');";
  html += "const row=document.getElementById('tank-'+idx+'-'+type+'-row');";
  html += "const line=document.getElementById('tank-'+idx+'-line-'+type);";
  html += "if(chk.checked){";
  html += "slider.disabled=false;row.classList.remove('disabled');val.classList.remove('disabled');if(line)line.style.display='block';";
  html += "updateExistingTankViz(idx);";
  html += "}else{";
  html += "slider.disabled=true;row.classList.add('disabled');val.classList.add('disabled');if(line)line.style.display='none';";
  html += "}";
  html += "markCardChanged('tank-card-'+idx);";
  html += "}";

  // Update existing tank viz
  html += "function updateExistingTankViz(idx){";
  html += "const lowEn=document.getElementById('tank-'+idx+'-low-en').checked;";
  html += "const highEn=document.getElementById('tank-'+idx+'-high-en').checked;";
  html += "const lowSlider=document.getElementById('tank-'+idx+'-low-slider');";
  html += "const highSlider=document.getElementById('tank-'+idx+'-high-slider');";
  html += "let low=parseInt(lowSlider.value);let high=parseInt(highSlider.value);";
  html += "if(lowEn&&highEn){if(high<low+5){high=low+5;highSlider.value=high;}highSlider.min=low+5;}else{highSlider.min=1;}";
  html += "document.getElementById('tank-'+idx+'-low-val').textContent=low+'%';";
  html += "document.getElementById('tank-'+idx+'-high-val').textContent=high+'%';";
  html += "const ll=document.getElementById('tank-'+idx+'-line-low');const hl=document.getElementById('tank-'+idx+'-line-high');";
  html += "if(ll)ll.style.bottom=low+'%';if(hl)hl.style.bottom=high+'%';";
  html += "markCardChanged('tank-card-'+idx);";
  html += "}";

  // Delete functions
  html += "function deleteTank(idx){";
  html += "if(!confirm('Delete this tank sensor?'))return;";
  html += "const fd=new FormData();fd.append('index',idx);";
  html += "fetch('/api/tank',{method:'DELETE',body:fd}).then(r=>r.json()).then(d=>{";
  html += "if(d.error)showToast(d.error,true);else{showToast('Deleted');loadConfigData();}";
  html += "});";
  html += "}";

  html += "function deleteWP(idx){";
  html += "if(!confirm('Delete this sensor?'))return;";
  html += "const fd=new FormData();fd.append('index',idx);";
  html += "fetch('/api/wp',{method:'DELETE',body:fd}).then(r=>r.json()).then(d=>{";
  html += "if(d.error)showToast(d.error,true);else{showToast('Deleted');loadConfigData();}";
  html += "});";
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
  html += "function refreshData(){hasUnsavedCards=false;loadConfigData();showToast('Data refreshed');}";

  // Handle back with unsaved changes warning
  html += "function handleBack(e){";
  html += "if(e)e.preventDefault();";
  html += "if(hasUnsavedCards){";
  html += "if(confirm('You have unsaved changes. Leave without saving?')){window.location.href='/';}";
  html += "}else{window.location.href='/';}}";

  // Browser beforeunload warning
  html += "window.addEventListener('beforeunload',function(e){";
  html += "if(hasUnsavedCards){e.preventDefault();e.returnValue='';return '';}});";

  // === WIZARD FUNCTIONS ===

  // Navigate wizard steps
  html += "function wizGoTo(step){";
  html += "document.querySelectorAll('.wizard-step').forEach(s=>s.classList.remove('active'));";
  // For WP sensors, step 3 goes to wiz-step-3wp
  html += "if(step===3&&wizSensorType==='WP'){document.getElementById('wiz-step-3wp').classList.add('active');return;}";
  html += "const el=document.getElementById('wiz-step-'+step);";
  html += "if(el)el.classList.add('active');";
  html += "}";

  // Wizard: Start scan
  html += "function wizStartScan(){";
  html += "const btn=document.getElementById('add-scan-btn');";
  html += "const status=document.getElementById('add-scan-status');";
  html += "const results=document.getElementById('add-scan-results');";
  html += "btn.disabled=true;btn.textContent='Scanning...';";
  html += "results.innerHTML='';status.textContent='Scanning for nearby sensors...';";
  html += "fetch('/api/scan?action=start').then(()=>{";
  html += "scanTimer=setInterval(()=>wizPollScan(),2000);";
  html += "setTimeout(()=>wizPollScan(),500);";
  html += "}).catch(()=>{btn.disabled=false;btn.textContent='Scan for Sensors';status.textContent='Scan failed. Try again.';});";
  html += "}";

  // Wizard: Poll scan results (filters out already configured sensors)
  html += "function wizPollScan(){";
  html += "const btn=document.getElementById('add-scan-btn');";
  html += "const status=document.getElementById('add-scan-status');";
  html += "const results=document.getElementById('add-scan-results');";
  html += "fetch('/api/scan').then(r=>r.json()).then(d=>{";
  html += "if(d.scanning){status.textContent='Scanning... '+d.remaining+'s remaining';}";
  html += "else{wizStopScanUI();";
  html += "}";
  // Check which devices are already configured
  html += "const configured=[];";
  html += "if(configData.tanks)configData.tanks.forEach(t=>{if(t.enabled)configured.push(t.nodeId);});";
  html += "if(configData.wpSensors)configData.wpSensors.forEach(w=>{if(w.enabled)configured.push(w.nodeId);});";
  html += "const available=d.devices.filter(dev=>!configured.includes(dev.id));";
  html += "const alreadyConf=d.devices.filter(dev=>configured.includes(dev.id));";
  html += "let html='';";
  html += "if(d.devices.length>0){";
  html += "if(!d.scanning&&available.length>0)status.textContent='Found '+available.length+' new sensor(s). Tap to select.';";
  html += "else if(!d.scanning&&available.length===0)status.textContent='All detected sensors are already configured.';";
  // New sensors first (clickable)
  html += "available.forEach(dev=>{";
  html += "const sigCls=dev.signal==='Excellent'?'sig-strong':dev.signal==='Good'?'sig-good':'sig-weak';";
  html += "const sType=dev.id.startsWith('WP')?'WP':'US';";
  html += "html+=`<div class='scan-item' onclick='wizSelectDevice(\"${dev.id}\",\"${sType}\")'>`;";
  html += "html+=`<span class='scan-item-id'>${dev.id}</span>`;";
  html += "html+=`<span class='scan-item-signal ${sigCls}'>${dev.signal}</span></div>`;";
  html += "});";
  // Already configured sensors (greyed out, not clickable)
  html += "alreadyConf.forEach(dev=>{";
  html += "html+=`<div class='scan-item configured'>`;";
  html += "html+=`<span class='scan-item-id'>${dev.id} <span class='scan-configured-tag'>Already configured</span></span>`;";
  html += "html+=`<span class='scan-item-signal' style='color:#999'>${dev.signal}</span></div>`;";
  html += "});";
  html += "}else if(!d.scanning){";
  html += "status.textContent='No sensors found. Make sure the sensor is powered on and nearby.';";
  html += "}";
  html += "results.innerHTML=html;";
  html += "}).catch(()=>{wizStopScanUI();status.textContent='Scan error. Try again.';});";
  html += "}";

  html += "function wizStopScanUI(){";
  html += "clearInterval(scanTimer);scanTimer=null;";
  html += "const btn=document.getElementById('add-scan-btn');";
  html += "btn.disabled=false;btn.textContent='Scan for Sensors';";
  html += "}";

  // Wizard: Select a scanned device
  html += "function wizSelectDevice(id,sType){";
  html += "clearInterval(scanTimer);scanTimer=null;";
  html += "fetch('/api/scan?action=stop').catch(()=>{});";
  html += "wizSelectedId=id;wizSensorType=sType;";
  html += "document.getElementById('wiz-selected-id').textContent=id+' ('+( sType==='US'?'Tank Sensor':'Water Presence')+')';";
  html += "document.getElementById('wiz-name').value='';";
  html += "document.getElementById('wiz-name-next').disabled=true;";
  html += "wizGoTo(2);";
  html += "}";

  // Wizard: Check name validity
  html += "function wizCheckName(){";
  html += "const name=document.getElementById('wiz-name').value.trim();";
  html += "document.getElementById('wiz-name-next').disabled=!(name.length>=1&&name.length<=8&&/^[a-zA-Z0-9]+$/.test(name));";
  html += "}";

  // Wizard: Check height validity
  html += "function wizCheckHeight(){";
  html += "const h=parseInt(document.getElementById('wiz-height').value);";
  html += "document.getElementById('wiz-height-next').disabled=!(h>=31&&h<=450);";
  html += "}";

  // Wizard: Toggle alarm slider
  html += "function wizToggleSlider(type){";
  html += "const chk=document.getElementById('wiz-'+type+'-enable');";
  html += "const slider=document.getElementById('wiz-'+type+'-slider');";
  html += "const val=document.getElementById('wiz-'+type+'-val');";
  html += "const row=document.getElementById('new-'+type+'-row');";
  html += "const line=document.getElementById('new-tank-line-'+type);";
  html += "if(chk.checked){slider.disabled=false;row.classList.remove('disabled');val.classList.remove('disabled');if(line)line.style.display='block';wizUpdateViz();}";
  html += "else{slider.disabled=true;row.classList.add('disabled');val.classList.add('disabled');if(line)line.style.display='none';}";
  html += "}";

  // Wizard: Update tank visualization
  html += "function wizUpdateViz(){";
  html += "const lowEn=document.getElementById('wiz-low-enable').checked;";
  html += "const highEn=document.getElementById('wiz-high-enable').checked;";
  html += "const lowSlider=document.getElementById('wiz-low-slider');";
  html += "const highSlider=document.getElementById('wiz-high-slider');";
  html += "let low=parseInt(lowSlider.value);let high=parseInt(highSlider.value);";
  html += "if(lowEn&&highEn){if(high<low+5){high=low+5;highSlider.value=high;}highSlider.min=low+5;}else{highSlider.min=1;}";
  html += "document.getElementById('wiz-low-val').textContent=low+'%';";
  html += "document.getElementById('wiz-high-val').textContent=high+'%';";
  html += "if(lowEn){const ll=document.getElementById('new-tank-line-low');if(ll)ll.style.bottom=low+'%';}";
  html += "if(highEn){const hl=document.getElementById('new-tank-line-high');if(hl)hl.style.bottom=high+'%';}";
  html += "}";

  // Wizard: Add tank sensor (final step)
  html += "function wizAddTank(){";
  html += "const nodeId=wizSelectedId;";
  html += "const name=document.getElementById('wiz-name').value.trim();";
  html += "const height=document.getElementById('wiz-height').value;";
  html += "const volume=document.getElementById('wiz-volume').value||0;";
  html += "const lowEn=document.getElementById('wiz-low-enable').checked;";
  html += "const highEn=document.getElementById('wiz-high-enable').checked;";
  html += "const lowAlarm=lowEn?document.getElementById('wiz-low-slider').value:0;";
  html += "const highAlarm=highEn?document.getElementById('wiz-high-slider').value:0;";
  html += "if(!nodeId||!name||!height){showToast('Missing required fields',true);return;}";
  html += "const fd=new FormData();";
  html += "fd.append('nodeId',nodeId);fd.append('name',name);fd.append('height',height);";
  html += "fd.append('volume',volume);fd.append('lowAlarm',lowAlarm);fd.append('highAlarm',highAlarm);";
  html += "fetch('/api/tank',{method:'POST',body:fd}).then(r=>r.json()).then(d=>{";
  html += "if(d.error)showToast(d.error,true);";
  html += "else{showToast('Tank sensor added!');wizReset();loadConfigData();}";
  html += "}).catch(e=>showToast('Error: '+e.message,true));";
  html += "}";

  // Wizard: Add WP sensor (final step)
  html += "function wizAddWP(){";
  html += "const nodeId=wizSelectedId;";
  html += "const name=document.getElementById('wiz-name').value.trim();";
  html += "const alarmMode=document.getElementById('wiz-wp-alarm-mode').value;";
  html += "if(!nodeId||!name){showToast('Missing required fields',true);return;}";
  html += "const fd=new FormData();";
  html += "fd.append('nodeId',nodeId);fd.append('name',name);fd.append('alarmMode',alarmMode);";
  html += "fetch('/api/wp',{method:'POST',body:fd}).then(r=>r.json()).then(d=>{";
  html += "if(d.error)showToast(d.error,true);";
  html += "else{showToast('Sensor added!');wizReset();loadConfigData();}";
  html += "}).catch(e=>showToast('Error: '+e.message,true));";
  html += "}";

  // Wizard: Reset to step 1
  html += "function wizReset(){";
  html += "wizSelectedId='';wizSensorType='';";
  html += "document.querySelectorAll('.wizard-step').forEach(s=>s.classList.remove('active'));";
  html += "document.getElementById('wiz-step-1').classList.add('active');";
  html += "document.getElementById('add-scan-status').textContent='';";
  html += "document.getElementById('add-scan-results').innerHTML='';";
  html += "document.getElementById('add-scan-btn').disabled=false;";
  html += "document.getElementById('add-scan-btn').textContent='Scan for Sensors';";
  html += "document.getElementById('wiz-name').value='';";
  html += "document.getElementById('wiz-height').value='';";
  html += "document.getElementById('wiz-volume').value='';";
  html += "document.getElementById('wiz-low-enable').checked=true;";
  html += "document.getElementById('wiz-high-enable').checked=true;";
  html += "document.getElementById('wiz-low-slider').value=20;";
  html += "document.getElementById('wiz-high-slider').value=90;";
  html += "document.getElementById('wiz-low-slider').disabled=false;";
  html += "document.getElementById('wiz-high-slider').disabled=false;";
  html += "const lr=document.getElementById('new-low-row');if(lr)lr.classList.remove('disabled');";
  html += "const hr=document.getElementById('new-high-row');if(hr)hr.classList.remove('disabled');";
  html += "const ll=document.getElementById('new-tank-line-low');if(ll)ll.style.display='block';";
  html += "const hl=document.getElementById('new-tank-line-high');if(hl)hl.style.display='block';";
  html += "wizUpdateViz();";
  html += "}";

  // Initialize
  html += "window.onload=function(){loadConfigData();";
  html += "setTimeout(()=>{const p=new URLSearchParams(window.location.search);const o=p.get('open');";
  html += "if(o){const card=document.getElementById(o.replace('tank-','tank-card-').replace('wp-','wp-card-'));";
  html += "if(card){card.classList.add('open');card.scrollIntoView({behavior:'smooth',block:'center'});}}";
  html += "},500);";
  html += "};";

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
  Serial.println("- OLED display optional - system works without it");
  Serial.println("- Web UI always available at http://192.168.4.1:7689");
  Serial.println("- Auto-cycling display with manual control");
  Serial.println("- Alarm confirmation system prevents false alarms");
  Serial.println("=====================================");
  
  // Initialize hardware with FIXED pins
  pinMode(BUZZER_PIN, OUTPUT);  // Pin 4 (safer than Pin 2, no boot strapping issues)
  digitalWrite(BUZZER_PIN, LOW);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Pin 15
  
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

  // Initialize sensor confirmation tracking
  for (int i = 0; i < MAX_TANKS; i++) {
    resetSensorConfirmation(i);
  }
  Serial.println("Sensor confirmation system initialized (requires fresh data after startup)");

  // Initialize LoRa with FIXED pins
  LoRa.setPins(SS, RST, DIO0);  // SS=5, RST=14, DIO0=26
  
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed!");
    if (oledAvailable) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tr);
      u8g2.drawStr(0, 15, "LoRa FAILED!");
      u8g2.drawStr(0, 30, "Check wiring");
      u8g2.sendBuffer();
    }

    while (1) {
      delay(1000);
      Serial.println("Retrying LoRa init...");
    }
  }
  
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setPreambleLength(LORA_PREAMBLE);
  LoRa.setSyncWord(LORA_SYNC_WORD);

  LoRa.receive();  // Explicitly enter continuous receive mode

  Serial.println("LoRa receiver ready");
  
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
    
    int configuredTanks = 0;
    int configuredWP = 0;
    
    for (int i = 0; i < MAX_TANKS; i++) {
      if (tankConfigs[i].enabled) configuredTanks++;
    }
    
    for (int i = 0; i < MAX_WP_SENSORS; i++) {
      if (wpConfigs[i].enabled) configuredWP++;
    }
    
    Serial.print(">>> Monitoring: ");
    Serial.print(configuredTanks);
    Serial.print(" tank sensor(s), ");
    Serial.print(configuredWP);
    Serial.println(" WP sensor(s) <<<");
  }
  
  // Initialize screen management
  screenMgr.lastUpdate = millis();
  updateDisplay();
  
  Serial.println("\nFEATURES CONFIRMED WORKING:");
  Serial.println("- Web UI shows tank status with acknowledge button");
  Serial.println("- OLED auto-cycling display (4s per screen)");
  Serial.println("- Buzzer test available in CONFIG > General Settings > Test Buzzer");
  Serial.println("- Button functions: Single=ACK, Double=System info (5s) + Tech details (5s), Long=Manual screen");
  Serial.println("- API endpoint working: /api/status");
  Serial.println("- All pin assignments fixed and verified");
  Serial.println("\nTERMINAL COMMANDS:");
  Serial.println("- CONFIG: Complete sensor setup menu");
  Serial.println("- WIFI: Direct WiFi SSID/password setup");
  Serial.println("- STATUS: Show all system and sensor status");
  Serial.println("- ALARM: Test buzzer system");
  Serial.println("- CONFIRM: View/change confirmations required (default: 5)");
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
  
  // Check for LoRa packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedData = "";
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }
    int rssi = LoRa.packetRssi();
    processPacket(receivedData, rssi);
    LoRa.receive();  // Re-enter continuous receive mode after processing packet
  }
  
  // Update alarm system
  updateAlarmSystem();
  
  // Update display with auto-cycling system
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 100) { // Check more frequently for smooth transitions
    updateDisplay();
    lastDisplayUpdate = millis();
  }
  
  // LoRa health check - reinitialize if no packets received for 60 seconds
  static unsigned long lastLoRaCheck = 0;
  if (millis() - lastLoRaCheck > 10000) { // Check every 10 seconds
    if (lastPacketTime > 0 && (millis() - lastPacketTime) > 60000) {
      loRaReinitCount++;
      Serial.print("WARNING: LoRa health check FAILED - no packets for 60s. Reinitializing... (count: ");
      Serial.print(loRaReinitCount);
      Serial.println(")");

      // Reinitialize LoRa
      LoRa.end();
      delay(100);
      LoRa.setPins(SS, RST, DIO0);
      if (LoRa.begin(433E6)) {
        LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
        LoRa.setSignalBandwidth(LORA_BANDWIDTH);
        LoRa.setCodingRate4(LORA_CODING_RATE);
        LoRa.setPreambleLength(LORA_PREAMBLE);
        LoRa.setSyncWord(LORA_SYNC_WORD);
        LoRa.receive();  // Enter continuous receive mode after reinitialization
        Serial.println("LoRa reinitialized successfully");
      } else {
        Serial.println("ERROR: LoRa reinitialization FAILED");
      }

      lastPacketTime = millis(); // Reset timer to prevent immediate re-trigger

      // Log every 5th reinitialization
      if (loRaReinitCount % 5 == 0) {
        Serial.print("WARNING: LoRa has been reinitialized ");
        Serial.print(loRaReinitCount);
        Serial.println(" times since startup");
      }
    }
    lastLoRaCheck = millis();
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

  // Check tank sensors
  for (int i = 0; i < MAX_TANKS; i++) {
    if (tankConfigs[i].enabled && tankData[i].hasData) {
      bool wasOnline = tankData[i].isOnline;
      tankData[i].isOnline = (currentTime - tankData[i].lastUpdate) <= offlineThreshold;

      if (wasOnline && !tankData[i].isOnline) {
        // Sensor went offline - reset confirmation tracking
        resetSensorConfirmation(i);
        Serial.print("Tank ");
        Serial.print(i + 1);
        Serial.print(" (");
        Serial.print(tankConfigs[i].customName);
        Serial.println(") went OFFLINE - confirmation reset");
      } else if (!wasOnline && tankData[i].isOnline) {
        // Clear stale alarm acknowledgments so alarms can re-trigger fresh after recovery
        if (alarmAck.tankLowThresholdAck[i]) {
          alarmAck.tankLowThresholdAck[i] = false;
          Serial.print("Tank ");
          Serial.print(i + 1);
          Serial.print(" (");
          Serial.print(tankConfigs[i].customName);
          Serial.println(") low alarm ack cleared on recovery");
        }
        if (alarmAck.tankHighThresholdAck[i]) {
          alarmAck.tankHighThresholdAck[i] = false;
          Serial.print("Tank ");
          Serial.print(i + 1);
          Serial.print(" (");
          Serial.print(tankConfigs[i].customName);
          Serial.println(") high alarm ack cleared on recovery");
        }
        if (alarmAck.tankOfflineAck[i]) {
          alarmAck.tankOfflineAck[i] = false;
        }
        saveAlarmAckData();
        Serial.print("Tank ");
        Serial.print(i + 1);
        Serial.print(" (");
        Serial.print(tankConfigs[i].customName);
        Serial.println(") came back ONLINE - alarm acks reset");
      }
    }
  }
  
  // Check WP sensors
  for (int i = 0; i < MAX_WP_SENSORS; i++) {
    if (wpConfigs[i].enabled && wpData[i].hasData) {
      bool wasOnline = wpData[i].isOnline;
      wpData[i].isOnline = (currentTime - wpData[i].lastUpdate) <= offlineThreshold;
      
      if (wasOnline && !wpData[i].isOnline) {
        Serial.print("WP sensor ");
        Serial.print(wpConfigs[i].customName);
        Serial.println(" went OFFLINE");
      } else if (!wasOnline && wpData[i].isOnline) {
        Serial.print("WP sensor ");
        Serial.print(wpConfigs[i].customName);
        Serial.println(" came back ONLINE");
      }
    }
  }
}