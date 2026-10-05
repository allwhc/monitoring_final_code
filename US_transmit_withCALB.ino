/*
 * ESP32 Ultrasonic LoRa Transmitter with WiFi Configuration
 * ==========================================================
 * Firmware Version: 9.0.0
 *
 * FEATURES:
 * - WiFi AP mode for configuration (2 min timeout, max 5 min)
 * - Web UI for Device ID and Sensor Mode configuration
 * - 3 Sensor Modes: US Only, WP Only, US+WP Dual
 * - mvsOTA firmware updates
 * - MAD filtering for robust ultrasonic readings
 * - Zone-based Smart Boundary Logic with EEPROM
 * - Session ID + Counter for duplicate packet rejection
 * - LoRa transmission continues during AP mode
 *
 * HARDWARE CONNECTIONS:
 * ---------------------
 * LoRa Module (SX1278) -> ESP32
 *   VCC  -> 3.3V
 *   GND  -> GND
 *   MOSI -> GPIO23
 *   MISO -> GPIO19
 *   SCK  -> GPIO18
 *   NSS  -> GPIO5
 *   RST  -> GPIO14
 *   DIO0 -> GPIO26
 *
 * Ultrasonic Sensor (HC-SR04) -> ESP32
 *   VCC  -> 5V
 *   GND  -> GND
 *   TRIG -> GPIO25
 *   ECHO -> GPIO34
 */

#include <SPI.h>
#include <LoRa.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <mvsota_esp32.h>

#define FIRMWARE_VERSION "8.0.7" // Fixed calibration: step='any', range<=450, raw/calibrated display

#define FIRMWARE_CODE "SF-US-2026-9867"

// Debug mode: 0 = minimal serial output, 1 = full debug output (for developers)
#define DEBUG_MODE 0

// LoRa pins for ESP32 (same as wireless.ino)
#define LORA_SS     5
#define LORA_RST    14
#define LORA_DIO0   26

// Ultrasonic sensor pins for ESP32 (same as WIRED.ino)
#define TRIG_PIN    25
#define ECHO_PIN    34

// Device configuration
#define COMPANY_SECRET "mvstech"
#define BASE_INTERVAL 10000
#define DEVICE_OFFSET 100

// WiFi AP Configuration
#define AP_PASSWORD "mvstech9867"
#define AP_SUFFIX "_mvstechsensors"
#define AP_DEFAULT_TIMEOUT_MS 120000      // 2 minutes default
#define AP_EXTEND_TIME_MS 60000           // Extend by 1 minute per activity
#define AP_MAX_TIME_MS 300000             // 5 minutes maximum
#define AP_INFINITE_MODE 0                // 0 = normal timeout, 1 = AP never stops

// Web UI Configuration
#define PASSWORD_PROTECTION 1             // 1 = login required, 0 = direct dashboard access

// Web Server Configuration
#define WEB_PORT 7689
#define WEB_PASSWORD "133611"
#define APP_USER_AGENT "MVStech7689"

// MAD-enhanced sampling configuration
#define SAMPLE_COUNT 15
#define MIN_VALID_SAMPLES 10
#define READING_DELAY 50
#define MAD_MULTIPLIER 2.5

// Zone definitions for Smart Boundary Logic
#define UPPER_ZONE_MIN 20
#define UPPER_ZONE_MAX 25
#define MIDDLE_ZONE_MIN 26
#define MIDDLE_ZONE_MAX 389
#define LOWER_ZONE_MIN 390

// EEPROM configuration
#define EEPROM_SIZE 112
#define EEPROM_ZONE_ADDR 0
#define EEPROM_VALID_FLAG_ADDR 1
#define EEPROM_DEVICE_ID_ADDR 2           // 10 bytes for device ID
#define EEPROM_DEVICE_ID_VALID_ADDR 12    // 1 byte magic number
// Sensor Mode EEPROM addresses (13-18)
#define EEPROM_SENSOR_MODE_ADDR 13        // 1 byte (0=US, 1=WP, 2=Dual)
#define EEPROM_WP_TANK_HEIGHT_ADDR 14     // 2 bytes (cm)
#define EEPROM_WP_MIN_WATER_LEVEL_ADDR 16 // 2 bytes (cm)
#define EEPROM_WP_VALID_ADDR 18           // 1 byte magic number
// UI Settings EEPROM addresses (19-86)
#define EEPROM_PASSWORD_PROT_ADDR 19      // 1 byte (0=disabled, 1=enabled)
#define EEPROM_AP_INFINITE_ADDR 20        // 1 byte (0=normal, 1=infinite)
#define EEPROM_WIFI_SSID_ADDR 21          // 32 bytes for WiFi SSID
#define EEPROM_WIFI_PASS_ADDR 53          // 32 bytes for WiFi Password
#define EEPROM_WIFI_ENABLED_ADDR 85       // 1 byte (0=disabled, 1=enabled)
#define EEPROM_UI_SETTINGS_VALID_ADDR 86  // 1 byte magic number
// Calibration EEPROM addresses (87-96)
#define EEPROM_CAL_OFFSET_ADDR 87         // 4 bytes (float offset)
#define EEPROM_CAL_SLOPE_ADDR 91          // 4 bytes (float slope)
#define EEPROM_CAL_ENABLED_ADDR 95        // 1 byte (0=disabled, 1=enabled)
#define EEPROM_CAL_VALID_ADDR 96          // 1 byte magic number
#define EEPROM_MAGIC_NUMBER 0xAB
#define EEPROM_DEVICE_MAGIC 0xCD
#define EEPROM_WP_MAGIC 0xEF
#define EEPROM_UI_SETTINGS_MAGIC 0xDE
#define EEPROM_CAL_MAGIC 0xBC

// Dual mode delay between US and WP packets (fixed)
#define DUAL_MODE_DELAY 1500              // 1.5 seconds

// Session ID and Packet Counter
#define COUNTER_RESET_LIMIT 65000
unsigned long SESSION_ID = 0;
unsigned long PACKET_COUNTER = 0;

// Tank zone enumeration
enum TankZone {
  NO_ZONE = 0,
  UPPER_ZONE = 1,
  MIDDLE_ZONE = 2,
  LOWER_ZONE = 3
};

// Sensor mode enumeration
enum SensorMode {
  MODE_US_ONLY = 0,    // Default: Send distance only
  MODE_WP_ONLY = 1,    // Send water presence only
  MODE_DUAL = 2        // Send both US and WP packets
};

// Global variables
TankZone lastStoredZone = NO_ZONE;
bool hasValidHistory = false;
unsigned long transmissionInterval;
char deviceID[12] = "US0000";             // Device ID (e.g., "US3847")
String apSSID = "";                        // WiFi AP name
bool apModeActive = false;
unsigned long apStartTime = 0;
unsigned long apTimeoutTime = 0;
unsigned long lastWebActivity = 0;
float lastReading = -1;
float lastRawReading = -1;                 // Raw reading before calibration (for UI display)
unsigned long lastLoRaTransmit = 0;
bool loraInitialized = false;

// Sensor Mode variables
SensorMode sensorMode = MODE_US_ONLY;     // Current sensor mode
uint16_t wpTankHeight = 200;              // Tank height in cm (default 200)
uint16_t wpMinWaterLevel = 60;            // Minimum water level threshold (default 60cm)
#define WP_MIN_TANK_HEIGHT 50             // Minimum tank height allowed
#define WP_MAX_TANK_HEIGHT 450            // Maximum tank height allowed
#define WP_DEFAULT_MIN_LEVEL 60           // Default minimum water level

// UI Settings variables (runtime configurable, saved to EEPROM)
bool passwordProtectionEnabled = true;    // Default: password required
bool apInfiniteModeEnabled = false;       // Default: normal timeout
bool wifiStationEnabled = false;          // Default: not connected to external WiFi
char wifiSSID[33] = "";                   // External WiFi SSID
char wifiPassword[33] = "";               // External WiFi Password
bool wifiConnected = false;               // Connection status
bool mdnsStarted = false;                 // mDNS service status
unsigned long lastWifiCheck = 0;          // Last WiFi status check time
#define WIFI_CHECK_INTERVAL 5000          // Check WiFi status every 5 seconds
#define WIFI_CONNECT_TIMEOUT 10000        // WiFi connection timeout (10 seconds)

// Calibration variables
bool calibrationEnabled = false;          // Use custom calibration
float calOffset = -0.456079;              // User offset (a)
float calSlope = 1.0301;                  // User slope (b)
#define CAL_DEFAULT_OFFSET -0.456079      // Default offset (hardcoded fallback)
#define CAL_DEFAULT_SLOPE 1.0301          // Default slope (hardcoded fallback)

// Web server and OTA
WebServer server(WEB_PORT);
MvsOTA mvsota;

// Function prototypes
void loadDeviceID();
void saveDeviceID();
void generateRandomDeviceID();
void setupAccessPoint();
void stopAccessPoint();
void setupWebServer();
void handleRoot();
void handleLogin();
void handleDashboard();
void handleSaveAll();
void handleStatus();
void handleGenerateID();
void handleNotFound();
bool checkUserAgent();
void extendAPTimeout();
float readDistanceCM();
void transmitLoRa();
void loadZoneFromEEPROM();
void resetEEPROM();
void updateZoneIfChanged(float calibratedReading);
TankZone getCurrentZone(float calibratedReading);
float applyCalibration(float rawReading);
void sortReadings(float arr[], int n);
int getDeviceNumber();
String getUptimeString();
// Sensor Mode functions
void loadSensorModeSettings();
void saveSensorModeSettings();
String getUSNodeID();
String getWPNodeID();
String getModeString();
void transmitUSPacket(float distance);
void transmitWPPacket(float distance);
// UI Settings functions
void loadUISettings();
void saveUISettings();
void connectToWiFiStation();
void checkWiFiConnection();
// Calibration functions
void loadCalibrationSettings();
void saveCalibrationSettings();
// mDNS functions
void startMDNS();
void stopMDNS();

void setup() {
  Serial.begin(115200);
  delay(1000);

  #if DEBUG_MODE
  Serial.println(F("\n========================================="));
  Serial.println(F("ESP32 Ultrasonic LoRa Transmitter"));
  Serial.print(F("Firmware: "));
  Serial.println(FIRMWARE_VERSION);
  Serial.println(F("=========================================\n"));
  #endif

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Initialize random seed
  randomSeed(analogRead(0) + millis());

  // Load or generate Device ID
  loadDeviceID();

  // Load Sensor Mode settings
  loadSensorModeSettings();

  // Load UI settings (password protection, AP mode, WiFi)
  loadUISettings();

  // Load Calibration settings
  loadCalibrationSettings();

  // Initialize session ID with random offset to ensure uniqueness on reset
  SESSION_ID = millis() + (unsigned long)esp_random();

  #if DEBUG_MODE
  Serial.print(F("Session ID: "));
  Serial.println(SESSION_ID);
  Serial.print(F("Packet Counter: "));
  Serial.println(PACKET_COUNTER);
  #endif

  // Calculate transmission interval
  transmissionInterval = BASE_INTERVAL + (getDeviceNumber() * DEVICE_OFFSET);

  // Load zone from EEPROM
  loadZoneFromEEPROM();

  // Setup WiFi Access Point
  setupAccessPoint();

  // Initialize mvsOTA
  mvsota.begin(deviceID, FIRMWARE_VERSION);
  mvsota.onStart([]() {
    Serial.println(F("OTA Update starting..."));
  });

  // Setup Web Server
  setupWebServer();

  // Initialize LoRa
  #if DEBUG_MODE
  Serial.println(F("Initializing LoRa..."));
  #endif
  SPI.begin();
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (LoRa.begin(433E6)) {
    LoRa.setTxPower(20);
    LoRa.setSpreadingFactor(10);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(8);
    LoRa.setPreambleLength(8);
    LoRa.setSyncWord(0x12);
    loraInitialized = true;
    #if DEBUG_MODE
    Serial.println(F("LoRa Ready!"));
    #endif
  } else {
    Serial.println(F("LoRa init failed!"));  // Always show errors
  }

  #if DEBUG_MODE
  Serial.println(F("\n--- Configuration ---"));
  Serial.print(F("Device ID: "));
  Serial.println(deviceID);
  Serial.print(F("AP SSID: "));
  Serial.println(apSSID);
  Serial.print(F("Web UI: http://192.168.4.1:"));
  Serial.println(WEB_PORT);
  Serial.print(F("Transmission Interval: "));
  Serial.print(transmissionInterval);
  Serial.println(F("ms"));
  Serial.println(F("---------------------\n"));
  #endif
}

void loop() {
  unsigned long currentMillis = millis();

  // Handle web server
  server.handleClient();

  // Handle OTA
  if (!mvsota.isUpdating()) {
    mvsota.handle();
  }

  // Check AP timeout (skip if infinite mode enabled - runtime check)
  if (!apInfiniteModeEnabled && apModeActive && currentMillis > apTimeoutTime) {
    #if DEBUG_MODE
    Serial.println(F("AP timeout - stopping Access Point"));
    #endif
    stopAccessPoint();
  }

  // Check WiFi connection status
  checkWiFiConnection();

  // LoRa transmission cycle (every transmissionInterval)
  if (loraInitialized && !mvsota.isUpdating() &&
      (currentMillis - lastLoRaTransmit >= transmissionInterval)) {
    transmitLoRa();
    lastLoRaTransmit = currentMillis;
  }

  delay(10);  // Small delay for stability
}

// ============================================
// DEVICE ID MANAGEMENT
// ============================================

void loadDeviceID() {
  if (EEPROM.read(EEPROM_DEVICE_ID_VALID_ADDR) == EEPROM_DEVICE_MAGIC) {
    // Load saved device ID
    for (int i = 0; i < 10; i++) {
      deviceID[i] = EEPROM.read(EEPROM_DEVICE_ID_ADDR + i);
      if (deviceID[i] == 0) break;
    }
    deviceID[10] = '\0';

    // Validate loaded ID format: must start with "US" and have 4 digits
    bool validFormat = (deviceID[0] == 'U' && deviceID[1] == 'S' && strlen(deviceID) == 6);
    if (validFormat) {
      for (int i = 2; i < 6; i++) {
        if (deviceID[i] < '0' || deviceID[i] > '9') {
          validFormat = false;
          break;
        }
      }
    }

    if (validFormat) {
      #if DEBUG_MODE
      Serial.print(F("Loaded Device ID: "));
      Serial.println(deviceID);
      #endif
    } else {
      // Invalid format - regenerate
      #if DEBUG_MODE
      Serial.println(F("Invalid Device ID format - regenerating"));
      #endif
      generateRandomDeviceID();
      saveDeviceID();
    }
  } else {
    // Generate random device ID
    #if DEBUG_MODE
    Serial.println(F("No Device ID found - generating random ID"));
    #endif
    generateRandomDeviceID();
    saveDeviceID();
  }
}

void saveDeviceID() {
  for (int i = 0; i < 10; i++) {
    EEPROM.write(EEPROM_DEVICE_ID_ADDR + i, deviceID[i]);
  }
  EEPROM.write(EEPROM_DEVICE_ID_VALID_ADDR, EEPROM_DEVICE_MAGIC);
  EEPROM.commit();
  #if DEBUG_MODE
  Serial.print(F("Saved Device ID: "));
  Serial.println(deviceID);
  #endif
}

void generateRandomDeviceID() {
  // Generate US + 4 random digits (e.g., US3847)
  int randomNum = random(1000, 9999);
  snprintf(deviceID, sizeof(deviceID), "US%04d", randomNum);
  #if DEBUG_MODE
  Serial.print(F("Generated Device ID: "));
  Serial.println(deviceID);
  #endif
}

int getDeviceNumber() {
  // Extract number from device ID (US3847 -> 3847, then mod 100 for interval offset)
  int num = 0;
  for (int i = 2; deviceID[i] != '\0' && i < 6; i++) {
    if (deviceID[i] >= '0' && deviceID[i] <= '9') {
      num = num * 10 + (deviceID[i] - '0');
    }
  }
  return num % 100;  // Return 0-99 for interval offset
}

// ============================================
// WIFI ACCESS POINT
// ============================================

void setupAccessPoint() {
  apSSID = String(deviceID) + AP_SUFFIX;

  // Use AP+STA mode if WiFi station is enabled, otherwise AP only
  if (wifiStationEnabled && strlen(wifiSSID) > 0) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_AP);
  }
  WiFi.softAP(apSSID.c_str(), AP_PASSWORD);

  apModeActive = true;
  apStartTime = millis();
  apTimeoutTime = apStartTime + AP_DEFAULT_TIMEOUT_MS;

  #if DEBUG_MODE
  Serial.println(F("\n--- WiFi Access Point Started ---"));
  Serial.print(F("SSID: "));
  Serial.println(apSSID);
  Serial.print(F("Password: "));
  Serial.println(AP_PASSWORD);
  Serial.print(F("IP: "));
  Serial.println(WiFi.softAPIP());
  Serial.print(F("Timeout: "));
  Serial.print(apInfiniteModeEnabled ? "Infinite" : String(AP_DEFAULT_TIMEOUT_MS / 1000).c_str());
  Serial.println(apInfiniteModeEnabled ? "" : " seconds");
  Serial.println(F("---------------------------------\n"));
  #endif

  // Connect to external WiFi if enabled
  if (wifiStationEnabled && strlen(wifiSSID) > 0) {
    connectToWiFiStation();
  }
}

void stopAccessPoint() {
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  apModeActive = false;
  #if DEBUG_MODE
  Serial.println(F("Access Point stopped"));
  #endif
}

void extendAPTimeout() {
  if (apModeActive) {
    unsigned long newTimeout = millis() + AP_EXTEND_TIME_MS;
    unsigned long maxTimeout = apStartTime + AP_MAX_TIME_MS;

    if (newTimeout > maxTimeout) {
      newTimeout = maxTimeout;
    }

    if (newTimeout > apTimeoutTime) {
      apTimeoutTime = newTimeout;
      #if DEBUG_MODE
      Serial.print(F("AP timeout extended. Remaining: "));
      Serial.print((apTimeoutTime - millis()) / 1000);
      Serial.println(F(" seconds"));
      #endif
    }
  }
}

// ============================================
// WEB SERVER
// ============================================

bool checkUserAgent() {
  if (server.hasHeader("User-Agent")) {
    String ua = server.header("User-Agent");
    return (ua.indexOf(APP_USER_AGENT) >= 0);
  }
  return false;
}

// Headers to collect for User-Agent check
const char* headerKeys[] = {"User-Agent"};
const size_t headerKeysCount = 1;

void setupWebServer() {
  server.collectHeaders(headerKeys, headerKeysCount);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/login", HTTP_POST, handleLogin);
  server.on("/dashboard", HTTP_GET, handleDashboard);
  server.on("/saveall", HTTP_POST, handleSaveAll);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/generate", HTTP_GET, handleGenerateID);
  server.onNotFound(handleNotFound);

  server.begin();
  #if DEBUG_MODE
  Serial.print(F("Web server started on port "));
  Serial.println(WEB_PORT);
  #endif
}

void handleRoot() {
  if (!checkUserAgent()) {
    server.send(403, "text/plain", "Access Denied");
    return;
  }
  extendAPTimeout();

  // Runtime password protection check
  if (!passwordProtectionEnabled) {
    // Password protection disabled - redirect directly to dashboard
    server.sendHeader("Location", "/dashboard");
    server.send(302, "text/plain", "");
    return;
  }

  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Sensor Login</title>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;background:#1a1a2e;margin:0;padding:20px;display:flex;justify-content:center;align-items:center;min-height:100vh}";
  html += ".container{background:#16213e;padding:30px;border-radius:10px;box-shadow:0 4px 20px rgba(0,0,0,0.3);width:100%;max-width:350px}";
  html += "h1{color:#e94560;text-align:center;margin-bottom:30px;font-size:1.5em}";
  html += ".form-group{margin-bottom:20px}";
  html += "label{display:block;color:#fff;margin-bottom:8px;font-weight:bold}";
  html += "input[type=password]{width:100%;padding:12px;border:2px solid #0f3460;border-radius:6px;background:#1a1a2e;color:#fff;font-size:1.1em;box-sizing:border-box}";
  html += "input:focus{border-color:#e94560;outline:none}";
  html += ".btn{width:100%;padding:14px;background:#e94560;color:#fff;border:none;border-radius:6px;font-size:1.1em;cursor:pointer;font-weight:bold}";
  html += ".btn:hover{background:#ff6b6b}";
  html += ".info{color:#aaa;text-align:center;margin-top:20px;font-size:0.9em}";
  html += "</style></head><body>";
  html += "<div class='container'>";
  html += "<h1>US Sensor Config</h1>";
  html += "<form action='/login' method='POST'>";
  html += "<div class='form-group'>";
  html += "<label>Password</label>";
  html += "<input type='password' name='password' placeholder='Enter password' required autofocus>";
  html += "</div>";
  html += "<button type='submit' class='btn'>Login</button>";
  html += "</form>";
  html += "<div class='info'>Device: " + String(deviceID) + "<br>v" + FIRMWARE_VERSION + "</div>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

void handleLogin() {
  if (!checkUserAgent()) {
    server.send(403, "text/plain", "Access Denied");
    return;
  }
  extendAPTimeout();

  if (server.hasArg("password") && server.arg("password") == WEB_PASSWORD) {
    server.sendHeader("Location", "/dashboard");
    server.send(302, "text/plain", "");
  } else {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>Login Failed</title>";
    html += "<style>body{font-family:Arial;background:#1a1a2e;color:#fff;text-align:center;padding:50px}";
    html += ".error{background:#e94560;padding:20px;border-radius:10px;max-width:300px;margin:0 auto}";
    html += "a{color:#4fc3f7;margin-top:20px;display:block}</style></head><body>";
    html += "<div class='error'><h2>Invalid Password</h2></div>";
    html += "<a href='/'>Try Again</a></body></html>";
    server.send(401, "text/html", html);
  }
}

void handleDashboard() {
  if (!checkUserAgent()) {
    server.send(403, "text/plain", "Access Denied");
    return;
  }
  extendAPTimeout();

  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Sensor Dashboard</title>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;background:#1a1a2e;margin:0;padding:20px}";
  html += ".container{background:#16213e;padding:20px;border-radius:10px;max-width:500px;margin:0 auto}";
  html += "h1{color:#e94560;text-align:center;font-size:1.4em;margin-bottom:20px}";
  html += ".card{background:#1a1a2e;padding:15px;border-radius:8px;margin-bottom:15px}";
  html += ".card h3{color:#4fc3f7;margin:0 0 10px 0;font-size:1em}";
  html += ".info-row{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #0f3460}";
  html += ".info-row:last-child{border-bottom:none}";
  html += ".label{color:#aaa}";
  html += ".value{color:#fff;font-weight:bold}";
  html += ".form-group{margin-bottom:15px}";
  html += "label{display:block;color:#fff;margin-bottom:5px}";
  html += "input[type=text],input[type=number]{width:100%;padding:10px;border:2px solid #0f3460;border-radius:6px;background:#1a1a2e;color:#fff;font-size:1em;box-sizing:border-box}";
  html += "input:focus{border-color:#e94560;outline:none}";
  html += ".btn{padding:12px 20px;border:none;border-radius:6px;font-size:1em;cursor:pointer;font-weight:bold;margin-right:10px;margin-bottom:10px}";
  html += ".btn-primary{background:#e94560;color:#fff}";
  html += ".btn-secondary{background:#0f3460;color:#fff}";
  html += ".btn:hover{opacity:0.9}";
  html += ".btn-group{margin-top:15px}";
  html += ".sticky-btn{position:fixed;bottom:0;left:0;right:0;background:#16213e;padding:15px 20px;box-shadow:0 -4px 10px rgba(0,0,0,0.5);z-index:100}";
  html += ".prefix{color:#4fc3f7;font-weight:bold}";
  html += ".success{background:#27ae60;color:#fff;padding:10px;border-radius:6px;text-align:center;margin-bottom:15px}";
  html += ".error{background:#e74c3c;color:#fff;padding:10px;border-radius:6px;text-align:center;margin-bottom:15px}";
  html += ".note{color:#aaa;font-size:0.85em;margin-top:5px}";
  // Toggle switch styles
  html += ".toggle-row{display:flex;justify-content:space-between;align-items:center;padding:10px 0}";
  html += ".toggle{position:relative;width:50px;height:26px}";
  html += ".toggle input{opacity:0;width:0;height:0}";
  html += ".slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#0f3460;border-radius:26px;transition:.3s}";
  html += ".slider:before{position:absolute;content:'';height:20px;width:20px;left:3px;bottom:3px;background:#fff;border-radius:50%;transition:.3s}";
  html += "input:checked+.slider{background:#27ae60}";
  html += "input:checked+.slider:before{transform:translateX(24px)}";
  // Tank visualization styles
  html += ".tank-config{display:flex;flex-direction:column;align-items:center;margin:15px 0}";
  html += ".tank-visual{position:relative;width:100px;height:140px;margin-bottom:10px}";
  html += ".tank-cap{width:55px;height:8px;background:linear-gradient(135deg,#34495e,#2c3e50);border-radius:50% 50% 0 0;margin:0 auto;box-shadow:0 2px 4px rgba(0,0,0,0.2)}";
  html += ".tank-top{width:100px;height:25px;background:linear-gradient(to bottom,#e8eef1,#d5dbdb);clip-path:polygon(15% 0,85% 0,100% 100%,0 100%);margin:0 auto;border-left:2px solid #34495e;border-right:2px solid #34495e}";
  html += ".tank-body{width:100px;height:110px;background:linear-gradient(to bottom,#ecf0f1,#d5dbdb);border:2px solid #34495e;border-top:none;border-radius:0 0 12px 12px;position:relative;overflow:hidden;margin:0 auto}";
  html += ".water-fill{position:absolute;bottom:0;width:100%;background:linear-gradient(to top,#1565c0,#42a5f5);transition:height 0.3s}";
  html += ".level-line{position:absolute;width:100%;height:3px;background:#e74c3c;left:0;box-shadow:0 0 6px rgba(231,76,60,0.8)}";
  html += ".level-label{position:absolute;right:-55px;transform:translateY(-50%);font-size:0.7em;color:#e74c3c;font-weight:bold;white-space:nowrap}";
  // Slider styles
  html += ".slider-group{width:100%;margin-top:10px}";
  html += ".slider-label{display:flex;justify-content:space-between;color:#fff;font-size:0.9em;margin-bottom:5px}";
  html += ".slider-val{color:#4fc3f7;font-weight:bold}";
  html += "input[type=range]{width:100%;height:8px;border-radius:4px;background:#0f3460;outline:none;-webkit-appearance:none}";
  html += "input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;border-radius:50%;background:#3498db;cursor:pointer}";
  html += ".wp-info{background:#0f3460;padding:10px;border-radius:6px;margin-top:10px;font-size:0.85em;color:#aaa}";
  html += ".wp-info strong{color:#4fc3f7}";
  // Radio button styles
  html += ".radio-group{display:flex;flex-direction:column;gap:10px;margin-bottom:15px}";
  html += ".radio-option{display:flex;align-items:center;padding:12px;background:#0f3460;border-radius:8px;cursor:pointer;transition:all 0.2s}";
  html += ".radio-option:hover{background:#1a4a7a}";
  html += ".radio-option.selected{background:#27ae60;border:2px solid #2ecc71}";
  html += ".radio-option input{display:none}";
  html += ".radio-circle{width:20px;height:20px;border:2px solid #fff;border-radius:50%;margin-right:12px;position:relative}";
  html += ".radio-option.selected .radio-circle:after{content:'';position:absolute;top:4px;left:4px;width:8px;height:8px;background:#fff;border-radius:50%}";
  html += ".radio-text{flex:1}";
  html += ".radio-title{color:#fff;font-weight:bold;font-size:0.95em}";
  html += ".radio-desc{color:#aaa;font-size:0.8em;margin-top:2px}";
  // Calibration styles
  html += ".cal-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}";
  html += ".cal-result{background:#0f3460;padding:10px;border-radius:6px;margin-top:10px;font-size:0.9em}";
  html += ".cal-result span{color:#4fc3f7}";
  html += "</style></head><body>";
  html += "<div class='container'>";

  // Single form for all settings
  html += "<form id='mainForm' action='/saveall' method='POST'>";

  html += "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:15px'>";
  html += "<h1 style='margin:0'>Sensor Configuration</h1>";
  html += "</div>";

  // Check for success/error messages
  if (server.hasArg("saved")) {
    html += "<div class='success'>All Settings Saved!</div>";
  }
  if (server.hasArg("error")) {
    html += "<div class='error'>Please enter Tank Height before enabling WP Mode!</div>";
  }

  // Status Card (values updated via AJAX)
  html += "<div class='card'>";
  html += "<h3>Status <span style='font-size:0.8em;color:#aaa'>(auto-refresh)</span></h3>";
  String displayNodeID;
  if (sensorMode == MODE_US_ONLY) {
    displayNodeID = getUSNodeID();
  } else if (sensorMode == MODE_WP_ONLY) {
    displayNodeID = getWPNodeID();
  } else {
    displayNodeID = getUSNodeID() + " / " + getWPNodeID();
  }
  html += "<div class='info-row'><span class='label'>Node ID</span><span class='value'>" + displayNodeID + "</span></div>";
  html += "<div class='info-row'><span class='label'>Mode</span><span class='value'>" + getModeString() + "</span></div>";
  html += "<div class='info-row'><span class='label'>Firmware</span><span class='value'>v" + String(FIRMWARE_VERSION) + "</span></div>";
  html += "<div class='info-row'><span class='label'>LoRa</span><span class='value'>" + String(loraInitialized ? "OK" : "Failed") + "</span></div>";
  html += "<div class='info-row'><span class='label'>Last Reading</span><span class='value' id='lastReading'>" + (lastReading > 0 ? String(lastReading, 1) + " cm" : "No data") + "</span></div>";
  html += "<div class='info-row'><span class='label'>Uptime</span><span class='value' id='uptime'>" + getUptimeString() + "</span></div>";
  html += "<div class='info-row'><span class='label'>AP Timeout</span><span class='value' id='apTimeout'>" + (apInfiniteModeEnabled ? "Always On" : String((apTimeoutTime - millis()) / 1000) + "s") + "</span></div>";
  html += "<div class='info-row'><span class='label'>WiFi Status</span><span class='value' id='wifiStatus'>";
  if (wifiStationEnabled && wifiConnected) {
    html += "Connected (" + WiFi.localIP().toString() + ")";
  } else if (wifiStationEnabled) {
    html += "Not Connected";
  } else {
    html += "Disabled";
  }
  html += "</span></div>";
  html += "</div>";

  // Device ID Configuration Card
  html += "<div class='card'>";
  html += "<h3>Device ID</h3>";
  html += "<div class='form-group'>";
  html += "<label>Device ID <span class='prefix'>(US + 4 digits)</span></label>";
  html += "<input type='text' name='deviceId' id='deviceId' value='" + getUSNodeID() + "' maxlength='10' pattern='US[0-9]{4}' required>";
  html += "<div class='note'>Format: US followed by 4 digits (e.g., US3847)</div>";
  html += "</div>";
  html += "<button type='button' class='btn btn-secondary' onclick='generateID()'>Random ID</button>";
  html += "</div>";

  // Sensor Mode Card
  html += "<div class='card'>";
  html += "<h3>Sensor Mode</h3>";
  html += "<div class='radio-group'>";
  // US Only
  html += "<label class='radio-option" + String(sensorMode == MODE_US_ONLY ? " selected" : "") + "' onclick='selectMode(0)'>";
  html += "<input type='radio' name='sensorMode' value='0'" + String(sensorMode == MODE_US_ONLY ? " checked" : "") + ">";
  html += "<div class='radio-circle'></div>";
  html += "<div class='radio-text'><div class='radio-title'>US Only (Default)</div>";
  html += "<div class='radio-desc'>Sends distance in cm as USxxxx</div></div></label>";
  // WP Only
  html += "<label class='radio-option" + String(sensorMode == MODE_WP_ONLY ? " selected" : "") + "' onclick='selectMode(1)'>";
  html += "<input type='radio' name='sensorMode' value='1'" + String(sensorMode == MODE_WP_ONLY ? " checked" : "") + ">";
  html += "<div class='radio-circle'></div>";
  html += "<div class='radio-text'><div class='radio-title'>WP Only</div>";
  html += "<div class='radio-desc'>Sends water presence (0/1) as WPxxxx</div></div></label>";
  // Dual Mode
  html += "<label class='radio-option" + String(sensorMode == MODE_DUAL ? " selected" : "") + "' onclick='selectMode(2)'>";
  html += "<input type='radio' name='sensorMode' value='2'" + String(sensorMode == MODE_DUAL ? " checked" : "") + ">";
  html += "<div class='radio-circle'></div>";
  html += "<div class='radio-text'><div class='radio-title'>US + WP (Dual)</div>";
  html += "<div class='radio-desc'>Sends both packets</div></div></label>";
  html += "</div>";

  // WP Settings
  bool showWPSettings = (sensorMode == MODE_WP_ONLY || sensorMode == MODE_DUAL);
  html += "<div id='wpSettings' style='display:" + String(showWPSettings ? "block" : "none") + "'>";
  html += "<div class='form-group'>";
  html += "<label>Tank Height (cm)</label>";
  html += "<input type='number' id='tankHeight' name='tankHeight' min='50' max='450' value='" + String(wpTankHeight) + "' onchange='updateTankVisual()'>";
  html += "</div>";
  // Tank Visualization
  html += "<div class='tank-config'>";
  html += "<div class='tank-visual'><div class='tank-cap'></div><div class='tank-top'></div>";
  html += "<div class='tank-body'>";
  int fillPercent = (wpTankHeight > 0) ? (wpMinWaterLevel * 100 / wpTankHeight) : 30;
  html += "<div class='water-fill' id='waterFill' style='height:" + String(fillPercent) + "%'></div>";
  html += "<div class='level-line' id='levelLine' style='bottom:" + String(fillPercent) + "%'>";
  html += "<span class='level-label' id='levelLabel'>" + String(wpMinWaterLevel) + " cm</span></div></div></div>";
  html += "<div class='slider-group'>";
  html += "<div class='slider-label'><span>Min Water Level</span><span class='slider-val' id='minLevelVal'>" + String(wpMinWaterLevel) + " cm</span></div>";
  html += "<input type='range' id='minLevelSlider' name='minWaterLevel' min='10' max='" + String(wpTankHeight > 0 ? wpTankHeight : 200) + "' value='" + String(wpMinWaterLevel) + "' oninput='updateMinLevel(this.value)'>";
  html += "</div></div></div>";
  html += "</div>";

  // Calibration Card
  html += "<div class='card'>";
  html += "<h3>Calibration</h3>";
  // Live reading display - show raw reading (before calibration)
  html += "<div class='info-row'><span class='label'>Raw Reading</span><span class='value' id='rawReading'>" + (lastRawReading > 0 ? String(lastRawReading, 1) + " cm" : "No data") + "</span></div>";
  float calibratedPreview = (lastRawReading > 21 && lastRawReading <= 450) ? (calOffset + calSlope * lastRawReading) : lastRawReading;
  html += "<div class='info-row'><span class='label'>Calibrated Preview</span><span class='value' id='calPreview'>" + (lastRawReading > 0 ? String(calibratedPreview, 1) + " cm" : "No data") + "</span></div>";
  // Direct offset/slope adjustment
  html += "<div style='margin-top:15px'><label style='color:#fff;margin-bottom:10px;display:block'>Manual Adjustment</label>";
  html += "<div class='cal-grid'>";
  html += "<div class='form-group'><label>Offset (a)</label><input type='number' step='any' name='calOffset' id='calOffsetIn' value='" + String(calOffset, 6) + "' onchange='updatePreview()'></div>";
  html += "<div class='form-group'><label>Slope (b)</label><input type='number' step='any' name='calSlope' id='calSlopeIn' value='" + String(calSlope, 6) + "' onchange='updatePreview()'></div>";
  html += "</div></div>";
  html += "<div class='cal-result'>Formula: <span id='calFormula'>" + String(calOffset, 3) + " + " + String(calSlope, 4) + " x raw</span></div>";
  // Auto calculate section
  html += "<div style='margin-top:15px;padding-top:15px;border-top:1px solid #0f3460'>";
  html += "<label style='color:#fff;margin-bottom:10px;display:block'>Auto Calculate from Actual</label>";
  html += "<div class='form-group'><label>Actual Distance (cm)</label><input type='number' id='actualDist' placeholder='Enter actual distance'></div>";
  html += "<div style='display:flex;gap:15px;margin:10px 0'>";
  html += "<label style='color:#aaa;display:flex;align-items:center;gap:5px'><input type='radio' name='adjustType' value='offset' checked style='width:auto'> Adjust Offset</label>";
  html += "<label style='color:#aaa;display:flex;align-items:center;gap:5px'><input type='radio' name='adjustType' value='slope' style='width:auto'> Adjust Slope</label>";
  html += "</div>";
  html += "<div id='calcResult' style='background:#0f3460;padding:10px;border-radius:6px;margin:10px 0;display:none'></div>";
  html += "<button type='button' class='btn btn-secondary' onclick='autoCalc()'>Calculate</button>";
  html += "<button type='button' class='btn btn-secondary' onclick='applyCalc()' style='margin-left:10px;display:none' id='applyBtn'>Apply</button>";
  html += "</div>";
  // Enable toggle and reset
  html += "<div class='toggle-row' style='margin-top:15px;padding-top:15px;border-top:1px solid #0f3460'>";
  html += "<div><span class='label' style='color:#fff'>Enable Custom Calibration</span><br><span class='note'>If disabled, uses default: " + String(CAL_DEFAULT_OFFSET, 3) + " + " + String(CAL_DEFAULT_SLOPE, 4) + " x raw</span></div>";
  html += "<label class='toggle'><input type='checkbox' name='calEnabled' value='1'" + String(calibrationEnabled ? " checked" : "") + ">";
  html += "<span class='slider'></span></label></div>";
  html += "<button type='button' class='btn btn-secondary' onclick='resetCalibration()' style='margin-top:10px'>Reset to Default</button>";
  html += "</div>";

  // System Settings Card
  html += "<div class='card'>";
  html += "<h3>System Settings</h3>";
  // Password Protection
  html += "<div class='toggle-row'>";
  html += "<div><span class='label' style='color:#fff'>Password Protection</span><br><span class='note'>Require login to access dashboard</span></div>";
  html += "<label class='toggle'><input type='checkbox' name='passwordProt' value='1'" + String(passwordProtectionEnabled ? " checked" : "") + ">";
  html += "<span class='slider'></span></label></div>";
  // AP Always On
  html += "<div class='toggle-row'>";
  html += "<div><span class='label' style='color:#fff'>AP Always On</span><br><span class='note'>Keep WiFi AP active indefinitely</span></div>";
  html += "<label class='toggle'><input type='checkbox' name='apInfinite' value='1'" + String(apInfiniteModeEnabled ? " checked" : "") + ">";
  html += "<span class='slider'></span></label></div>";
  // WiFi Connection
  html += "<div class='toggle-row' style='border-bottom:1px solid #0f3460;padding-bottom:15px'>";
  html += "<div><span class='label' style='color:#fff'>WiFi Connection</span><br><span class='note'>Connect to external WiFi</span></div>";
  html += "<label class='toggle'><input type='checkbox' name='wifiEnabled' value='1' id='wifiToggle' onchange='toggleWifiSettings()'" + String(wifiStationEnabled ? " checked" : "") + ">";
  html += "<span class='slider'></span></label></div>";
  // WiFi fields
  html += "<div id='wifiFields' style='display:" + String(wifiStationEnabled ? "block" : "none") + "'>";
  html += "<div class='form-group'><label>WiFi SSID</label>";
  html += "<input type='text' name='wifiSSID' value='" + String(wifiSSID) + "' maxlength='32' placeholder='WiFi network name'></div>";
  html += "<div class='form-group'><label>WiFi Password</label>";
  html += "<input type='text' name='wifiPass' value='" + String(wifiPassword) + "' maxlength='32' placeholder='WiFi password'></div>";
  html += "</div>";
  html += "</div>";

  html += "</form>";
  html += "</div>";

  // Sticky Save All Button
  html += "<div class='sticky-btn'>";
  html += "<button type='submit' form='mainForm' class='btn btn-primary' style='width:100%'>Save All Settings</button>";
  html += "</div>";
  html += "<div style='height:70px'></div>";

  // JavaScript
  html += "<script>";
  // Global raw value for calibration
  html += "var currentRaw=0;";
  html += "var calcOffset=0,calcSlope=0;";
  // AJAX status refresh
  html += "function refreshStatus(){";
  html += "fetch('/status').then(r=>r.json()).then(d=>{";
  html += "document.getElementById('lastReading').textContent=d.reading;";
  html += "document.getElementById('uptime').textContent=d.uptime;";
  html += "document.getElementById('apTimeout').textContent=d.apTimeout;";
  html += "document.getElementById('wifiStatus').textContent=d.wifiStatus;";
  html += "document.getElementById('rawReading').textContent=d.rawReading;";
  html += "currentRaw=d.rawValue||0;";
  html += "updatePreview();";
  html += "}).catch(e=>{});}";
  html += "setInterval(refreshStatus,5000);";

  // Generate random ID
  html += "function generateID(){fetch('/generate').then(r=>r.text()).then(id=>{document.getElementById('deviceId').value=id;});}";

  // Device ID input validation
  html += "document.getElementById('deviceId').addEventListener('input',function(){";
  html += "let v=this.value.toUpperCase();";
  html += "if(!v.startsWith('US'))v='US'+v.replace(/[^0-9]/g,'');";
  html += "else v='US'+v.substring(2).replace(/[^0-9]/g,'');";
  html += "if(v.length>6)v=v.substring(0,6);this.value=v;});";

  // Mode selection
  html += "function selectMode(mode){";
  html += "var radios=document.querySelectorAll('input[name=sensorMode]');";
  html += "var options=document.querySelectorAll('.radio-option');";
  html += "radios.forEach(function(r,i){r.checked=(i==mode);});";
  html += "options.forEach(function(o,i){o.classList.toggle('selected',i==mode);});";
  html += "document.getElementById('wpSettings').style.display=(mode>0)?'block':'none';}";

  // Tank visualization
  html += "function updateMinLevel(val){";
  html += "document.getElementById('minLevelVal').textContent=val+' cm';";
  html += "document.getElementById('levelLabel').textContent=val+' cm';";
  html += "var tankH=parseInt(document.getElementById('tankHeight').value)||200;";
  html += "var pct=(val/tankH)*100;";
  html += "document.getElementById('waterFill').style.height=pct+'%';";
  html += "document.getElementById('levelLine').style.bottom=pct+'%';}";

  html += "function updateTankVisual(){";
  html += "var tankH=parseInt(document.getElementById('tankHeight').value)||200;";
  html += "var slider=document.getElementById('minLevelSlider');";
  html += "slider.max=tankH;";
  html += "if(parseInt(slider.value)>tankH)slider.value=tankH;";
  html += "updateMinLevel(slider.value);}";

  // WiFi toggle
  html += "function toggleWifiSettings(){";
  html += "var toggle=document.getElementById('wifiToggle');";
  html += "document.getElementById('wifiFields').style.display=toggle.checked?'block':'none';}";

  // Update preview when offset/slope changes
  html += "function updatePreview(){";
  html += "var offset=parseFloat(document.getElementById('calOffsetIn').value)||0;";
  html += "var slope=parseFloat(document.getElementById('calSlopeIn').value)||1;";
  html += "document.getElementById('calFormula').textContent=offset.toFixed(3)+' + '+slope.toFixed(4)+' x raw';";
  html += "if(currentRaw>21&&currentRaw<=450){";
  html += "var cal=offset+(slope*currentRaw);";
  html += "document.getElementById('calPreview').textContent=cal.toFixed(1)+' cm';";
  html += "}}";

  // Auto calculate from actual
  html += "function autoCalc(){";
  html += "var actual=parseFloat(document.getElementById('actualDist').value);";
  html += "if(!actual||currentRaw<=0){alert('Enter actual distance and wait for raw reading');return;}";
  html += "var adjustType=document.querySelector('input[name=adjustType]:checked').value;";
  html += "var curOffset=parseFloat(document.getElementById('calOffsetIn').value)||0;";
  html += "var curSlope=parseFloat(document.getElementById('calSlopeIn').value)||1;";
  html += "var resultDiv=document.getElementById('calcResult');";
  html += "var applyBtn=document.getElementById('applyBtn');";
  html += "if(adjustType=='offset'){";
  html += "calcOffset=actual-(curSlope*currentRaw);calcSlope=curSlope;";
  html += "resultDiv.innerHTML='<span style=\"color:#4fc3f7\">New Offset: '+calcOffset.toFixed(6)+'</span><br>Slope stays: '+calcSlope.toFixed(6);";
  html += "}else{";
  html += "calcSlope=(actual-curOffset)/currentRaw;calcOffset=curOffset;";
  html += "resultDiv.innerHTML='Offset stays: '+calcOffset.toFixed(6)+'<br><span style=\"color:#4fc3f7\">New Slope: '+calcSlope.toFixed(6)+'</span>';";
  html += "}";
  html += "resultDiv.style.display='block';applyBtn.style.display='inline-block';}";

  // Apply calculated values
  html += "function applyCalc(){";
  html += "document.getElementById('calOffsetIn').value=calcOffset.toFixed(6);";
  html += "document.getElementById('calSlopeIn').value=calcSlope.toFixed(6);";
  html += "updatePreview();";
  html += "document.getElementById('calcResult').style.display='none';";
  html += "document.getElementById('applyBtn').style.display='none';}";

  // Reset calibration to defaults
  html += "function resetCalibration(){";
  html += "document.getElementById('calOffsetIn').value='" + String(CAL_DEFAULT_OFFSET, 6) + "';";
  html += "document.getElementById('calSlopeIn').value='" + String(CAL_DEFAULT_SLOPE, 6) + "';";
  html += "updatePreview();}";

  html += "</script>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// Status endpoint for AJAX polling (returns JSON)
void handleStatus() {
  if (!checkUserAgent()) {
    server.send(403, "text/plain", "Access Denied");
    return;
  }
  extendAPTimeout();

  String json = "{";
  json += "\"reading\":\"" + (lastReading > 0 ? String(lastReading, 1) + " cm" : "No data") + "\",";
  json += "\"rawValue\":" + (lastRawReading > 0 ? String(lastRawReading, 2) : "0") + ",";
  json += "\"rawReading\":\"" + (lastRawReading > 0 ? String(lastRawReading, 1) + " cm" : "No data") + "\",";
  json += "\"calibratedValue\":" + (lastReading > 0 ? String(lastReading, 2) : "0") + ",";
  json += "\"uptime\":\"" + getUptimeString() + "\",";
  json += "\"apTimeout\":\"" + (apInfiniteModeEnabled ? String("Always On") : String((apTimeoutTime - millis()) / 1000) + "s") + "\",";

  String wifiStatus;
  if (wifiStationEnabled && wifiConnected) {
    wifiStatus = "Connected (" + WiFi.localIP().toString() + ")";
  } else if (wifiStationEnabled) {
    wifiStatus = "Not Connected";
  } else {
    wifiStatus = "Disabled";
  }
  json += "\"wifiStatus\":\"" + wifiStatus + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

// Unified save handler for all settings
void handleSaveAll() {
  if (!checkUserAgent()) {
    server.send(403, "text/plain", "Access Denied");
    return;
  }
  extendAPTimeout();

  bool wasApInfinite = apInfiniteModeEnabled;
  bool wasWifiEnabled = wifiStationEnabled;
  String oldWifiSSID = String(wifiSSID);

  // === Device ID ===
  if (server.hasArg("deviceId")) {
    String newID = server.arg("deviceId");
    newID.trim();
    newID.toUpperCase();
    if (newID.length() >= 4 && newID.startsWith("US")) {
      strncpy(deviceID, newID.c_str(), sizeof(deviceID) - 1);
      deviceID[sizeof(deviceID) - 1] = '\0';
      saveDeviceID();
      apSSID = String(deviceID) + AP_SUFFIX;
      transmissionInterval = BASE_INTERVAL + (getDeviceNumber() * DEVICE_OFFSET);
    }
  }

  // === Sensor Mode ===
  SensorMode newMode = MODE_US_ONLY;
  if (server.hasArg("sensorMode")) {
    int modeVal = server.arg("sensorMode").toInt();
    if (modeVal >= 0 && modeVal <= 2) {
      newMode = (SensorMode)modeVal;
    }
  }

  // Validate WP mode requires tank height
  if ((newMode == MODE_WP_ONLY || newMode == MODE_DUAL) &&
      (!server.hasArg("tankHeight") || server.arg("tankHeight").toInt() < WP_MIN_TANK_HEIGHT)) {
    server.sendHeader("Location", "/dashboard?error=1");
    server.send(302, "text/plain", "");
    return;
  }

  sensorMode = newMode;

  if (server.hasArg("tankHeight")) {
    int newTankHeight = server.arg("tankHeight").toInt();
    if (newTankHeight >= WP_MIN_TANK_HEIGHT && newTankHeight <= WP_MAX_TANK_HEIGHT) {
      wpTankHeight = newTankHeight;
    }
  }

  if (server.hasArg("minWaterLevel")) {
    int newMinLevel = server.arg("minWaterLevel").toInt();
    if (newMinLevel >= 10 && newMinLevel <= wpTankHeight) {
      wpMinWaterLevel = newMinLevel;
    }
  }

  saveSensorModeSettings();

  // === Calibration ===
  if (server.hasArg("calOffset")) {
    calOffset = server.arg("calOffset").toFloat();
  }
  if (server.hasArg("calSlope")) {
    calSlope = server.arg("calSlope").toFloat();
    // Validate slope (must be positive and reasonable)
    if (calSlope <= 0 || calSlope > 10) {
      calSlope = CAL_DEFAULT_SLOPE;
    }
  }

  calibrationEnabled = server.hasArg("calEnabled");
  saveCalibrationSettings();

  // === System Settings ===
  passwordProtectionEnabled = server.hasArg("passwordProt");
  apInfiniteModeEnabled = server.hasArg("apInfinite");

  // Reset AP timeout if infinite mode was just turned OFF
  if (wasApInfinite && !apInfiniteModeEnabled) {
    apStartTime = millis();
    apTimeoutTime = apStartTime + AP_DEFAULT_TIMEOUT_MS;
  }

  wifiStationEnabled = server.hasArg("wifiEnabled");

  if (server.hasArg("wifiSSID")) {
    String newSSID = server.arg("wifiSSID");
    newSSID.trim();
    strncpy(wifiSSID, newSSID.c_str(), 32);
    wifiSSID[32] = '\0';
  }

  if (server.hasArg("wifiPass")) {
    String newPass = server.arg("wifiPass");
    strncpy(wifiPassword, newPass.c_str(), 32);
    wifiPassword[32] = '\0';
  }

  saveUISettings();

  // Handle WiFi mode changes
  if (wifiStationEnabled && (!wasWifiEnabled || oldWifiSSID != String(wifiSSID))) {
    if (strlen(wifiSSID) > 0) {
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP(apSSID.c_str(), AP_PASSWORD);
      connectToWiFiStation();
    }
  } else if (!wifiStationEnabled && wasWifiEnabled) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSSID.c_str(), AP_PASSWORD);
    wifiConnected = false;
  }

  #if DEBUG_MODE
  Serial.println(F("All settings saved"));
  #endif

  server.sendHeader("Location", "/dashboard?saved=1");
  server.send(302, "text/plain", "");
}

void handleGenerateID() {
  if (!checkUserAgent()) {
    server.send(403, "text/plain", "Access Denied");
    return;
  }
  extendAPTimeout();

  int randomNum = random(1000, 9999);
  char newID[12];
  // Always generate US prefix (WP prefix derived from it)
  snprintf(newID, sizeof(newID), "US%04d", randomNum);

  server.send(200, "text/plain", newID);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}

String getUptimeString() {
  unsigned long secs = millis() / 1000;
  unsigned long mins = secs / 60;
  unsigned long hours = mins / 60;

  secs %= 60;
  mins %= 60;

  char buf[20];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hours, mins, secs);
  return String(buf);
}

// ============================================
// UI SETTINGS MANAGEMENT
// ============================================

void loadUISettings() {
  if (EEPROM.read(EEPROM_UI_SETTINGS_VALID_ADDR) == EEPROM_UI_SETTINGS_MAGIC) {
    // Load password protection setting
    passwordProtectionEnabled = (EEPROM.read(EEPROM_PASSWORD_PROT_ADDR) == 1);

    // Load AP infinite mode setting
    apInfiniteModeEnabled = (EEPROM.read(EEPROM_AP_INFINITE_ADDR) == 1);

    // Load WiFi station enabled setting
    wifiStationEnabled = (EEPROM.read(EEPROM_WIFI_ENABLED_ADDR) == 1);

    // Load WiFi SSID
    for (int i = 0; i < 32; i++) {
      wifiSSID[i] = EEPROM.read(EEPROM_WIFI_SSID_ADDR + i);
    }
    wifiSSID[32] = '\0';

    // Load WiFi Password
    for (int i = 0; i < 32; i++) {
      wifiPassword[i] = EEPROM.read(EEPROM_WIFI_PASS_ADDR + i);
    }
    wifiPassword[32] = '\0';

    #if DEBUG_MODE
    Serial.println(F("UI Settings loaded:"));
    Serial.print(F("  Password Protection: "));
    Serial.println(passwordProtectionEnabled ? "ON" : "OFF");
    Serial.print(F("  AP Infinite Mode: "));
    Serial.println(apInfiniteModeEnabled ? "ON" : "OFF");
    Serial.print(F("  WiFi Station: "));
    Serial.println(wifiStationEnabled ? "ON" : "OFF");
    if (wifiStationEnabled && strlen(wifiSSID) > 0) {
      Serial.print(F("  WiFi SSID: "));
      Serial.println(wifiSSID);
    }
    #endif
  } else {
    // Set defaults
    passwordProtectionEnabled = true;
    apInfiniteModeEnabled = false;
    wifiStationEnabled = false;
    wifiSSID[0] = '\0';
    wifiPassword[0] = '\0';
    #if DEBUG_MODE
    Serial.println(F("UI Settings: Not configured (using defaults)"));
    #endif
  }
}

void saveUISettings() {
  // Save password protection setting
  EEPROM.write(EEPROM_PASSWORD_PROT_ADDR, passwordProtectionEnabled ? 1 : 0);

  // Save AP infinite mode setting
  EEPROM.write(EEPROM_AP_INFINITE_ADDR, apInfiniteModeEnabled ? 1 : 0);

  // Save WiFi station enabled setting
  EEPROM.write(EEPROM_WIFI_ENABLED_ADDR, wifiStationEnabled ? 1 : 0);

  // Save WiFi SSID
  for (int i = 0; i < 32; i++) {
    EEPROM.write(EEPROM_WIFI_SSID_ADDR + i, wifiSSID[i]);
  }

  // Save WiFi Password
  for (int i = 0; i < 32; i++) {
    EEPROM.write(EEPROM_WIFI_PASS_ADDR + i, wifiPassword[i]);
  }

  // Save magic number
  EEPROM.write(EEPROM_UI_SETTINGS_VALID_ADDR, EEPROM_UI_SETTINGS_MAGIC);
  EEPROM.commit();

  #if DEBUG_MODE
  Serial.println(F("UI Settings saved"));
  #endif
}

void connectToWiFiStation() {
  if (!wifiStationEnabled || strlen(wifiSSID) == 0) {
    return;
  }

  #if DEBUG_MODE
  Serial.print(F("Connecting to WiFi: "));
  Serial.println(wifiSSID);
  #endif

  WiFi.begin(wifiSSID, wifiPassword);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < WIFI_CONNECT_TIMEOUT) {
    delay(500);
    #if DEBUG_MODE
    Serial.print(F("."));
    #endif
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    // Start mDNS for device discovery
    startMDNS();
    #if DEBUG_MODE
    Serial.println(F(" Connected!"));
    Serial.print(F("IP: "));
    Serial.println(WiFi.localIP());
    #endif
  } else {
    wifiConnected = false;
    #if DEBUG_MODE
    Serial.println(F(" Failed!"));
    #endif
  }
}

void checkWiFiConnection() {
  if (!wifiStationEnabled) {
    wifiConnected = false;
    stopMDNS();
    return;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastWifiCheck >= WIFI_CHECK_INTERVAL) {
    lastWifiCheck = currentMillis;

    if (WiFi.status() == WL_CONNECTED) {
      if (!wifiConnected) {
        // Just reconnected - start mDNS
        wifiConnected = true;
        startMDNS();
      }
    } else {
      if (wifiConnected) {
        // Just disconnected - stop mDNS
        stopMDNS();
      }
      wifiConnected = false;
      // Try to reconnect
      if (strlen(wifiSSID) > 0) {
        WiFi.begin(wifiSSID, wifiPassword);
      }
    }
  }
}

// ============================================
// CALIBRATION MANAGEMENT
// ============================================


void loadCalibrationSettings() {
  if (EEPROM.read(EEPROM_CAL_VALID_ADDR) == EEPROM_CAL_MAGIC) {
    // Load calibration offset and slope as floats
    EEPROM.get(EEPROM_CAL_OFFSET_ADDR, calOffset);
    EEPROM.get(EEPROM_CAL_SLOPE_ADDR, calSlope);
    calibrationEnabled = (EEPROM.read(EEPROM_CAL_ENABLED_ADDR) == 1);

    // Validate values (slope must be positive and reasonable)
    if (isnan(calOffset) || calOffset < -100 || calOffset > 100) {
      calOffset = CAL_DEFAULT_OFFSET;
    }
    if (isnan(calSlope) || calSlope <= 0 || calSlope > 10) {
      calSlope = CAL_DEFAULT_SLOPE;
    }

    #if DEBUG_MODE
    Serial.println(F("Calibration loaded:"));
    Serial.print(F("  Offset: "));
    Serial.println(calOffset, 6);
    Serial.print(F("  Slope: "));
    Serial.println(calSlope, 6);
    Serial.print(F("  Enabled: "));
    Serial.println(calibrationEnabled ? "YES" : "NO");
    #endif
  } else {
    // Set defaults
    calibrationEnabled = false;
    calOffset = CAL_DEFAULT_OFFSET;
    calSlope = CAL_DEFAULT_SLOPE;

    #if DEBUG_MODE
    Serial.println(F("Calibration: Not configured (using defaults)"));
    #endif
  }
}

void saveCalibrationSettings() {
  // Save calibration offset and slope as floats
  EEPROM.put(EEPROM_CAL_OFFSET_ADDR, calOffset);
  EEPROM.put(EEPROM_CAL_SLOPE_ADDR, calSlope);
  EEPROM.write(EEPROM_CAL_ENABLED_ADDR, calibrationEnabled ? 1 : 0);
  EEPROM.write(EEPROM_CAL_VALID_ADDR, EEPROM_CAL_MAGIC);
  EEPROM.commit();

  #if DEBUG_MODE
  Serial.print(F("Calibration saved - Offset: "));
  Serial.print(calOffset, 6);
  Serial.print(F(", Slope: "));
  Serial.println(calSlope, 6);
  #endif
}

// ============================================
// mDNS MANAGEMENT
// ============================================

void startMDNS() {
  if (mdnsStarted) {
    return;
  }

  // Create mDNS hostname: deviceID + "_mvstechsensors" (e.g., "US3847_mvstechsensors")
  String mdnsHostname = String(deviceID) + "_mvstechsensors";

  if (MDNS.begin(mdnsHostname.c_str())) {
    mdnsStarted = true;
    // Add HTTP service for app discovery
    MDNS.addService("http", "tcp", WEB_PORT);

    #if DEBUG_MODE
    Serial.print(F("mDNS started: "));
    Serial.print(mdnsHostname);
    Serial.println(F(".local"));
    #endif
  } else {
    #if DEBUG_MODE
    Serial.println(F("mDNS start failed"));
    #endif
  }
}

void stopMDNS() {
  if (mdnsStarted) {
    MDNS.end();
    mdnsStarted = false;

    #if DEBUG_MODE
    Serial.println(F("mDNS stopped"));
    #endif
  }
}

// ============================================
// SENSOR MODE MANAGEMENT
// ============================================

void loadSensorModeSettings() {
  if (EEPROM.read(EEPROM_WP_VALID_ADDR) == EEPROM_WP_MAGIC) {
    byte modeVal = EEPROM.read(EEPROM_SENSOR_MODE_ADDR);
    if (modeVal <= 2) {
      sensorMode = (SensorMode)modeVal;
    } else {
      sensorMode = MODE_US_ONLY;
    }

    wpTankHeight = (EEPROM.read(EEPROM_WP_TANK_HEIGHT_ADDR) << 8) |
                   EEPROM.read(EEPROM_WP_TANK_HEIGHT_ADDR + 1);
    wpMinWaterLevel = (EEPROM.read(EEPROM_WP_MIN_WATER_LEVEL_ADDR) << 8) |
                      EEPROM.read(EEPROM_WP_MIN_WATER_LEVEL_ADDR + 1);

    // Validate loaded values
    if (wpTankHeight < WP_MIN_TANK_HEIGHT || wpTankHeight > WP_MAX_TANK_HEIGHT) {
      wpTankHeight = 200;
    }
    if (wpMinWaterLevel > wpTankHeight) {
      wpMinWaterLevel = WP_DEFAULT_MIN_LEVEL;
    }

    #if DEBUG_MODE
    Serial.print(F("Sensor Mode: "));
    Serial.println(getModeString());
    Serial.print(F("Tank Height: "));
    Serial.print(wpTankHeight);
    Serial.println(F("cm"));
    Serial.print(F("Min Water Level: "));
    Serial.print(wpMinWaterLevel);
    Serial.println(F("cm"));
    #endif
  } else {
    // Set defaults
    sensorMode = MODE_US_ONLY;
    wpTankHeight = 200;
    wpMinWaterLevel = WP_DEFAULT_MIN_LEVEL;
    #if DEBUG_MODE
    Serial.println(F("Sensor Mode: Not configured (using defaults)"));
    #endif
  }
}

void saveSensorModeSettings() {
  EEPROM.write(EEPROM_SENSOR_MODE_ADDR, (byte)sensorMode);
  EEPROM.write(EEPROM_WP_TANK_HEIGHT_ADDR, wpTankHeight >> 8);
  EEPROM.write(EEPROM_WP_TANK_HEIGHT_ADDR + 1, wpTankHeight & 0xFF);
  EEPROM.write(EEPROM_WP_MIN_WATER_LEVEL_ADDR, wpMinWaterLevel >> 8);
  EEPROM.write(EEPROM_WP_MIN_WATER_LEVEL_ADDR + 1, wpMinWaterLevel & 0xFF);
  EEPROM.write(EEPROM_WP_VALID_ADDR, EEPROM_WP_MAGIC);
  EEPROM.commit();

  #if DEBUG_MODE
  Serial.println(F("Sensor Mode settings saved"));
  #endif
}

String getUSNodeID() {
  // Returns US node ID (e.g., US3847)
  char nodeID[12];
  strcpy(nodeID, deviceID);
  // Ensure US prefix
  nodeID[0] = 'U';
  nodeID[1] = 'S';
  return String(nodeID);
}

String getWPNodeID() {
  // Returns WP node ID (e.g., WP3847)
  char nodeID[12];
  strcpy(nodeID, deviceID);
  // Change to WP prefix
  nodeID[0] = 'W';
  nodeID[1] = 'P';
  return String(nodeID);
}

String getModeString() {
  switch(sensorMode) {
    case MODE_US_ONLY: return "US Only";
    case MODE_WP_ONLY: return "WP Only";
    case MODE_DUAL: return "US + WP";
    default: return "Unknown";
  }
}

// ============================================
// EEPROM ZONE MANAGEMENT
// ============================================

void loadZoneFromEEPROM() {
  if (EEPROM.read(EEPROM_VALID_FLAG_ADDR) == EEPROM_MAGIC_NUMBER) {
    byte storedZone = EEPROM.read(EEPROM_ZONE_ADDR);
    if (storedZone >= UPPER_ZONE && storedZone <= LOWER_ZONE) {
      lastStoredZone = (TankZone)storedZone;
      hasValidHistory = true;
      #if DEBUG_MODE
      Serial.print(F("EEPROM: Zone "));
      Serial.println(storedZone);
      #endif
    } else {
      #if DEBUG_MODE
      Serial.println(F("EEPROM: Corrupted"));
      #endif
      resetEEPROM();
    }
  } else {
    #if DEBUG_MODE
    Serial.println(F("EEPROM: Empty"));
    #endif
    resetEEPROM();
  }
}

void resetEEPROM() {
  EEPROM.write(EEPROM_ZONE_ADDR, NO_ZONE);
  EEPROM.write(EEPROM_VALID_FLAG_ADDR, 0);
  EEPROM.commit();
  lastStoredZone = NO_ZONE;
  hasValidHistory = false;
}

float applyCalibration(float rawReading) {
  if (rawReading > 21 && rawReading <= 450) {
    if (calibrationEnabled) {
      // Use custom calibration values
      return calOffset + (calSlope * rawReading);
    } else {
      // Use default hardcoded calibration
      return CAL_DEFAULT_OFFSET + (CAL_DEFAULT_SLOPE * rawReading);
    }
  }
  return rawReading;
}

TankZone getCurrentZone(float calibratedReading) {
  if (calibratedReading >= UPPER_ZONE_MIN && calibratedReading <= UPPER_ZONE_MAX) {
    return UPPER_ZONE;
  }
  else if (calibratedReading >= LOWER_ZONE_MIN) {
    return LOWER_ZONE;
  }
  else if (calibratedReading >= MIDDLE_ZONE_MIN && calibratedReading <= MIDDLE_ZONE_MAX) {
    return MIDDLE_ZONE;
  }
  return NO_ZONE;
}

void updateZoneIfChanged(float calibratedReading) {
  TankZone currentZone = getCurrentZone(calibratedReading);

  if (currentZone != lastStoredZone && currentZone != NO_ZONE) {
    EEPROM.write(EEPROM_ZONE_ADDR, currentZone);
    EEPROM.write(EEPROM_VALID_FLAG_ADDR, EEPROM_MAGIC_NUMBER);
    EEPROM.commit();
    lastStoredZone = currentZone;
    hasValidHistory = true;

    #if DEBUG_MODE
    Serial.print(F("Zone: "));
    Serial.print(currentZone);
    Serial.print(F(" ("));
    Serial.print(calibratedReading, 1);
    Serial.println(F("cm)"));
    #endif
  }
}

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

// ============================================
// ULTRASONIC SENSOR (MAD FILTERING)
// ============================================

float readDistanceCM() {
  float readings[SAMPLE_COUNT];
  int count = 0;

  #if DEBUG_MODE
  Serial.print(F("MAD: "));
  #endif

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(15);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 60000);
    float distance = duration * 0.0343 / 2;

    if (duration > 0 && distance > 21 && distance <= 450) {
      readings[count++] = distance;
      #if DEBUG_MODE
      Serial.print(distance, 0);
      Serial.print(F(" "));
      #endif
    } else {
      #if DEBUG_MODE
      Serial.print(F("X "));
      #endif
    }

    delay(READING_DELAY);
  }
  #if DEBUG_MODE
  Serial.println();
  #endif

  if (count >= MIN_VALID_SAMPLES) {
    sortReadings(readings, count);

    float median;
    if (count % 2 == 1) {
      median = readings[count / 2];
    } else {
      median = (readings[count / 2 - 1] + readings[count / 2]) / 2.0;
    }

    float deviations[SAMPLE_COUNT];
    for (int i = 0; i < count; i++) {
      deviations[i] = abs(readings[i] - median);
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

    for (int i = 0; i < count; i++) {
      if (abs(readings[i] - median) <= threshold) {
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

    #if DEBUG_MODE
    Serial.print(F("Valid: "));
    Serial.print(count);
    Serial.print(F(" Clean: "));
    Serial.print(cleanCount);
    Serial.print(F(" Final: "));
    Serial.print(median, 1);
    Serial.println(F("cm"));
    #endif

    if (median > 21 && median <= 450) {
      lastRawReading = median;  // Store raw value before calibration
      float calibratedValue = applyCalibration(median);
      updateZoneIfChanged(calibratedValue);
      return calibratedValue;
    }
  } else {
    #if DEBUG_MODE
    Serial.print(F("Only "));
    Serial.print(count);
    Serial.println(F(" valid samples"));
    #endif
  }

  // Zone-based boundary logic
  if (hasValidHistory) {
    if (lastStoredZone == UPPER_ZONE) {
      #if DEBUG_MODE
      Serial.println(F("Zone: UPPER->21cm"));
      #endif
      return applyCalibration(21.0);
    }
    else if (lastStoredZone == LOWER_ZONE) {
      #if DEBUG_MODE
      Serial.println(F("Zone: LOWER->400cm"));
      #endif
      return 400.0;
    }
  }

  return -1.0;
}

// ============================================
// LORA TRANSMISSION
// ============================================

void transmitUSPacket(float distance) {
  int transmitValue = (int)(distance + 0.5);
  String nodeID = getUSNodeID();

  // Increment packet counter
  PACKET_COUNTER++;
  if (PACKET_COUNTER >= COUNTER_RESET_LIMIT) {
    SESSION_ID = millis();
    PACKET_COUNTER = 1;
    #if DEBUG_MODE
    Serial.println(F("Counter reset - new session"));
    #endif
  }

  // Build JSON
  char jsonBuffer[140];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"node_id\":\"%s\",\"msg_id\":\"%s_%lu_%lu\",\"data\":\"%d\",\"secret_code\":\"%s\"}",
    nodeID.c_str(), nodeID.c_str(), SESSION_ID, PACKET_COUNTER, transmitValue, COMPANY_SECRET);

  #if DEBUG_MODE
  Serial.print(F("US TX: "));
  Serial.println(jsonBuffer);
  #endif

  // Transmit with retry
  bool success = false;
  for (int attempt = 1; attempt <= 3; attempt++) {
    #if DEBUG_MODE
    Serial.print(F("Try "));
    Serial.print(attempt);
    Serial.print(F("/3: "));
    #endif

    if (LoRa.beginPacket()) {
      LoRa.print(jsonBuffer);
      if (LoRa.endPacket(true)) {
        #if DEBUG_MODE
        Serial.println(F("OK"));
        #endif
        success = true;
        break;
      }
    }
    #if DEBUG_MODE
    Serial.println(F("FAIL"));
    #endif

    if (attempt < 3) {
      delay(500);
    }
  }

  if (!success) {
    #if DEBUG_MODE
    Serial.println(F("US TX failed"));
    #endif
  }
}

void transmitWPPacket(float distance) {
  // Calculate water level and determine presence
  int waterLevel = (int)(wpTankHeight - distance);
  if (waterLevel < 0) waterLevel = 0;

  int transmitValue = (waterLevel >= wpMinWaterLevel) ? 1 : 0;
  String nodeID = getWPNodeID();

  #if DEBUG_MODE
  Serial.print(F("WP Mode - Water Level: "));
  Serial.print(waterLevel);
  Serial.print(F("cm, Threshold: "));
  Serial.print(wpMinWaterLevel);
  Serial.print(F("cm, Status: "));
  Serial.println(transmitValue ? "PRESENT" : "ABSENT");
  #endif

  // Increment packet counter
  PACKET_COUNTER++;
  if (PACKET_COUNTER >= COUNTER_RESET_LIMIT) {
    SESSION_ID = millis();
    PACKET_COUNTER = 1;
    #if DEBUG_MODE
    Serial.println(F("Counter reset - new session"));
    #endif
  }

  // Build JSON
  char jsonBuffer[140];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"node_id\":\"%s\",\"msg_id\":\"%s_%lu_%lu\",\"data\":\"%d\",\"secret_code\":\"%s\"}",
    nodeID.c_str(), nodeID.c_str(), SESSION_ID, PACKET_COUNTER, transmitValue, COMPANY_SECRET);

  #if DEBUG_MODE
  Serial.print(F("WP TX: "));
  Serial.println(jsonBuffer);
  #endif

  // Transmit with retry
  bool success = false;
  for (int attempt = 1; attempt <= 3; attempt++) {
    #if DEBUG_MODE
    Serial.print(F("Try "));
    Serial.print(attempt);
    Serial.print(F("/3: "));
    #endif

    if (LoRa.beginPacket()) {
      LoRa.print(jsonBuffer);
      if (LoRa.endPacket(true)) {
        #if DEBUG_MODE
        Serial.println(F("OK"));
        #endif
        success = true;
        break;
      }
    }
    #if DEBUG_MODE
    Serial.println(F("FAIL"));
    #endif

    if (attempt < 3) {
      delay(500);
    }
  }

  if (!success) {
    #if DEBUG_MODE
    Serial.println(F("WP TX failed"));
    #endif
  }
}

void transmitLoRa() {
  #if DEBUG_MODE
  Serial.println(F("\n--- TX Cycle ---"));
  #endif

  float calibratedDistance = readDistanceCM();

  if (calibratedDistance < 0) {
    #if DEBUG_MODE
    Serial.println(F("Skip TX - no valid reading"));
    #endif
    return;
  }

  lastReading = calibratedDistance;

  #if DEBUG_MODE
  Serial.print(F("Distance: "));
  Serial.print(calibratedDistance, 1);
  Serial.print(F("cm, Mode: "));
  Serial.println(getModeString());
  #endif

  // Transmit based on sensor mode
  switch(sensorMode) {
    case MODE_US_ONLY:
      // Send US packet only
      transmitUSPacket(calibratedDistance);
      break;

    case MODE_WP_ONLY:
      // Send WP packet only
      transmitWPPacket(calibratedDistance);
      break;

    case MODE_DUAL:
      // Send US packet first
      transmitUSPacket(calibratedDistance);

      // Wait fixed delay
      #if DEBUG_MODE
      Serial.print(F("Dual mode delay: "));
      Serial.print(DUAL_MODE_DELAY);
      Serial.println(F("ms"));
      #endif
      delay(DUAL_MODE_DELAY);

      // Then send WP packet
      transmitWPPacket(calibratedDistance);
      break;
  }
}

/*
ESP32 ULTRASONIC LORA TRANSMITTER WITH WIFI CONFIGURATION
==========================================================

CONFIGURATION SUMMARY:
---------------------
Firmware Version: 9.0.0
Web Port: 7689
AP Suffix: _mvstechsensors
AP Password: mvstech9867
Web Password: 133611
User-Agent: MVStech7689
AP Timeout: 2 min default, extends on activity, max 5 min
DEBUG_MODE: 0 = minimal output, 1 = full debug (for developers)

DEVICE ID:
---------
- Format: US + 4 random digits (e.g., US3847)
- Generated randomly on first boot
- Configurable via web UI
- Saved to EEPROM
- WP prefix (WP3847) auto-derived from US ID

AP NAMING:
---------
- Format: <DeviceID>_mvstechsensors
- Example: US3847_mvstechsensors
- App identifies sensors by _mvstechsensors suffix

WEB UI PAGES:
------------
/ - Login page (password: 133611)
/dashboard - Configuration page with sensor mode settings

SENSOR MODES:
------------
1. US Only (Default)
   - Sends distance in cm as USxxxx
   - JSON: {"node_id":"US3847","data":"150",...}

2. WP Only
   - Sends water presence (0/1) as WPxxxx
   - JSON: {"node_id":"WP3847","data":"1",...}

3. US + WP (Dual Mode)
   - Sends BOTH packets in sequence
   - First: US packet with distance
   - Wait: 1.5 seconds
   - Then: WP packet with water presence
   - Same session ID and counter for both (different node_id)

WP MODE CONFIGURATION:
--------------------
- Tank Height: Distance from sensor to tank bottom (50-450 cm)
- Min Water Level: Threshold for water presence (default 60 cm)
- Logic: Water Level = Tank Height - Sensor Distance
         If Water Level >= Min Level -> Sends "1" (PRESENT)
         If Water Level < Min Level -> Sends "0" (ABSENT)

OTA UPDATE:
----------
TCP-based OTA on port 8089 (via MVS App or Python script)

LORA PARAMETERS:
---------------
- Frequency: 433 MHz
- Spreading Factor: 10
- Bandwidth: 125 kHz
- Coding Rate: 4/8
- Preamble: 8
- Sync Word: 0x12
- TX Power: 20 dBm

HARDWARE PINS:
-------------
LoRa: SS=5, RST=14, DIO0=26
Ultrasonic: TRIG=25, ECHO=34

EEPROM LAYOUT:
-------------
0: Zone address
1: Zone valid flag
2-11: Device ID (10 bytes)
12: Device ID magic number
13: Sensor mode (0=US, 1=WP, 2=Dual)
14-15: WP tank height (2 bytes)
16-17: WP min water level (2 bytes)
18: Settings magic number

USAGE:
-----
1. Power on device
2. Connect to WiFi: USxxxx_mvstechsensors
3. Open app or browser: http://192.168.4.1:7689
4. Enter password: 133611
5. Configure Device ID
6. Select Sensor Mode (US Only, WP Only, or Dual)
7. If WP or Dual: Configure Tank Height and Min Water Level
8. Device will transmit via LoRa automatically
*/
