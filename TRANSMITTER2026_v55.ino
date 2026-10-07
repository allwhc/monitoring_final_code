/*
 * SenseFlow Tank Sensor (Transmitter) - single tank, ultrasonic
 * ===============================================================
 * Firmware Version: 55.0.0
 *
 * FEATURES:
 * - Connection type chosen on the setup page (saved in EEPROM):
 *     Auto          - Long Range if the module is fitted, else Home Network + NearbyConnect
 *                     (WiFi set) or NearbyConnect
 *     Long Range    - LoRa radio, up to several km
 *     NearbyConnect - direct link to a nearby display (~200 m, no router)
 *     Home Network  - sensor joins the home WiFi; the display finds it and reads it
 *     Home Network + NearbyConnect - both, NearbyConnect keeps working if the router is off
 * - Micro-payload every cycle, ALWAYS sent (heartbeat when the reading fails):
 *     {"i":"US3847","s":51,"c":102,"d":143}   data (distance cm)
 *     {"i":"US3847","s":51,"c":103,"h":1}     heartbeat - sensor alive, reading failed
 * - WiFi AP setup page (2 min, extends with activity, max 5 min) - closing the
 *   AP keeps the radio running for NearbyConnect / Home Network
 * - mvsOTA firmware updates
 * - MAD filtering + calibration + zone boundary logic
 *
 * HARDWARE CONNECTIONS:
 * ---------------------
 * LoRa Module (SX1278) -> ESP32 (optional - only for Long Range)
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
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <mvsota_esp32.h>

#define FIRMWARE_VERSION "55.0.0"
#define FIRMWARE_CODE "SF-TC-2026-9867"

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
// 13-18: reserved (old WP sensor mode settings - no longer used)
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
// Connection EEPROM addresses (97-99)
#define EEPROM_LINK_TYPE_ADDR 97          // 1 byte connection type
#define EEPROM_LINK_VALID_ADDR 98         // 1 byte magic number
#define EEPROM_NEARBY_CHANNEL_ADDR 99     // 1 byte last NearbyConnect channel (1-13)
#define EEPROM_MAGIC_NUMBER 0xAB
#define EEPROM_DEVICE_MAGIC 0xCD
#define EEPROM_WP_MAGIC 0xEF
#define EEPROM_UI_SETTINGS_MAGIC 0xDE
#define EEPROM_CAL_MAGIC 0xBC
#define EEPROM_LINK_MAGIC 0xA5

// NearbyConnect timing
#define NEARBY_ACK_WAIT_MS 300            // Wait for the display's acknowledgement
#define NEARBY_RETRIES 3                  // Sends per reading while locked
#define NEARBY_SEARCH_DWELL_MS 150        // Time spent on each channel while searching
#define NEARBY_MISSED_BEFORE_SEARCH 3     // Readings without any ack before searching channels again

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

// Connection type (stored in EEPROM)
enum LinkType : uint8_t {
  TX_LINK_AUTO = 0,
  TX_LINK_LONG_RANGE = 1,     // LoRa
  TX_LINK_NEARBY = 2,         // ESP-NOW
  TX_LINK_HOME_NETWORK = 3,   // Home WiFi, display polls /api/r
  TX_LINK_HN_NEARBY = 4       // Both
};
#define TX_LINK_COUNT 5

// NearbyConnect sender state
enum NearbyState : uint8_t {
  NB_IDLE = 0,
  NB_WAIT_ACK = 1,
  NB_SEARCH = 2
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
unsigned long lastTransmit = 0;
bool loraInitialized = false;             // Long Range module detected

// Connection variables
LinkType linkType = TX_LINK_AUTO;
bool useLoRa = false;                     // Active links after resolving Auto at boot
bool useNearby = false;
bool useHomeNet = false;
bool nearbyReady = false;
bool restartPending = false;
unsigned long restartAt = 0;

// Last payload (also served to the display at /api/r for Home Network)
char txPayload[96] = "";
bool lastSentHasDistance = false;
int lastSentDistance = 0;

// NearbyConnect sender
const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
NearbyState nearbyState = NB_IDLE;
uint8_t nearbyChannel = 1;                // Channel the display was last found on
bool nearbyLocked = false;                // Display acknowledged on nearbyChannel
uint8_t nearbyMissed = 0;
uint8_t nearbyAttempt = 0;
uint8_t nearbySearchIndex = 0;
unsigned long nearbySentAt = 0;
unsigned long lastNearbyAck = 0;
uint32_t nearbyCounter = 0;               // Counter of the payload being delivered
volatile bool nearbyAckFlag = false;      // Set in the WiFi task when the display acknowledges
volatile uint32_t nearbyAckCounter = 0;

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
void measurementCycle();
void loadZoneFromEEPROM();
void resetEEPROM();
void updateZoneIfChanged(float calibratedReading);
TankZone getCurrentZone(float calibratedReading);
float applyCalibration(float rawReading);
void sortReadings(float arr[], int n);
int getDeviceNumber();
String getUptimeString();
// Connection functions
String getUSNodeID();
void loadLinkSettings();
void saveLinkSettings();
void resolveActiveLinks();
const char* linkTypeName(LinkType t);
String getLinkStatusString();
bool startLoRa();
bool startNearby();
void serviceNearby();
void nearbyBeginDelivery();
void setRadioChannel(uint8_t ch);
void sendLoRaPayload(const char* payload);
void handleReading();
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

  // Load connection type
  loadLinkSettings();

  // Load UI settings (password protection, AP mode, WiFi)
  loadUISettings();

  // Load Calibration settings
  loadCalibrationSettings();

  // New session on every boot (short number keeps the payload small)
  SESSION_ID = (esp_random() % 65535UL) + 1;

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

  // Detect the Long Range module first - Auto mode depends on it
  SPI.begin();
  loraInitialized = startLoRa();

  // Decide which links run this session
  resolveActiveLinks();
  if (!useLoRa && loraInitialized) {
    LoRa.sleep();  // Module fitted but not used
  }

  // Setup WiFi Access Point (+ station for Home Network)
  setupAccessPoint();

  // Initialize mvsOTA
  mvsota.begin(deviceID, FIRMWARE_VERSION, FIRMWARE_CODE);
  mvsota.onStart([]() {
    Serial.println(F("OTA Update starting..."));
  });

  // Setup Web Server
  setupWebServer();

  // NearbyConnect starts after WiFi so it shares the radio cleanly
  if (useNearby) {
    nearbyReady = startNearby();
  }

  Serial.print(F("Connection: "));
  Serial.println(linkTypeName(linkType));
  if (linkType == TX_LINK_LONG_RANGE && !loraInitialized) {
    Serial.println(F("WARNING: Long Range module not detected"));  // Always show errors
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

  // Measurement + transmission cycle (every transmissionInterval) - always sends
  if (!mvsota.isUpdating() && (currentMillis - lastTransmit >= transmissionInterval)) {
    measurementCycle();
    lastTransmit = currentMillis;
  }

  // NearbyConnect delivery / channel search (non-blocking)
  serviceNearby();

  // Restart requested by the setup page (connection type changed)
  if (restartPending && currentMillis > restartAt) {
    ESP.restart();
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

  // AP+STA when joining home WiFi or using NearbyConnect (it sends on the station interface)
  if ((wifiStationEnabled && strlen(wifiSSID) > 0) || useNearby) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_AP);
  }
  // NearbyConnect: open the AP on the channel the display was last found on
  WiFi.softAP(apSSID.c_str(), AP_PASSWORD, useNearby ? nearbyChannel : 1);

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
  if (useNearby || useHomeNet) {
    // Keep the radio on: only the setup AP closes, the link to the display stays up
    WiFi.mode(WIFI_STA);
    if (useNearby && WiFi.status() != WL_CONNECTED) {
      setRadioChannel(nearbyChannel);
    }
  } else {
    WiFi.mode(WIFI_OFF);  // Long Range only - WiFi not needed
  }
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
  server.on("/api/r", HTTP_GET, handleReading);  // Home Network: the display polls this
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
    html += "<div class='error'>Home Network needs your WiFi name and password below.</div>";
  }
  if (server.hasArg("restart")) {
    html += "<div class='success'>Connection type changed - the sensor is restarting.</div>";
  }

  // Status Card (values updated via AJAX)
  html += "<div class='card'>";
  html += "<h3>Status <span style='font-size:0.8em;color:#aaa'>(auto-refresh)</span></h3>";
  html += "<div class='info-row'><span class='label'>Sensor ID</span><span class='value'>" + getUSNodeID() + "</span></div>";
  html += "<div class='info-row'><span class='label'>Connection</span><span class='value' id='linkStatus'>" + getLinkStatusString() + "</span></div>";
  html += "<div class='info-row'><span class='label'>Firmware</span><span class='value'>v" + String(FIRMWARE_VERSION) + "</span></div>";
  html += "<div class='info-row'><span class='label'>Long Range module</span><span class='value'>" + String(loraInitialized ? "Detected" : "Not fitted") + "</span></div>";
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

  // Connection Type Card (customer-facing names only)
  html += "<div class='card'>";
  html += "<h3>Connection Type</h3>";
  html += "<div class='radio-group'>";
  const char* linkTitles[TX_LINK_COUNT] = {
    "Auto (recommended)", "Long Range", "NearbyConnect", "Home Network", "Home Network + NearbyConnect"
  };
  const char* linkDescs[TX_LINK_COUNT] = {
    "Picks the best connection on its own: Long Range if the module is fitted, otherwise your home WiFi or NearbyConnect.",
    "For tanks far away - up to several kilometres in open areas. Needs the Long Range module in the sensor and the display.",
    "Talks directly to a display nearby, up to about 200 m in open space (less through thick walls). No router needed.",
    "Joins your home WiFi so the display can read it from anywhere on the network. The display must be on the same WiFi.",
    "Uses your home WiFi, and switches to NearbyConnect automatically if the router is off."
  };
  for (int i = 0; i < TX_LINK_COUNT; i++) {
    bool sel = ((int)linkType == i);
    html += "<label class='radio-option" + String(sel ? " selected" : "") + "' onclick='selectLink(" + String(i) + ")'>";
    html += "<input type='radio' name='linkType' value='" + String(i) + "'" + String(sel ? " checked" : "") + ">";
    html += "<div class='radio-circle'></div>";
    html += "<div class='radio-text'><div class='radio-title'>" + String(linkTitles[i]) + "</div>";
    html += "<div class='radio-desc'>" + String(linkDescs[i]) + "</div></div></label>";
  }
  html += "</div>";
  html += "<div class='note' id='linkNote'>Home Network options need the WiFi details in System Settings. The sensor restarts when the connection type changes.</div>";
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
  html += "<div><span class='label' style='color:#fff'>WiFi Connection</span><br><span class='note'>Needed for Home Network - use the same WiFi as the display</span></div>";
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
  html += "document.getElementById('linkStatus').textContent=d.linkStatus;";
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

  // Connection type selection
  html += "function selectLink(v){";
  html += "var radios=document.querySelectorAll('input[name=linkType]');";
  html += "var options=document.querySelectorAll('.radio-option');";
  html += "radios.forEach(function(r,i){r.checked=(i==v);});";
  html += "options.forEach(function(o,i){o.classList.toggle('selected',i==v);});";
  html += "if(v==3||v==4){var t=document.getElementById('wifiToggle');if(!t.checked){t.checked=true;toggleWifiSettings();}}}";

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
  json += "\"wifiStatus\":\"" + wifiStatus + "\",";
  json += "\"linkStatus\":\"" + getLinkStatusString() + "\"";
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

  // === Connection Type === (applied after the WiFi settings below)
  LinkType newLink = linkType;
  if (server.hasArg("linkType")) {
    int v = server.arg("linkType").toInt();
    if (v >= 0 && v < TX_LINK_COUNT) {
      newLink = (LinkType)v;
    }
  }

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

  // Home Network needs WiFi details
  bool needsWifi = (newLink == TX_LINK_HOME_NETWORK || newLink == TX_LINK_HN_NEARBY);
  if (needsWifi && strlen(wifiSSID) == 0) {
    server.sendHeader("Location", "/dashboard?error=1");
    server.send(302, "text/plain", "");
    return;
  }
  if (needsWifi && !wifiStationEnabled) {
    wifiStationEnabled = true;
    saveUISettings();
  }

  // A new connection type takes effect after a restart
  if (newLink != linkType) {
    linkType = newLink;
    saveLinkSettings();
    restartPending = true;
    restartAt = millis() + 2000;
    server.sendHeader("Location", "/dashboard?saved=1&restart=1");
    server.send(302, "text/plain", "");
    return;
  }

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
    // Display discovery (Home Network): _mvstech._tcp with the sensor ID in TXT "device"
    MDNS.addService("mvstech", "tcp", WEB_PORT);
    MDNS.addServiceTxt("mvstech", "tcp", "device", deviceID);

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
// CONNECTION MANAGEMENT
// ============================================

void loadLinkSettings() {
  if (EEPROM.read(EEPROM_LINK_VALID_ADDR) == EEPROM_LINK_MAGIC) {
    uint8_t t = EEPROM.read(EEPROM_LINK_TYPE_ADDR);
    linkType = (t < TX_LINK_COUNT) ? (LinkType)t : TX_LINK_AUTO;
    uint8_t ch = EEPROM.read(EEPROM_NEARBY_CHANNEL_ADDR);
    nearbyChannel = (ch >= 1 && ch <= 13) ? ch : 1;
  } else {
    linkType = TX_LINK_AUTO;  // Existing sensors with a Long Range module keep using it
    nearbyChannel = 1;
  }
}

void saveLinkSettings() {
  EEPROM.write(EEPROM_LINK_TYPE_ADDR, (uint8_t)linkType);
  EEPROM.write(EEPROM_NEARBY_CHANNEL_ADDR, nearbyChannel);
  EEPROM.write(EEPROM_LINK_VALID_ADDR, EEPROM_LINK_MAGIC);
  EEPROM.commit();
}

// Decide which links run this session (Auto is resolved once at boot)
void resolveActiveLinks() {
  bool wifiDetails = strlen(wifiSSID) > 0;
  useLoRa = useNearby = useHomeNet = false;

  switch (linkType) {
    case TX_LINK_LONG_RANGE:   useLoRa = true; break;
    case TX_LINK_NEARBY:       useNearby = true; break;
    case TX_LINK_HOME_NETWORK: useHomeNet = true; break;
    case TX_LINK_HN_NEARBY:    useHomeNet = true; useNearby = true; break;
    default:
      if (loraInitialized) {
        useLoRa = true;
      } else if (wifiDetails) {
        useHomeNet = true;
        useNearby = true;
      } else {
        useNearby = true;
      }
      break;
  }

  // Home Network always joins the configured WiFi
  if (useHomeNet && wifiDetails) {
    wifiStationEnabled = true;
  }

  #if DEBUG_MODE
  Serial.print(F("Links: LoRa="));
  Serial.print(useLoRa);
  Serial.print(F(" ESP-NOW="));
  Serial.print(useNearby);
  Serial.print(F(" HomeNet="));
  Serial.println(useHomeNet);
  #endif
}

// Customer-facing names - internal technology names never appear in the UI
const char* linkTypeName(LinkType t) {
  switch (t) {
    case TX_LINK_LONG_RANGE: return "Long Range";
    case TX_LINK_NEARBY: return "NearbyConnect";
    case TX_LINK_HOME_NETWORK: return "Home Network";
    case TX_LINK_HN_NEARBY: return "Home Network + NearbyConnect";
    default: return "Auto";
  }
}

// One line for the setup page: what is actually running and whether the display answers
String getLinkStatusString() {
  String st;
  if (useLoRa) {
    st = loraInitialized ? "Long Range" : "Long Range (module not detected)";
  }
  if (useHomeNet) {
    if (st.length()) st += " + ";
    st += (WiFi.status() == WL_CONNECTED) ? "Home Network (" + WiFi.localIP().toString() + ")"
                                         : String("Home Network (WiFi not connected)");
  }
  if (useNearby) {
    if (st.length()) st += " + ";
    bool linked = nearbyLocked && lastNearbyAck > 0 && (millis() - lastNearbyAck) < 120000UL;
    st += linked ? "NearbyConnect (display linked)" : "NearbyConnect (looking for display)";
  }
  if (linkType == TX_LINK_AUTO) {
    st = "Auto: " + st;
  }
  return st;
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
// LONG RANGE (LoRa)
// ============================================

// Returns false when no module answers (LoRa.begin checks the chip version)
bool startLoRa() {
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    return false;
  }
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);
  return true;
}

void sendLoRaPayload(const char* payload) {
  for (int attempt = 1; attempt <= 3; attempt++) {
    if (LoRa.beginPacket()) {
      LoRa.print(payload);
      if (LoRa.endPacket(true)) {
        #if DEBUG_MODE
        Serial.println(F("LoRa TX OK"));
        #endif
        return;
      }
    }
    if (attempt < 3) {
      delay(500);
    }
  }
  #if DEBUG_MODE
  Serial.println(F("LoRa TX failed"));
  #endif
}

// ============================================
// NEARBYCONNECT (ESP-NOW)
// ============================================

// Display acknowledgement: {"a":"US3847","c":123} - runs in the WiFi task, keep it short
void handleNearbyAck(const uint8_t* data, int len) {
  if (len <= 0 || len >= 64) return;
  char buf[64];
  memcpy(buf, data, len);
  buf[len] = '\0';
  if (strstr(buf, "\"a\"") == NULL || strstr(buf, deviceID) == NULL) return;
  const char* c = strstr(buf, "\"c\":");
  if (c == NULL) return;
  nearbyAckCounter = strtoul(c + 4, NULL, 10);
  nearbyAckFlag = true;
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onNearbyReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  handleNearbyAck(data, len);
}
#else
void onNearbyReceive(const uint8_t* mac, const uint8_t* data, int len) {
  handleNearbyAck(data, len);
}
#endif

bool startNearby() {
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("NearbyConnect init failed"));  // Always show errors
    return false;
  }
  esp_now_register_recv_cb(onNearbyReceive);

  // Broadcast peer - channel 0 means "whatever channel the radio is on"
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = 0;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  if (WiFi.status() != WL_CONNECTED) {
    setRadioChannel(nearbyChannel);
  }
  return true;
}

void setRadioChannel(uint8_t ch) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

// Hopping channels is only safe when nothing else pins the radio:
// not joined to a router, and nobody connected to the setup AP.
bool nearbyCanHop() {
  if (WiFi.status() == WL_CONNECTED) return false;
  if (apModeActive && WiFi.softAPgetStationNum() > 0) return false;
  return true;
}

void nearbySend() {
  esp_now_send(BROADCAST_MAC, (const uint8_t*)txPayload, strlen(txPayload));
  nearbySentAt = millis();
}

// Search order: last known channel first, then 1..13
uint8_t nearbySearchChannel(uint8_t index) {
  if (index == 0) return nearbyChannel;
  uint8_t ch = index;
  return (ch >= nearbyChannel) ? ch + 1 : ch;  // Skip the channel already tried
}

// Called once per reading: deliver txPayload to the display
void nearbyBeginDelivery() {
  if (!nearbyReady) return;
  nearbyCounter = PACKET_COUNTER;
  nearbyAckFlag = false;
  nearbyAttempt = 1;

  if (nearbyLocked || !nearbyCanHop()) {
    nearbyState = NB_WAIT_ACK;
    nearbySend();
  } else {
    // Display not found yet - sweep the channels
    nearbyState = NB_SEARCH;
    nearbySearchIndex = 0;
    setRadioChannel(nearbySearchChannel(0));
    nearbySend();
  }
}

void nearbyLockChannel(uint8_t ch) {
  nearbyLocked = true;
  nearbyMissed = 0;
  lastNearbyAck = millis();
  if (ch >= 1 && ch <= 13 && ch != nearbyChannel) {
    nearbyChannel = ch;
    saveLinkSettings();
  }
  #if DEBUG_MODE
  Serial.print(F("NearbyConnect: display found on channel "));
  Serial.println(ch);
  #endif
}

// Non-blocking delivery: retries while locked, channel sweep while searching
void serviceNearby() {
  if (!nearbyReady || nearbyState == NB_IDLE) return;

  unsigned long now = millis();

  if (nearbyAckFlag && nearbyAckCounter == nearbyCounter) {
    nearbyAckFlag = false;
    uint8_t ch = (nearbyState == NB_SEARCH) ? nearbySearchChannel(nearbySearchIndex) : (uint8_t)WiFi.channel();
    nearbyLockChannel(ch);
    nearbyState = NB_IDLE;
    return;
  }

  if (nearbyState == NB_WAIT_ACK && now - nearbySentAt >= NEARBY_ACK_WAIT_MS) {
    if (nearbyAttempt < NEARBY_RETRIES) {
      nearbyAttempt++;
      nearbySend();
    } else {
      nearbyState = NB_IDLE;
      if (nearbyMissed < 255) nearbyMissed++;
      if (nearbyMissed >= NEARBY_MISSED_BEFORE_SEARCH) {
        nearbyLocked = false;  // Next reading sweeps the channels
        #if DEBUG_MODE
        Serial.println(F("NearbyConnect: display not answering - will search channels"));
        #endif
      }
    }
  } else if (nearbyState == NB_SEARCH && now - nearbySentAt >= NEARBY_SEARCH_DWELL_MS) {
    if (!nearbyCanHop() || nearbySearchIndex >= 12) {
      // Swept all 13 channels (or hopping became unsafe) - try again next reading
      nearbyState = NB_IDLE;
      setRadioChannel(nearbyChannel);
      return;
    }
    nearbySearchIndex++;
    setRadioChannel(nearbySearchChannel(nearbySearchIndex));
    nearbySend();
  }
}

// ============================================
// MEASUREMENT CYCLE (every transmissionInterval)
// ============================================

// Read the sensor and ALWAYS send: data when the reading worked, heartbeat when it did not
void measurementCycle() {
  #if DEBUG_MODE
  Serial.println(F("\n--- TX Cycle ---"));
  #endif

  float calibratedDistance = readDistanceCM();
  bool hasDistance = calibratedDistance > 0;
  if (hasDistance) {
    lastReading = calibratedDistance;
  }

  PACKET_COUNTER++;
  if (PACKET_COUNTER >= COUNTER_RESET_LIMIT) {
    SESSION_ID = (esp_random() % 65535UL) + 1;
    PACKET_COUNTER = 1;
  }

  lastSentHasDistance = hasDistance;
  lastSentDistance = hasDistance ? (int)(calibratedDistance + 0.5) : 0;

  if (hasDistance) {
    snprintf(txPayload, sizeof(txPayload), "{\"i\":\"%s\",\"s\":%lu,\"c\":%lu,\"d\":%d}",
             deviceID, SESSION_ID, PACKET_COUNTER, lastSentDistance);
  } else {
    snprintf(txPayload, sizeof(txPayload), "{\"i\":\"%s\",\"s\":%lu,\"c\":%lu,\"h\":1}",
             deviceID, SESSION_ID, PACKET_COUNTER);
  }

  #if DEBUG_MODE
  Serial.print(F("TX: "));
  Serial.println(txPayload);
  #endif

  if (useLoRa && loraInitialized) {
    sendLoRaPayload(txPayload);
  }
  if (useNearby) {
    nearbyBeginDelivery();
  }
  // Home Network: nothing to send - the display reads /api/r
}

// Home Network: latest payload plus this sensor's WiFi signal
void handleReading() {
  if (!checkUserAgent()) {
    server.send(403, "text/plain", "Access Denied");
    return;
  }

  char buf[112];
  long rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  if (PACKET_COUNTER == 0) {
    // No reading taken yet since boot
    snprintf(buf, sizeof(buf), "{\"i\":\"%s\",\"s\":%lu,\"c\":0,\"h\":1,\"r\":%ld}", deviceID, SESSION_ID, rssi);
  } else if (lastSentHasDistance) {
    snprintf(buf, sizeof(buf), "{\"i\":\"%s\",\"s\":%lu,\"c\":%lu,\"d\":%d,\"r\":%ld}",
             deviceID, SESSION_ID, PACKET_COUNTER, lastSentDistance, rssi);
  } else {
    snprintf(buf, sizeof(buf), "{\"i\":\"%s\",\"s\":%lu,\"c\":%lu,\"h\":1,\"r\":%ld}",
             deviceID, SESSION_ID, PACKET_COUNTER, rssi);
  }
  server.send(200, "application/json", buf);
}

/*
SENSEFLOW TANK SENSOR (TRANSMITTER) v55.0.0
===========================================

CONFIGURATION SUMMARY:
---------------------
Firmware Version: 55.0.0 (code SF-TC-2026-9867)
Web Port: 7689
AP Suffix: _mvstechsensors
AP Password: mvstech9867
Web Password: 133611
User-Agent: MVStech7689
AP Timeout: 2 min default, extends on activity, max 5 min
            (closing the AP keeps NearbyConnect / Home Network running)
DEBUG_MODE: 0 = minimal output, 1 = full debug (for developers)

DEVICE ID:
---------
- Format: US + 4 random digits (e.g., US3847)
- Generated randomly on first boot, configurable via web UI, saved to EEPROM

CONNECTION TYPES (setup page > Connection Type):
-----------------------------------------------
0 Auto          - Long Range if the module is fitted, else Home Network +
                  NearbyConnect (WiFi details set), else NearbyConnect
1 Long Range    - LoRa 433 MHz, SF10, BW 125k, CR 4/8, sync 0x12, 20 dBm
2 NearbyConnect - ESP-NOW broadcast on the display's WiFi channel. The display
                  answers {"a":"US3847","c":<counter>}. No answer for 3 readings
                  -> sweep channels 1-13 (skipped while a phone is on the setup
                  AP or the sensor is joined to a router).
3 Home Network  - joins home WiFi, advertises mDNS _mvstech._tcp (TXT device=ID);
                  the display polls http://<ip>:7689/api/r
4 Home Network + NearbyConnect - both at once

PAYLOAD (every cycle, all links):
--------------------------------
{"i":"US3847","s":51,"c":102,"d":143}   data, d = calibrated distance in cm
{"i":"US3847","s":51,"c":103,"h":1}     heartbeat - sensor alive, reading failed
s = session (new random number each boot), c = counter (duplicate rejection)
/api/r adds "r": the sensor's WiFi RSSI.

OTA UPDATE:
----------
TCP-based OTA on port 8089 (MVS App or Python script), firmware code SF-TC-2026-9867

HARDWARE PINS:
-------------
LoRa: SS=5, RST=14, DIO0=26 (optional)
Ultrasonic: TRIG=25, ECHO=34

EEPROM LAYOUT:
-------------
0: Zone address            1: Zone valid flag
2-11: Device ID            12: Device ID magic number
13-18: reserved (old WP settings)
19-86: UI settings (password, AP, WiFi)
87-96: Calibration
97: Connection type        98: Connection magic     99: NearbyConnect channel

USAGE:
-----
1. Power on device
2. Connect to WiFi: USxxxx_mvstechsensors (within 5 minutes of power-on)
3. Open app or browser: http://192.168.4.1:7689, password 133611
4. Pick the Connection Type (and WiFi details for Home Network)
5. On the display: Settings > Sensor Connection > Scan, tap this sensor's ID
*/
