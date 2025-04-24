/**
 * Enhanced CAN Logger with Hex Data Decoding
 * 
 * Version: 2.2.0
 * Date: 2025-04-24
 * Author: caseyn
 */

#include <ESP32-TWAI-CAN.hpp>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD.h>
#include "esp_pm.h"
#include "time.h"

// Pin definitions
#define CAN_TX     5
#define CAN_RX     4

#define SD_CS   13
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK  18

// SD Card settings
#define SPI_FREQUENCY  25000000  // 25MHz SPI clock
#define MAX_LOG_SIZE   5000000   // 5MB max file size before rotation
#define BUFFER_SIZE    50        // Number of messages to buffer before writing
#define DEFAULT_LOG_FILE "/current_log.csv"  // Default current log filename

// WiFi settings
#define WIFI_SSID ""        // Your SSID
#define WIFI_PASSWORD ""    // Your password
#define AP_SSID "ESP32_CAN_Logger"
#define AP_PASSWORD "12345678"

// Time settings
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC 0    // UTC+0
#define DAYLIGHT_OFFSET_SEC 0

// Message buffering
String messageBuffer[BUFFER_SIZE];
int bufferIndex = 0;
unsigned long lastFlushTime = 0;
const unsigned long FLUSH_INTERVAL = 5000;  // 5 seconds max time between writes

// File management
File logFile;
bool fileOpen = false;
unsigned long logFileSize = 0;
int fileCounter = 0;
const unsigned long LOG_CHECK_INTERVAL = 30000;  // Check log size every 30 seconds
unsigned long lastLogCheckTime = 0;

// Statistics
unsigned long messagesLogged = 0;
unsigned long lastFrameTime = 0;
const unsigned long DATA_TIMEOUT = 5000;  // 5 seconds timeout for CAN data
unsigned long startupTime = 0;  // Track ESP32 uptime since boot

// Recent message cache for display
#define MAX_RECENT_MESSAGES 50
struct RecentMessage {
  unsigned long timestamp;
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
  String decodedValue;
};
RecentMessage recentMessages[MAX_RECENT_MESSAGES];
int recentMessageIndex = 0;
int totalRecentMessages = 0;

WebServer server(80);
bool sdCardOK = false;

// CAN message decoders
String decodeCANData(uint32_t id, uint8_t dlc, const uint8_t* data);
String decodeVehicleSpeed(const uint8_t* data);
String decodeEngineRPM(const uint8_t* data);
String decodeOilPressure(const uint8_t* data);
String decodeCoolantTemp(const uint8_t* data);
String decodeFuelLevel(const uint8_t* data);
String decodeGearPosition(const uint8_t* data);
String decodeThrottlePosition(const uint8_t* data);
String decodeEngineLoad(const uint8_t* data);
String decodeBrakeStatus(const uint8_t* data);
String decodeOBDResponse(uint8_t dlc, const uint8_t* data);

void setup() {
  // Record startup time first thing
  startupTime = millis();
  
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n----- Enhanced CAN Logger with Data Decoding -----");
  Serial.println("v2.2.0 - 2025-04-24");
  Serial.println("User: caseyn");
  
  // Set up power management
  setupPowerSaving();
  
  // Initialize WiFi and time
  if (connectToWiFi()) {
    configureTime();
  } else {
    startAccessPoint();
  }
  
  // Initialize SD card with optimized settings
  initializeSDCard();
  
  // Rotate any existing log file before starting
  rotateLogFile();
  
  // Set up web server
  setupWebServer();
  
  // Initialize CAN bus
  ESP32Can.setPins(CAN_TX, CAN_RX);
  Serial.println("Starting CAN...");
  if (ESP32Can.begin(ESP32Can.convertSpeed(500))) {
    Serial.println("CAN bus started successfully!");
  } else {
    Serial.println("CAN bus failed to start!");
  }
  
  // Create initial log file
  createLogFile();
}

void loop() {
  server.handleClient();
  
  // Process incoming CAN messages
  CanFrame rxFrame;
  if (ESP32Can.readFrame(rxFrame)) {
    lastFrameTime = millis();
    
    // Add to recent messages cache
    storeRecentMessage(rxFrame);
    
    // Only log messages that pass our filter
    if (shouldLogMessage(rxFrame.identifier)) {
      logCANFrame(rxFrame);
    }
  }
  
  // Check if we need to flush the buffer
  unsigned long currentTime = millis();
  if (bufferIndex > 0 && 
      (currentTime - lastFlushTime > FLUSH_INTERVAL || bufferIndex >= BUFFER_SIZE)) {
    writeBufferToSD();
  }
  
  // Periodically check log file size
  if (currentTime - lastLogCheckTime > LOG_CHECK_INTERVAL) {
    checkLogFileSize();
    lastLogCheckTime = currentTime;
  }
  
  // Handle WiFi reconnection if needed
  handleWiFiConnection();
}

void storeRecentMessage(const CanFrame &frame) {
  // Store in circular buffer
  recentMessages[recentMessageIndex].timestamp = millis();
  recentMessages[recentMessageIndex].id = frame.identifier;
  recentMessages[recentMessageIndex].dlc = frame.data_length_code;
  
  // Copy data bytes
  for (int i = 0; i < frame.data_length_code && i < 8; i++) {
    recentMessages[recentMessageIndex].data[i] = frame.data[i];
  }
  
  // Decode the data
  recentMessages[recentMessageIndex].decodedValue = 
      decodeCANData(frame.identifier, frame.data_length_code, frame.data);
  
  // Update indices
  recentMessageIndex = (recentMessageIndex + 1) % MAX_RECENT_MESSAGES;
  if (totalRecentMessages < MAX_RECENT_MESSAGES) {
    totalRecentMessages++;
  }
}

// Main decoder function that dispatches to specific decoders based on ID
String decodeCANData(uint32_t id, uint8_t dlc, const uint8_t* data) {
  switch (id) {
    case 0x100:
      return decodeVehicleSpeed(data);
    case 0x101:
      return decodeEngineRPM(data);
    case 0x102:
      return decodeOilPressure(data);
    case 0x103:
      return decodeCoolantTemp(data);
    case 0x104:
      return decodeFuelLevel(data);
    case 0x105:
      return decodeGearPosition(data);
    case 0x106:
      return decodeThrottlePosition(data);
    case 0x107:
      return decodeEngineLoad(data);
    case 0x108:
      return decodeBrakeStatus(data);
    case 0x7E8:
      return decodeOBDResponse(dlc, data);
    default:
      // If unknown ID, return empty string - we'll show raw hex
      return "";
  }
}

// Individual decoder functions
String decodeVehicleSpeed(const uint8_t* data) {
  uint16_t speed = (data[0] << 8) | data[1];
  float speedValue = speed / 10.0f;  // 0.1 mph resolution
  return String(speedValue, 1) + " mph";
}

String decodeEngineRPM(const uint8_t* data) {
  uint16_t rpm = (data[0] << 8) | data[1];
  return String(rpm) + " RPM";
}

String decodeOilPressure(const uint8_t* data) {
  return String(data[0]) + " PSI";
}

String decodeCoolantTemp(const uint8_t* data) {
  return String(data[0]) + " °F";
}

String decodeFuelLevel(const uint8_t* data) {
  return String(data[0]) + "%";
}

String decodeGearPosition(const uint8_t* data) {
  char gear = (char)data[0];
  
  // Handle known gear positions
  switch (gear) {
    case 'P': return "Park (P)";
    case 'R': return "Reverse (R)";
    case 'N': return "Neutral (N)";
    case 'D': return "Drive (D)";
    case '1': return "1st Gear";
    case '2': return "2nd Gear";
    case '3': return "3rd Gear";
    case '4': return "4th Gear";
    case '5': return "5th Gear";
    case '6': return "6th Gear";
    default: return "Unknown (" + String(gear) + ")";
  }
}

String decodeThrottlePosition(const uint8_t* data) {
  return String(data[0]) + "% Throttle";
}

String decodeEngineLoad(const uint8_t* data) {
  return String(data[0]) + "% Load";
}

String decodeBrakeStatus(const uint8_t* data) {
  return data[0] ? "Brakes ON" : "Brakes OFF";
}

String decodeOBDResponse(uint8_t dlc, const uint8_t* data) {
  if (dlc < 3) return "Invalid OBD";
  
  if (data[1] == 0x41) { // OBD mode 01 response
    switch (data[2]) {
      case 0x05: // Coolant temperature
        return "Coolant: " + String(data[3] - 40) + " °C";
      case 0x0C: // RPM (need 4+ bytes)
        if (dlc >= 5) {
          uint16_t A = data[3];
          uint16_t B = data[4];
          float rpm = ((A * 256.0) + B) / 4.0;
          return "OBD RPM: " + String(rpm, 0);
        }
        break;
      case 0x0D: // Vehicle speed
        return "OBD Speed: " + String(data[3]) + " km/h";
      case 0x11: // Throttle position
        return "OBD Throttle: " + String((data[3] * 100) / 255) + "%";
    }
  }
  
  // If we can't decode the specific OBD response
  return "OBD Mode " + String(data[1], HEX) + " PID " + String(data[2], HEX);
}

void setupPowerSaving() {
  #ifdef CONFIG_PM_ENABLE
    esp_pm_config_esp32_t pm_config = {
      .max_freq_mhz = 240,
      .min_freq_mhz = 80,
      .light_sleep_enable = true
    };
    esp_pm_configure(&pm_config);
    Serial.println("Power management configured");
  #else
    Serial.println("Power management not available");
  #endif
}

void configureTime() {
  // Configure time with NTP
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  Serial.println("Waiting for NTP time sync...");
  
  int retries = 0;
  time_t now;
  struct tm timeinfo;
  while (retries < 5) {
    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_year >= 120) { // 2020 or later means time is set
      Serial.printf("Time synchronized: %04d-%02d-%02d %02d:%02d:%02d\n", 
                  timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      break;
    }
    Serial.print(".");
    delay(1000);
    retries++;
  }
  
  if (timeinfo.tm_year < 120) {
    Serial.println("\nFailed to get NTP time. Using default timestamp format.");
  }
}

void initializeSDCard() {
  // Initialize SPI with optimized settings
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  SPI.setFrequency(SPI_FREQUENCY);
  
  Serial.println("Initializing SD card with optimized settings...");
  
  // Try to initialize SD card with retries
  int retryCount = 0;
  while (!SD.begin(SD_CS) && retryCount < 3) {
    Serial.println("SD card initialization failed! Retrying...");
    delay(1000);
    retryCount++;
  }
  
  if (retryCount < 3) {
    Serial.println("SD card initialized successfully.");
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    
    // List files in root directory
    listDir("/", 0);
    
    sdCardOK = true;
  } else {
    Serial.println("SD card failed after multiple attempts.");
    sdCardOK = false;
  }
}

void rotateLogFile() {
  if (!sdCardOK) {
    Serial.println("Cannot rotate log file - SD card not available");
    return;
  }
  
  // Check if current log file exists
  if (SD.exists(DEFAULT_LOG_FILE)) {
    char newFilename[40];
    
    // Get current time for filename
    time_t now;
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    
    // Format timestamped filename
    if (timeinfo.tm_year >= 120) { // Valid time available
      sprintf(newFilename, "/log_%04d-%02d-%02d_%02d-%02d-%02d.csv", 
              timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
              timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
      // Fallback if time not available
      sprintf(newFilename, "/log_archive_%06lu.csv", millis());
    }
    
    Serial.printf("Renaming current log to: %s\n", newFilename);
    
    // Close file if open
    if (fileOpen && logFile) {
      logFile.close();
      fileOpen = false;
    }
    
    // Rename the file
    if (SD.rename(DEFAULT_LOG_FILE, newFilename)) {
      Serial.println("Log file renamed successfully");
    } else {
      Serial.println("Error renaming log file");
      // If rename fails, try to delete the current log as a fallback
      if (SD.remove(DEFAULT_LOG_FILE)) {
        Serial.println("Removed old log file instead");
      }
    }
    
    // Update file listing after rotation
    listDir("/", 0);
  } else {
    Serial.println("No existing log file to rotate");
  }
}

void listDir(const char* dirname, uint8_t levels) {
  if (!sdCardOK) return;
  
  Serial.printf("Listing directory: %s\n", dirname);
  
  File root = SD.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }
  
  File file = root.openNextFile();
  int fileCount = 0;
  unsigned long totalSize = 0;
  
  while (file && fileCount < 10) { // Limit to first 10 files to avoid console spam
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      
      unsigned long fileSize = file.size();
      totalSize += fileSize;
      
      if (fileSize < 1024) {
        Serial.print(fileSize);
        Serial.println(" B");
      } else if (fileSize < (1024 * 1024)) {
        Serial.print(fileSize / 1024);
        Serial.println(" KB");
      } else {
        Serial.print(fileSize / (1024 * 1024));
        Serial.println(" MB");
      }
    }
    file = root.openNextFile();
    fileCount++;
  }
  
  if (file) { // More files exist
    Serial.println("  ... more files exist (not shown)");
  }
  
  Serial.printf("Total size of listed files: %.2f MB\n", totalSize / (1024.0 * 1024.0));
  
  root.close();
}

bool shouldLogMessage(uint32_t canId) {
  // High priority messages - log every occurrence
  static const uint32_t criticalIds[] = {
    0x100,  // Vehicle speed
    0x101,  // Engine RPM
    0x102,  // Oil pressure
    0x103   // Coolant temperature
  };
  
  for (int i = 0; i < sizeof(criticalIds)/sizeof(criticalIds[0]); i++) {
    if (canId == criticalIds[i]) {
      return true;
    }
  }
  
  // Medium priority messages - log at medium frequency
  static const uint32_t mediumIds[] = {
    0x104,  // Fuel level
    0x105,  // Gear position
    0x106   // Throttle position
  };
  
  static unsigned long lastMediumLog[256] = {0};
  for (int i = 0; i < sizeof(mediumIds)/sizeof(mediumIds[0]); i++) {
    if (canId == mediumIds[i]) {
      unsigned long currentTime = millis();
      uint8_t bucket = canId & 0xFF;
      if (currentTime - lastMediumLog[bucket] > 1000) {  // 1 second interval
        lastMediumLog[bucket] = currentTime;
        return true;
      }
      return false;
    }
  }
  
  // Low priority messages - log less frequently
  static unsigned long lastLowPriorityLog[256] = {0};
  uint8_t bucket = canId & 0xFF;
  unsigned long currentTime = millis();
  
  if (currentTime - lastLowPriorityLog[bucket] > 5000) {  // 5 second interval
    lastLowPriorityLog[bucket] = currentTime;
    return true;
  }
  
  return false;
}

void createLogFile() {
  if (!sdCardOK) {
    Serial.println("Cannot create log file - SD card not available");
    return;
  }
  
  // Close current file if open
  if (fileOpen && logFile) {
    logFile.close();
    fileOpen = false;
  }
  
  // Use the fixed default log filename
  Serial.printf("Creating new log file: %s\n", DEFAULT_LOG_FILE);
  
  logFile = SD.open(DEFAULT_LOG_FILE, FILE_WRITE);
  if (logFile) {
    // Write CSV header - now includes decoded value column
    // DLC - Data Length Code - bytes
    logFile.println("Timestamp,ID,DLC,Data,DecodedValue");
    logFile.flush();
    fileOpen = true;
    logFileSize = logFile.size();
    Serial.printf("Log file created with header, initial size: %lu bytes\n", logFileSize);
  } else {
    Serial.println("Failed to create log file");
    fileOpen = false;
  }
}

void checkLogFileSize() {
  if (!fileOpen || !sdCardOK) return;
  
  if (logFileSize > MAX_LOG_SIZE) {
    Serial.printf("Log file size (%lu bytes) exceeded limit (%d bytes), rotating...\n", 
                 logFileSize, MAX_LOG_SIZE);
    
    // Close current file
    logFile.close();
    fileOpen = false;
    
    // Rotate the file
    rotateLogFile();
    
    // Create a new file
    createLogFile();
  } else {
    // Just print current size every 30 seconds
    Serial.printf("Current log file size: %lu bytes (%.2f MB)\n", 
                 logFileSize, logFileSize / (1024.0 * 1024.0));
  }
}

void logCANFrame(const CanFrame &frame) {
  // Format timestamp with milliseconds
  unsigned long currentTime = millis();
  
  // Format the log entry - hex data
  char hexData[24] = {0};  // 8 bytes * 3 chars (2 hex + space)
  for (int i = 0; i < frame.data_length_code; i++) {
    sprintf(&hexData[i*3], "%02X ", frame.data[i]);
  }
  
  // Decode the data
  String decodedValue = decodeCANData(frame.identifier, frame.data_length_code, frame.data);
  
  String logEntry = String(currentTime) + ",";
  logEntry += "0x" + String(frame.identifier, HEX) + ",";
  logEntry += String(frame.data_length_code) + ",";
  
  // Fix for the compiler error - append hexData first, then comma
  logEntry += hexData;
  logEntry += ",";  
  
  logEntry += decodedValue;  // Add the decoded value
  
  // Add to buffer
  if (bufferIndex < BUFFER_SIZE) {
    messageBuffer[bufferIndex++] = logEntry;
    messagesLogged++;
  }
  
  // Update last flush time
  if (bufferIndex == 1) {
    lastFlushTime = currentTime;
  }
}

void writeBufferToSD() {
  if (bufferIndex == 0 || !sdCardOK) return;
  
  unsigned long startTime = millis();
  
  if (!fileOpen) {
    createLogFile();
    if (!fileOpen) {
      Serial.println("Failed to open log file for writing");
      bufferIndex = 0;  // Clear buffer
      return;
    }
  }
  
  // Write all buffered entries
  int successCount = 0;
  for (int i = 0; i < bufferIndex; i++) {
    size_t bytesWritten = logFile.println(messageBuffer[i]);
    if (bytesWritten > 0) {
      logFileSize += bytesWritten + 1;  // +1 for newline
      successCount++;
    }
  }
  
  // Flush to SD card
  logFile.flush();
  
  unsigned long duration = millis() - startTime;
  Serial.printf("Wrote %d/%d messages in %lu ms, log size: %.2f MB\n", 
               successCount, bufferIndex, duration, logFileSize / (1024.0 * 1024.0));
  
  // Clear buffer
  bufferIndex = 0;
  lastFlushTime = millis();
}

bool connectToWiFi() {
  if (strlen(WIFI_SSID) == 0) {
    Serial.println("No WiFi SSID configured, skipping connection");
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to WiFi...");

  unsigned long startTime = millis();
  int retryCount = 0;
  const int maxRetries = 10;

  while (WiFi.status() != WL_CONNECTED && retryCount < maxRetries) {
    delay(1000);
    retryCount++;
    Serial.print(".");
  }
  
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected! IP Address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("Failed to connect to WiFi.");
    return false;
  }
}

void startAccessPoint() {
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.print("Access Point started. IP Address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Failed to start Access Point.");
  }
}

void handleWiFiConnection() {
  if (WiFi.getMode() == WIFI_STA && WiFi.status() != WL_CONNECTED) {
    static unsigned long lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > 30000) {  // Try every 30 seconds
      Serial.println("WiFi disconnected. Reconnecting...");
      lastReconnectAttempt = millis();
      if (!connectToWiFi()) {
        Serial.println("Switching to Access Point mode...");
        startAccessPoint();
      }
    }
  }
}

void setupWebServer() {
  server.enableCORS(true);  // Enable CORS for better web compatibility
  
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/log", HTTP_GET, handleLog);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/rotate", HTTP_GET, handleRotate);  // New endpoint to force rotation
  server.on("/list", HTTP_GET, handleListFiles); // New endpoint to list files
  
  // Serve static files
  server.serveStatic("/logs", SD, "/", "max-age=86400");
  
  server.begin();
  Serial.println("Web server started");
}

void handleRoot() {
  String html = R"html(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 CAN Logger</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f4; }
        .container { max-width: 1200px; margin: 0 auto; }
        .card { background: white; border-radius: 8px; padding: 20px; margin-bottom: 20px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        h1 { color: #333; }
        .status { padding: 10px; margin: 10px 0; border-radius: 5px; }
        .online { background-color: #d4edda; color: #155724; }
        .offline { background-color: #f8d7da; color: #721c24; }
        .warning { background-color: #fff3cd; color: #856404; }
        .btn {
            background-color: #4CAF50;
            border: none;
            color: white;
            padding: 10px 20px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 16px;
            margin: 4px 2px;
            cursor: pointer;
            border-radius: 4px;
        }
        .btn-blue { background-color: #2196F3; }
        .btn-orange { background-color: #FF9800; }
        .stats {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(200px, 1fr));
            gap: 10px;
            margin-bottom: 20px;
        }
        .stat-box {
            background-color: #e9ecef;
            padding: 15px;
            border-radius: 5px;
            text-align: center;
        }
        .stat-value {
            font-size: 1.5em;
            font-weight: bold;
        }
        table {
            width: 100%;
            border-collapse: collapse;
        }
        table, th, td {
            border: 1px solid #ddd;
        }
        th, td {
            padding: 10px;
            text-align: left;
        }
        th {
            background-color: #f0f0f0;
        }
        .raw-data { 
            font-family: monospace; 
            color: #555;
        }
        .decoded-value { 
            font-weight: bold; 
            color: #0066cc;
        }
        .can-id {
            font-family: monospace;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <h1>ESP32 CAN Logger</h1>
            <div id="sdStatus" class="status">SD Card: Checking...</div>
            <div id="canStatus" class="status">CAN Bus: Checking...</div>
        </div>
        
        <div class="card">
            <h2>Statistics</h2>
            <div class="stats">
                <div class="stat-box">
                    <div>Messages Logged</div>
                    <div class="stat-value" id="messagesLogged">0</div>
                </div>
                <div class="stat-box">
                    <div>Current File Size</div>
                    <div class="stat-value" id="fileSize">0 KB</div>
                </div>
                <div class="stat-box">
                    <div>Uptime</div>
                    <div class="stat-value" id="uptime">0 min</div>
                </div>
            </div>
        </div>
        
        <div class="card">
            <h2>Actions</h2>
            <button class="btn" onclick="window.location.href='/log'">Download Current Log</button>
            <button class="btn btn-blue" onclick="refreshData()">Refresh Status</button>
            <button class="btn btn-orange" onclick="rotateLog()">Rotate Log File</button>
            <button class="btn" onclick="window.location.href='/list'">List All Logs</button>
        </div>
        
        <div class="card">
            <h2>Recent CAN Messages</h2>
            <div id="canMessages">Loading...</div>
        </div>
    </div>

    <script>
        // Initial data load
        window.onload = function() {
            refreshData();
            setInterval(refreshData, 5000); // Auto refresh every 5 seconds
        };
        
        function refreshData() {
            // Update status information
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('sdStatus').className = 
                        data.sdCardOK ? 'status online' : 'status offline';
                    document.getElementById('sdStatus').innerHTML = 
                        data.sdCardOK ? 'SD Card: Connected' : 'SD Card: Error';
                        
                    document.getElementById('canStatus').className = 
                        data.canActive ? 'status online' : 'status warning';
                    document.getElementById('canStatus').innerHTML = 
                        data.canActive ? 'CAN Bus: Active' : 'CAN Bus: No recent messages';
                    
                    document.getElementById('messagesLogged').innerText = data.messagesLogged.toLocaleString();
                    
                    // Format file size nicely
                    let size = data.currentFileSize;
                    let sizeStr;
                    if (size < 1024) {
                        sizeStr = size + ' B';
                    } else if (size < 1024 * 1024) {
                        sizeStr = (size / 1024).toFixed(1) + ' KB';
                    } else {
                        sizeStr = (size / (1024 * 1024)).toFixed(1) + ' MB';
                    }
                    document.getElementById('fileSize').innerText = sizeStr;
                    
                    // Using server-provided uptime
                    document.getElementById('uptime').innerText = data.uptimeMinutes + ' min';
                });
                
            // Get recent messages
            fetch('/data')
                .then(response => response.text())
                .then(data => {
                    document.getElementById('canMessages').innerHTML = data;
                });
        }
        
        function rotateLog() {
            if (confirm('Rotate log file now? Current log will be archived.')) {
                fetch('/rotate')
                    .then(response => response.text())
                    .then(data => {
                        alert(data);
                        refreshData();
                    });
            }
        }
    </script>
</body>
</html>
)html";

  server.send(200, "text/html", html);
}

void handleData() {
  // Display the recent messages instead of reading from the file
  String html = "<table><tr><th>Time</th><th>ID</th><th>Data</th><th>Decoded Value</th></tr>";
  
  if (totalRecentMessages == 0) {
    html += "<tr><td colspan='4'>No messages received yet</td></tr>";
  } else {
    // Display messages in reverse chronological order (newest first)
    for (int i = 0; i < totalRecentMessages; i++) {
      int idx = (recentMessageIndex - 1 - i + MAX_RECENT_MESSAGES) % MAX_RECENT_MESSAGES;
      
      // Format timestamp as relative time
      unsigned long age = (millis() - recentMessages[idx].timestamp) / 1000; // in seconds
      String timeStr;
      
      if (age < 60) {
        timeStr = String(age) + "s ago";
      } else if (age < 3600) {
        timeStr = String(age / 60) + "m " + String(age % 60) + "s ago";
      } else {
        timeStr = String(age / 3600) + "h " + String((age % 3600) / 60) + "m ago";
      }
      
      html += "<tr>";
      html += "<td>" + timeStr + "</td>";
      html += "<td class='can-id'>0x" + String(recentMessages[idx].id, HEX) + "</td>";
      
      // Format data bytes
      String dataHex = "";
      for (int j = 0; j < recentMessages[idx].dlc; j++) {
        char hexByte[4];
        sprintf(hexByte, "%02X ", recentMessages[idx].data[j]);
        dataHex += hexByte;
      }
      
      html += "<td class='raw-data'>" + dataHex + "</td>";
      
      // Decoded value
      if (recentMessages[idx].decodedValue.length() > 0) {
        html += "<td class='decoded-value'>" + recentMessages[idx].decodedValue + "</td>";
      } else {
        html += "<td class='raw-data'>-</td>";
      }
      
      html += "</tr>";
      
      // Only show the most recent 20 messages
      if (i >= 19) break;
    }
  }
  
  html += "</table>";
  server.send(200, "text/html", html);
}

void handleLog() {
  // Force buffer flush
  writeBufferToSD();
  
  if (sdCardOK) {
    // Close current file if open for writing
    bool wasOpen = fileOpen;
    if (fileOpen && logFile) {
      logFile.close();
      fileOpen = false;
    }
    
    File file = SD.open(DEFAULT_LOG_FILE);
    if (file) {
      server.streamFile(file, "text/csv");
      file.close();
    } else {
      server.send(404, "text/plain", "Log file not found");
    }
    
    // Reopen for writing
    if (wasOpen) {
      logFile = SD.open(DEFAULT_LOG_FILE, FILE_APPEND);
      if (logFile) {
        fileOpen = true;
      }
    }
  } else {
    server.send(503, "text/plain", "SD card not available");
  }
}

void handleStatus() {
  // Force buffer flush to ensure all data is written
  writeBufferToSD();
  
  // Calculate if CAN bus is active (received message in last 5 seconds)
  bool canActive = (millis() - lastFrameTime < DATA_TIMEOUT);
  
  // Calculate uptime in minutes - fixed to use ESP uptime
  unsigned long uptimeMinutes = (millis() - startupTime) / 60000;
  
  String json = "{\"sdCardOK\":" + String(sdCardOK ? "true" : "false") + 
                ",\"canActive\":" + String(canActive ? "true" : "false") + 
                ",\"messagesLogged\":" + String(messagesLogged) + 
                ",\"currentFileSize\":" + String(logFileSize) + 
                ",\"uptimeMinutes\":" + String(uptimeMinutes) + "}";
                
  server.send(200, "application/json", json);
}

void handleRotate() {
  // Force rotation of the log file
  if (sdCardOK) {
    // Close current file if open
    if (fileOpen && logFile) {
      logFile.close();
      fileOpen = false;
    }
    
    rotateLogFile();
    createLogFile();
    
    server.send(200, "text/plain", "Log file rotated successfully");
  } else {
    server.send(503, "text/plain", "SD card not available");
  }
}

void handleListFiles() {
  // Implementation remains the same
  if (!sdCardOK) {
    server.send(503, "text/plain", "SD card not available");
    return;
  }
  
  String html = R"html(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 CAN Logger - Files</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f4; }
        .container { max-width: 1200px; margin: 0 auto; }
        .card { background: white; border-radius: 8px; padding: 20px; margin-bottom: 20px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        h1, h2 { color: #333; }
        table { width: 100%; border-collapse: collapse; }
        table, th, td { border: 1px solid #ddd; }
        th, td { padding: 10px; text-align: left; }
        th { background-color: #f0f0f0; }
        .btn {
            background-color: #4CAF50;
            border: none;
            color: white;
            padding: 10px 20px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 16px;
            margin: 4px 2px;
            cursor: pointer;
            border-radius: 4px;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <h1>Log Files</h1>
            <a class="btn" href="/">Back to Dashboard</a>
            <p>Click on a file name to download it.</p>
            <table>
                <tr>
                    <th>Filename</th>
                    <th>Size</th>
                    <th>Date</th>
                </tr>
)html";

  File root = SD.open("/");
  if (!root) {
    html += "<tr><td colspan='3'>Failed to open root directory</td></tr>";
  } else if (!root.isDirectory()) {
    html += "<tr><td colspan='3'>Root is not a directory</td></tr>";
  } else {
    File file = root.openNextFile();
    while (file) {
      if (!file.isDirectory()) {
        String filename = file.name();
        
        // Skip system files
        if (!filename.startsWith(".")) {
          html += "<tr><td><a href='/logs";
          html += filename;
          html += "'>";
          html += filename;
          html += "</a></td><td>";
          
          // Format file size
          size_t fileSize = file.size();
          if (fileSize < 1024) {
            html += String(fileSize) + " B";
          } else if (fileSize < (1024 * 1024)) {
            html += String(fileSize / 1024) + " KB";
          } else {
            html += String(fileSize / (1024 * 1024)) + " MB";
          }
          
          html += "</td><td>";
          
          // Get file date from filename if it contains a date pattern
          if (filename.indexOf("_20") >= 0) {
            int dateStart = filename.indexOf("_20") + 1;
            html += filename.substring(dateStart, dateStart + 10);
          } else {
            html += "Unknown date";
          }
          
          html += "</td></tr>";
        }
      }
      file = root.openNextFile();
    }
  }
  
  html += R"html(
            </table>
        </div>
    </div>
</body>
</html>
)html";

  server.send(200, "text/html", html);
}
