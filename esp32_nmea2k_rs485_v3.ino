/*
 * esp32 NMEA 2k to RS485 gateway
 * Author: caseyn
 * Date: 2025-08-01
 */

#include <ESP32-TWAI-CAN.hpp>
#include <inttypes.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>

// Version information
#define SW_VERSION_MAJOR 3
#define SW_VERSION_MINOR 0
#define SW_VERSION_PATCH 0
#define SW_VERSION_STRING "v3.0.0"
#define SW_BUILD_DATE "2025-08-01"

// --- CAN Bus Definitions ---
#define CAN_TX 5
#define CAN_RX 4
#define CAN_SPEED 250  // NMEA2000 uses 250kbps

// RS485 Definitions for HW-097 module
#define RS485_RX_PIN 16     // Connect to RO pin on HW-097
#define RS485_TX_PIN 17     // Connect to DI pin on HW-097
#define RS485_DE_RE_PIN 21  // Connect to DE and RE pins on HW-097

#define RS485_BAUDRATE 76800
#define RESPONSE_TIMEOUT 3000

// RS485 message format
#define REQUEST_LEN 4
#define RESPONSE_LEN 13
#define MSG_TYPE_REQUEST 0x04
#define MSG_CMD_GET_DEPTH 0x09

HardwareSerial RS485Serial(2);

// --- Debug Output Control ---
bool debugNMEA2000 = false;  // Set true for verbose NMEA 2000 (CAN) debug
bool debugRS485 = false;     // Set true for verbose RS485 debug

#define DEBUG_PRINT_NMEA2000(...) do { if (debugNMEA2000) Serial.printf(__VA_ARGS__); } while (0)
#define DEBUG_PRINT_RS485(...)    do { if (debugRS485) Serial.printf(__VA_ARGS__); } while (0)

// --- Depth data storage --- rs485
uint16_t depth_tenths = 0;  // Depth in tenths of a foot (e.g., 123 = 12.3 ft)
uint8_t toggle_bit = 0;

// --- timeouts and defaults
unsigned long nmeaDepthTimeoutMs = 5000;  // Default: 5 seconds
uint16_t depth_timeout_value = 0;         // Value when timeout occurs (e.g., 0 = no depth)
unsigned long lastDepthReceivedMs = 0;
bool depth_valid = false;

// --- Data Structs ---
struct CurrentDepth {
  float currentDepth;
  float currentOffset;
  uint8_t currentRange;
};
CurrentDepth currentDepth;

struct CurrentGPS {
  double latitude;
  double longitude;
  double altitude;
  bool valid;
};
CurrentGPS currentGPS;

struct CurrentCOG {
  float cog;
  float sog;
  bool valid;
};
CurrentCOG currentCOG;

struct CurrentTemp {
  float waterTemp;
  float seaTemp;
};
CurrentTemp currentTemp;

struct CurrentMagVar {
  float variation;
  bool valid;
};
CurrentMagVar currentMagVar;

// --- PGN Handler Type ---
typedef void (*PGNHandler)(const CanFrame&);

// --- PGN Decoders ---
void handleDepthPGN(const CanFrame& frame) {
  // PGN 128267 Water Depth
  if (frame.data_length_code < 7) return;
  uint8_t sid = frame.data[0];
  uint32_t depthRaw = frame.data[1] | (frame.data[2] << 8) | (frame.data[3] << 16);
  int16_t offsetRaw = frame.data[4] | (frame.data[5] << 8);
  uint8_t rangeRaw = frame.data[6];
  currentDepth.currentDepth = depthRaw * 0.01f;
  currentDepth.currentOffset = offsetRaw * 0.001f;
  currentDepth.currentRange = (rangeRaw == 255) ? 0 : rangeRaw * 10;
  DEBUG_PRINT_NMEA2000("128267 Water Depth: SID = %s; Depth = %.2f m; Offset = %.3f m; Range = %s\n",
                       "Unknown", currentDepth.currentDepth, currentDepth.currentOffset,
                       (rangeRaw == 255) ? "Unknown" : String(currentDepth.currentRange).c_str());

  float depth_ft = currentDepth.currentDepth * 3.28084f;
  depth_tenths = (uint16_t)(depth_ft * 10.0f + 0.5f);  // tenths of a foot
  lastDepthReceivedMs = millis();
  depth_valid = true;
}

void handleGPSPGN_129025(const CanFrame& frame) {
  // PGN 129025 Position, Rapid Update
  if (frame.data_length_code < 8) return;
  int32_t lonRaw = (int32_t)(((uint32_t)frame.data[0]) | ((uint32_t)frame.data[1] << 8) | ((uint32_t)frame.data[2] << 16) | ((uint32_t)frame.data[3] << 24));
  int32_t latRaw = (int32_t)(((uint32_t)frame.data[4]) | ((uint32_t)frame.data[5] << 8) | ((uint32_t)frame.data[6] << 16) | ((uint32_t)frame.data[7] << 24));
  currentGPS.longitude = lonRaw * 1e-7;
  currentGPS.latitude = latRaw * 1e-7;
  currentGPS.valid = true;
  DEBUG_PRINT_NMEA2000("129025 Position, Rapid Update:  Latitude = %.7f; Longitude = %.7f\n", currentGPS.latitude, currentGPS.longitude);
}

void handleGPSPGN_129029(const CanFrame& frame) {
  // PGN 129029 GNSS Position Data
  if (frame.data_length_code < 43) return;
  uint8_t sid = frame.data[0];
  uint16_t dateRaw = frame.data[1] | (frame.data[2] << 8);
  uint32_t timeRaw = frame.data[3] | (frame.data[4] << 8) | (frame.data[5] << 16) | (frame.data[6] << 24);
  int32_t latRaw = (int32_t)(frame.data[9] | (frame.data[10] << 8) | (frame.data[11] << 16) | (frame.data[12] << 24));
  int32_t lonRaw = (int32_t)(frame.data[13] | (frame.data[14] << 8) | (frame.data[15] << 16) | (frame.data[16] << 24));
  int32_t altRaw = (int32_t)(frame.data[17] | (frame.data[18] << 8) | (frame.data[19] << 16) | (frame.data[20] << 24));
  currentGPS.latitude = latRaw * 1e-7;
  currentGPS.longitude = lonRaw * 1e-7;
  currentGPS.altitude = altRaw * 0.001;
  currentGPS.valid = true;
  DEBUG_PRINT_NMEA2000("129029 GNSS Position Data:  SID = Unknown; Date = %u; Time = %.4f; Latitude = %.7f; Longitude = %.7f; Altitude = %.6f m\n",
                       dateRaw, timeRaw / 10000.0, currentGPS.latitude, currentGPS.longitude, currentGPS.altitude);
}

void handleCOGPGN(const CanFrame& frame) {
  // PGN 129026 COG & SOG, Rapid Update
  if (frame.data_length_code < 8) return;
  uint8_t sid = frame.data[0];
  uint8_t ref = frame.data[1];
  uint16_t cogRaw = frame.data[2] | (frame.data[3] << 8);
  uint16_t sogRaw = frame.data[4] | (frame.data[5] << 8);
  currentCOG.cog = cogRaw * 0.0001f * 180.0f / 3.14159265358979323846f;  // radians to degrees
  currentCOG.sog = sogRaw * 0.01f;                                       // m/s
  currentCOG.valid = true;
  DEBUG_PRINT_NMEA2000("129026 COG & SOG, Rapid Update:  SID = Unknown; COG Reference = %s; COG = %.1f deg; SOG = %.2f m/s\n",
                       (ref ? "True" : "False"), currentCOG.cog, currentCOG.sog);
}

void handleMagVarPGN(const CanFrame& frame) {
  // PGN 127258 Magnetic Variation
  if (frame.data_length_code < 6) return;
  uint8_t sid = frame.data[0];
  uint8_t src = frame.data[1];
  int16_t varRaw = frame.data[4] | (frame.data[5] << 8);
  currentMagVar.variation = varRaw * 0.0001 * 180.0 / 3.14159265358979323846;
  currentMagVar.valid = true;
  DEBUG_PRINT_NMEA2000("127258 Magnetic Variation:  SID = Unknown; Source = %s; Variation = %.1f deg\n",
                       "Automatic Table", currentMagVar.variation);
}

void handleGNSSSatsInViewPGN(const CanFrame& frame) {
  // PGN 129540 GNSS Sats in View
  if (frame.data_length_code < 8) return;
  uint8_t sid = frame.data[0];
  uint8_t satsInView = frame.data[2];
  DEBUG_PRINT_NMEA2000("129540 GNSS Sats in View:  SID = Unknown; Sats in View = %u; ...\n", satsInView);
}

void handleGNSSDOPPGN(const CanFrame& frame) {
  // PGN 129539 GNSS DOPs
  if (frame.data_length_code < 8) return;
  uint8_t sid = frame.data[0];
  float hdop = (frame.data[4] | (frame.data[5] << 8)) * 0.01;
  float vdop = (frame.data[6] | (frame.data[7] << 8)) * 0.01;
  DEBUG_PRINT_NMEA2000("129539 GNSS DOPs:  SID = Unknown; HDOP = %.2f; VDOP = %.2f\n", hdop, vdop);
}

void handleEnvParamsPGN(const CanFrame& frame) {
  // PGN 130310 Environmental Parameters (obsolete)
  if (frame.data_length_code < 8) return;
  float waterTemp = (frame.data[2] | (frame.data[3] << 8)) * 0.01f;
  DEBUG_PRINT_NMEA2000("130310 Environmental Parameters (obsolete): Water Temperature = %.2f C\n", waterTemp);
  currentTemp.waterTemp = waterTemp;
}

void handleTempPGN(const CanFrame& frame) {
  // PGN 130312 Temperature
  if (frame.data_length_code < 8) return;
  float temp = (frame.data[2] | (frame.data[3] << 8)) * 0.01f;
  DEBUG_PRINT_NMEA2000("130312 Temperature: Actual Temperature = %.2f C\n", temp);
  currentTemp.seaTemp = temp;
}

void handleNavigationDataPGN(const CanFrame& frame) {
  // PGN 129284 Navigation Data
  DEBUG_PRINT_NMEA2000("129284 Navigation Data: ...\n");
}

void handleCrossTrackErrorPGN(const CanFrame& frame) {
  // PGN 129283 Cross Track Error
  DEBUG_PRINT_NMEA2000("129283 Cross Track Error: ...\n");
}

// --- Proprietary PGN 126720 Handler ---
void handleProprietary126720PGN(const CanFrame& frame) {
  // CANBOAT style: output Manufacturer, Industry, Data as hex
  const char* manufacturer = "Garmin";
  const char* industry = "Marine";
  if (!debugNMEA2000) return;
  Serial.print("Manufacturer Proprietary fast-packet addressed:  Manufacturer Code = ");
  Serial.print(manufacturer);
  Serial.print("; Industry Code = ");
  Serial.print(industry);
  Serial.print("; Data = ");
  for (int i = 0; i < frame.data_length_code; i++) {
    Serial.printf("%02X ", frame.data[i]);
  }
  Serial.println();
}

// --- PGN Table ---
struct PGNEntry {
  uint32_t pgn;
  PGNHandler handler;
};

PGNEntry pgnTable[] = {
  { 128267, handleDepthPGN },
  { 129025, handleGPSPGN_129025 },
  { 129029, handleGPSPGN_129029 },
  { 129026, handleCOGPGN },
  { 127258, handleMagVarPGN },
  { 129540, handleGNSSSatsInViewPGN },
  { 129539, handleGNSSDOPPGN },
  { 130310, handleEnvParamsPGN },
  { 130312, handleTempPGN },
  { 129284, handleNavigationDataPGN },
  { 129283, handleCrossTrackErrorPGN },
  { 126720, handleProprietary126720PGN },
};
const int pgnTableSize = sizeof(pgnTable) / sizeof(PGNEntry);

// --- RS485 State Tracking ---
uint8_t currentByteIndex = 0;
uint8_t expectedLen = 0;
unsigned long lastByteTime = 0;
const unsigned long MESSAGE_TIMEOUT = 1000;
unsigned long totalMessages = 0;
unsigned long invalidMessages = 0;
unsigned long depthRequests = 0;

// --- PGN extraction for J1939/NMEA2000 ---
uint32_t extractPGN(uint32_t canId) {
  uint8_t dp = (canId >> 24) & 0x01;
  uint8_t pf = (canId >> 16) & 0xFF;
  uint8_t ps = (canId >> 8) & 0xFF;
  if (pf < 240) {
    return (dp << 16) | (pf << 8);
  } else {
    return (dp << 16) | (pf << 8) | ps;
  }
}

// --- CAN Message Processing with Handler Table ---
void canReceiver() {
  CanFrame rxFrame;
  if (ESP32Can.readFrame(rxFrame)) {
    uint32_t pgn = extractPGN(rxFrame.identifier);
    bool handled = false;
    for (int i = 0; i < pgnTableSize; i++) {
      if (pgn == pgnTable[i].pgn) {
        if (pgnTable[i].handler) {
          pgnTable[i].handler(rxFrame);
        } else {
          DEBUG_PRINT_NMEA2000("[PGN %lu] Handler not implemented\n", pgn);
        }
        handled = true;
        break;
      }
    }
    if (!handled) {
      DEBUG_PRINT_NMEA2000("PGN %lu not handled\n", pgn);
    }
  }
}

// --- RS485 Request/Response Handling ---
void readAndProcessRS485Message() {
  static uint8_t messageBuffer[32];
  static uint8_t currentByteIndex = 0;
  static uint8_t expectedLen = 0;

  while (RS485Serial.available()) {
    uint8_t b = RS485Serial.read();
    if (currentByteIndex == 0) {
      expectedLen = b;
      if (expectedLen > 32 || expectedLen < 4) {
        currentByteIndex = 0;
        expectedLen = 0;
        continue;
      }
    }
    if (currentByteIndex < 32) {
      messageBuffer[currentByteIndex++] = b;
    }
    if (currentByteIndex >= expectedLen) {
      // Verify checksum if present
      if (verifyRS485Checksum(messageBuffer, expectedLen)) {
        storeRS485Message(messageBuffer, expectedLen);
      } else {
        DEBUG_PRINT_RS485("[RS485] Checksum failed. Ignoring message.\n");
      }
      currentByteIndex = 0;
      expectedLen = 0;
    }
  }
}

bool validateMessage(uint8_t* msg, uint8_t len) {
  if (msg[0] != len) {
    DEBUG_PRINT_RS485("RS485 Length mismatch: header %d, actual %d\n", msg[0], len);
    return false;
  }
  return true;
}

void storeRS485Message(uint8_t* msg, uint8_t len) {
  DEBUG_PRINT_RS485("RS485: Processing message (len: %d)\n", len);
  if (debugRS485) {
    Serial.print("HEX: ");
    for (int i = 0; i < len; i++) { Serial.printf("%02X ", msg[i]); }
    Serial.println();
  }

  // Only handle depth requests: 04 09 XX XX
  if (len == REQUEST_LEN && msg[0] == MSG_TYPE_REQUEST && msg[1] == MSG_CMD_GET_DEPTH) {
    DEBUG_PRINT_RS485("[RS485] Depth request received. Sending response.\n");
    sendDepthData();
    toggle_bit ^= 1;
  } else {
    DEBUG_PRINT_RS485("[RS485] Unknown or invalid request received.\n");
  }
}

// --- RS485 Checksum Calculation ---
uint8_t calculate_checksum(const uint8_t* data, uint8_t len) {
  uint8_t total = 0;
  for (uint8_t i = 0; i < len; i++) {
    total = (total + data[i]) & 0xFF;
  }
  return ((~total + 1) & 0xFF);  // 8-bit two's complement
}

bool verifyRS485Checksum(const uint8_t* msg, uint8_t len) {
  if (len < 2) return false;
  uint8_t checksum = msg[len - 1];
  uint8_t calculated = calculate_checksum(msg, len - 1);
  return checksum == calculated;
}

// --- RS485 Transmit/Receive Direction ---
void enableTransmit() {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
}

void enableReceive() {
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

void sendDepthData() {
  uint16_t depth_to_send;
  if (!depth_valid || (millis() - lastDepthReceivedMs > nmeaDepthTimeoutMs)) {
    depth_to_send = depth_timeout_value;
  } else {
    depth_to_send = depth_tenths;
  }

  uint8_t LL = depth_to_send & 0xFF;
  uint8_t HH = (depth_to_send >> 8) & 0xFF;
  uint8_t response[RESPONSE_LEN] = {
    RESPONSE_LEN,       // Length
    MSG_CMD_GET_DEPTH,  // Command (0x09)
    0x14,               // Sub-command
    0xAA,               // Status/ID
    LL, HH,             // Depth bytes (tenths of a foot, low/high)
    0xFF, 0x03,         // Filler
    0xFF, 0x03,         // Filler
    toggle_bit,         // Toggle
    0x02,               // Filler
    0                   // Placeholder for checksum
  };
  response[12] = calculate_checksum(response, 12);  // Set checksum
  enableTransmit();
  delayMicroseconds(200);
  RS485Serial.write(response, RESPONSE_LEN);
  RS485Serial.flush();
  delayMicroseconds(200);
  enableReceive();

  if (debugRS485) {
    Serial.print("[RS485] Response sent: ");
    for (int i = 0; i < RESPONSE_LEN; i++) {
      Serial.printf("%02X ", response[i]);
    }
    Serial.println();
  }
}

// --- Debug/Serial Command Handler ---
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "debug nmea on") {
      debugNMEA2000 = true;
      Serial.println("NMEA2000 debug output ON");
    } else if (command == "debug nmea off") {
      debugNMEA2000 = false;
      Serial.println("NMEA2000 debug output OFF");
    } else if (command == "debug rs485 on") {
      debugRS485 = true;
      Serial.println("RS485 debug output ON");
    } else if (command == "debug rs485 off") {
      debugRS485 = false;
      Serial.println("RS485 debug output OFF");
    } else if (command == "depth") {
      Serial.printf("Current depth: %.1f ft (tenths: %u)\n", depth_tenths / 10.0, depth_tenths);
    } else {
      Serial.println("Unknown command.");
    }
  }
}

void printMessageStats() {
  unsigned long total = totalMessages + invalidMessages;
  float successRate = total > 0 ? (float)totalMessages / total * 100.0 : 0.0;

  Serial.println("MESSAGE STATISTICS:");
  Serial.printf("   Total processed: %lu\n", totalMessages);
  Serial.printf("   Invalid/rejected: %lu\n", invalidMessages);
  Serial.printf("   Success rate: %.1f%%\n", successRate);
  Serial.printf("   Depth requests: %lu\n", depthRequests);
  Serial.printf("   Latest depth: %.2f m\n", currentDepth.currentDepth);
  if (currentGPS.valid)
    Serial.printf("   Latest GPS: Lat %.7f, Lon %.7f, Alt %.3f\n", currentGPS.latitude, currentGPS.longitude, currentGPS.altitude);
  else
    Serial.printf("   Latest GPS: not available\n");
  if (currentCOG.valid)
    Serial.printf("   Latest COG/SOG: COG %.1f deg, SOG %.2f m/s\n", currentCOG.cog, currentCOG.sog);
  if (currentMagVar.valid)
    Serial.printf("   Latest Mag Variation: %.1f deg\n", currentMagVar.variation);
  Serial.printf("   Uptime: %.1f minutes\n", millis() / 60000.0);
}

void printHelp() {
  Serial.println("Available commands:");
  Serial.println("  stats            - Show message statistics");
  Serial.println("  test             - Send test depth data");
  Serial.println("  reset            - Reset message buffer");
  Serial.println("  help             - Show this help");
  Serial.println("  debug nmea on    - Enable NMEA 2000 (CAN) debug output");
  Serial.println("  debug nmea off   - Disable NMEA 2000 (CAN) debug output");
  Serial.println("  debug rs485 on   - Enable RS485 debug output");
  Serial.println("  debug rs485 off  - Disable RS485 debug output");
}

// --- Dashboard HTML Embedded ---
const char* dashboard_html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>ESP32 Metrics Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; background: #1c2833; color: #eaecee; margin: 0; }
    .container { max-width: 700px; margin: 2em auto; background: #273746; border-radius: 8px; padding: 2em; box-shadow: 0 2px 8px #0006;}
    h1 { text-align: center; color: #48c9b0; margin-bottom: 1em;}
    .metrics { display: grid; grid-template-columns: 1fr 1fr; gap: 1em; }
    .metric { background: #212f3d; border-radius: 6px; padding: 1em; }
    .metric-label { font-weight: bold; color: #aed6f1;}
    .metric-value { font-size: 1.3em; margin-top: 0.3em;}
    .status-ok { color: #48c9b0; }
    .status-warn { color: #f7ca18; }
    .status-err { color: #e74c3c; }
    .controls { margin-top: 2em; display: flex; gap: 1em; flex-wrap: wrap; justify-content: center;}
    .controls button, .controls input[type=number] { padding: 0.5em 1em; font-size: 1em; border-radius: 4px; border: none; }
    .controls button { background: #48c9b0; color: #212f3d; cursor: pointer;}
    .controls button:hover { background: #1abc9c;}
    .log { background: #212f3d; border-radius: 6px; padding: 1em; margin-top: 2em; max-height: 200px; overflow-y: auto; font-size: 0.95em;}
    .log-line { border-bottom: 1px solid #273746; padding: 0.2em 0;}
    @media (max-width: 600px) {
      .metrics { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>ESP32 Gateway Metrics</h1>
    <div id="metrics" class="metrics">
      <!-- Metrics injected by JS -->
    </div>
    <div class="controls">
      <label>
        Timeout (ms):
        <input type="number" id="timeout" min="1000" max="60000" step="500" value="5000">
      </label>
      <label>
        Fallback Depth (tenths ft):
        <input type="number" id="timeoutval" min="0" max="10000" step="1" value="400">
      </label>
      <button onclick="sendConfig()">Set Config</button>
      <button onclick="sendReboot()">Reboot ESP32</button>
    </div>
    <div class="log" id="log">
      <!-- Log injected by JS -->
    </div>
  </div>
  <script>
    let metricsUrl = "/metrics.json"; // ESP32 should serve this endpoint
    let logUrl = "/log.txt"; // optional endpoint for recent logs

    function statusClass(val, warn, err) {
      return val < warn ? 'status-ok' : (val < err ? 'status-warn' : 'status-err');
    }

    function renderMetrics(data) {
      let html = '';
      html += `<div class="metric"><div class="metric-label">Uptime</div><div class="metric-value">${data.uptime_min} min</div></div>`;
      html += `<div class="metric"><div class="metric-label">Heap Free</div><div class="metric-value ${statusClass(data.heap_free, 20000, 12000)}">${data.heap_free} bytes</div></div>`;
      html += `<div class="metric"><div class="metric-label">RS485 Requests</div><div class="metric-value">${data.rs485_requests}</div></div>`;
      html += `<div class="metric"><div class="metric-label">NMEA Depth Frames</div><div class="metric-value">${data.nmea_depth_frames}</div></div>`;
      html += `<div class="metric"><div class="metric-label">Depth (ft)</div><div class="metric-value">${data.depth_ft.toFixed(1)}</div></div>`;
      html += `<div class="metric"><div class="metric-label">Fallback Active</div><div class="metric-value ${data.depth_fallback ? "status-warn" : "status-ok"}">${data.depth_fallback ? "YES" : "NO"}</div></div>`;
      html += `<div class="metric"><div class="metric-label">Timeout</div><div class="metric-value">${data.nmea_depth_timeout_ms} ms</div></div>`;
      html += `<div class="metric"><div class="metric-label">Fallback Value</div><div class="metric-value">${(data.depth_timeout_value/10.0).toFixed(1)} ft</div></div>`;
      document.getElementById('metrics').innerHTML = html;
      document.getElementById('timeout').value = data.nmea_depth_timeout_ms;
      document.getElementById('timeoutval').value = data.depth_timeout_value;
    }

    function renderLog(text) {
      let lines = text.trim().split("\n").slice(-25);
      document.getElementById('log').innerHTML = lines.map(l => `<div class="log-line">${l}</div>`).join('');
    }

    function fetchMetrics() {
      fetch(metricsUrl)
        .then(r => r.json())
        .then(data => renderMetrics(data))
        .catch(err => { document.getElementById('metrics').innerHTML = "<div class='metric-value status-err'>Error loading metrics</div>"; });
      fetch(logUrl)
        .then(r => r.text())
        .then(text => renderLog(text))
        .catch(err => { document.getElementById('log').innerHTML = "<div class='log-line status-err'>No log available</div>"; });
    }

    function sendConfig() {
      let timeout = document.getElementById('timeout').value;
      let timeoutval = document.getElementById('timeoutval').value;
      fetch(`/setconfig?timeout=${timeout}&timeoutval=${timeoutval}`)
        .then(r => r.text())
        .then(t => alert(t))
        .catch(err => alert("Failed to set config!"));
    }

    function sendReboot() {
      fetch("/reboot")
        .then(r => r.text())
        .then(t => alert("Rebooting ESP32..."))
        .catch(err => alert("Failed to send reboot!"));
    }

    setInterval(fetchMetrics, 2000); // update every 2s
    window.onload = fetchMetrics;
  </script>
</body>
</html>
)rawliteral";

// --- Web Server ---
WebServer server(80);

// --- Setup ---
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("\n\n--- esp32 NMEA2k RS485 gateway ---");
  Serial.printf("FW Version: %s (Build: %s)\n", SW_VERSION_STRING, SW_BUILD_DATE);
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  RS485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  Serial.println("Starting RS485 transceiver...");
  Serial.printf("RS485 initialized at %d baud \n", RS485_BAUDRATE);
  Serial.printf("   RX Pin: GPIO%d, TX Pin: GPIO%d, DE/RE Pin: GPIO%d\n",
                RS485_RX_PIN, RS485_TX_PIN, RS485_DE_RE_PIN);

  ESP32Can.setPins(CAN_TX, CAN_RX);
  Serial.println("Starting NMEA 2000...");
  Serial.printf("NMEA 2000 initialized at %d kbps \n", CAN_SPEED);
  Serial.printf("   CAN_H: GPIO%d, CAN_L: GPIO%d\n",
                CAN_TX, CAN_RX);
  if (!ESP32Can.begin(ESP32Can.convertSpeed(CAN_SPEED))) {
    Serial.println("CAN bus failed to start!");
    while (1) {}
  }

  Serial.println("Starting WiFi...");
  WiFi.softAP("esp32-gw", "1234567890");
  delay(1000); // Wait for AP to be up
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  
  Serial.println("Starting webserver...");
  server.on("/", []() {
    server.send(200, "text/html", dashboard_html);
  });
  server.on("/metrics.json", []() {
  String json = "{";
  json += "\"uptime_min\":" + String(millis() / 60000.0, 1);
  json += ",\"heap_free\":" + String(ESP.getFreeHeap());
  json += ",\"rs485_requests\":" + String(depthRequests);
  json += ",\"nmea_depth_frames\":" + String(totalMessages);
  json += ",\"depth_ft\":" + String(depth_tenths / 10.0, 1);
  json += ",\"depth_fallback\":" + String((!depth_valid || (millis() - lastDepthReceivedMs > nmeaDepthTimeoutMs)) ? "true" : "false");
  json += ",\"nmea_depth_timeout_ms\":" + String(nmeaDepthTimeoutMs);
  json += ",\"depth_timeout_value\":" + String(depth_timeout_value);
  json += "}";
  server.send(200, "application/json", json);
  });

  // Optional: Implement /log.txt endpoint for JS dashboard (stub for now)
  server.on("/log.txt", []() {
  server.send(200, "text/plain", "No log available\n");
  });

  server.begin();
  Serial.println("Web server started");

  Serial.println("System startup complete.");
}

// --- Main Loop ---
void loop() {
  canReceiver();
  readAndProcessRS485Message();
  handleSerialCommands();
  server.handleClient();
  delay(10);
}