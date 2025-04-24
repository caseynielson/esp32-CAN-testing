/**
 * Optimized Vehicle CAN Simulator
 * 
 * Version: 2.0.0
 * Date: 2025-04-23
 * Author: caseyn
 * 
 * Features:
 * - Realistic vehicle parameter simulation
 * - OBD-II compatible messages
 * - Prioritized message scheduling
 * - Power-efficient operation
 */

#include <ESP32-TWAI-CAN.hpp>
#include "esp_pm.h"

#define CAN_TX     5
#define CAN_RX     4
#define CAN_SPEED  500

// Vehicle state
struct VehicleState {
  float vehicleSpeed;    // mph
  float engineRPM;       // RPM
  float oilPressure;     // PSI
  float coolantTemp;     // °F
  float fuelLevel;       // Percentage
  uint8_t gearPosition;  // Index into gearStates (0-3: P, R, N, D; 4-9: 1-6 gears)
  bool engineRunning;
  bool accelerating;
  bool braking;          // Brake status
  uint8_t throttlePos;   // Throttle position 0-100%
  float engineLoad;      // Engine load 0-100%
  unsigned long lastUpdateTime;
} state;

// Constants
const float MAX_SPEED = 120.0f;
const float MAX_RPM = 7000.0f;
const float IDLE_RPM = 800.0f;
const float MIN_OIL_PRESSURE = 20.0f;
const float MAX_OIL_PRESSURE = 80.0f;
const float MIN_COOLANT_TEMP = 135.0f;
const float MAX_COOLANT_TEMP = 230.0f;
const float GEAR_RATIOS[] = {0, -3.2f, 0, 3.4f, 2.8f, 2.3f, 1.8f, 1.4f, 1.0f, 0.7f}; // P, R, N, D, 1–6
const char GEAR_STATES[] = {'P', 'R', 'N', 'D', '1', '2', '3', '4', '5', '6'};

// Timing constants
const unsigned long ACCEL_TOGGLE_INTERVAL = 5000;  // How often to toggle acceleration
const unsigned long BRAKE_INTERVAL = 2000;         // How often to toggle brake
const unsigned long THROTTLE_UPDATE_MS = 100;      // Throttle position update interval

// Performance metrics
unsigned long framesSent = 0;
unsigned long lastMetricsUpdate = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n\n----- Optimized Vehicle CAN Simulator -----");
  Serial.println("v2.0.0 - 2025-04-23");
  Serial.println("User: caseyn");

  // Set up power management for better efficiency
  setupPowerSaving();

  // Initialize CAN bus
  ESP32Can.setPins(CAN_TX, CAN_RX);
  Serial.println("Starting CAN...");

  if (ESP32Can.begin(ESP32Can.convertSpeed(CAN_SPEED))) {
    Serial.println("CAN bus started successfully!");
  } else {
    Serial.println("CAN bus failed to start!");
    while(1);
  }

  // Initialize vehicle state
  initializeVehicle();
  
  Serial.println("Vehicle simulator initialized.");
  Serial.println("Starting in Park with engine running.");
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime = ((long)(currentTime - state.lastUpdateTime)) / 1000.0f; // Safe for rollover
  state.lastUpdateTime = currentTime;

  // Update vehicle physics
  updateVehicleState(deltaTime);

  // Prioritized message sending based on importance
  static int cyclecCounter = 0;
  cyclecCounter = (cyclecCounter + 1) % 5;
  
  // HIGH PRIORITY: Send every cycle (500ms)
  sendVehicleSpeed();
  sendEngineRPM();
  
  // MEDIUM PRIORITY: Alternate between these sets every cycle
  if (cyclecCounter % 2 == 0) {
    sendOilPressure();
    sendCoolantTemp();
  } else {
    sendGearPosition();
    sendThrottlePosition();
  }
  
  // LOW PRIORITY: Send every 5th cycle (2500ms)
  if (cyclecCounter == 0) {
    sendFuelLevel();
    sendEngineLoad();
    sendBrakeStatus();
    sendOBDData();
  }

  // Display performance metrics every 10 seconds
  if (currentTime - lastMetricsUpdate > 10000) {
    Serial.printf("Performance Metrics - Frames sent: %lu, Uptime: %lu seconds\n", 
                  framesSent, currentTime / 1000);
    lastMetricsUpdate = currentTime;
  }

  delay(500); // 2Hz update rate
}

void setupPowerSaving() {
  #ifdef CONFIG_PM_ENABLE
    // Configure CPU frequency scaling
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

void initializeVehicle() {
  state.vehicleSpeed = 0;
  state.engineRPM = IDLE_RPM;
  state.oilPressure = MIN_OIL_PRESSURE;
  state.coolantTemp = MIN_COOLANT_TEMP;
  state.fuelLevel = 100.0f;
  state.gearPosition = 0; // Park
  state.engineRunning = true;
  state.accelerating = false;
  state.braking = false;
  state.throttlePos = 0;
  state.engineLoad = 0;
  state.lastUpdateTime = millis();
  state.accelerating = true; // Start accelerating
}

void updateVehicleState(float deltaTime) {
  // Debug output with reduced frequency (once per second)
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 1000) {
    Serial.printf("Vehicle State - Speed: %.1f MPH, RPM: %.0f, Gear: %c, Throttle: %d%%\n", 
                 state.vehicleSpeed, state.engineRPM, GEAR_STATES[state.gearPosition], 
                 state.throttlePos);
    lastDebugTime = millis();
  }

  // Toggle acceleration every 5 seconds
  static unsigned long lastAccelToggle = 0;
  if (millis() - lastAccelToggle > ACCEL_TOGGLE_INTERVAL) {
    state.accelerating = !state.accelerating;
    lastAccelToggle = millis();
    
    // If we're accelerating but still in Park, automatically shift to Drive
    if (state.accelerating && state.gearPosition == 0) {
      state.gearPosition = 3; // Shift to Drive
      Serial.println("Shifting from Park to Drive");
    }
    
    // Higher throttle position for better acceleration
    state.throttlePos = state.accelerating ? random(60, 90) : random(0, 10);
  }
  
  // Toggle brake occasionally when decelerating
  static unsigned long lastBrakeToggle = 0;
  if (millis() - lastBrakeToggle > BRAKE_INTERVAL) {
    state.braking = !state.accelerating && (random(100) < 70); // 70% chance of braking when not accelerating
    lastBrakeToggle = millis();
  }
  
  // Throttle position variation - less frequent updates to save processing
  static unsigned long lastThrottleUpdate = 0;
  if (millis() - lastThrottleUpdate > THROTTLE_UPDATE_MS) {
    if (state.accelerating) {
      // Add small variations to throttle when accelerating
      state.throttlePos = constrain(state.throttlePos + random(-5, 6), 40, 90);
    } else {
      state.throttlePos = constrain(state.throttlePos + random(-3, 4), 0, 10);
    }
    lastThrottleUpdate = millis();
  }

  // Drive mode or manual gears physics
  if (state.accelerating && state.gearPosition >= 3) {
    float acceleration = 8.0f * (state.throttlePos / 100.0f);
    state.vehicleSpeed = min(state.vehicleSpeed + acceleration * deltaTime, MAX_SPEED);

    float gearRatio = GEAR_RATIOS[state.gearPosition];
    float wheelCircumference = 2 * PI * 1.083f; // In feet (assumes ~26" diameter wheels)
    float transmissionRatio = 3.5f; // Typical final drive ratio
    float targetRPM = (state.vehicleSpeed * 88.0f * gearRatio * transmissionRatio) / wheelCircumference;
    
    // Ensure minimum RPM when in gear
    targetRPM = max(targetRPM, IDLE_RPM + 200);
    state.engineRPM = min(targetRPM, MAX_RPM);
  } else {
    float deceleration = state.braking ? 12.0f : 4.0f; 
    state.vehicleSpeed = max(0.0f, state.vehicleSpeed - deceleration * deltaTime);
    
    float rpmDecelRate = 300.0f;
    state.engineRPM = max(IDLE_RPM, state.engineRPM - rpmDecelRate * deltaTime);
  }
  
  // Calculate engine load based on RPM and throttle
  float rpmFactor = (state.engineRPM - IDLE_RPM) / (MAX_RPM - IDLE_RPM);
  state.engineLoad = 0.3f * rpmFactor * 100.0f + 0.7f * state.throttlePos;
  state.engineLoad = constrain(state.engineLoad, 0, 100);

  // Oil pressure correlates with RPM
  state.oilPressure = MIN_OIL_PRESSURE + rpmFactor * (MAX_OIL_PRESSURE - MIN_OIL_PRESSURE);

  // Coolant temperature rises more with engine load
  float tempRiseRate = 0.5f + 0.5f * (state.engineLoad / 100.0f);
  if (state.coolantTemp < MAX_COOLANT_TEMP) {
    state.coolantTemp += tempRiseRate * deltaTime;
  }

  // Fuel consumption based on engine load
  float fuelConsumption = (state.engineLoad / 100.0f) * 0.05f * deltaTime;
  state.fuelLevel = max(0.0f, state.fuelLevel - fuelConsumption);

  // Gear selection logic
  updateGearSelection();
}

void updateGearSelection() {
  // Shift from D into gears 1–6
  if (state.gearPosition == 3) {
    if (state.vehicleSpeed < 15.0f) state.gearPosition = 4;  // 1st gear
    else if (state.vehicleSpeed < 25.0f) state.gearPosition = 5;  // 2nd gear
    else if (state.vehicleSpeed < 40.0f) state.gearPosition = 6;  // 3rd gear
    else if (state.vehicleSpeed < 50.0f) state.gearPosition = 7;  // 4th gear
    else if (state.vehicleSpeed < 65.0f) state.gearPosition = 8;  // 5th gear
    else state.gearPosition = 9;  // 6th gear
  }
  
  // Return to 'D' when fully stopped from any gear
  if (state.vehicleSpeed < 1.0f && state.gearPosition > 3) {
    state.gearPosition = 3;  // Return to D
  }
  
  // Prevent gear index from going out of bounds
  state.gearPosition = constrain(state.gearPosition, 0, 9);
}

// ----------------- CAN SEND FUNCTIONS ----------------------

void sendVehicleSpeed() {
  CanFrame frame = { 0 };
  frame.identifier = 0x100;
  frame.extd = 0;
  frame.data_length_code = 2;
  uint16_t speed = (uint16_t)(state.vehicleSpeed * 10); // 0.1 mph resolution
  frame.data[0] = (speed >> 8) & 0xFF;
  frame.data[1] = speed & 0xFF;

  if (ESP32Can.writeFrame(frame)) {
    framesSent++;
  } else {
    Serial.println("CAN send failed: Speed");
  }
}

void sendEngineRPM() {
  CanFrame frame = { 0 };
  frame.identifier = 0x101;
  frame.extd = 0;
  frame.data_length_code = 2;
  uint16_t rpm = (uint16_t)state.engineRPM;
  frame.data[0] = (rpm >> 8) & 0xFF;
  frame.data[1] = rpm & 0xFF;

  if (ESP32Can.writeFrame(frame)) {
    framesSent++;
  } else {
    Serial.println("CAN send failed: RPM");
  }
}

void sendOilPressure() {
  CanFrame frame = { 0 };
  frame.identifier = 0x102;
  frame.extd = 0;
  frame.data_length_code = 1;
  frame.data[0] = (uint8_t)state.oilPressure;

  if (ESP32Can.writeFrame(frame)) {
    framesSent++;
  } else {
    Serial.println("CAN send failed: Oil Pressure");
  }
}

void sendCoolantTemp() {
  CanFrame frame = { 0 };
  frame.identifier = 0x103;
  frame.extd = 0;
  frame.data_length_code = 1;
  frame.data[0] = (uint8_t)state.coolantTemp;

  if (ESP32Can.writeFrame(frame)) {
    framesSent++;
  } else {
    Serial.println("CAN send failed: Coolant Temp");
  }
}

void sendFuelLevel() {
  CanFrame frame = { 0 };
  frame.identifier = 0x104;
  frame.extd = 0;
  frame.data_length_code = 1;
  frame.data[0] = (uint8_t)state.fuelLevel;

  if (ESP32Can.writeFrame(frame)) {
    framesSent++;
  } else {
    Serial.println("CAN send failed: Fuel Level");
  }
}

void sendGearPosition() {
  CanFrame frame = { 0 };
  frame.identifier = 0x105;
  frame.extd = 0;
  frame.data_length_code = 1;
  frame.data[0] = GEAR_STATES[state.gearPosition];

  if (ESP32Can.writeFrame(frame)) {
    framesSent++;
  } else {
    Serial.println("CAN send failed: Gear");
  }
}

void sendThrottlePosition() {
  CanFrame frame = { 0 };
  frame.identifier = 0x106;
  frame.extd = 0;
  frame.data_length_code = 1;
  frame.data[0] = state.throttlePos;

  if (ESP32Can.writeFrame(frame)) {
    framesSent++;
  } else {
    Serial.println("CAN send failed: Throttle");
  }
}

void sendEngineLoad() {
  CanFrame frame = { 0 };
  frame.identifier = 0x107;
  frame.extd = 0;
  frame.data_length_code = 1;
  frame.data[0] = (uint8_t)state.engineLoad;

  if (ESP32Can.writeFrame(frame)) {
    framesSent++;
  } else {
    Serial.println("CAN send failed: Engine Load");
  }
}

void sendBrakeStatus() {
  CanFrame frame = { 0 };
  frame.identifier = 0x108;
  frame.extd = 0;
  frame.data_length_code = 1;
  frame.data[0] = state.braking ? 1 : 0;

  if (ESP32Can.writeFrame(frame)) {
    framesSent++;
  } else {
    Serial.println("CAN send failed: Brake Status");
  }
}

void sendOBDData() {
  // OBD-II style messages - Standard PID format
  bool success = true;
  CanFrame frame = { 0 };
  
  // Engine coolant temperature (PID 0x05)
  frame.identifier = 0x7E8; // Standard OBD-II response ID
  frame.extd = 0;
  frame.data_length_code = 4;
  frame.data[0] = 0x03; // Number of additional bytes
  frame.data[1] = 0x41; // Response to service 01
  frame.data[2] = 0x05; // PID for coolant temp
  frame.data[3] = (uint8_t)(state.coolantTemp - 40); // Convert to OBD format
  success &= ESP32Can.writeFrame(frame);
  
  // Vehicle speed (PID 0x0D)
  frame.data[2] = 0x0D; // PID for vehicle speed
  frame.data[3] = (uint8_t)state.vehicleSpeed;
  success &= ESP32Can.writeFrame(frame);
  
  // Engine RPM (PID 0x0C)
  frame.data_length_code = 5;
  frame.data[0] = 0x04; // Number of additional bytes
  frame.data[2] = 0x0C; // PID for RPM
  uint16_t scaledRPM = (uint16_t)(state.engineRPM * 4); // OBD formula: RPM = ((A*256)+B)/4
  frame.data[3] = (scaledRPM >> 8) & 0xFF;
  frame.data[4] = scaledRPM & 0xFF;
  success &= ESP32Can.writeFrame(frame);
  
  // Throttle position (PID 0x11)
  frame.data_length_code = 4;
  frame.data[0] = 0x03;
  frame.data[2] = 0x11; // PID for throttle position
  frame.data[3] = (uint8_t)((state.throttlePos * 255) / 100); // Scale to 0-255
  success &= ESP32Can.writeFrame(frame);
  
  if (success) {
    framesSent += 4;
  } else {
    Serial.println("One or more OBD messages failed to send");
  }
}
