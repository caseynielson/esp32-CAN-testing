/*
 Attempt to create a standard J1939 simulator

*/

#include <ESP32-TWAI-CAN.hpp>

// Pin definitions
#define CAN_TX     5
#define CAN_RX     4
#define CAN_SPEED  250  // J1939 typically uses 250kbps

// J1939 message formatting constants
#define J1939_EXT_BIT      0x80000000  // Extended frame bit

// J1939 Source Addresses
#define J1939_SA_ENGINE    0x00        // Engine source address
#define J1939_SA_TRANS     0x03        // Transmission source address
#define J1939_SA_BRAKE     0x0B        // Brake system controller
#define J1939_SA_DISPLAY   0x17        // Instrument cluster
#define J1939_SA_FUEL      0x1C        // Fuel system

// J1939 PGNs
#define J1939_PGN_EEC1     0xF004      // Electronic Engine Controller 1 (61444)
#define J1939_PGN_EEC2     0xF003      // Electronic Engine Controller 2 (61443)
#define J1939_PGN_EEC3     0xFEF3      // Electronic Engine Controller 3 (65267)
#define J1939_PGN_TURBO    0xFEF6      // Turbocharger (65270)
#define J1939_PGN_ETC1     0xF005      // Electronic Transmission Controller 1 (61445)
#define J1939_PGN_LFE1     0xFEF2      // Liquid Fuel Economy (65266)
#define J1939_PGN_AMB      0xFEF5      // Ambient Conditions (65269)
#define J1939_PGN_VDHR     0xFEC1      // Vehicle Distance/Time (65217) 
#define J1939_PGN_ET1      0xFEEE      // Engine Temperature 1 (65262)
#define J1939_PGN_FLUID    0xFEF1      // Engine Fluid Level/Pressure (65263)
#define J1939_PGN_VEP1     0xFEEF      // Vehicle Electrical Power (65263)

// Simulation values - these are the values we want to transmit
struct J1939SimValues {
  // EEC1 values
  uint16_t engineSpeed;              // RPM
  uint8_t  engineTorqueMode;         // Mode index
  int8_t   driverDemandTorque;       // Percent (-125 to 125%)
  int8_t   actualEngineTorque;       // Percent (-125 to 125%)
  bool     cruiseControlActive;
  bool     brakeSwitch;

  // EEC2 values
  uint8_t  acceleratorPosition;      // Percent (0-100%)
  uint8_t  engineLoad;               // Percent (0-125%)  
  int8_t   engineDemandTorque;       // Percent (-125 to 125%)

  // EEC3 values
  uint8_t  nominalFrictionTorque;    // Percent (0-125%)

  // Turbocharger values
  uint32_t turboSpeed;               // RPM - Changed to uint32_t to handle larger values
  uint8_t  boostPressure;            // kPa
  int16_t  intakeTemp;               // Degrees C

  // ETC1 values
  uint8_t  selectedGear;             // Gear number
  uint8_t  currentGear;              // Gear number

  // Liquid Fuel Economy values
  uint16_t fuelRate;                 // L/h with 0.05 resolution
  uint16_t instantFuelEconomy;       // km/L with 0.01 resolution
  
  // Ambient Conditions values
  int16_t  ambientAirTemp;           // Degrees C
  uint8_t  airPressure;              // kPa
  
  // Engine Temperature values
  int16_t  coolantTemp;              // Degrees C
  int16_t  oilTemp;                  // Degrees C
  int16_t  fuelTemp;                 // Degrees C
  
  // Engine Fluid values
  uint16_t engineOilPressure;        // kPa
  
  // Vehicle Electrical values  
  uint16_t batteryVoltage;           // Volts with 0.05 resolution
  uint16_t alternatoCurrent;         // Amps with 0.05 resolution
};

// Initialize with some realistic values
J1939SimValues simValues = {
  .engineSpeed = 3500,               // 1500 RPM
  .engineTorqueMode = 0,             // Low idle governor
  .driverDemandTorque = 60,          // 60% driver demand
  .actualEngineTorque = 50,          // 50% actual engine torque
  .cruiseControlActive = false,       // Cruise control active
  .brakeSwitch = false,              // Brake not activated
  
  .acceleratorPosition = 47,         // 45% pedal position  
  .engineLoad = 55,                  // 35% engine load
  .engineDemandTorque = 45,          // 40% engine demand torque
  
  .nominalFrictionTorque = 10,       // 10% nominal friction
  
  .turboSpeed = 80000UL,             // 80,000 RPM - Added UL to make it unsigned long
  .boostPressure = 155,              // 150 kPa
  .intakeTemp = 40,                  // 40°C
  
  .selectedGear = 6,                 // 6th gear
  .currentGear = 6,                  // 6th gear
  
  .fuelRate = 240,                   // 12.0 L/h
  .instantFuelEconomy = 950,         // 9.5 km/L
  
  .ambientAirTemp = 25,              // 25°C
  .airPressure = 101,                // 101 kPa (standard atmosphere)
  
  .coolantTemp = 85,                 // 85°C
  .oilTemp = 95,                     // 95°C
  .fuelTemp = 45,                    // 45°C
  
  .engineOilPressure = 350,          // 350 kPa
  
  .batteryVoltage = 270,             // 13.5V (270 * 0.05)
  .alternatoCurrent = 600            // 30A (600 * 0.05)
};

// Message transmission intervals (milliseconds)
const uint16_t intervalEEC1 = 100;    // 100ms (10Hz)
const uint16_t intervalEEC2 = 50;     // 50ms (20Hz)
const uint16_t intervalEEC3 = 250;    // 250ms (4Hz)
const uint16_t intervalTURBO = 500;   // 500ms (2Hz)
const uint16_t intervalETC1 = 100;    // 100ms (10Hz)
const uint16_t intervalLFE1 = 1000;   // 1000ms (1Hz)
const uint16_t intervalAMB = 1000;    // 1000ms (1Hz)
const uint16_t intervalET1 = 1000;    // 1000ms (1Hz)
const uint16_t intervalFLUID = 100;   // 100ms (10Hz) - Faster updates for testing
const uint16_t intervalVEP1 = 1000;   // 1000ms (1Hz)

// Timing variables
unsigned long lastEEC1Time = 0;
unsigned long lastEEC2Time = 0;
unsigned long lastEEC3Time = 0;
unsigned long lastTURBOTime = 0;
unsigned long lastETC1Time = 0;
unsigned long lastLFE1Time = 0;
unsigned long lastAMBTime = 0;
unsigned long lastET1Time = 0;
unsigned long lastFLUIDTime = 0;
unsigned long lastVEP1Time = 0;

// Runtime variables for simulated values
unsigned long totalDistanceMeters = 0;
unsigned long engineHours = 36000;    // 10 hours in seconds
unsigned long vehicleHours = 36000;   // 10 hours in seconds

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n\n----- J1939 Multi-PGN Simulator -----");
  Serial.println("v1.0.0 - 2025-04-29");
  Serial.println("User: caseynielson");

  ESP32Can.setPins(CAN_TX, CAN_RX);
  Serial.println("Starting CAN...");

  if (ESP32Can.begin(ESP32Can.convertSpeed(CAN_SPEED))) {
    Serial.println("CAN bus started successfully at 250kbps!");
  } else {
    Serial.println("CAN bus failed to start!");
    while(1); // Stop if CAN initialization fails
  }
  
  Serial.println("J1939 simulator initialized.");
  Serial.println("Simulating multiple PGNs according to J1939 standard.");
  printSimValues();
}

void loop() {
  unsigned long currentTime = millis();

  // Update runtime variables
  updateRuntimeVariables();

  // Send J1939 messages at specified intervals
  if (currentTime - lastEEC1Time >= intervalEEC1) {
    sendEEC1();
    lastEEC1Time = currentTime;
  }
  
  if (currentTime - lastEEC2Time >= intervalEEC2) {
    sendEEC2();
    lastEEC2Time = currentTime;
  }
  
  if (currentTime - lastEEC3Time >= intervalEEC3) {
    sendEEC3();
    lastEEC3Time = currentTime;
  }
  
  if (currentTime - lastTURBOTime >= intervalTURBO) {
    sendTurbocharger();
    lastTURBOTime = currentTime;
  }
  
  if (currentTime - lastETC1Time >= intervalETC1) {
    sendETC1();
    lastETC1Time = currentTime;
  }
  
  if (currentTime - lastLFE1Time >= intervalLFE1) {
    sendLFE1();
    lastLFE1Time = currentTime;
  }
  
  if (currentTime - lastAMBTime >= intervalAMB) {
    sendAmbientConditions();
    lastAMBTime = currentTime;
  }
  
  if (currentTime - lastET1Time >= intervalET1) {
    sendEngineTemperature1();
    lastET1Time = currentTime;
  }
  
  if (currentTime - lastFLUIDTime >= intervalFLUID) {
    sendEngineFluidLevelPressure();
    // Try several different formats for oil pressure in case one works
    sendOilPressureAlternateFormats();
    lastFLUIDTime = currentTime;
  }
  
  if (currentTime - lastVEP1Time >= intervalVEP1) {
    sendVehicleElectricalPower();
    lastVEP1Time = currentTime;
  }
}

// J1939 message formatting helper
uint32_t formatJ1939Identifier(uint8_t priority, uint32_t pgn, uint8_t sourceAddr) {
  return J1939_EXT_BIT | ((priority & 0x07) << 26) | ((pgn & 0xFFFF) << 8) | sourceAddr;
}

// Print simulation values
void printSimValues() {
  Serial.println("\n--- Simulation Values ---");
  
  Serial.println("\nEEC1 (Engine Controller 1):");
  Serial.println("  Engine Speed: " + String(simValues.engineSpeed) + " RPM");
  Serial.println("  Engine Torque Mode: " + String(simValues.engineTorqueMode));
  Serial.println("  Driver Demand Torque: " + String(simValues.driverDemandTorque) + "%");
  Serial.println("  Actual Engine Torque: " + String(simValues.actualEngineTorque) + "%");
  Serial.println("  Cruise Control: " + String(simValues.cruiseControlActive ? "Active" : "Inactive"));
  Serial.println("  Brake Switch: " + String(simValues.brakeSwitch ? "On" : "Off"));
  
  Serial.println("\nEEC2 (Engine Controller 2):");
  Serial.println("  Accelerator Position: " + String(simValues.acceleratorPosition) + "%");
  Serial.println("  Engine Load: " + String(simValues.engineLoad) + "%");
  
  Serial.println("\nTurbocharger:");
  Serial.println("  Turbo Speed: " + String(simValues.turboSpeed) + " RPM");
  Serial.println("  Boost Pressure: " + String(simValues.boostPressure) + " kPa");
  Serial.println("  Intake Temperature: " + String(simValues.intakeTemp) + "°C");
  
  Serial.println("\nEngine Temperature:");
  Serial.println("  Coolant Temperature: " + String(simValues.coolantTemp) + "°C");
  Serial.println("  Oil Temperature: " + String(simValues.oilTemp) + "°C");
  Serial.println("  Fuel Temperature: " + String(simValues.fuelTemp) + "°C");
  
  Serial.println("\nEngine Fluid:");
  Serial.println("  Oil Pressure: " + String(simValues.engineOilPressure) + " kPa (" + 
               String(simValues.engineOilPressure * 0.145) + " PSI)");
  
  Serial.println("\nFuel Economy:");
  Serial.println("  Fuel Rate: " + String(simValues.fuelRate * 0.05) + " L/h");
  Serial.println("  Instant Fuel Economy: " + String(simValues.instantFuelEconomy * 0.01) + " km/L");
  
  Serial.println("\nAmbient Conditions:");
  Serial.println("  Ambient Temperature: " + String(simValues.ambientAirTemp) + "°C");
  Serial.println("  Air Pressure: " + String(simValues.airPressure) + " kPa");
  
  Serial.println("\n--- End of Simulation Values ---\n");
}

// Update runtime variables like hours, distance, etc.
void updateRuntimeVariables() {
  // Increase engine and vehicle hours (in seconds)
  static unsigned long lastUpdate = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastUpdate >= 1000) { // Every second
    // Calculate how many seconds have passed
    unsigned long secondsPassed = (currentTime - lastUpdate) / 1000;
    
    // Update counters
    engineHours += secondsPassed;
    vehicleHours += secondsPassed;
    
    // Update distance based on simulated speed (assuming speed in km/h)
    // Distance = speed * time
    // If speed is 60 km/h, then in 1 second we travel 60/3600 = 0.0166... km = 16.67 meters
    float speedKmH = simValues.engineSpeed / 30.0; // Fake calculation just for simulation
    totalDistanceMeters += (unsigned long)(speedKmH * 1000.0 * secondsPassed / 3600.0);
    
    lastUpdate = currentTime;
  }
}

// Send Electronic Engine Controller 1 (EEC1) message
void sendEEC1() {
  CanFrame frame = { 0 };
  
  // Format standard J1939 ID for EEC1 (PGN 61444 / 0xF004) from Engine (SA 0)
  // Using priority 3 which is standard for EEC1
  frame.identifier = formatJ1939Identifier(3, J1939_PGN_EEC1, J1939_SA_ENGINE);
  frame.extd = 1;  // Extended CAN frame
  frame.data_length_code = 8;  // J1939 uses 8-byte messages
  
  // According to SAE J1939 standard:
  // - Engine Torque Mode (byte 1): Tells how torque is being calculated
  frame.data[0] = simValues.engineTorqueMode & 0x0F;
  
  // - Driver's Demand Engine - Percent Torque (byte 2)
  // Range: -125% to +125%, 1% per bit, offset by 125%
  frame.data[1] = simValues.driverDemandTorque + 125;
  
  // - Actual Engine - Percent Torque (byte 3)
  // Range: -125% to +125%, 1% per bit, offset by 125%
  frame.data[2] = simValues.actualEngineTorque + 125;
  
  // - Engine Speed (bytes 4-5)
  // Resolution: 0.125 rpm/bit, 0 offset
  // This is 2 bytes, least significant byte first (little endian)
  uint16_t rpm_scaled = simValues.engineSpeed * 8;  // 0.125 rpm/bit = multiply by 8
  frame.data[3] = rpm_scaled & 0xFF;         // LSB
  frame.data[4] = (rpm_scaled >> 8) & 0xFF;  // MSB
  
  // - Source Address of Controlling Device for Engine Control (byte 6)
  // 0 = engine control device
  frame.data[5] = 0x00;
  
  // - Engine Starter Mode (byte 7)
  // Setting to 0 (not cranking)
  frame.data[6] = 0x00;
  
  // - Engine Status (byte 8)
  // Bit 1: Cruise control
  // Bit 0: Brake switch
  uint8_t engineStatus = 0;
  if (simValues.cruiseControlActive) engineStatus |= 0x02;
  if (simValues.brakeSwitch) engineStatus |= 0x01;
  frame.data[7] = engineStatus;
  
  // Send the frame
  ESP32Can.writeFrame(frame);
  
  // Simple debug output
  Serial.print("EEC1: RPM=");
  Serial.print(simValues.engineSpeed);
  Serial.print(", Actual Torque=");
  Serial.print(simValues.actualEngineTorque);
  Serial.print("%, Status=0x");
  Serial.println(engineStatus, HEX);
}

// Send Electronic Engine Controller 2 (EEC2) message
void sendEEC2() {
  CanFrame frame = { 0 };
  
  // Format standard J1939 ID for EEC2 (PGN 61443)
  frame.identifier = formatJ1939Identifier(3, J1939_PGN_EEC2, J1939_SA_ENGINE);
  frame.extd = 1;
  frame.data_length_code = 8;
  
  // Initialize data bytes to 0xFF (not available)
  for (int i = 0; i < 8; i++) {
    frame.data[i] = 0xFF;
  }
  
  // Accelerator pedal position 1
  // Resolution: 0.4% per bit, offset 0
  uint8_t accelScaled = (uint8_t)(simValues.acceleratorPosition * 2.5);
  frame.data[1] = (accelScaled > 250) ? 250 : accelScaled;
  
  // Engine percent load
  // Resolution: 1% per bit, offset 0
  frame.data[2] = (simValues.engineLoad > 250) ? 250 : simValues.engineLoad;
  
  // Remote accelerator position
  // Not used in this simulation, set to 0xFF (not available)
  frame.data[3] = 0xFF;
  
  // Accelerator pedal position 2
  // Not used in this simulation, set to 0xFF (not available)
  frame.data[4] = 0xFF;
  
  // Engine demand percent torque
  // Range: -125% to +125%, 1% per bit, offset by 125%
  frame.data[5] = simValues.engineDemandTorque + 125;
  
  // Send the frame
  ESP32Can.writeFrame(frame);
  
  // Simple debug output
  Serial.print("EEC2: Accel=");
  Serial.print(simValues.acceleratorPosition);
  Serial.print("%, Load=");
  Serial.print(simValues.engineLoad);
  Serial.println("%");
}

// Send Electronic Engine Controller 3 (EEC3) message
void sendEEC3() {
  CanFrame frame = { 0 };
  
  // Format standard J1939 ID for EEC3 (PGN 65267)
  frame.identifier = formatJ1939Identifier(6, J1939_PGN_EEC3, J1939_SA_ENGINE);
  frame.extd = 1;
  frame.data_length_code = 8;
  
  // Initialize all bytes to 0xFF (not available)
  for (int i = 0; i < 8; i++) {
    frame.data[i] = 0xFF;
  }
  
  // Nominal Friction - Percent Torque
  // Resolution: 1% per bit, offset 0
  // Range: 0 to 250%
  frame.data[0] = simValues.nominalFrictionTorque;
  
  // Engine's Desired Operating Speed
  // Not implemented in this simulation
  
  // Send the frame
  ESP32Can.writeFrame(frame);
}

// Send Turbocharger message
void sendTurbocharger() {
  CanFrame frame = { 0 };
  
  // Format standard J1939 ID for Turbocharger (PGN 65270)
  frame.identifier = formatJ1939Identifier(6, J1939_PGN_TURBO, J1939_SA_ENGINE);
  frame.extd = 1;
  frame.data_length_code = 8;
  
  // Initialize all bytes to 0xFF (not available)
  for (int i = 0; i < 8; i++) {
    frame.data[i] = 0xFF;
  }
  
  // Turbocharger Speed
  // Resolution: 4 rpm/bit, offset 0
  uint16_t turboSpeedScaled = (simValues.turboSpeed / 4 > 65534UL) ? 65534 : (uint16_t)(simValues.turboSpeed / 4);
  frame.data[0] = turboSpeedScaled & 0xFF;        // LSB
  frame.data[1] = (turboSpeedScaled >> 8) & 0xFF; // MSB
  
  // Boost Pressure
  // Resolution: 2 kPa/bit, offset 0
  uint8_t boostScaled = (uint8_t)(simValues.boostPressure / 2);
  frame.data[2] = (boostScaled > 250) ? 250 : boostScaled;
  
  // Intake Manifold Temperature
  // Resolution: 1 °C/bit, offset -40 °C
  // Range: -40 °C to 210 °C
  int16_t tempScaled = simValues.intakeTemp + 40;
  if (tempScaled < 0) tempScaled = 0;
  if (tempScaled > 250) tempScaled = 250;
  frame.data[3] = (uint8_t)tempScaled;
  
  // Send the frame
  ESP32Can.writeFrame(frame);
  
  // Simple debug output
  Serial.print("TURBO: Speed=");
  Serial.print(simValues.turboSpeed);
  Serial.print(" RPM, Boost=");
  Serial.print(simValues.boostPressure);
  Serial.print(" kPa, Intake=");
  Serial.print(simValues.intakeTemp);
  Serial.println("°C");
}

// Send Electronic Transmission Controller 1 (ETC1) message
void sendETC1() {
  CanFrame frame = { 0 };
  
  // Format standard J1939 ID for ETC1 (PGN 61445)
  frame.identifier = formatJ1939Identifier(3, J1939_PGN_ETC1, J1939_SA_TRANS);
  frame.extd = 1;
  frame.data_length_code = 8;
  
  // Initialize all bytes to 0xFF (not available)
  for (int i = 0; i < 8; i++) {
    frame.data[i] = 0xFF;
  }
  
  // Bytes 1-2: Transmission gear and mode data
  // Byte 1, bits 0-4: Transmission Selected Gear
  // Byte 1, bits 5-7: Transmission Mode Status
  frame.data[0] = (simValues.selectedGear & 0x1F);
  
  // Byte 2: bits 0-4: Current Gear
  frame.data[1] = (simValues.currentGear & 0x1F);
  
  // Other parameters not implemented in this simulation
  
  // Send the frame
  ESP32Can.writeFrame(frame);
}

// Send Liquid Fuel Economy (LFE1) message
void sendLFE1() {
  CanFrame frame = { 0 };
  
  // Format standard J1939 ID for LFE1 (PGN 65266)
  frame.identifier = formatJ1939Identifier(6, J1939_PGN_LFE1, J1939_SA_ENGINE);
  frame.extd = 1;
  frame.data_length_code = 8;
  
  // Initialize all bytes to 0xFF (not available)
  for (int i = 0; i < 8; i++) {
    frame.data[i] = 0xFF;
  }
  
  // Fuel Rate
  // Resolution: 0.05 L/h per bit, offset 0
  // 10 L/h = 200 bits
  uint16_t fuelRateScaled = simValues.fuelRate;
  frame.data[0] = fuelRateScaled & 0xFF;        // LSB
  frame.data[1] = (fuelRateScaled >> 8) & 0xFF; // MSB
  
  // Instantaneous Fuel Economy
  // Resolution: 0.01 km/L per bit, offset 0
  // 10 km/L = 1000 bits
  uint16_t fuelEconomyScaled = simValues.instantFuelEconomy;
  frame.data[2] = fuelEconomyScaled & 0xFF;        // LSB
  frame.data[3] = (fuelEconomyScaled >> 8) & 0xFF; // MSB
  
  // Send the frame
  ESP32Can.writeFrame(frame);
  
  // Simple debug output
  Serial.print("FUEL: Rate=");
  Serial.print(simValues.fuelRate * 0.05, 2);
  Serial.print(" L/h, Economy=");
  Serial.print(simValues.instantFuelEconomy * 0.01, 2);
  Serial.println(" km/L");
}

// Send Ambient Conditions message
void sendAmbientConditions() {
  CanFrame frame = { 0 };
  
  // Format standard J1939 ID for AMB (PGN 65269)
  frame.identifier = formatJ1939Identifier(6, J1939_PGN_AMB, J1939_SA_ENGINE);
  frame.extd = 1;
  frame.data_length_code = 8;
  
  // Initialize all bytes to 0xFF (not available)
  for (int i = 0; i < 8; i++) {
    frame.data[i] = 0xFF;
  }
  
  // Barometric Pressure
  // Resolution: 0.5 kPa/bit, offset 0
  uint8_t pressureScaled = (uint8_t)(simValues.airPressure * 2);
  frame.data[0] = (pressureScaled > 250) ? 250 : pressureScaled;
  
  // Ambient Air Temperature
  // Resolution: 1 °C/bit, offset -40 °C
  // Range: -40 °C to 215 °C
  int16_t tempScaled = simValues.ambientAirTemp + 40;
  if (tempScaled < 0) tempScaled = 0;
  if (tempScaled > 255) tempScaled = 255;
  frame.data[1] = (uint8_t)tempScaled;
  
  // Send the frame
  ESP32Can.writeFrame(frame);
  
  // Simple debug output
  Serial.print("AMB: Ambient=");
  Serial.print(simValues.ambientAirTemp);
  Serial.print("°C, Pressure=");
  Serial.print(simValues.airPressure);
  Serial.println(" kPa");
}

// Send Engine Temperature 1 message
void sendEngineTemperature1() {
  CanFrame frame = { 0 };
  
  // Format standard J1939 ID for ET1 (PGN 65262)
  frame.identifier = formatJ1939Identifier(6, J1939_PGN_ET1, J1939_SA_ENGINE);
  frame.extd = 1;
  frame.data_length_code = 8;
  
  // Initialize all bytes to 0xFF (not available)
  for (int i = 0; i < 8; i++) {
    frame.data[i] = 0xFF;
  }
  
  // Engine Coolant Temperature
  // Resolution: 1 °C/bit, offset -40 °C
  // Range: -40 °C to 210 °C
  int16_t coolantScaled = simValues.coolantTemp + 40;
  if (coolantScaled < 0) coolantScaled = 0;
  if (coolantScaled > 250) coolantScaled = 250;
  frame.data[0] = (uint8_t)coolantScaled;
  
  // Engine Fuel Temperature
  // Resolution: 1 °C/bit, offset -40 °C
  // Range: -40 °C to 210 °C
  int16_t fuelTempScaled = simValues.fuelTemp + 40;
  if (fuelTempScaled < 0) fuelTempScaled = 0;
  if (fuelTempScaled > 250) fuelTempScaled = 250;
  frame.data[1] = (uint8_t)fuelTempScaled;
  
  // Engine Oil Temperature
  // Resolution: 1 °C/bit, offset -40 °C
  // Range: -40 °C to 210 °C
  int16_t oilTempScaled = simValues.oilTemp + 40;
  if (oilTempScaled < 0) oilTempScaled = 0;
  if (oilTempScaled > 250) oilTempScaled = 250;
  frame.data[2] = (uint8_t)oilTempScaled;
  
  // Send the frame
  ESP32Can.writeFrame(frame);
  
  // Simple debug output
  Serial.print("TEMP: Coolant=");
  Serial.print(simValues.coolantTemp);
  Serial.print("°C, Oil=");
  Serial.print(simValues.oilTemp);
  Serial.print("°C, Fuel=");
  Serial.print(simValues.fuelTemp);
  Serial.println("°C");
}

// Send Engine Fluid Level/Pressure message
void sendEngineFluidLevelPressure() {
  CanFrame frame = { 0 };
  
  // Format standard J1939 ID for Engine Fluid Level/Pressure (PGN 65263 / 0xFEF1)
  frame.identifier = formatJ1939Identifier(6, J1939_PGN_FLUID, J1939_SA_ENGINE);
  frame.extd = 1;
  frame.data_length_code = 8;
  
  // Initialize all bytes to 0xFF (not available)
  for (int i = 0; i < 8; i++) {
    frame.data[i] = 0xFF;
  }
  
  // Engine Oil Pressure
  // Resolution: 4 kPa/bit, offset 0
  // Range: 0 to 1000 kPa
  uint8_t oilPressureScaled = (uint8_t)(simValues.engineOilPressure / 4);
  if (oilPressureScaled > 250) oilPressureScaled = 250;
  frame.data[3] = oilPressureScaled;
  
  // Send the frame
  ESP32Can.writeFrame(frame);
}

// Send alternative oil pressure formats in case Scanguage uses a different format
void sendOilPressureAlternateFormats() {
  static const uint32_t PGN_ALT_OIL_1 = 0xFEEA;  // Arbitrary test PGNs for oil
  static const uint32_t PGN_ALT_OIL_2 = 0xFEB5;
  static const uint32_t PGN_ALT_OIL_3 = 0xFEB6;
  
  // Format 1: Fluid PGN with different byte order
  {
    CanFrame frame = { 0 };
    frame.identifier = formatJ1939Identifier(3, PGN_ALT_OIL_1, J1939_SA_ENGINE);
    frame.extd = 1;
    frame.data_length_code = 8;
    
    // Initialize all bytes to 0xFF
    for (int i = 0; i < 8; i++) {
      frame.data[i] = 0xFF;
    }
    
    // Try all possible byte positions with standard 4 kPa/bit scaling
    uint8_t oilPressureScaled = (uint8_t)(simValues.engineOilPressure / 4);
    if (oilPressureScaled > 250) oilPressureScaled = 250;
    
    // Try different byte positions
    frame.data[0] = oilPressureScaled;
    frame.data[1] = oilPressureScaled;
    frame.data[2] = oilPressureScaled;
    // (Standard position is byte 3)
    frame.data[4] = oilPressureScaled;
    frame.data[5] = oilPressureScaled;
    
    ESP32Can.writeFrame(frame);
  }
  
  // Format 2: Direct PSI value
  {
    CanFrame frame = { 0 };
    frame.identifier = formatJ1939Identifier(3, PGN_ALT_OIL_2, J1939_SA_ENGINE);
    frame.extd = 1;
    frame.data_length_code = 8;
    
    // Initialize all bytes to 0xFF
    for (int i = 0; i < 8; i++) {
      frame.data[i] = 0xFF;
    }
    
    // Try direct PSI value (1 PSI per bit)
    uint8_t psiValue = (uint8_t)(simValues.engineOilPressure * 0.145);
    if (psiValue > 250) psiValue = 250;
    
    // Try all possible byte positions
    for (int i = 0; i < 6; i++) {
      frame.data[i] = psiValue;
    }
    
    ESP32Can.writeFrame(frame);
  }
  
  // Format 3: 0.1 PSI per bit, 2-byte value
  {
    CanFrame frame = { 0 };
    frame.identifier = formatJ1939Identifier(3, PGN_ALT_OIL_3, J1939_SA_ENGINE);
    frame.extd = 1;
    frame.data_length_code = 8;
    
    // Initialize all bytes to 0xFF
    for (int i = 0; i < 8; i++) {
      frame.data[i] = 0xFF;
    }
    
    // Try 0.1 PSI per bit (2-byte value)
    uint16_t tenthPsiValue = (uint16_t)(simValues.engineOilPressure * 0.145 * 10);
    if (tenthPsiValue > 65535) tenthPsiValue = 65535;
    
    // Try in bytes 0-1
    frame.data[0] = tenthPsiValue & 0xFF;        // LSB
    frame.data[1] = (tenthPsiValue >> 8) & 0xFF; // MSB
    
    // And in bytes 2-3
    frame.data[2] = tenthPsiValue & 0xFF;        // LSB
    frame.data[3] = (tenthPsiValue >> 8) & 0xFF; // MSB
    
    ESP32Can.writeFrame(frame);
  }
}

// Send Vehicle Electrical Power message
void sendVehicleElectricalPower() {
  CanFrame frame = { 0 };
  
  // Format standard J1939 ID for VEP1 (PGN 65263)
  frame.identifier = formatJ1939Identifier(6, J1939_PGN_VEP1, J1939_SA_ENGINE);
  frame.extd = 1;
  frame.data_length_code = 8;
  
  // Initialize all bytes to 0xFF (not available)
  for (int i = 0; i < 8; i++) {
    frame.data[i] = 0xFF;
  }
  
  // Electrical Potential (Voltage)
  // Resolution: 0.05 V/bit, offset 0
  // 12V battery = 240 bits
  frame.data[0] = simValues.batteryVoltage & 0xFF;
  frame.data[1] = (simValues.batteryVoltage >> 8) & 0xFF;
  
  // Charging System Potential (Voltage)
  // Resolution: 0.05 V/bit, offset 0
  frame.data[2] = simValues.batteryVoltage & 0xFF;  // Using same value as battery
  frame.data[3] = (simValues.batteryVoltage >> 8) & 0xFF;
  
  // Battery Potential / Power Input 1
  frame.data[4] = simValues.batteryVoltage & 0xFF;
  frame.data[5] = (simValues.batteryVoltage >> 8) & 0xFF;
  
  // Alternator Current
  // Resolution: 0.05 A/bit, offset 0
  frame.data[6] = simValues.alternatoCurrent & 0xFF;
  frame.data[7] = (simValues.alternatoCurrent >> 8) & 0xFF;
  
  // Send the frame
  ESP32Can.writeFrame(frame);
  
  // Simple debug output
  Serial.print("ELEC: Battery=");
  Serial.print(simValues.batteryVoltage * 0.05, 2);
  Serial.print("V, Alternator=");
  Serial.print(simValues.alternatoCurrent * 0.05, 2);
  Serial.println("A");
}