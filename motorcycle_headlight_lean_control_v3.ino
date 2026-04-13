/*
 * Motorcycle Adaptive Cornering Lights V3
 * Xiao ESP32-C6 (or ESP32-C3-WROOM-02)
 *
 * Hardware:
 * - ESP32-C6 microcontroller
 * - TCA9548A I2C multiplexer
 * - 2x DFRobot Laser Distance Sensors (SEN0590, address 0x74, via TCA channels 0/1)
 * - BMI160 IMU (I2C directly on D4/D5, address 0x68)
 * - TMC2209 stepper driver (STEP/DIR/EN on D0/D1/D2, standalone mode)
 * - NEMA 8 stepper motor driving worm gear (72:1) -> bi-LED projector rotation
 * - 2x limit switches (D6/D7, active LOW with pull-ups)
 *
 * Homing sequence (triggered manually via web UI button):
 *   1. Rotate toward left limit switch until triggered -> record pot ADC
 *   2. Rotate toward right limit switch until triggered -> record pot ADC
 *   3. Move to center (midpoint) -> set center ADC, enable closed loop, save to NVS
 *   On subsequent boots: pot calibration loaded from NVS, homing skipped
 *
 * Position control:
 *   Closed loop (default after first homing): pot ADC drives step direction
 *   Open loop (fallback): step counter used if pot not calibrated
 *
 * Pin Assignments (ESP32-C6 Xiao):
 *   D0  (GPIO0):  STEP -> TMC2209
 *   D1  (GPIO1):  DIR  -> TMC2209
 *   D2  (GPIO2):  EN   -> TMC2209 (active LOW)
 *   D3  (GPIO21): TMC2209 UART TX (via 1kΩ) + RX (D9) for current control
 *   D4  (GPIO22): I2C SDA -> TCA9548A, BMI160
 *   D5  (GPIO23): I2C SCL -> TCA9548A, BMI160
 *   D6  (GPIO16): Limit switch LEFT  (INPUT_PULLUP, active LOW)
 *   D7  (GPIO17): Limit switch RIGHT (INPUT_PULLUP, active LOW)
 *   D8  (GPIO19): Speed sensor input
 *   D9  (GPIO20): TMC2209 UART RX
 *   D10 (GPIO18): Position pot ADC input
 */

#include <Wire.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <Preferences.h>

// ============================================================================
// FIRMWARE VERSION
// ============================================================================
#define FIRMWARE_VERSION "20260413_01"


// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define STEP_PIN            0     // D0 - TMC2209 STEP
#define DIR_PIN             1     // D1 - TMC2209 DIR
#define EN_PIN              2     // D2 - TMC2209 EN (active LOW = enabled)
#define TMC_TX_PIN          21    // D3 - TMC2209 UART TX via 1k resistor
#define I2C_SDA             22    // D4
#define I2C_SCL             23    // D5
#define LIMIT_LEFT_PIN      16    // D6 - left limit switch (INPUT_PULLUP, LOW = triggered)
#define LIMIT_RIGHT_PIN     17    // D7 - right limit switch (INPUT_PULLUP, LOW = triggered)
#define SPEED_SENSOR_PIN    19    // D8 - speed sensor input
#define TMC_RX_PIN          20    // D9 - TMC2209 UART RX
#define POT_PIN             18    // D10 - position feedback potentiometer (ADC)

// ============================================================================
// I2C ADDRESSES
// ============================================================================
#define TCA9548A_ADDR       0x70
#define SENSOR_ADDR         0x74
#define BMI160_ADDR         0x68

#define LEFT_SENSOR_CH      0
#define RIGHT_SENSOR_CH     1

// ============================================================================
// BMI160 REGISTERS
// ============================================================================
#define BMI160_CHIP_ID        0x00
#define BMI160_DATA_GYRO_X_L  0x0C
#define BMI160_GYR_CONF       0x42
#define BMI160_GYR_RANGE      0x43
#define BMI160_CMD            0x7E
#define BMI160_CHIP_ID_VAL    0xD1
#define BMI160_CMD_GYRO_NORMAL 0x15
#define BMI160_CMD_SOFT_RESET  0xB6
#define BMI160_GYR_RANGE_250  0x03
#define BMI160_GYR_CONF_200HZ 0x29
#define GYRO_SCALE_250        131.2f

// ============================================================================
// DFROBOT SENSOR REGISTERS
// ============================================================================
#define SENSOR_START_REG    0x10
#define SENSOR_DATA_REG     0x02
#define SENSOR_START_CMD    0xB0

// ============================================================================
// STEPPER MOTOR CONFIG
// ============================================================================
#define STEPPER_MICROSTEPPING   8       // MS1=GND, MS2=GND -> 1/8 step
#define STEPPER_STEPS_PER_REV   200     // NEMA 8, 1.8 deg/step
#define WORM_RATIO              72      // 72:1 worm drive
// Steps per degree of projector rotation
#define STEPS_PER_DEGREE  ((STEPPER_STEPS_PER_REV * STEPPER_MICROSTEPPING * WORM_RATIO) / 360)
// = 200 * 8 * 72 / 360 = 320 steps/degree

#define HOMING_SPEED_US     800     // Microseconds between steps during homing (slow)
#define NORMAL_SPEED_US     200     // Microseconds between steps during normal operation
#define HOMING_BACKOFF_STEPS 100    // Steps to back off from limit switch after triggering
#define MAX_HOMING_STEPS    50000   // Safety: abort homing if this many steps without trigger

// ============================================================================
// DEFAULTS
// ============================================================================
#define DEFAULT_SENSOR_SPACING    800.0   // mm between sensors
#define DEFAULT_HYSTERESIS        2.0     // degrees — dead band to prevent jitter at target
#define DEFAULT_DEAD_ZONE         5.0     // degrees — lean below this = center, motor disabled
#define DEFAULT_MAX_LEAN          45.0    // degrees full travel each side
#define DEFAULT_MIN_DISTANCE      50      // mm
#define DEFAULT_MAX_DISTANCE      2000    // mm
#define DEFAULT_SAMPLE_INTERVAL   100     // ms


// ============================================================================
// SENSOR GEOMETRY CONFIG
// ============================================================================
struct SensorGeometry {
  float heightMm;       // Height above ground (mm)
  float widthMm;        // Distance from vehicle centerline (mm), positive = outboard
  float angleDeg;       // Mount angle offset (degrees), 0 = pointing straight down
};

// ============================================================================
// CONFIG STRUCT
// ============================================================================
struct Config {
  char deviceName[32];
  char wifiSSID[64];
  char wifiPassword[64];
  bool useAPMode;
  char apPassword[64];

  // Sensor geometry
  SensorGeometry leftSensor;
  SensorGeometry rightSensor;
  float sensorSpacing;        // Effective horizontal spacing used for lean calc (mm)
  float leftSensorOffset;     // Distance offset correction (mm)
  float rightSensorOffset;
  uint16_t minDistance;
  uint16_t maxDistance;
  uint16_t sampleInterval;

  // IMU
  bool useIMU;
  bool useDistanceSensors;
  int  imuYawAxis;            // 0=X, 1=Y, 2=Z
  bool imuYawInvert;
  float imuGyroScale;

  // Lean to projector mapping
  float maxLeanAngle;         // Lean angle (deg) that corresponds to full projector travel
  float hysteresis;           // Dead band around target position to prevent jitter
  float deadZone;             // Lean below this (deg) = center position, motor disabled

  // Position pot feedback
  bool  usePot;               // true = closed loop via pot, false = open loop step count
  int   potLeftADC;           // ADC reading at left limit switch (saved during homing)
  int   potRightADC;          // ADC reading at right limit switch (saved during homing)
  int   potCenterADC;         // ADC reading at center = (left+right)/2 + offset
  int   potCenterOffset;      // User-adjustable offset in ADC counts (+/- 200)
  int   potDeadband;          // ADC counts of deadband around target (prevents hunting)

  // TMC2209 current control via UART
  int   motorCurrentMA;       // Motor RMS current in mA (50–400)

  // Speed sensor
  uint16_t pulsesPerRev;
  float wheelCircumference;
};

Config config = {
  "CL-V3",
  "", "", true, "cornering123",
  // Left sensor geometry defaults
  { 300.0f, 400.0f, 0.0f },
  // Right sensor geometry defaults
  { 300.0f, 400.0f, 0.0f },
  DEFAULT_SENSOR_SPACING,
  0.0f, 0.0f,
  DEFAULT_MIN_DISTANCE,
  DEFAULT_MAX_DISTANCE,
  DEFAULT_SAMPLE_INTERVAL,
  // IMU
  true, true, 2, false, 0.0f,
  // Lean mapping
  DEFAULT_MAX_LEAN,
  DEFAULT_HYSTERESIS,
  DEFAULT_DEAD_ZONE,
  // Pot feedback (disabled until homing run)
  false,   // usePot
  0,       // potLeftADC
  4095,    // potRightADC
  2047,    // potCenterADC
  0,       // potCenterOffset
  30,      // potDeadband (~0.5° at 1:1)
  // TMC2209 current
  300,     // motorCurrentMA — validated at 300mA, 15min continuous, ~60-65°C surface
  // Speed sensor
  4, 1.95f
};

// ============================================================================
// SYSTEM STATE
// ============================================================================
struct SystemState {
  // Sensors
  uint16_t leftDistance;
  uint16_t rightDistance;
  bool leftValid;
  bool rightValid;

  // Lean angle
  float leanAngle;
  float leanAngleDist;
  float leanAngleIMU;

  // IMU
  float yawRate;
  float speedMs;
  float gyroZeroBias;
  bool imuInitialized;
  bool imuCalibrated;

  // Stepper / projector
  bool homingComplete;
  bool stepperActive;         // true when motor is enabled and tracking
  long currentSteps;          // Steps from center (negative = left, positive = right)
  long homingMaxSteps;        // Total steps left-to-right measured during homing
  long targetSteps;           // Requested target position in steps from center
  float projectorAngle;       // Current projector angle in degrees
  int  potADC;                // Raw ADC reading from position pot (0-4095)
  float potAngle;             // Pot-derived projector angle in degrees

  // Test mode
  bool testMode;
  float testAngle;            // Manual test angle (-45 to +45)

  // Diagnostics
  uint32_t errorCount;
  uint32_t sensorErrorCount;
  unsigned long lastUpdate;
  unsigned long uptime;
  bool systemInitialized;
};

SystemState state = {};


// ============================================================================
// OBJECTS
// ============================================================================
WebServer server(80);
Preferences preferences;

unsigned long lastSampleTime    = 0;
unsigned long lastStepTime      = 0;

// ============================================================================
// TMC2209 UART CURRENT CONTROL (raw protocol, no TMCStepper library)
// Wiring: D3/IO21 -1k-> TMC TX pad, D9/IO20 -> TMC TX pad, 10k pullup to 3.3V
// ============================================================================
uint8_t tmcCRC(uint8_t* d, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t b = d[i];
    for (uint8_t j = 0; j < 8; j++) {
      if ((crc >> 7) ^ (b & 1)) crc = (crc << 1) ^ 0x07;
      else crc = crc << 1;
      b >>= 1;
    }
  }
  return crc;
}

void tmcWriteReg(uint8_t reg, uint32_t val) {
  uint8_t pkt[7] = {0x05, 0x00, (uint8_t)(reg | 0x80),
    (uint8_t)(val >> 24), (uint8_t)(val >> 16),
    (uint8_t)(val >> 8),  (uint8_t)(val)};
  uint8_t crc = tmcCRC(pkt, 7);
  for (int i = 0; i < 7; i++) Serial1.write(pkt[i]);
  Serial1.write(crc);
  Serial1.flush();
  delay(2);
  while (Serial1.available()) Serial1.read(); // drain echo
}

void tmcSetCurrent(uint16_t mA) {
  // CS = I_rms * 32 * sqrt(2) * Rsense / V_FS - 1
  // V_FS = 0.325V, Rsense = 0.11 ohm (adjust if needed)
  float cs = mA / 1000.0f * 32.0f * 1.41421f * 0.11f / 0.325f - 1.0f;
  uint8_t irun  = (uint8_t)constrain((int)round(cs), 0, 31);
  uint8_t ihold = irun / 2;
  uint32_t regval = ((uint32_t)3 << 16) | ((uint32_t)irun << 8) | ihold;
  Serial.printf("[TMC] Setting current: %umA  IRUN=%d  IHOLD=%d\n", mA, irun, ihold);
  // GCONF: all defaults (I_scale_analog=0, pdn_disable=0, mstep_reg_select=0)
  tmcWriteReg(0x00, 0x00000000);
  delay(5);
  tmcWriteReg(0x10, regval);
}

void initTMCUart() {
  Serial.printf("[TMC] Initializing UART at 115200, target current %umA\n",
                config.motorCurrentMA);
  Serial1.begin(115200, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);
  delay(50);
  tmcSetCurrent(config.motorCurrentMA);
  Serial1.end();
  // Restore stepper pins
  pinMode(STEP_PIN, OUTPUT); digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN,  OUTPUT); digitalWrite(DIR_PIN,  LOW);
  pinMode(EN_PIN,   OUTPUT); digitalWrite(EN_PIN,   HIGH);
  Serial.println("[TMC] UART done, stepper pins restored");
}

// ============================================================================
// POSITION POTENTIOMETER
// ============================================================================
int readPotADC() {
  // Average 4 samples to reduce noise
  long sum = 0;
  for (int i = 0; i < 4; i++) { sum += analogRead(POT_PIN); delay(1); }
  return (int)(sum / 4);
}

// Convert pot ADC reading to projector angle in degrees
// Uses potLeftADC, potRightADC, potCenterADC from config
float potADCToAngle(int adc) {
  if (!config.usePot) return 0.0f;
  int center = config.potCenterADC;
  float range = (float)(config.potRightADC - config.potLeftADC);
  if (fabsf(range) < 10) return 0.0f; // not calibrated
  float maxAngle = config.maxLeanAngle;
  return (float)(adc - center) / (range / 2.0f) * maxAngle;
}

void updatePotReading() {
  state.potADC   = readPotADC();
  state.potAngle = potADCToAngle(state.potADC);
  if (config.usePot) state.projectorAngle = state.potAngle;
}

// ============================================================================
// SPEED SENSOR (interrupt-based)
// ============================================================================
volatile uint32_t speedPulseCount = 0;
volatile unsigned long lastPulseTime = 0;
unsigned long lastSpeedCalcTime = 0;
uint32_t lastPulseSnapshot = 0;

void IRAM_ATTR speedPulseISR() {
  speedPulseCount++;
  lastPulseTime = micros();
}

void updateSpeed() {
  unsigned long now = millis();
  if (now - lastSpeedCalcTime < 100) return;
  uint32_t pulses = speedPulseCount - lastPulseSnapshot;
  lastPulseSnapshot = speedPulseCount;
  unsigned long elapsed = now - lastSpeedCalcTime;
  lastSpeedCalcTime = now;
  float revs = (float)pulses / config.pulsesPerRev;
  state.speedMs = revs * config.wheelCircumference / (elapsed / 1000.0f);
  if ((micros() - lastPulseTime) > 500000UL) state.speedMs = 0.0f;
}

// ============================================================================
// BMI160 IMU
// ============================================================================
uint8_t bmiRead(uint8_t reg) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BMI160_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void bmiWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

float readRawYawRate() {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(BMI160_DATA_GYRO_X_L);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BMI160_ADDR, (uint8_t)6);
  if (Wire.available() < 6) return 0.0f;
  int16_t gx = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t gy = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t gz = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t raw = 0;
  switch (config.imuYawAxis) {
    case 0: raw = gx; break;
    case 1: raw = gy; break;
    case 2: raw = gz; break;
  }
  if (config.imuYawInvert) raw = -raw;
  return raw / config.imuGyroScale;
}

bool initIMU() {
  uint8_t chipId = bmiRead(BMI160_CHIP_ID);
  if (chipId != BMI160_CHIP_ID_VAL) {
    Serial.printf("BMI160 not found (0x%02X)\n", chipId);
    return false;
  }
  bmiWrite(BMI160_CMD, BMI160_CMD_SOFT_RESET);
  delay(100);
  bmiWrite(BMI160_GYR_RANGE, BMI160_GYR_RANGE_250);
  delay(1);
  bmiWrite(BMI160_GYR_CONF, BMI160_GYR_CONF_200HZ);
  delay(1);
  bmiWrite(BMI160_CMD, BMI160_CMD_GYRO_NORMAL);
  delay(80);
  config.imuGyroScale = GYRO_SCALE_250;
  state.imuInitialized = true;
  Serial.println("BMI160 initialized");
  return true;
}

void calibrateGyroBias(int samples = 200) {
  if (!state.imuInitialized) return;
  Serial.println("Calibrating gyro - keep still...");
  float sum = 0;
  for (int i = 0; i < samples; i++) { sum += readRawYawRate(); delay(5); }
  state.gyroZeroBias = sum / samples;
  state.imuCalibrated = true;
  Serial.printf("Gyro bias: %.4f deg/s\n", state.gyroZeroBias);
}

void updateIMULean() {
  if (!state.imuInitialized || !state.imuCalibrated) return;
  state.yawRate = readRawYawRate() - state.gyroZeroBias;
  if (state.speedMs < 2.0f) { state.leanAngleIMU = 0.0f; return; }
  float omegaRad = state.yawRate * (M_PI / 180.0f);
  float leanRad  = atanf(state.speedMs * omegaRad / 9.81f);
  state.leanAngleIMU = leanRad * (180.0f / M_PI);
}


// ============================================================================
// LEAN ANGLE FUSION
// ============================================================================
void updateLeanAngle() {
  bool distValid = state.leftValid && state.rightValid;
  if (config.useIMU && config.useDistanceSensors) {
    if (distValid && state.speedMs >= 2.0f)
      state.leanAngle = 0.3f * state.leanAngleDist + 0.7f * state.leanAngleIMU;
    else if (distValid)
      state.leanAngle = state.leanAngleDist;
    else if (state.speedMs >= 2.0f)
      state.leanAngle = state.leanAngleIMU;
    else
      state.leanAngle = 0.0f;
  } else if (config.useIMU) {
    state.leanAngle = state.leanAngleIMU;
  } else {
    state.leanAngle = distValid ? state.leanAngleDist : 0.0f;
  }
}

// ============================================================================
// TCA9548A
// ============================================================================
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void tcaDisableAll() {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

// ============================================================================
// DFROBOT DISTANCE SENSORS
// ============================================================================
bool sensorWriteReg(uint8_t reg, const void* pBuf, size_t size) {
  uint8_t* _pBuf = (uint8_t*)pBuf;
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(&reg, 1);
  for (uint16_t i = 0; i < size; i++) Wire.write(_pBuf[i]);
  return (Wire.endTransmission() == 0);
}

uint8_t sensorReadReg(uint8_t reg, void* pBuf, size_t size) {
  uint8_t* _pBuf = (uint8_t*)pBuf;
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(&reg, 1);
  if (Wire.endTransmission() != 0) return 0;
  delay(20);
  Wire.requestFrom(SENSOR_ADDR, (uint8_t)size);
  for (uint16_t i = 0; i < size; i++) _pBuf[i] = Wire.read();
  return size;
}

uint16_t readDistance() {
  uint8_t cmd = SENSOR_START_CMD;
  uint8_t buf[2] = {0};
  if (!sensorWriteReg(SENSOR_START_REG, &cmd, 1)) return 0;
  delay(50);
  if (sensorReadReg(SENSOR_DATA_REG, buf, 2) != 2) return 0;
  return buf[0] * 256 + buf[1] + 10;
}

bool initializeSensors() {
  tcaSelect(LEFT_SENSOR_CH);
  delay(10);
  uint16_t d = readDistance();
  if (d == 0 || d > 5000) { Serial.println("Left sensor init failed"); return false; }
  Serial.printf("Left sensor OK (%dmm)\n", d);
  tcaSelect(RIGHT_SENSOR_CH);
  delay(10);
  d = readDistance();
  if (d == 0 || d > 5000) { Serial.println("Right sensor init failed"); return false; }
  Serial.printf("Right sensor OK (%dmm)\n", d);
  tcaDisableAll();
  return true;
}

float calculateLeanAngle(uint16_t leftDist, uint16_t rightDist) {
  float l = leftDist  + config.leftSensorOffset;
  float r = rightDist + config.rightSensorOffset;
  float diff = r - l;
  return atan2(diff, config.sensorSpacing) * (180.0f / M_PI);
}

void readSensors() {
  tcaSelect(LEFT_SENSOR_CH);
  state.leftDistance = readDistance();
  state.leftValid = (state.leftDistance >= config.minDistance &&
                     state.leftDistance <= config.maxDistance);
  tcaSelect(RIGHT_SENSOR_CH);
  state.rightDistance = readDistance();
  state.rightValid = (state.rightDistance >= config.minDistance &&
                      state.rightDistance <= config.maxDistance);
  tcaDisableAll();
  if (state.leftValid && state.rightValid) {
    state.leanAngleDist = calculateLeanAngle(state.leftDistance, state.rightDistance);
  } else {
    state.sensorErrorCount++;
  }
  state.lastUpdate = millis();
}


// ============================================================================
// STEPPER MOTOR / PROJECTOR CONTROL
// ============================================================================

// Step the motor one step in the given direction
// dir: true = toward right limit, false = toward left limit
void stepMotor(bool dir, unsigned int delayUs) {
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
  delayMicroseconds(1);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(delayUs / 2);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delayUs / 2);
}

bool leftLimitTriggered()  { return digitalRead(LIMIT_LEFT_PIN)  == LOW; }
bool rightLimitTriggered() { return digitalRead(LIMIT_RIGHT_PIN) == LOW; }

// Enable / disable TMC2209 (active LOW)
void stepperEnable()  { digitalWrite(EN_PIN, LOW);  }
void stepperDisable() { digitalWrite(EN_PIN, HIGH); }

// ============================================================================
// HOMING SEQUENCE
// Rotates to left limit, records step 0, then rotates to right limit,
// records maxSteps, then moves to center.
// ============================================================================
bool homeProjector() {
  Serial.println("Homing: moving to left limit...");
  stepperEnable();
  delay(10);

  // Phase 1: move toward left limit
  long steps = 0;
  while (!leftLimitTriggered()) {
    stepMotor(false, HOMING_SPEED_US);
    steps++;
    if (steps > MAX_HOMING_STEPS) {
      Serial.println("Homing FAILED: left limit not found");
      stepperDisable();
      return false;
    }
    if (steps % 100 == 0) { server.handleClient(); ArduinoOTA.handle(); }
  }
  Serial.printf("Left limit hit after %ld steps\n", steps);

  // Record pot ADC at left limit
  delay(50);
  config.potLeftADC = readPotADC();
  Serial.printf("  Pot ADC at left limit: %d\n", config.potLeftADC);

  // Back off from left limit
  for (int i = 0; i < HOMING_BACKOFF_STEPS; i++) stepMotor(true, HOMING_SPEED_US);
  state.currentSteps = 0;

  // Phase 2: move toward right limit, counting steps
  Serial.println("Homing: moving to right limit...");
  steps = 0;
  while (!rightLimitTriggered()) {
    stepMotor(true, HOMING_SPEED_US);
    steps++;
    state.currentSteps++;
    if (steps > MAX_HOMING_STEPS) {
      Serial.println("Homing FAILED: right limit not found");
      stepperDisable();
      return false;
    }
    if (steps % 100 == 0) { server.handleClient(); ArduinoOTA.handle(); }
  }
  Serial.printf("Right limit hit. Total travel: %ld steps\n", steps);

  // Record pot ADC at right limit
  delay(50);
  config.potRightADC = readPotADC();
  Serial.printf("  Pot ADC at right limit: %d\n", config.potRightADC);

  // Back off from right limit
  for (int i = 0; i < HOMING_BACKOFF_STEPS; i++) {
    stepMotor(false, HOMING_SPEED_US);
    state.currentSteps--;
  }
  state.homingMaxSteps = state.currentSteps;

  // Phase 3: move to center
  Serial.println("Homing: moving to center...");
  long center = state.homingMaxSteps / 2;
  while (state.currentSteps > center) {
    stepMotor(false, HOMING_SPEED_US);
    state.currentSteps--;
  }
  while (state.currentSteps < center) {
    stepMotor(true, HOMING_SPEED_US);
    state.currentSteps++;
  }

  // Calculate and save pot center ADC
  config.potCenterADC = (config.potLeftADC + config.potRightADC) / 2
                        + config.potCenterOffset;
  config.usePot = true;
  Serial.printf("  Pot center ADC: %d (offset %d)\n",
                config.potCenterADC, config.potCenterOffset);
  saveConfig();

  // Re-zero: center is now step 0 reference
  state.currentSteps  = 0;
  state.homingComplete = true;
  state.projectorAngle = 0.0f;

  Serial.printf("Homing complete. Travel: %ld steps (%.1f deg)\n",
                state.homingMaxSteps,
                (float)state.homingMaxSteps / STEPS_PER_DEGREE);
  Serial.printf("Pot range: %d (left) to %d (right), center %d\n",
                config.potLeftADC, config.potRightADC, config.potCenterADC);
  return true;
}

// Move projector to target angle (degrees, negative = left, positive = right)
// Called from main loop - moves one step per call for non-blocking operation.
//
// Control strategy:
//   |lean| < deadZone  → target is center (0 steps), disable motor when reached
//   |lean| >= deadZone → enable motor, track lean angle mapped to projector travel
//   Motor is only enabled when actively moving toward a target.
//   Once target is reached the worm gear holds position and motor is disabled.
void updateProjectorPosition() {
  if (!state.homingComplete) return;

  float targetAngle = state.testMode ? state.testAngle : state.leanAngle;

  // Dead zone: lean below threshold snaps target to center
  if (!state.testMode && fabsf(targetAngle) < config.deadZone) {
    targetAngle = 0.0f;
  }

  // Clamp to safe mechanical range
  float maxAngle = (float)state.homingMaxSteps / 2.0f / STEPS_PER_DEGREE;
  targetAngle = constrain(targetAngle, -maxAngle, maxAngle);

  // Map angle to steps from center
  state.targetSteps = (long)(targetAngle * STEPS_PER_DEGREE);
}

// Non-blocking single-step toward target.
// In closed-loop mode (usePot=true): uses pot ADC as position feedback.
// In open-loop mode: uses step counter.
// Enables motor when movement needed, disables when target reached.
void stepTowardTarget() {
  if (!state.homingComplete) return;

  long error;
  if (config.usePot) {
    // Closed loop: compare pot ADC to target ADC
    // Convert targetSteps to target ADC
    float targetAngle = (float)state.targetSteps / STEPS_PER_DEGREE;
    float adcRange    = (float)(config.potRightADC - config.potLeftADC);
    int   targetADC   = config.potCenterADC + (int)(targetAngle / config.maxLeanAngle
                        * adcRange / 2.0f);
    error = targetADC - state.potADC;
    if (abs(error) <= config.potDeadband) {
      if (state.stepperActive) { stepperDisable(); state.stepperActive = false; }
      return;
    }
  } else {
    // Open loop: use step counter
    error = state.targetSteps - state.currentSteps;
    if (abs(error) <= 1) {
      if (state.stepperActive) { stepperDisable(); state.stepperActive = false; }
      return;
    }
  }

  // Need to move — enable motor if not already
  if (!state.stepperActive) { stepperEnable(); state.stepperActive = true; }

  // Safety: don't drive into limits
  if (leftLimitTriggered()  && error < 0) { stepperDisable(); state.stepperActive = false; return; }
  if (rightLimitTriggered() && error > 0) { stepperDisable(); state.stepperActive = false; return; }

  // Rate limit
  unsigned long now = micros();
  if (now - lastStepTime < (unsigned long)NORMAL_SPEED_US) return;
  lastStepTime = now;

  bool dir = (error > 0);
  stepMotor(dir, 1);
  state.currentSteps += dir ? 1 : -1;
  if (!config.usePot) state.projectorAngle = (float)state.currentSteps / STEPS_PER_DEGREE;
}


// ============================================================================
// CONFIG: LOAD / SAVE / RESET
// ============================================================================
void loadConfig() {
  preferences.begin("cl_v3", false);
  preferences.getString("deviceName", config.deviceName, sizeof(config.deviceName));
  preferences.getString("wifiSSID",   config.wifiSSID,   sizeof(config.wifiSSID));
  preferences.getString("wifiPass",   config.wifiPassword, sizeof(config.wifiPassword));
  config.useAPMode = preferences.getBool("useAPMode", true);
  preferences.getString("apPass",     config.apPassword, sizeof(config.apPassword));

  config.sensorSpacing     = preferences.getFloat("spacing",   DEFAULT_SENSOR_SPACING);
  config.leftSensorOffset  = preferences.getFloat("leftOff",   0.0f);
  config.rightSensorOffset = preferences.getFloat("rightOff",  0.0f);
  config.minDistance       = preferences.getUShort("minDist",  DEFAULT_MIN_DISTANCE);
  config.maxDistance       = preferences.getUShort("maxDist",  DEFAULT_MAX_DISTANCE);
  config.sampleInterval    = preferences.getUShort("sampleInt",DEFAULT_SAMPLE_INTERVAL);

  config.leftSensor.heightMm  = preferences.getFloat("lsH",  300.0f);
  config.leftSensor.widthMm   = preferences.getFloat("lsW",  400.0f);
  config.leftSensor.angleDeg  = preferences.getFloat("lsA",  0.0f);
  config.rightSensor.heightMm = preferences.getFloat("rsH",  300.0f);
  config.rightSensor.widthMm  = preferences.getFloat("rsW",  400.0f);
  config.rightSensor.angleDeg = preferences.getFloat("rsA",  0.0f);

  config.maxLeanAngle  = preferences.getFloat("maxLean",   DEFAULT_MAX_LEAN);
  config.hysteresis    = preferences.getFloat("hysteresis",DEFAULT_HYSTERESIS);
  config.deadZone      = preferences.getFloat("deadZone",  DEFAULT_DEAD_ZONE);
  config.usePot        = preferences.getBool("usePot",     false);
  config.potLeftADC    = preferences.getInt("potLeftADC",  0);
  config.potRightADC   = preferences.getInt("potRightADC", 4095);
  config.potCenterADC  = preferences.getInt("potCenterADC",2047);
  config.potCenterOffset = preferences.getInt("potOffset", 0);
  config.potDeadband   = preferences.getInt("potDeadband", 30);
  config.motorCurrentMA = preferences.getInt("motorMA",    200);
  config.useIMU        = preferences.getBool("useIMU",     true);
  config.useDistanceSensors = preferences.getBool("useDist", true);
  config.imuYawAxis    = preferences.getInt("imuAxis",     0);  // X confirmed on bench
  config.imuYawInvert  = preferences.getBool("imuInv",     false);
  config.pulsesPerRev  = preferences.getUShort("ppr",      4);
  config.wheelCircumference = preferences.getFloat("wheelC", 1.95f);

  preferences.end();
  Serial.println("Config loaded");
}

void saveConfig() {
  preferences.begin("cl_v3", false);
  preferences.putString("deviceName", config.deviceName);
  preferences.putString("wifiSSID",   config.wifiSSID);
  preferences.putString("wifiPass",   config.wifiPassword);
  preferences.putBool("useAPMode",    config.useAPMode);
  preferences.putString("apPass",     config.apPassword);

  preferences.putFloat("spacing",     config.sensorSpacing);
  preferences.putFloat("leftOff",     config.leftSensorOffset);
  preferences.putFloat("rightOff",    config.rightSensorOffset);
  preferences.putUShort("minDist",    config.minDistance);
  preferences.putUShort("maxDist",    config.maxDistance);
  preferences.putUShort("sampleInt",  config.sampleInterval);

  preferences.putFloat("lsH",  config.leftSensor.heightMm);
  preferences.putFloat("lsW",  config.leftSensor.widthMm);
  preferences.putFloat("lsA",  config.leftSensor.angleDeg);
  preferences.putFloat("rsH",  config.rightSensor.heightMm);
  preferences.putFloat("rsW",  config.rightSensor.widthMm);
  preferences.putFloat("rsA",  config.rightSensor.angleDeg);

  preferences.putFloat("maxLean",     config.maxLeanAngle);
  preferences.putFloat("hysteresis",  config.hysteresis);
  preferences.putFloat("deadZone",    config.deadZone);
  preferences.putBool("usePot",       config.usePot);
  preferences.putInt("potLeftADC",    config.potLeftADC);
  preferences.putInt("potRightADC",   config.potRightADC);
  preferences.putInt("potCenterADC",  config.potCenterADC);
  preferences.putInt("potOffset",     config.potCenterOffset);
  preferences.putInt("potDeadband",   config.potDeadband);
  preferences.putInt("motorMA",       config.motorCurrentMA);
  preferences.putBool("useIMU",       config.useIMU);
  preferences.putBool("useDist",      config.useDistanceSensors);
  preferences.putInt("imuAxis",       config.imuYawAxis);
  preferences.putBool("imuInv",       config.imuYawInvert);
  preferences.putUShort("ppr",        config.pulsesPerRev);
  preferences.putFloat("wheelC",      config.wheelCircumference);

  preferences.end();
  Serial.println("Config saved");
}

void resetConfig() {
  preferences.begin("cl_v3", false);
  preferences.clear();
  preferences.end();
  ESP.restart();
}


// ============================================================================
// WIFI
// ============================================================================
void setupWiFi() {
  if (config.useAPMode || strlen(config.wifiSSID) == 0) {
    WiFi.mode(WIFI_AP);
    IPAddress local_IP(192, 168, 5, 1);
    IPAddress gateway(192, 168, 5, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(config.deviceName, config.apPassword);
    delay(500);
    Serial.printf("AP: %s  IP: %s\n", config.deviceName, WiFi.softAPIP().toString().c_str());
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.wifiSSID, config.wifiPassword);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); attempts++; }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("WiFi: %s\n", WiFi.localIP().toString().c_str());
    } else {
      config.useAPMode = true;
      WiFi.mode(WIFI_AP);
      IPAddress local_IP(192, 168, 5, 1);
      IPAddress gateway(192, 168, 5, 1);
      IPAddress subnet(255, 255, 255, 0);
      WiFi.softAPConfig(local_IP, gateway, subnet);
      WiFi.softAP(config.deviceName, config.apPassword);
    }
  }
  if (MDNS.begin(config.deviceName))
    Serial.printf("mDNS: http://%s.local\n", config.deviceName);
}

// ============================================================================
// SHARED HTML HELPERS
// ============================================================================
String htmlHeader(const String& title) {
  String h = "<!DOCTYPE html><html lang='en'><head>";
  h += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>" + title + " | " + String(config.deviceName) + "</title>";
  h += "<style>";
  h += "*{margin:0;padding:0;box-sizing:border-box;}";
  h += "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:#1a1a1a;color:#e0e0e0;line-height:1.6;}";
  h += "nav{background:#0a0a0a;padding:16px 0;box-shadow:0 2px 10px rgba(0,0,0,0.5);position:sticky;top:0;z-index:1000;}";
  h += ".nav-inner{max-width:960px;margin:0 auto;padding:0 20px;display:flex;align-items:center;justify-content:space-between;}";
  h += ".nav-title{color:#fff;font-size:1em;font-weight:400;letter-spacing:2px;text-transform:uppercase;}";
  h += ".nav-links{display:flex;gap:24px;list-style:none;}";
  h += ".nav-links a{color:#999;text-decoration:none;font-size:0.9em;font-weight:500;letter-spacing:1px;text-transform:uppercase;transition:color 0.2s;}";
  h += ".nav-links a:hover,.nav-links a.active{color:#fff;}";
  h += ".wrap{max-width:960px;margin:0 auto;padding:30px 20px;}";
  h += "h2{font-size:0.75em;font-weight:600;letter-spacing:2px;text-transform:uppercase;color:#666;margin:32px 0 14px;padding-bottom:8px;border-bottom:1px solid #2a2a2a;}";
  h += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin-bottom:8px;}";
  h += ".card{background:#111;border:1px solid #222;padding:16px;border-radius:4px;}";
  h += ".card h3{font-size:0.7em;font-weight:500;letter-spacing:1.5px;text-transform:uppercase;color:#555;margin-bottom:8px;}";
  h += ".card .val{font-size:1.8em;font-weight:300;color:#e0e0e0;}";
  h += ".card .unit{font-size:0.75em;color:#444;margin-top:2px;}";
  h += "label{display:block;margin:16px 0 6px;font-size:0.75em;font-weight:500;letter-spacing:1px;text-transform:uppercase;color:#666;}";
  h += "input[type=number],input[type=text],input[type=password]{width:100%;padding:10px 12px;background:#111;border:1px solid #2a2a2a;border-radius:4px;color:#e0e0e0;font-size:0.95em;box-sizing:border-box;}";
  h += "input:focus{outline:none;border-color:#555;}";
  h += "button{background:#e0e0e0;color:#111;border:none;padding:10px 24px;border-radius:4px;cursor:pointer;font-size:0.85em;font-weight:600;letter-spacing:1px;text-transform:uppercase;margin:6px 6px 0 0;}";
  h += "button:hover{background:#fff;}";
  h += "button.sec{background:transparent;color:#666;border:1px solid #333;}button.sec:hover{border-color:#666;color:#e0e0e0;}";
  h += "button.danger{background:transparent;color:#666;border:1px solid #333;}button.danger:hover{border-color:#888;color:#e0e0e0;}";
  h += ".info{background:#111;border:1px solid #222;border-left:2px solid #444;padding:12px 16px;border-radius:4px;margin:12px 0;font-size:0.85em;color:#777;}";
  h += ".row2{display:grid;grid-template-columns:1fr 1fr;gap:12px;}";
  h += ".row3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px;}";
  h += "@media(max-width:600px){.row2,.row3{grid-template-columns:1fr;}.nav-title{display:none;}}";
  h += "</style></head><body>";
  h += "<nav><div class='nav-inner'>";
  h += "<span class='nav-title'>" + String(config.deviceName) + "</span>";
  h += "<ul class='nav-links'>";
  h += "<li><a href='/'>Dashboard</a></li>";
  h += "<li><a href='/calibrate'>Calibration</a></li>";
  h += "<li><a href='/config'>Config</a></li>";
  h += "<li><a href='/update'>OTA</a></li>";
  h += "<li><a href='/log'>Log</a></li>";
  h += "<li><a href='/test'>" + String(state.testMode ? "&#9679; Test" : "Test") + "</a></li>";
  h += "</ul></div></nav><div class='wrap'>";
  return h;
}

String htmlFooter() { return "</div></body></html>"; }


// ============================================================================
// DASHBOARD
// ============================================================================
void handleRoot() {
  String html = htmlHeader("Dashboard");
  html += "<script>setInterval(function(){fetch('/api/status').then(r=>r.json()).then(d=>{";
  html += "document.getElementById('lean').innerText=d.leanAngle.toFixed(1);";
  html += "document.getElementById('leanDist').innerText=d.leanDist.toFixed(1);";
  html += "document.getElementById('leanIMU').innerText=d.leanIMU.toFixed(1);";
  html += "document.getElementById('projAngle').innerText=d.projAngle.toFixed(1);";
  html += "document.getElementById('projSteps').innerText=d.projSteps;";
  html += "document.getElementById('lDist').innerText=d.leftDist;";
  html += "document.getElementById('rDist').innerText=d.rightDist;";
  html += "document.getElementById('speed').innerText=d.speed.toFixed(1);";
  html += "document.getElementById('homed').innerText=d.homed?'Yes':'No';";
  html += "document.getElementById('motorState').innerText=d.motorActive?'ACTIVE':'idle';";
  html += "if(document.getElementById('potADC'))document.getElementById('potADC').innerText=d.potADC;";
  html += "document.getElementById('errs').innerText=d.errors;";
  html += "document.getElementById('uptime').innerText=Math.floor(d.uptime/1000)+'s';";
  html += "var pb=document.getElementById('projBar');";
  html += "var pct=((d.projAngle+45)/90*100).toFixed(0);pct=Math.max(0,Math.min(100,pct));";
  html += "pb.style.left=pct+'%';";
  html += "});},400);</script>";

  // Projector position visual
  html += "<h2>Projector Position</h2>";
  html += "<div style='background:#111;border:1px solid #222;border-radius:4px;padding:24px 20px;margin-bottom:8px;'>";
  html += "<div style='position:relative;height:8px;background:#222;border-radius:4px;margin:20px 10px;'>";
  html += "<div id='projBar' style='position:absolute;top:-6px;width:20px;height:20px;background:#e0e0e0;border-radius:50%;margin-left:-10px;left:50%;transition:left 0.2s;'></div>";
  html += "</div>";
  html += "<div style='display:flex;justify-content:space-between;font-size:11px;color:#555;margin-top:8px;'>";
  html += "<span>45° L</span><span>Center</span><span>45° R</span></div></div>";

  html += "<h2>Lean Angle</h2><div class='grid'>";
  html += "<div class='card'><h3>Fused Lean</h3><div class='val' id='lean'>" + String(state.leanAngle,1) + "</div><div class='unit'>degrees</div></div>";
  html += "<div class='card'><h3>Distance Lean</h3><div class='val' id='leanDist'>" + String(state.leanAngleDist,1) + "</div><div class='unit'>degrees</div></div>";
  html += "<div class='card'><h3>IMU Lean</h3><div class='val' id='leanIMU'>" + String(state.leanAngleIMU,1) + "</div><div class='unit'>degrees</div></div>";
  html += "</div>";

  html += "<h2>Projector</h2><div class='grid'>";
  html += "<div class='card'><h3>Projector Angle</h3><div class='val' id='projAngle'>" + String(state.projectorAngle,1) + "</div><div class='unit'>degrees</div></div>";
  html += "<div class='card'><h3>Step Position</h3><div class='val' id='projSteps'>" + String(state.currentSteps) + "</div><div class='unit'>steps from center</div></div>";
  html += "<div class='card'><h3>Motor</h3><div class='val' id='motorState'>" + String(state.stepperActive?"ACTIVE":"idle") + "</div><div class='unit'>dead zone ±" + String(config.deadZone,0) + "°</div></div>";
  html += "<div class='card'><h3>Pot ADC</h3><div class='val' id='potADC'>" + String(state.potADC) + "</div><div class='unit'>" + String(config.usePot?"closed loop":"open loop") + "</div></div>";
  html += "</div>";

  html += "<h2>Sensors</h2><div class='grid'>";
  html += "<div class='card'><h3>Left Distance</h3><div class='val' id='lDist'>" + String(state.leftDistance) + "</div><div class='unit'>mm</div></div>";
  html += "<div class='card'><h3>Right Distance</h3><div class='val' id='rDist'>" + String(state.rightDistance) + "</div><div class='unit'>mm</div></div>";
  html += "<div class='card'><h3>Speed</h3><div class='val' id='speed'>" + String(state.speedMs,1) + "</div><div class='unit'>m/s</div></div>";
  html += "<div class='card'><h3>Errors</h3><div class='val' id='errs'>" + String(state.errorCount) + "</div></div>";
  html += "<div class='card'><h3>Uptime</h3><div class='val' id='uptime'>" + String(millis()/1000) + "s</div></div>";
  html += "</div>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleAPIStatus() {
  String j = "{";
  j += "\"leanAngle\":"   + String(state.leanAngle, 2)      + ",";
  j += "\"leanDist\":"    + String(state.leanAngleDist, 2)  + ",";
  j += "\"leanIMU\":"     + String(state.leanAngleIMU, 2)   + ",";
  j += "\"projAngle\":"   + String(state.projectorAngle, 2) + ",";
  j += "\"projSteps\":"   + String(state.currentSteps)      + ",";
  j += "\"leftDist\":"    + String(state.leftDistance)       + ",";
  j += "\"rightDist\":"   + String(state.rightDistance)      + ",";
  j += "\"leftValid\":"   + String(state.leftValid?"true":"false")  + ",";
  j += "\"rightValid\":"  + String(state.rightValid?"true":"false") + ",";
  j += "\"speed\":"       + String(state.speedMs, 2)        + ",";
  j += "\"homed\":"       + String(state.homingComplete?"true":"false") + ",";
  j += "\"motorActive\":" + String(state.stepperActive?"true":"false")  + ",";
  j += "\"potADC\":"      + String(state.potADC)                        + ",";
  j += "\"errors\":"      + String(state.errorCount)        + ",";
  j += "\"uptime\":"      + String(millis())                + "}";
  server.send(200, "application/json", j);
}


// ============================================================================
// CALIBRATION PAGE
// ============================================================================
void handleCalibrate() {
  String html = htmlHeader("Calibration");
  html += "<form action='/api/calibrate' method='POST'>";

  html += "<h2>Lean Mapping</h2>";
  html += "<div class='info'>Dead zone: lean below this angle keeps projector centered and motor disabled. Hysteresis prevents hunting once projector reaches target.</div>";
  html += "<div class='row3'>";
  html += "<div><label>Dead Zone (°)</label><input type='number' name='deadZone' step='0.5' value='" + String(config.deadZone,1) + "'></div>";
  html += "<div><label>Max Lean Angle (°)</label><input type='number' name='maxLean' step='1' value='" + String(config.maxLeanAngle,0) + "'></div>";
  html += "<div><label>Position Hysteresis (°)</label><input type='number' name='hysteresis' step='0.5' value='" + String(config.hysteresis,1) + "'></div>";
  html += "</div>";

  html += "<h2>Motor Current</h2>";
  html += "<div class='info'>RMS current sent to TMC2209 via UART on startup. Applied at next boot. Range 50-400mA. Motor rated 200mA.</div>";
  html += "<div class='row2'>";
  html += "<div><label>Motor Current (mA)</label><input type='number' name='motorCurrentMA' step='10' min='50' max='400' value='" + String(config.motorCurrentMA) + "'></div>";
  html += "<div><label>Pot Deadband (ADC counts)</label><input type='number' name='potDeadband' step='1' min='5' max='200' value='" + String(config.potDeadband) + "'></div>";
  html += "</div>";

  html += "<h2>Position Pot</h2>";
  html += "<div class='info'>Pot calibration is set automatically during homing. Use Center Offset to fine-tune the zero position after homing. Positive = shift right, negative = shift left.</div>";
  html += "<div class='row3'>";
  html += "<div><label>Pot Enabled</label><input type='text' name='' value='" + String(config.usePot?"YES - Closed loop":"NO - Open loop") + "' readonly style='background:#111'></div>";
  html += "<div><label>Center Offset (ADC counts)</label><input type='number' name='potCenterOffset' step='1' min='-200' max='200' value='" + String(config.potCenterOffset) + "'></div>";
  html += "<div><label>Current Pot ADC</label><input type='text' value='" + String(state.potADC) + "' readonly style='background:#111'></div>";
  html += "</div>";
  html += "<div class='row3'>";
  html += "<div><label>Left Limit ADC</label><input type='text' value='" + String(config.potLeftADC) + "' readonly style='background:#111'></div>";
  html += "<div><label>Center ADC</label><input type='text' value='" + String(config.potCenterADC) + "' readonly style='background:#111'></div>";
  html += "<div><label>Right Limit ADC</label><input type='text' value='" + String(config.potRightADC) + "' readonly style='background:#111'></div>";
  html += "</div>";

  html += "<h2>Sensor Geometry — Left</h2>";
  html += "<div class='info'>Physical position of the left distance sensor on the bike.</div>";
  html += "<div class='row3'>";
  html += "<div><label>Height (mm)</label><input type='number' name='lsH' step='1' value='" + String(config.leftSensor.heightMm,0) + "'></div>";
  html += "<div><label>Width from CL (mm)</label><input type='number' name='lsW' step='1' value='" + String(config.leftSensor.widthMm,0) + "'></div>";
  html += "<div><label>Mount Angle (°)</label><input type='number' name='lsA' step='0.5' value='" + String(config.leftSensor.angleDeg,1) + "'></div>";
  html += "</div>";

  html += "<h2>Sensor Geometry — Right</h2>";
  html += "<div class='row3'>";
  html += "<div><label>Height (mm)</label><input type='number' name='rsH' step='1' value='" + String(config.rightSensor.heightMm,0) + "'></div>";
  html += "<div><label>Width from CL (mm)</label><input type='number' name='rsW' step='1' value='" + String(config.rightSensor.widthMm,0) + "'></div>";
  html += "<div><label>Mount Angle (°)</label><input type='number' name='rsA' step='0.5' value='" + String(config.rightSensor.angleDeg,1) + "'></div>";
  html += "</div>";

  html += "<h2>Distance Sensor Settings</h2>";
  html += "<div class='row2'>";
  html += "<div><label>Effective Spacing (mm)</label><input type='number' name='spacing' step='1' value='" + String(config.sensorSpacing,0) + "'></div>";
  html += "<div><label>Sample Interval (ms)</label><input type='number' name='sampleInt' step='10' value='" + String(config.sampleInterval) + "'></div>";
  html += "<div><label>Left Offset (mm)</label><input type='number' name='leftOff' step='0.1' value='" + String(config.leftSensorOffset,1) + "'></div>";
  html += "<div><label>Right Offset (mm)</label><input type='number' name='rightOff' step='0.1' value='" + String(config.rightSensorOffset,1) + "'></div>";
  html += "<div><label>Min Distance (mm)</label><input type='number' name='minDist' step='1' value='" + String(config.minDistance) + "'></div>";
  html += "<div><label>Max Distance (mm)</label><input type='number' name='maxDist' step='1' value='" + String(config.maxDistance) + "'></div>";
  html += "</div>";

  html += "<h2>IMU Settings</h2>";
  html += "<div class='row2'>";
  html += "<div><label>Gyro Axis (0=X 1=Y 2=Z)</label><input type='number' name='imuAxis' min='0' max='2' value='" + String(config.imuYawAxis) + "'></div>";
  html += "<div><label>Invert Gyro</label><input type='number' name='imuInv' min='0' max='1' value='" + String(config.imuYawInvert?1:0) + "'></div>";
  html += "</div>";

  html += "<h2>Speed Sensor</h2>";
  html += "<div class='row2'>";
  html += "<div><label>Pulses per Revolution</label><input type='number' name='ppr' step='1' value='" + String(config.pulsesPerRev) + "'></div>";
  html += "<div><label>Wheel Circumference (m)</label><input type='number' name='wheelC' step='0.01' value='" + String(config.wheelCircumference,2) + "'></div>";
  html += "</div>";

  html += "<button type='submit'>Save Calibration</button>";
  html += "</form>";
  html += "<br><button onclick=\"if(confirm('Run homing sequence? Motor will move to both limit switches.')){fetch('/api/setzero').then(r=>r.json()).then(d=>alert(d.msg))}\">&#8962; Run Homing &amp; Set Zero</button>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleAPICalibrate() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  if (server.hasArg("deadZone"))       config.deadZone            = server.arg("deadZone").toFloat();
  if (server.hasArg("maxLean"))        config.maxLeanAngle        = server.arg("maxLean").toFloat();
  if (server.hasArg("hysteresis"))     config.hysteresis          = server.arg("hysteresis").toFloat();
  if (server.hasArg("motorCurrentMA")) config.motorCurrentMA      = server.arg("motorCurrentMA").toInt();
  if (server.hasArg("potDeadband"))    config.potDeadband         = server.arg("potDeadband").toInt();
  if (server.hasArg("potCenterOffset")) {
    config.potCenterOffset = server.arg("potCenterOffset").toInt();
    // Recalculate center ADC with new offset
    config.potCenterADC = (config.potLeftADC + config.potRightADC) / 2 + config.potCenterOffset;
  }
  if (server.hasArg("hysteresis"))config.hysteresis          = server.arg("hysteresis").toFloat();
  if (server.hasArg("lsH"))       config.leftSensor.heightMm = server.arg("lsH").toFloat();
  if (server.hasArg("lsW"))       config.leftSensor.widthMm  = server.arg("lsW").toFloat();
  if (server.hasArg("lsA"))       config.leftSensor.angleDeg = server.arg("lsA").toFloat();
  if (server.hasArg("rsH"))       config.rightSensor.heightMm= server.arg("rsH").toFloat();
  if (server.hasArg("rsW"))       config.rightSensor.widthMm = server.arg("rsW").toFloat();
  if (server.hasArg("rsA"))       config.rightSensor.angleDeg= server.arg("rsA").toFloat();
  if (server.hasArg("spacing"))   config.sensorSpacing       = server.arg("spacing").toFloat();
  if (server.hasArg("sampleInt")) config.sampleInterval      = server.arg("sampleInt").toInt();
  if (server.hasArg("leftOff"))   config.leftSensorOffset    = server.arg("leftOff").toFloat();
  if (server.hasArg("rightOff"))  config.rightSensorOffset   = server.arg("rightOff").toFloat();
  if (server.hasArg("minDist"))   config.minDistance         = server.arg("minDist").toInt();
  if (server.hasArg("maxDist"))   config.maxDistance         = server.arg("maxDist").toInt();
  if (server.hasArg("imuAxis"))   config.imuYawAxis          = server.arg("imuAxis").toInt();
  if (server.hasArg("imuInv"))    config.imuYawInvert        = server.arg("imuInv").toInt() != 0;
  if (server.hasArg("ppr"))       config.pulsesPerRev        = server.arg("ppr").toInt();
  if (server.hasArg("wheelC"))    config.wheelCircumference  = server.arg("wheelC").toFloat();
  saveConfig();
  server.sendHeader("Location", "/calibrate");
  server.send(303);
}


// ============================================================================
// CONFIG PAGE
// ============================================================================
void handleConfig() {
  String html = htmlHeader("Configuration");
  html += "<form action='/api/config' method='POST'>";
  html += "<h2>Device</h2>";
  html += "<label>Device Name (AP SSID / mDNS)</label>";
  html += "<input type='text' name='deviceName' value='" + String(config.deviceName) + "' maxlength='31'>";
  html += "<h2>WiFi</h2>";
  html += "<label>WiFi SSID (leave empty for AP mode)</label>";
  html += "<input type='text' name='wifiSSID' value='" + String(config.wifiSSID) + "'>";
  html += "<label>WiFi Password</label>";
  html += "<input type='password' name='wifiPassword' value='" + String(config.wifiPassword) + "'>";
  html += "<label>AP Mode Password</label>";
  html += "<input type='password' name='apPassword' value='" + String(config.apPassword) + "'>";
  html += "<h2>Sensor Sources</h2>";
  html += "<div class='row2'>";
  html += "<div><label>Use IMU (1=yes 0=no)</label><input type='number' name='useIMU' min='0' max='1' value='" + String(config.useIMU?1:0) + "'></div>";
  html += "<div><label>Use Distance Sensors</label><input type='number' name='useDist' min='0' max='1' value='" + String(config.useDistanceSensors?1:0) + "'></div>";
  html += "</div>";
  html += "<br><button type='submit'>Save &amp; Restart</button>";
  html += "<button type='button' class='danger' onclick=\"if(confirm('Reset ALL settings?'))window.location='/api/reset';\">Factory Reset</button>";
  html += "</form>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleAPIConfig() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  if (server.hasArg("deviceName"))   strncpy(config.deviceName,   server.arg("deviceName").c_str(),   sizeof(config.deviceName)-1);
  if (server.hasArg("wifiSSID"))     strncpy(config.wifiSSID,     server.arg("wifiSSID").c_str(),     sizeof(config.wifiSSID)-1);
  if (server.hasArg("wifiPassword")) strncpy(config.wifiPassword, server.arg("wifiPassword").c_str(), sizeof(config.wifiPassword)-1);
  if (server.hasArg("apPassword"))   strncpy(config.apPassword,   server.arg("apPassword").c_str(),   sizeof(config.apPassword)-1);
  if (server.hasArg("useIMU"))       config.useIMU              = server.arg("useIMU").toInt() != 0;
  if (server.hasArg("useDist"))      config.useDistanceSensors  = server.arg("useDist").toInt() != 0;
  config.useAPMode = (strlen(config.wifiSSID) == 0);
  saveConfig();
  server.send(200, "text/html",
    "<html><body style='background:#1a1a1a;color:#e0e0e0;font-family:-apple-system,sans-serif;"
    "display:flex;align-items:center;justify-content:center;height:100vh;margin:0;'>"
    "<p style='font-size:0.8em;letter-spacing:2px;text-transform:uppercase;color:#666;'>Saved &mdash; Restarting</p>"
    "<script>setTimeout(()=>window.location='/',6000)</script></body></html>");
  delay(1000);
  ESP.restart();
}

void handleAPIReset() { resetConfig(); }

// ============================================================================
// TEST MODE PAGE
// ============================================================================
void handleTest() {
  String html = htmlHeader("Test Mode");

  if (!state.homingComplete) {
    html += "<div class='info'>Homing not complete. Stepper is not yet ready for test control.</div>";
    html += "<br><a href='/api/rehome' style='display:inline-block;background:#e0e0e0;color:#111;padding:10px 24px;border-radius:4px;font-size:0.85em;font-weight:600;letter-spacing:1px;text-transform:uppercase;text-decoration:none;'>Run Homing Sequence</a>";
    html += htmlFooter();
    server.send(200, "text/html", html);
    return;
  }

  if (state.testMode) {
    html += "<div style='background:#111;border:1px solid #444;border-left:2px solid #e0e0e0;padding:12px 16px;border-radius:4px;margin-bottom:24px;font-size:0.85em;color:#aaa;'>";
    html += "Test mode active &mdash; lean control suspended. Projector: <strong style='color:#e0e0e0;'>" + String(state.projectorAngle,1) + "°</strong></div>";
  } else {
    html += "<div class='info'>Set projector to a fixed angle for testing. Lean-based control suspended while active.</div>";
  }

  html += "<h2>Projector Angle</h2>";
  html += "<form action='/api/test' method='POST' style='display:flex;gap:8px;align-items:flex-end;flex-wrap:wrap;'>";
  html += "<div><label style='margin-top:0;'>Angle (° — negative=left, positive=right)</label>";
  html += "<input type='number' name='angle' min='-45' max='45' step='1' value='" + String(state.testMode?(int)state.testAngle:0) + "' style='width:120px;'></div>";
  html += "<button type='submit' name='action' value='set'>Set Angle</button>";
  if (state.testMode) {
    html += "<button type='submit' name='action' value='release' class='sec'>Return to Lean Control</button>";
  }
  html += "</form>";

  html += "<h2>Quick Positions</h2>";
  html += "<div style='display:flex;gap:8px;flex-wrap:wrap;margin-top:8px;'>";
  const int presets[] = {-40, -20, 0, 20, 40};
  for (int p : presets) {
    html += "<a href='/api/test?action=set&angle=" + String(p) + "' style='background:#111;color:#e0e0e0;border:1px solid #333;padding:8px 16px;border-radius:4px;font-size:0.85em;text-decoration:none;'>" + String(p) + "°</a>";
  }
  html += "</div>";

  html += "<h2>Homing</h2>";
  html += "<a href='/api/rehome' style='display:inline-block;background:transparent;color:#666;border:1px solid #333;padding:10px 24px;border-radius:4px;font-size:0.85em;font-weight:600;letter-spacing:1px;text-transform:uppercase;text-decoration:none;'>Re-run Homing Sequence</a>";

  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleAPITest() {
  String action = server.hasArg("action") ? server.arg("action") : "";
  if (action == "release") {
    state.testMode  = false;
    state.testAngle = 0.0f;
    Serial.println("Test mode: released");
  } else if (action == "set") {
    float angle = server.hasArg("angle") ? constrain(server.arg("angle").toFloat(), -45.0f, 45.0f) : 0.0f;
    state.testAngle = angle;
    state.testMode  = true;
    Serial.printf("Test mode: angle set to %.1f\n", angle);
  }
  server.sendHeader("Location", "/test");
  server.send(303);
}

// Set zero / rehome via limit switches
void handleAPISetZero() {
  // Trigger a new homing sequence and save pot calibration
  state.homingComplete = false;
  if (homeProjector()) {
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Homing complete, zero set\"}");
  } else {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"Homing failed\"}");
  }
}

void handleAPIRehome() {
  server.send(200, "text/html",
    "<html><body style='background:#1a1a1a;color:#e0e0e0;font-family:-apple-system,sans-serif;"
    "display:flex;align-items:center;justify-content:center;height:100vh;margin:0;'>"
    "<p style='font-size:0.8em;letter-spacing:2px;text-transform:uppercase;color:#666;'>Homing in progress...</p>"
    "<script>setTimeout(()=>window.location='/',15000)</script></body></html>");
  delay(200);
  state.homingComplete = false;
  state.testMode = false;
  homeProjector();
}


// ============================================================================
// OTA UPDATE PAGE
// ============================================================================
void handleUpdate() {
  String html = htmlHeader("OTA Update");
  html += "<h2>Current Version</h2>";
  html += "<div class='info' style='font-family:monospace;'>" + String(FIRMWARE_VERSION) + "</div>";
  html += "<h2>Web Upload</h2>";
  html += "<p style='color:#666;font-size:0.9em;margin-bottom:12px;'>Sketch → Export Compiled Binary in Arduino IDE, then upload .bin here.</p>";
  html += "<form id='upForm' enctype='multipart/form-data'>";
  html += "<input type='file' id='file' name='update' accept='.bin' style='width:100%;padding:10px;background:#111;color:#e0e0e0;border:1px solid #2a2a2a;border-radius:4px;margin-bottom:10px;'>";
  html += "<button type='submit'>Upload Firmware</button></form>";
  html += "<div id='prog' style='display:none;margin:16px 0;'><div style='background:#222;border-radius:4px;overflow:hidden;border:1px solid #2a2a2a;'>";
  html += "<div id='bar' style='height:28px;background:#e0e0e0;width:0%;transition:width 0.3s;text-align:center;line-height:28px;color:#111;font-size:0.8em;font-weight:600;letter-spacing:1px;'>0%</div></div></div>";
  html += "<div id='msg' style='display:none;padding:12px;border-radius:5px;margin:15px 0;'></div>";
  html += "<script>document.getElementById('upForm').addEventListener('submit',function(e){";
  html += "e.preventDefault();var f=document.getElementById('file').files[0];if(!f){alert('Select a file');return;}";
  html += "var fd=new FormData();fd.append('update',f);var x=new XMLHttpRequest();";
  html += "x.upload.onprogress=function(e){if(e.lengthComputable){var p=Math.round(e.loaded/e.total*100);";
  html += "document.getElementById('prog').style.display='block';document.getElementById('bar').style.width=p+'%';document.getElementById('bar').innerText=p+'%';}};";
  html += "x.onload=function(){var m=document.getElementById('msg');m.style.display='block';";
  html += "if(x.status===200){m.style.cssText='background:#111;border:1px solid #2a2a2a;border-left:2px solid #e0e0e0;color:#e0e0e0;padding:12px 16px;border-radius:4px;';m.innerHTML='Update complete &mdash; restarting';setTimeout(()=>window.location='/',12000);}";
  html += "else{m.style.cssText='background:#111;border:1px solid #2a2a2a;border-left:2px solid #888;color:#999;padding:12px 16px;border-radius:4px;';m.innerHTML='Update failed: '+x.responseText;}};";
  html += "x.open('POST','/updateUpload',true);x.send(fd);});</script>";
  html += htmlFooter();
  server.send(200, "text/html", html);
}

void handleUpdateUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("[OTA] Start: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) Serial.printf("[OTA] Done: %u bytes\n", upload.totalSize);
    else Update.printError(Serial);
  }
}

void handleUpdateComplete() {
  if (Update.hasError()) {
    server.send(500, "text/plain", Update.errorString());
  } else {
    server.send(200, "text/plain", "OK");
    delay(1000);
    ESP.restart();
  }
}

// ============================================================================
// DATALOG PAGE
// Streams CSV rows to the browser. The page accumulates rows in memory and
// lets the user download the result as a .csv file.
// Columns: timestamp_ms, lean_fused, lean_dist, lean_imu, left_dist,
//          right_dist, speed_ms, projector_angle, left_valid, right_valid
// ============================================================================
void handleLog() {
  String html = htmlHeader("Datalog");

  html += "<div style='display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin-bottom:16px;'>";
  html += "<button id='btnStart' onclick='startLog()'>Start Logging</button>";
  html += "<button id='btnStop' onclick='stopLog()' class='sec' disabled>Stop</button>";
  html += "<button onclick='clearLog()' class='sec'>Clear</button>";
  html += "<button onclick='downloadCSV()'>Download CSV</button>";
  html += "<span id='rowCount' style='color:#555;font-size:0.85em;'>0 rows</span>";
  html += "<span id='status' style='color:#555;font-size:0.85em;'></span>";
  html += "</div>";

  html += "<div style='background:#111;border:1px solid #222;border-radius:4px;padding:12px;margin-bottom:12px;'>";
  html += "<div style='font-size:0.7em;letter-spacing:1px;text-transform:uppercase;color:#555;margin-bottom:8px;'>Live — last 10 rows</div>";
  html += "<div style='overflow-x:auto;'>";
  html += "<table id='preview' style='width:100%;border-collapse:collapse;font-size:0.8em;font-family:monospace;'>";
  html += "<thead><tr style='color:#555;text-align:right;'>";
  html += "<th style='padding:4px 8px;text-align:left;'>time(ms)</th>";
  html += "<th style='padding:4px 8px;'>lean_fused</th>";
  html += "<th style='padding:4px 8px;'>lean_dist</th>";
  html += "<th style='padding:4px 8px;'>lean_imu</th>";
  html += "<th style='padding:4px 8px;'>l_dist</th>";
  html += "<th style='padding:4px 8px;'>r_dist</th>";
  html += "<th style='padding:4px 8px;'>speed</th>";
  html += "<th style='padding:4px 8px;'>proj_ang</th>";
  html += "</tr></thead>";
  html += "<tbody id='previewBody'></tbody></table></div></div>";

  html += "<script>";
  html += "var rows=[];var logging=false;var timer=null;";
  html += "function startLog(){";
  html += "logging=true;";
  html += "document.getElementById('btnStart').disabled=true;";
  html += "document.getElementById('btnStop').disabled=false;";
  html += "document.getElementById('status').innerText='Logging...';";
  html += "timer=setInterval(fetchRow,200);";  // 5 Hz
  html += "}";
  html += "function stopLog(){";
  html += "logging=false;clearInterval(timer);";
  html += "document.getElementById('btnStart').disabled=false;";
  html += "document.getElementById('btnStop').disabled=true;";
  html += "document.getElementById('status').innerText='Stopped ('+rows.length+' rows)';";
  html += "}";
  html += "function clearLog(){";
  html += "rows=[];";
  html += "document.getElementById('previewBody').innerHTML='';";
  html += "document.getElementById('rowCount').innerText='0 rows';";
  html += "}";
  html += "function fetchRow(){";
  html += "fetch('/api/logrow').then(r=>r.json()).then(d=>{";
  html += "var row=[d.t,d.lf.toFixed(2),d.ld.toFixed(2),d.li.toFixed(2),";
  html += "d.ldist,d.rdist,d.spd.toFixed(2),d.pa.toFixed(2),d.lv?1:0,d.rv?1:0];";
  html += "rows.push(row);";
  html += "document.getElementById('rowCount').innerText=rows.length+' rows';";
  html += "updatePreview(row);";
  html += "}).catch(()=>{});";
  html += "}";
  html += "function updatePreview(row){";
  html += "var tb=document.getElementById('previewBody');";
  html += "var tr=document.createElement('tr');";
  html += "var cols=[0,1,2,3,4,5,6,7];";  // first 8 cols in preview
  html += "cols.forEach(function(i){";
  html += "var td=document.createElement('td');";
  html += "td.style.cssText='padding:3px 8px;text-align:'+(i===0?'left':'right')+';color:#aaa;border-top:1px solid #1a1a1a;';";
  html += "td.innerText=row[i];tr.appendChild(td);});";
  html += "tb.appendChild(tr);";
  html += "while(tb.rows.length>10)tb.deleteRow(0);";
  html += "}";
  html += "function downloadCSV(){";
  html += "if(rows.length===0){alert('No data logged yet');return;}";
  html += "var hdr='timestamp_ms,lean_fused,lean_dist,lean_imu,left_dist,right_dist,speed_ms,projector_angle,left_valid,right_valid\\n';";
  html += "var csv=hdr+rows.map(r=>r.join(',')).join('\\n');";
  html += "var a=document.createElement('a');";
  html += "a.href='data:text/csv;charset=utf-8,'+encodeURIComponent(csv);";
  html += "a.download='lean_log_'+Date.now()+'.csv';";
  html += "document.body.appendChild(a);a.click();document.body.removeChild(a);";
  html += "}";
  html += "</script>";

  html += htmlFooter();
  server.send(200, "text/html", html);
}

// Single JSON row for the logger — fetched at 5 Hz by the log page
void handleAPILogRow() {
  String j = "{";
  j += "\"t\":"     + String(millis())               + ",";
  j += "\"lf\":"    + String(state.leanAngle, 2)     + ",";
  j += "\"ld\":"    + String(state.leanAngleDist, 2) + ",";
  j += "\"li\":"    + String(state.leanAngleIMU, 2)  + ",";
  j += "\"ldist\":" + String(state.leftDistance)      + ",";
  j += "\"rdist\":" + String(state.rightDistance)     + ",";
  j += "\"spd\":"   + String(state.speedMs, 2)       + ",";
  j += "\"pa\":"    + String(state.projectorAngle, 2) + ",";
  j += "\"lv\":"    + String(state.leftValid?"true":"false")  + ",";
  j += "\"rv\":"    + String(state.rightValid?"true":"false") + "}";
  server.send(200, "application/json", j);
}

// ============================================================================
// WEB SERVER SETUP
// ============================================================================
void setupWebServer() {
  server.on("/",              handleRoot);
  server.on("/calibrate",     handleCalibrate);
  server.on("/config",        handleConfig);
  server.on("/update",  HTTP_GET, handleUpdate);
  server.on("/updateUpload", HTTP_POST, handleUpdateComplete, handleUpdateUpload);
  server.on("/test",          handleTest);
  server.on("/log",           handleLog);
  server.on("/api/status",    handleAPIStatus);
  server.on("/api/calibrate", handleAPICalibrate);
  server.on("/api/config",    handleAPIConfig);
  server.on("/api/reset",     handleAPIReset);
  server.on("/api/test",      handleAPITest);
  server.on("/api/rehome",    handleAPIRehome);
  server.on("/api/setzero",   handleAPISetZero);
  server.on("/api/logrow",    handleAPILogRow);
  server.begin();
  Serial.println("Web server started");
}

// ============================================================================
// OTA (Arduino IDE)
// ============================================================================
void setupOTA() {
  ArduinoOTA.setHostname(config.deviceName);
  ArduinoOTA.onStart([]() { stepperDisable(); Serial.println("[OTA] Start"); });
  ArduinoOTA.onEnd([]()   { Serial.println("[OTA] Done"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("[OTA] %u%%\r", p / (t / 100));
  });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("[OTA] Error %u\n", e); });
  ArduinoOTA.begin();
}


// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  // Wait for USB CDC on Xiao ESP32-C6
  unsigned long t = millis();
  while (!Serial && millis() - t < 3000) delay(10);

  Serial.println("\n========================================");
  Serial.println("Motorcycle Cornering Lights V3");
  Serial.println("========================================");

  // Stepper pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(EN_PIN,   OUTPUT);
  stepperDisable();
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN,  LOW);

  // Limit switch pins (active LOW, internal pull-up)
  pinMode(LIMIT_LEFT_PIN,  INPUT_PULLUP);
  pinMode(LIMIT_RIGHT_PIN, INPUT_PULLUP);

  // Position pot ADC pin
  pinMode(POT_PIN, INPUT);
  analogReadResolution(12);  // 0-4095

  // Set default device name with MAC suffix
  {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(config.deviceName, sizeof(config.deviceName),
             "CL-V3-%02X%02X%02X", mac[3], mac[4], mac[5]);
  }

  loadConfig();

  // TMC2209 UART current control
  initTMCUart();

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // Distance sensors
  if (config.useDistanceSensors) {
    state.systemInitialized = initializeSensors();
  }

  // IMU
  if (config.useIMU) {
    if (initIMU()) {
      calibrateGyroBias(200);
    }
  }

  // Speed sensor
  // Uncomment and set SPEED_PIN when speed sensor is connected:
  // pinMode(SPEED_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(SPEED_PIN), speedPulseISR, RISING);

  // WiFi and services
  setupWiFi();
  setupWebServer();
  setupOTA();

  Serial.printf("Access: http://%s.local  or  http://192.168.5.1\n", config.deviceName);

  // If pot is calibrated from a previous homing, skip homing and use pot for position
  if (config.usePot && config.potLeftADC != config.potRightADC) {
    Serial.println("Pot calibration found — skipping homing, using pot for position.");
    Serial.printf("  Pot range: %d (left) to %d (right), center %d\n",
                  config.potLeftADC, config.potRightADC, config.potCenterADC);
    // Read current pot position to determine where we are
    updatePotReading();
    state.currentSteps  = (long)(state.potAngle * STEPS_PER_DEGREE);
    state.homingMaxSteps = (long)(config.maxLeanAngle * 2.0f * STEPS_PER_DEGREE);
    state.homingComplete = true;
    state.projectorAngle = state.potAngle;
    Serial.printf("  Current position: %.1f° (ADC %d)\n", state.potAngle, state.potADC);
  } else {
    Serial.println("Starting homing sequence...");
    if (!homeProjector()) {
      Serial.println("WARNING: Homing failed - stepper control disabled");
      Serial.println("Check limit switches and wiring, then use /api/rehome to retry");
    }
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  ArduinoOTA.handle();
  server.handleClient();

  unsigned long now = millis();

  // Speed update every 100ms
  updateSpeed();

  // IMU lean update every loop
  if (config.useIMU && state.imuInitialized && state.imuCalibrated) {
    updateIMULean();
  }

  // Distance sensor read
  if (!state.testMode && config.useDistanceSensors &&
      (now - lastSampleTime >= config.sampleInterval)) {
    lastSampleTime = now;
    readSensors();
  }

  // Fuse lean angle sources
  if (!state.testMode) {
    updateLeanAngle();
  }

  // Compute target stepper position and step toward it
  if (state.homingComplete) {
    updatePotReading();  // always read pot — used for closed loop and dashboard
    updateProjectorPosition();
    stepTowardTarget();
  }

  state.uptime = now;

  // Periodic diagnostics
  static unsigned long lastHeap = 0;
  if (now - lastHeap >= 5000) {
    lastHeap = now;
    Serial.printf("[MEM] Free heap: %u  Lean: %.1f°  Projector: %.1f° (step %ld)\n",
                  ESP.getFreeHeap(), state.leanAngle,
                  state.projectorAngle, state.currentSteps);
  }
}
