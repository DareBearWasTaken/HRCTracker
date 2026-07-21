#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <string.h>

// ============================================================
// HANDHELD CADENCE TRACKER -- low-latency ESP32-C5 edition
// ============================================================
//
// Default wiring (matches the original tracker):
//   OLED SDA -> GPIO23      OLED SCL -> GPIO24
//   MPU SDA  -> GPIO25      MPU SCL  -> GPIO26
//   LED +    -> GPIO2 through a 220-330 ohm resistor
//   LED -    -> GND
//
// ESP32-C5 has only one high-performance I2C controller that can use
// these arbitrary pins. This version safely locks that controller,
// changes pins for the OLED frame, restores the MPU pins, and then
// releases the sensor task. The OLED updates at 2 Hz, keeping the
// unavoidable sampling interruption short and infrequent.
// ============================================================

#ifndef USE_ORIGINAL_WIRING
#define USE_ORIGINAL_WIRING 1
#endif

#ifndef ENABLE_STEP_DEBUG
#define ENABLE_STEP_DEBUG 0
#endif

// ------------------------------------------------------------
// Hardware
// ------------------------------------------------------------

constexpr int MPU_SDA_PIN = 25;
constexpr int MPU_SCL_PIN = 26;

#if USE_ORIGINAL_WIRING
constexpr int OLED_SDA_PIN = 23;
constexpr int OLED_SCL_PIN = 24;
constexpr int LED_PIN = 2;
#define OLED_WIRE Wire
#else
// Optional maximum-performance wiring using the C5's fixed LP-I2C pins.
constexpr int OLED_SDA_PIN = 2;
constexpr int OLED_SCL_PIN = 3;
constexpr int LED_PIN = 4;
#define OLED_WIRE Wire1
#endif

constexpr bool LED_ACTIVE_HIGH = true;
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint16_t I2C_TIMEOUT_MS = 8;

// ------------------------------------------------------------
// OLED
// ------------------------------------------------------------

constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int OLED_RESET_PIN = -1;

// Keeping clkDuring and clkAfter at 400 kHz avoids an extra clock change
// during each safely time-sliced OLED update.
Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &OLED_WIRE,
  OLED_RESET_PIN,
  I2C_CLOCK_HZ,
  I2C_CLOCK_HZ
);

bool displayAvailable = false;
uint8_t oledAddress = 0;

// ------------------------------------------------------------
// MPU9250 registers and scale factors
// ------------------------------------------------------------

constexpr uint8_t MPU_ADDRESS_LOW = 0x68;
constexpr uint8_t MPU_ADDRESS_HIGH = 0x69;

constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG_2 = 0x1D;
constexpr uint8_t REG_INT_PIN_CFG = 0x37;
constexpr uint8_t REG_INT_ENABLE = 0x38;
constexpr uint8_t REG_INT_STATUS = 0x3A;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr float ACCEL_COUNTS_PER_G = 8192.0f;   // +/-4 g
constexpr float GYRO_COUNTS_PER_DPS = 65.5f;    // +/-500 dps

uint8_t mpuAddress = 0;

// ------------------------------------------------------------
// Application and UI state
// ------------------------------------------------------------

enum AppState : uint8_t {
  STATE_CALIBRATING,
  STATE_TRACKING,
  STATE_ERROR
};

struct UiSnapshot {
  AppState state;
  float calibrationProgress;
  bool calibrationMoving;
  bool rhythmLocked;
  float averageCadence;
  float currentCadence;
  float motionScore;
  uint32_t stepCount;
  uint32_t sensorFailures;
  uint32_t missedDeadlines;
  uint32_t sessionStartedAt;
  char errorMessage[24];
};

UiSnapshot uiSnapshot = {
  STATE_CALIBRATING,
  0.0f,
  false,
  false,
  0.0f,
  0.0f,
  0.0f,
  0,
  0,
  0,
  0,
  "NONE"
};

portMUX_TYPE uiMux = portMUX_INITIALIZER_UNLOCKED;

AppState appState = STATE_CALIBRATING;
char errorMessage[24] = "NONE";

// ------------------------------------------------------------
// Timing
// ------------------------------------------------------------

constexpr uint32_t SENSOR_PERIOD_MS = 8;      // MPU configured for 125 Hz
constexpr uint32_t DISPLAY_PERIOD_MS = 500;   // 2 FPS; protects sensor timing
constexpr uint32_t CALIBRATION_SAMPLE_COUNT = 375; // 3 stable seconds

uint32_t lastDisplayAt = 0;
uint32_t sessionStartedAt = 0;

#if USE_ORIGINAL_WIRING
enum PrimaryBusTarget : uint8_t {
  PRIMARY_BUS_NONE,
  PRIMARY_BUS_MPU,
  PRIMARY_BUS_OLED
};

PrimaryBusTarget primaryBusTarget = PRIMARY_BUS_NONE;
SemaphoreHandle_t i2cAccessMutex = nullptr;
#endif

// ------------------------------------------------------------
// LED -- immediate, non-blocking pulse
// ------------------------------------------------------------

constexpr uint32_t LED_PULSE_MS = 45;
volatile bool ledIsOn = false;
volatile uint32_t ledOffAt = 0;
portMUX_TYPE ledMux = portMUX_INITIALIZER_UNLOCKED;

// ------------------------------------------------------------
// Motion sample
// ------------------------------------------------------------

float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 1.0f;
float accelMagnitude = 1.0f;

float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;
float gyroMagnitude = 0.0f;

uint32_t sensorFailures = 0;
uint32_t consecutiveSensorFailures = 0;
uint32_t missedDeadlines = 0;

enum SampleResult : uint8_t {
  SAMPLE_READY,
  SAMPLE_NOT_READY,
  SAMPLE_BUS_ERROR
};

// ------------------------------------------------------------
// Calibration (Welford running statistics)
// ------------------------------------------------------------

uint32_t calibrationSamples = 0;
uint16_t unstableCalibrationSamples = 0;
bool calibrationMoving = false;

float accelCalibrationMean = 0.0f;
float accelCalibrationM2 = 0.0f;

float gyroMeanX = 0.0f;
float gyroMeanY = 0.0f;
float gyroMeanZ = 0.0f;

float gyroM2X = 0.0f;
float gyroM2Y = 0.0f;
float gyroM2Z = 0.0f;

float accelCalibrationNoise = 0.0f;
float gyroCalibrationNoise = 0.0f;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

// ------------------------------------------------------------
// Filters and adaptive thresholds
// ------------------------------------------------------------

// Coefficients assume an 8 ms sample interval. They correspond to a
// slow 0.35 Hz baseline and roughly 8-9 Hz motion low-pass filters.
constexpr float GRAVITY_ALPHA = 0.0173f;
constexpr float ACCEL_ENVELOPE_ALPHA = 0.3110f;
constexpr float GYRO_ENVELOPE_ALPHA = 0.2870f;
constexpr float NOISE_FLOOR_ALPHA = 0.0030f;

float gravityEstimate = 1.0f;
float accelEnvelope = 0.0f;
float gyroEnvelope = 0.0f;
float accelNoiseFloor = 0.005f;
float gyroNoiseFloor = 1.0f;

float baseAccelThreshold = 0.055f;
float baseGyroThreshold = 10.0f;
float activeAccelThreshold = 0.055f;
float activeGyroThreshold = 10.0f;
float combinedScore = 0.0f;

// A peak is opened at the high threshold and confirmed after the
// signal drops through the release threshold. This gives hysteresis
// without the double triggering common to simple threshold tests.
constexpr float PEAK_TRIGGER_SCORE = 1.00f;
constexpr float PEAK_RELEASE_SCORE = 0.58f;
constexpr uint32_t MAX_PEAK_WIDTH_MS = 190;

bool peakActive = false;
uint32_t peakStartedAt = 0;
uint32_t peakAt = 0;
float peakScore = 0.0f;
float peakAccel = 0.0f;
float peakGyro = 0.0f;

// ------------------------------------------------------------
// Cadence and rhythm
// ------------------------------------------------------------

constexpr uint32_t MIN_STEP_INTERVAL_MS = 260;   // 231 SPM ceiling
constexpr uint32_t MAX_STEP_INTERVAL_MS = 1500;  // 40 SPM floor
constexpr uint32_t MIN_LOCK_TIMEOUT_MS = 2400;

bool rhythmLocked = false;
uint32_t pendingCandidateAt = 0;
uint32_t lastCandidateAt = 0;

uint32_t stepCount = 0;
uint32_t lastStepAt = 0;

constexpr uint8_t INTERVAL_HISTORY_SIZE = 5;
uint32_t intervalHistory[INTERVAL_HISTORY_SIZE] = {0};
uint8_t intervalCount = 0;
uint8_t intervalWriteIndex = 0;

float smoothedIntervalMs = 0.0f;
float currentCadence = 0.0f;
float averageCadence = 0.0f;
uint64_t activeIntervalTotalMs = 0;
uint32_t activeIntervalCount = 0;

// ============================================================
// Forward declarations
// ============================================================

bool initializeBuses();
bool selectMPUBus();
bool selectOLEDBus();
bool transmitDisplayFrame();
bool initializeDisplay();
bool initializeMPU();
bool addressResponds(TwoWire &bus, uint8_t address);
bool writeMPURegister(uint8_t reg, uint8_t value);
bool readMPURegister(uint8_t reg, uint8_t &value);
SampleResult readMotionSample();

void sensorTask(void *parameter);
void beginCalibration();
void resetCalibrationStatistics(bool movementDetected);
void updateCalibration();
void finishCalibration();

void beginTracking();
void processHandheldMotion(uint32_t sampleAt);
void finishMotionPeak();
void processStepCandidate(uint32_t candidateAt);
void establishRhythmLock(uint32_t firstAt, uint32_t secondAt);
void registerConfirmedStep(
  uint32_t stepAt,
  uint8_t stepsToAdd,
  uint32_t normalizedInterval
);
void loseRhythmLock(uint32_t newPendingAt);
void checkRhythmTimeout(uint32_t now);

void clearIntervals();
void addInterval(uint32_t interval);
float robustIntervalMilliseconds();
void updateCadence();

void setLed(bool on);
void pulseLed();
void serviceLed();

void publishUiSnapshot();
UiSnapshot copyUiSnapshot();
void updateDisplay();
void drawCalibrationScreen(const UiSnapshot &snapshot);
void drawTrackingScreen(const UiSnapshot &snapshot);
void drawErrorScreen(const UiSnapshot &snapshot);
void drawTopLine(const char *leftText, const char *rightText);
void drawBar(int x, int y, int width, int height, float fraction);
void formatTime(uint32_t totalSeconds, char *buffer, size_t size);

void enterError(const char *message);

// ============================================================
// I2C and hardware initialization
// ============================================================

bool initializeBuses() {
#if USE_ORIGINAL_WIRING
  i2cAccessMutex = xSemaphoreCreateMutex();
  if (i2cAccessMutex == nullptr) {
    return false;
  }

  return selectMPUBus();
#else
  if (!Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, I2C_CLOCK_HZ)) {
    return false;
  }

  if (!Wire1.begin(OLED_SDA_PIN, OLED_SCL_PIN, I2C_CLOCK_HZ)) {
    return false;
  }

  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  Wire1.setClock(I2C_CLOCK_HZ);
  Wire1.setTimeOut(I2C_TIMEOUT_MS);
  return true;
#endif
}

bool selectMPUBus() {
#if USE_ORIGINAL_WIRING
  if (primaryBusTarget == PRIMARY_BUS_MPU) {
    return true;
  }

  if (primaryBusTarget != PRIMARY_BUS_NONE) {
    Wire.end();
    delayMicroseconds(80);
  }

  if (!Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN, I2C_CLOCK_HZ)) {
    primaryBusTarget = PRIMARY_BUS_NONE;
    return false;
  }

  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  primaryBusTarget = PRIMARY_BUS_MPU;
#endif
  return true;
}

bool selectOLEDBus() {
#if USE_ORIGINAL_WIRING
  if (primaryBusTarget == PRIMARY_BUS_OLED) {
    return true;
  }

  if (primaryBusTarget != PRIMARY_BUS_NONE) {
    Wire.end();
    delayMicroseconds(80);
  }

  if (!Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN, I2C_CLOCK_HZ)) {
    primaryBusTarget = PRIMARY_BUS_NONE;
    return false;
  }

  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  primaryBusTarget = PRIMARY_BUS_OLED;
#endif
  return true;
}

bool transmitDisplayFrame() {
#if USE_ORIGINAL_WIRING
  if (i2cAccessMutex == nullptr ||
      xSemaphoreTake(i2cAccessMutex, portMAX_DELAY) != pdTRUE) {
    return false;
  }

  bool selected = selectOLEDBus();
  if (selected) {
    display.display();
  }

  bool restored = selectMPUBus();
  xSemaphoreGive(i2cAccessMutex);
  return selected && restored;
#else
  display.display();
  return true;
#endif
}

bool addressResponds(TwoWire &bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

bool initializeDisplay() {
  if (!selectOLEDBus()) {
    return false;
  }

  if (addressResponds(OLED_WIRE, 0x3C)) {
    oledAddress = 0x3C;
  } else if (addressResponds(OLED_WIRE, 0x3D)) {
    oledAddress = 0x3D;
  }

  bool initialized = false;

  // periphBegin=false preserves the bus and pins configured above.
  if (oledAddress != 0 &&
      display.begin(SSD1306_SWITCHCAPVCC, oledAddress, true, false)) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    display.display();
    initialized = true;
  }

  bool restored = selectMPUBus();
  return initialized && restored;
}

bool writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMPURegister(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(mpuAddress, static_cast<size_t>(1), true) != 1) {
    return false;
  }

  value = static_cast<uint8_t>(Wire.read());
  return true;
}

bool initializeMPU() {
  if (addressResponds(Wire, MPU_ADDRESS_LOW)) {
    mpuAddress = MPU_ADDRESS_LOW;
  } else if (addressResponds(Wire, MPU_ADDRESS_HIGH)) {
    mpuAddress = MPU_ADDRESS_HIGH;
  } else {
    return false;
  }

  uint8_t whoAmI = 0;
  if (!readMPURegister(REG_WHO_AM_I, whoAmI)) {
    return false;
  }

  // 0x71=MPU9250, 0x73=MPU9255, 0x70=MPU6500.
  if (whoAmI != 0x71 && whoAmI != 0x73 && whoAmI != 0x70) {
    Serial.printf("Unexpected MPU WHO_AM_I: 0x%02X\n", whoAmI);
    return false;
  }

  // Reset, wake, and use the gyro PLL as the clock source.
  if (!writeMPURegister(REG_PWR_MGMT_1, 0x80)) {
    return false;
  }
  delay(100);

  if (!writeMPURegister(REG_PWR_MGMT_1, 0x01) ||
      !writeMPURegister(REG_PWR_MGMT_2, 0x00)) {
    return false;
  }
  delay(50);

  // 1 kHz internal rate / (1 + 7) = 125 Hz output.
  // DLPF setting 4 is approximately 20/21 Hz for gyro/accelerometer.
  // The lower bandwidth rejects handheld vibration while preserving gait.
  if (!writeMPURegister(REG_CONFIG, 0x04) ||
      !writeMPURegister(REG_SMPLRT_DIV, 0x07) ||
      !writeMPURegister(REG_GYRO_CONFIG, 0x08) ||
      !writeMPURegister(REG_ACCEL_CONFIG, 0x08) ||
      !writeMPURegister(REG_ACCEL_CONFIG_2, 0x04) ||
      !writeMPURegister(REG_INT_PIN_CFG, 0x00) ||
      !writeMPURegister(REG_INT_ENABLE, 0x01)) {
    return false;
  }

  delay(20);

  // Verify the scale registers so a wiring glitch cannot silently produce
  // incorrectly scaled motion data.
  uint8_t gyroConfig = 0;
  uint8_t accelConfig = 0;

  if (!readMPURegister(REG_GYRO_CONFIG, gyroConfig) ||
      !readMPURegister(REG_ACCEL_CONFIG, accelConfig)) {
    return false;
  }

  return (gyroConfig & 0x18) == 0x08 &&
         (accelConfig & 0x18) == 0x08;
}

SampleResult readMotionSample() {
  // Read INT_STATUS plus the complete 14-byte motion block in one I2C
  // transaction. This both checks data-ready and minimizes bus overhead.
  Wire.beginTransmission(mpuAddress);
  Wire.write(REG_INT_STATUS);

  if (Wire.endTransmission(false) != 0) {
    return SAMPLE_BUS_ERROR;
  }

  constexpr size_t BLOCK_SIZE = 15;
  uint8_t data[BLOCK_SIZE];

  size_t received = Wire.requestFrom(mpuAddress, BLOCK_SIZE, true);
  if (received != BLOCK_SIZE) {
    return SAMPLE_BUS_ERROR;
  }

  for (size_t i = 0; i < BLOCK_SIZE; ++i) {
    data[i] = static_cast<uint8_t>(Wire.read());
  }

  if ((data[0] & 0x01) == 0) {
    return SAMPLE_NOT_READY;
  }

  auto makeInt16 = [](uint8_t highByte, uint8_t lowByte) -> int16_t {
    uint16_t combined =
      (static_cast<uint16_t>(highByte) << 8) |
      static_cast<uint16_t>(lowByte);
    return static_cast<int16_t>(combined);
  };

  int16_t rawAccelX = makeInt16(data[1], data[2]);
  int16_t rawAccelY = makeInt16(data[3], data[4]);
  int16_t rawAccelZ = makeInt16(data[5], data[6]);

  // data[7] and data[8] are temperature and intentionally ignored.
  int16_t rawGyroX = makeInt16(data[9], data[10]);
  int16_t rawGyroY = makeInt16(data[11], data[12]);
  int16_t rawGyroZ = makeInt16(data[13], data[14]);

  accelX = static_cast<float>(rawAccelX) / ACCEL_COUNTS_PER_G;
  accelY = static_cast<float>(rawAccelY) / ACCEL_COUNTS_PER_G;
  accelZ = static_cast<float>(rawAccelZ) / ACCEL_COUNTS_PER_G;

  gyroX = static_cast<float>(rawGyroX) / GYRO_COUNTS_PER_DPS;
  gyroY = static_cast<float>(rawGyroY) / GYRO_COUNTS_PER_DPS;
  gyroZ = static_cast<float>(rawGyroZ) / GYRO_COUNTS_PER_DPS;

  accelMagnitude = sqrtf(
    accelX * accelX +
    accelY * accelY +
    accelZ * accelZ
  );

  gyroMagnitude = sqrtf(
    gyroX * gyroX +
    gyroY * gyroY +
    gyroZ * gyroZ
  );

  return SAMPLE_READY;
}

// ============================================================
// Setup, loop, and high-priority sensor task
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(250);

  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  if (!initializeBuses()) {
    enterError("I2C INIT FAILED");
    while (true) {
      delay(1000);
    }
  }

  displayAvailable = initializeDisplay();
  if (!displayAvailable) {
    Serial.println("OLED not found; tracking will continue without it.");
  }

  if (!initializeMPU()) {
    enterError("MPU INIT FAILED");
    updateDisplay();
    while (true) {
      serviceLed();
      delay(10);
    }
  }

  // Quick hardware LED test. It happens before sampling begins and therefore
  // cannot disturb cadence timing.
  for (uint8_t i = 0; i < 3; ++i) {
    setLed(true);
    delay(45);
    setLed(false);
    delay(55);
  }

  // Discard the first quarter-second of sensor settling.
  uint32_t warmupStartedAt = millis();
  while (millis() - warmupStartedAt < 250) {
    if (readMotionSample() == SAMPLE_READY) {
      gravityEstimate = accelMagnitude;
    }
    delay(SENSOR_PERIOD_MS);
  }

  beginCalibration();
  publishUiSnapshot();
  updateDisplay();

  BaseType_t taskCreated = xTaskCreate(
    sensorTask,
    "cadence-sensor",
    4096,
    nullptr,
    3,
    nullptr
  );

  if (taskCreated != pdPASS) {
    enterError("SENSOR TASK FAILED");
    updateDisplay();
    while (true) {
      delay(1000);
    }
  }

  Serial.println();
  Serial.println("HANDHELD CADENCE TRACKER READY");
  Serial.println("Hold completely still until calibration reaches 100%.");
}

void loop() {
  serviceLed();

  uint32_t now = millis();
  if (now - lastDisplayAt >= DISPLAY_PERIOD_MS) {
    lastDisplayAt = now;
    updateDisplay();
  }

  // Let the higher-priority sensor task wake precisely every 8 ms.
  delay(1);
}

void sensorTask(void *parameter) {
  (void)parameter;

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t periodTicks = pdMS_TO_TICKS(SENSOR_PERIOD_MS);

  while (true) {
    vTaskDelayUntil(&lastWake, periodTicks);

    TickType_t nowTicks = xTaskGetTickCount();
    if (nowTicks - lastWake > periodTicks) {
      missedDeadlines++;
      lastWake = nowTicks;
    }

    SampleResult result = SAMPLE_BUS_ERROR;

#if USE_ORIGINAL_WIRING
    if (xSemaphoreTake(i2cAccessMutex, portMAX_DELAY) == pdTRUE) {
      if (selectMPUBus()) {
        result = readMotionSample();
      }
      xSemaphoreGive(i2cAccessMutex);
    }
#else
    result = readMotionSample();
#endif

    if (result == SAMPLE_BUS_ERROR) {
      sensorFailures++;
      consecutiveSensorFailures++;

      if (consecutiveSensorFailures == 20) {
        enterError("MPU READ FAILED");
      }
    } else if (result == SAMPLE_READY) {
      consecutiveSensorFailures = 0;
      uint32_t sampleAt = millis();

      if (appState == STATE_CALIBRATING) {
        updateCalibration();
      } else if (appState == STATE_TRACKING) {
        processHandheldMotion(sampleAt);
        checkRhythmTimeout(sampleAt);
      }
    }

    publishUiSnapshot();
  }
}

// ============================================================
// Calibration
// ============================================================

void beginCalibration() {
  appState = STATE_CALIBRATING;
  resetCalibrationStatistics(false);
  Serial.println("Calibration started.");
}

void resetCalibrationStatistics(bool movementDetected) {
  calibrationSamples = 0;
  unstableCalibrationSamples = 0;
  calibrationMoving = movementDetected;

  accelCalibrationMean = 0.0f;
  accelCalibrationM2 = 0.0f;

  gyroMeanX = 0.0f;
  gyroMeanY = 0.0f;
  gyroMeanZ = 0.0f;

  gyroM2X = 0.0f;
  gyroM2Y = 0.0f;
  gyroM2Z = 0.0f;
}

void updateCalibration() {
  bool stationary =
    accelMagnitude > 0.78f &&
    accelMagnitude < 1.22f &&
    gyroMagnitude < 35.0f;

  if (stationary && calibrationSamples >= 24) {
    float gyroDeltaX = gyroX - gyroMeanX;
    float gyroDeltaY = gyroY - gyroMeanY;
    float gyroDeltaZ = gyroZ - gyroMeanZ;
    float gyroDeltaMagnitude = sqrtf(
      gyroDeltaX * gyroDeltaX +
      gyroDeltaY * gyroDeltaY +
      gyroDeltaZ * gyroDeltaZ
    );

    stationary =
      fabsf(accelMagnitude - accelCalibrationMean) < 0.065f &&
      gyroDeltaMagnitude < 10.0f;
  }

  if (!stationary) {
    calibrationMoving = true;
    unstableCalibrationSamples++;

    // Require a continuous still window. Short tremors merely pause the
    // progress; sustained motion restarts the three-second window.
    if (unstableCalibrationSamples >= 10) {
      resetCalibrationStatistics(true);
    }
    return;
  }

  unstableCalibrationSamples = 0;
  calibrationSamples++;

  float n = static_cast<float>(calibrationSamples);

  float accelDelta = accelMagnitude - accelCalibrationMean;
  accelCalibrationMean += accelDelta / n;
  accelCalibrationM2 +=
    accelDelta * (accelMagnitude - accelCalibrationMean);

  float gyroDeltaX = gyroX - gyroMeanX;
  gyroMeanX += gyroDeltaX / n;
  gyroM2X += gyroDeltaX * (gyroX - gyroMeanX);

  float gyroDeltaY = gyroY - gyroMeanY;
  gyroMeanY += gyroDeltaY / n;
  gyroM2Y += gyroDeltaY * (gyroY - gyroMeanY);

  float gyroDeltaZ = gyroZ - gyroMeanZ;
  gyroMeanZ += gyroDeltaZ / n;
  gyroM2Z += gyroDeltaZ * (gyroZ - gyroMeanZ);

  if (calibrationSamples >= 24) {
    calibrationMoving = false;
  }

  if (calibrationSamples >= CALIBRATION_SAMPLE_COUNT) {
    finishCalibration();
  }
}

void finishCalibration() {
  if (calibrationSamples < 2) {
    enterError("NO SENSOR SAMPLES");
    return;
  }

  float divisor = static_cast<float>(calibrationSamples - 1);
  accelCalibrationNoise = sqrtf(accelCalibrationM2 / divisor);

  float gyroVarianceX = gyroM2X / divisor;
  float gyroVarianceY = gyroM2Y / divisor;
  float gyroVarianceZ = gyroM2Z / divisor;
  gyroCalibrationNoise = sqrtf(
    gyroVarianceX + gyroVarianceY + gyroVarianceZ
  );

  gyroBiasX = gyroMeanX;
  gyroBiasY = gyroMeanY;
  gyroBiasZ = gyroMeanZ;
  gravityEstimate = accelCalibrationMean;

  baseAccelThreshold = constrain(
    0.035f + 5.0f * accelCalibrationNoise,
    0.050f,
    0.130f
  );

  baseGyroThreshold = constrain(
    6.0f + 4.0f * gyroCalibrationNoise,
    9.0f,
    28.0f
  );

  accelNoiseFloor = fmaxf(0.005f, accelCalibrationNoise);
  gyroNoiseFloor = fmaxf(1.0f, gyroCalibrationNoise);

  Serial.printf(
    "Calibration complete: gravity=%.3fg accelNoise=%.4fg "
    "gyroNoise=%.2fdps\n",
    gravityEstimate,
    accelCalibrationNoise,
    gyroCalibrationNoise
  );
  Serial.printf(
    "Starting thresholds: accel=%.3fg gyro=%.1fdps\n",
    baseAccelThreshold,
    baseGyroThreshold
  );

  beginTracking();
}

// ============================================================
// Motion filtering and peak detection
// ============================================================

void beginTracking() {
  rhythmLocked = false;
  pendingCandidateAt = 0;
  lastCandidateAt = 0;

  stepCount = 0;
  lastStepAt = 0;

  clearIntervals();
  currentCadence = 0.0f;
  averageCadence = 0.0f;
  smoothedIntervalMs = 0.0f;
  activeIntervalTotalMs = 0;
  activeIntervalCount = 0;

  accelEnvelope = 0.0f;
  gyroEnvelope = 0.0f;
  combinedScore = 0.0f;

  peakActive = false;
  peakStartedAt = 0;
  peakAt = 0;
  peakScore = 0.0f;
  peakAccel = 0.0f;
  peakGyro = 0.0f;

  gravityEstimate = accelCalibrationMean;
  sessionStartedAt = millis();
  appState = STATE_TRACKING;

  Serial.println("Automatic tracking started.");
}

void processHandheldMotion(uint32_t sampleAt) {
  float correctedGyroX = gyroX - gyroBiasX;
  float correctedGyroY = gyroY - gyroBiasY;
  float correctedGyroZ = gyroZ - gyroBiasZ;

  float correctedGyroMagnitude = sqrtf(
    correctedGyroX * correctedGyroX +
    correctedGyroY * correctedGyroY +
    correctedGyroZ * correctedGyroZ
  );

  // Magnitude makes the detector independent of how the unit is held.
  // The slow baseline removes gravity and gradual orientation changes.
  gravityEstimate +=
    GRAVITY_ALPHA * (accelMagnitude - gravityEstimate);

  float dynamicAcceleration = fabsf(accelMagnitude - gravityEstimate);

  accelEnvelope += ACCEL_ENVELOPE_ALPHA *
    (dynamicAcceleration - accelEnvelope);
  gyroEnvelope += GYRO_ENVELOPE_ALPHA *
    (correctedGyroMagnitude - gyroEnvelope);

  activeAccelThreshold = fmaxf(
    baseAccelThreshold,
    accelNoiseFloor * 3.6f + 0.024f
  );
  activeAccelThreshold = constrain(
    activeAccelThreshold,
    baseAccelThreshold,
    0.240f
  );

  activeGyroThreshold = fmaxf(
    baseGyroThreshold,
    gyroNoiseFloor * 3.2f + 5.0f
  );
  activeGyroThreshold = constrain(
    activeGyroThreshold,
    baseGyroThreshold,
    65.0f
  );

  float accelRatio =
    accelEnvelope / fmaxf(activeAccelThreshold, 0.001f);
  float gyroRatio =
    gyroEnvelope / fmaxf(activeGyroThreshold, 0.1f);

  combinedScore = 0.70f * accelRatio + 0.30f * gyroRatio;

  bool corroboratedMotion =
    (accelRatio >= 1.00f && gyroRatio >= 0.55f) ||
    (gyroRatio >= 1.15f && accelRatio >= 0.65f) ||
    (accelRatio >= 1.60f);

  if (!peakActive) {
    // Learn quiet handheld motion only below the release level. This keeps
    // real walking from raising its own threshold and disappearing.
    if (combinedScore < PEAK_RELEASE_SCORE) {
      accelNoiseFloor += NOISE_FLOOR_ALPHA *
        (accelEnvelope - accelNoiseFloor);
      gyroNoiseFloor += NOISE_FLOOR_ALPHA *
        (gyroEnvelope - gyroNoiseFloor);
    }

    bool outsideRefractory =
      lastCandidateAt == 0 ||
      sampleAt - lastCandidateAt >= MIN_STEP_INTERVAL_MS;

    if (outsideRefractory &&
        combinedScore >= PEAK_TRIGGER_SCORE &&
        corroboratedMotion) {
      peakActive = true;
      peakStartedAt = sampleAt;
      peakAt = sampleAt;
      peakScore = combinedScore;
      peakAccel = accelEnvelope;
      peakGyro = gyroEnvelope;
    }
    return;
  }

  if (combinedScore > peakScore) {
    peakAt = sampleAt;
    peakScore = combinedScore;
    peakAccel = accelEnvelope;
    peakGyro = gyroEnvelope;
  }

  if (combinedScore <= PEAK_RELEASE_SCORE ||
      sampleAt - peakStartedAt >= MAX_PEAK_WIDTH_MS) {
    finishMotionPeak();
  }
}

void finishMotionPeak() {
  bool validShape =
    peakScore >= PEAK_TRIGGER_SCORE &&
    peakAccel < 2.5f &&
    peakGyro < 420.0f;

  if (validShape) {
    lastCandidateAt = peakAt;
    processStepCandidate(peakAt);
  }

  peakActive = false;
}

// ============================================================
// Rhythm validation and step registration
// ============================================================

void processStepCandidate(uint32_t candidateAt) {
  if (!rhythmLocked) {
    if (pendingCandidateAt == 0) {
      pendingCandidateAt = candidateAt;

      // The flash happens at detection time. If the next interval is valid,
      // this becomes the first confirmed step without a delayed replay.
      pulseLed();

#if ENABLE_STEP_DEBUG
      Serial.printf("SEEK candidate score=%.2f\n", peakScore);
#endif
      return;
    }

    uint32_t interval = candidateAt - pendingCandidateAt;

    if (interval < MIN_STEP_INTERVAL_MS) {
      // Keep the first peak; a close second peak is usually the same step.
      return;
    }

    if (interval <= MAX_STEP_INTERVAL_MS) {
      establishRhythmLock(pendingCandidateAt, candidateAt);
      pulseLed();
      pendingCandidateAt = 0;
      return;
    }

    // The old candidate is stale. The current peak becomes a fresh start.
    pendingCandidateAt = candidateAt;
    pulseLed();
    return;
  }

  uint32_t interval = candidateAt - lastStepAt;
  if (interval < MIN_STEP_INTERVAL_MS) {
    return;
  }

  float expectedInterval = robustIntervalMilliseconds();
  if (expectedInterval <= 0.0f) {
    registerConfirmedStep(candidateAt, 1, interval);
    pulseLed();
    return;
  }

  float intervalRatio =
    static_cast<float>(interval) / expectedInterval;

  // Normal cadence variation. Tighter bounds reject random hand motion but
  // still allow a runner to accelerate or slow down naturally.
  if (intervalRatio >= 0.62f && intervalRatio <= 1.55f &&
      interval <= MAX_STEP_INTERVAL_MS) {
    registerConfirmedStep(candidateAt, 1, interval);
    pulseLed();
    return;
  }

  // A gap near two periods almost always means one impact was too weak to
  // cross the threshold. Recover that count and feed half the gap into the
  // cadence filter. Only the actually observed step flashes the LED.
  if (intervalRatio > 1.55f && intervalRatio <= 2.35f) {
    uint32_t normalizedInterval = interval / 2U;
    registerConfirmedStep(candidateAt, 2, normalizedInterval);
    pulseLed();
    return;
  }

  if (intervalRatio < 0.62f) {
    // Likely a secondary hand-motion peak. Retain the existing rhythm.
    return;
  }

  // The old rhythm is no longer credible. This strong current peak is kept
  // as the first candidate of the next lock instead of being discarded.
  loseRhythmLock(candidateAt);
  pulseLed();
}

void establishRhythmLock(uint32_t firstAt, uint32_t secondAt) {
  rhythmLocked = true;

  // Add instead of assigning 2. The original sketch erased the session
  // total every time cadence lock was reacquired.
  stepCount += 2;
  lastStepAt = secondAt;

  addInterval(secondAt - firstAt);
  updateCadence();

#if ENABLE_STEP_DEBUG
  Serial.printf(
    "LOCK steps=%lu cadence=%.1f\n",
    static_cast<unsigned long>(stepCount),
    currentCadence
  );
#endif
}

void registerConfirmedStep(
  uint32_t stepAt,
  uint8_t stepsToAdd,
  uint32_t normalizedInterval
) {
  stepCount += stepsToAdd;
  lastStepAt = stepAt;

  // Weight a recovered two-step gap as two equal intervals.
  for (uint8_t i = 0; i < stepsToAdd; ++i) {
    addInterval(normalizedInterval);
  }
  updateCadence();

#if ENABLE_STEP_DEBUG
  Serial.printf(
    "STEP total=%lu add=%u cadence=%.1f\n",
    static_cast<unsigned long>(stepCount),
    stepsToAdd,
    currentCadence
  );
#endif
}

void loseRhythmLock(uint32_t newPendingAt) {
  rhythmLocked = false;
  pendingCandidateAt = newPendingAt;
  clearIntervals();
  currentCadence = 0.0f;
  smoothedIntervalMs = 0.0f;

#if ENABLE_STEP_DEBUG
  Serial.println("Rhythm lock lost.");
#endif
}

void checkRhythmTimeout(uint32_t now) {
  if (!rhythmLocked || lastStepAt == 0) {
    return;
  }

  float expected = robustIntervalMilliseconds();
  uint32_t timeout = MIN_LOCK_TIMEOUT_MS;

  if (expected > 0.0f) {
    uint32_t adaptiveTimeout =
      static_cast<uint32_t>(expected * 2.6f);
    timeout = constrain(adaptiveTimeout, MIN_LOCK_TIMEOUT_MS, 4000U);
  }

  if (now - lastStepAt > timeout) {
    loseRhythmLock(0);
  }
}

// ============================================================
// Robust cadence smoothing
// ============================================================

void clearIntervals() {
  intervalCount = 0;
  intervalWriteIndex = 0;
  for (uint8_t i = 0; i < INTERVAL_HISTORY_SIZE; ++i) {
    intervalHistory[i] = 0;
  }
}

void addInterval(uint32_t interval) {
  intervalHistory[intervalWriteIndex] = interval;
  intervalWriteIndex =
    (intervalWriteIndex + 1U) % INTERVAL_HISTORY_SIZE;

  if (intervalCount < INTERVAL_HISTORY_SIZE) {
    intervalCount++;
  }

  // Session average uses only accepted active step intervals. Pauses and
  // rejected hand movements therefore do not drag the average toward zero.
  activeIntervalTotalMs += interval;
  activeIntervalCount++;
}

float robustIntervalMilliseconds() {
  if (intervalCount == 0) {
    return 0.0f;
  }

  uint32_t sorted[INTERVAL_HISTORY_SIZE];
  for (uint8_t i = 0; i < intervalCount; ++i) {
    sorted[i] = intervalHistory[i];
  }

  for (uint8_t i = 1; i < intervalCount; ++i) {
    uint32_t value = sorted[i];
    int j = static_cast<int>(i) - 1;

    while (j >= 0 && sorted[j] > value) {
      sorted[j + 1] = sorted[j];
      --j;
    }
    sorted[j + 1] = value;
  }

  if (intervalCount <= 2) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < intervalCount; ++i) {
      sum += sorted[i];
    }
    return static_cast<float>(sum) / intervalCount;
  }

  // Trim the fastest and slowest samples. Compared with a 7-value median,
  // this rejects outliers but responds several steps sooner to pace changes.
  uint32_t trimmedSum = 0;
  for (uint8_t i = 1; i < intervalCount - 1; ++i) {
    trimmedSum += sorted[i];
  }

  return static_cast<float>(trimmedSum) /
         static_cast<float>(intervalCount - 2);
}

void updateCadence() {
  if (activeIntervalCount > 0 && activeIntervalTotalMs > 0) {
    averageCadence =
      60000.0f * static_cast<float>(activeIntervalCount) /
      static_cast<float>(activeIntervalTotalMs);
  }

  float robustInterval = robustIntervalMilliseconds();
  if (robustInterval <= 0.0f) {
    currentCadence = 0.0f;
    return;
  }

  if (smoothedIntervalMs <= 0.0f) {
    smoothedIntervalMs = robustInterval;
  } else {
    constexpr float CADENCE_RESPONSE_ALPHA = 0.38f;
    smoothedIntervalMs += CADENCE_RESPONSE_ALPHA *
      (robustInterval - smoothedIntervalMs);
  }

  currentCadence = 60000.0f / smoothedIntervalMs;
}

// ============================================================
// LED
// ============================================================

void setLed(bool on) {
  bool outputHigh = LED_ACTIVE_HIGH ? on : !on;
  digitalWrite(LED_PIN, outputHigh ? HIGH : LOW);
}

void pulseLed() {
  uint32_t offAt = millis() + LED_PULSE_MS;

  portENTER_CRITICAL(&ledMux);
  ledIsOn = true;
  ledOffAt = offAt;
  portEXIT_CRITICAL(&ledMux);

  setLed(true);
}

void serviceLed() {
  uint32_t now = millis();
  bool turnOff = false;

  portENTER_CRITICAL(&ledMux);
  if (ledIsOn && static_cast<int32_t>(now - ledOffAt) >= 0) {
    ledIsOn = false;
    turnOff = true;
  }
  portEXIT_CRITICAL(&ledMux);

  if (turnOff) {
    setLed(false);
  }
}

// ============================================================
// Thread-safe UI snapshot and display
// ============================================================

void publishUiSnapshot() {
  float progress = 0.0f;
  if (appState == STATE_CALIBRATING) {
    progress = constrain(
      static_cast<float>(calibrationSamples) /
        static_cast<float>(CALIBRATION_SAMPLE_COUNT),
      0.0f,
      1.0f
    );
  }

  portENTER_CRITICAL(&uiMux);
  uiSnapshot.state = appState;
  uiSnapshot.calibrationProgress = progress;
  uiSnapshot.calibrationMoving = calibrationMoving;
  uiSnapshot.rhythmLocked = rhythmLocked;
  uiSnapshot.averageCadence = averageCadence;
  uiSnapshot.currentCadence = currentCadence;
  uiSnapshot.motionScore = combinedScore;
  uiSnapshot.stepCount = stepCount;
  uiSnapshot.sensorFailures = sensorFailures;
  uiSnapshot.missedDeadlines = missedDeadlines;
  uiSnapshot.sessionStartedAt = sessionStartedAt;
  strncpy(
    uiSnapshot.errorMessage,
    errorMessage,
    sizeof(uiSnapshot.errorMessage) - 1
  );
  uiSnapshot.errorMessage[sizeof(uiSnapshot.errorMessage) - 1] = '\0';
  portEXIT_CRITICAL(&uiMux);
}

UiSnapshot copyUiSnapshot() {
  UiSnapshot copy;
  portENTER_CRITICAL(&uiMux);
  copy = uiSnapshot;
  portEXIT_CRITICAL(&uiMux);
  return copy;
}

void updateDisplay() {
  if (!displayAvailable) {
    return;
  }

  UiSnapshot snapshot = copyUiSnapshot();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);
  display.setTextSize(1);

  if (snapshot.state == STATE_CALIBRATING) {
    drawCalibrationScreen(snapshot);
  } else if (snapshot.state == STATE_TRACKING) {
    drawTrackingScreen(snapshot);
  } else {
    drawErrorScreen(snapshot);
  }

  if (!transmitDisplayFrame()) {
    Serial.println("OLED frame transfer failed.");
  }
}

void drawTopLine(const char *leftText, const char *rightText) {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(leftText);

  int rightWidth = static_cast<int>(strlen(rightText)) * 6;
  int rightX = SCREEN_WIDTH - rightWidth;
  display.setCursor(rightX < 0 ? 0 : rightX, 0);
  display.print(rightText);

  display.drawLine(0, 9, SCREEN_WIDTH - 1, 9, SSD1306_WHITE);
}

void drawBar(int x, int y, int width, int height, float fraction) {
  fraction = constrain(fraction, 0.0f, 1.0f);
  display.drawRect(x, y, width, height, SSD1306_WHITE);

  int innerWidth = width - 4;
  int fillWidth = static_cast<int>(innerWidth * fraction);
  if (fillWidth > 0) {
    display.fillRect(
      x + 2,
      y + 2,
      fillWidth,
      height - 4,
      SSD1306_WHITE
    );
  }
}

void formatTime(uint32_t totalSeconds, char *buffer, size_t size) {
  if (totalSeconds < 3600U) {
    uint32_t minutes = totalSeconds / 60U;
    uint32_t seconds = totalSeconds % 60U;
    snprintf(
      buffer,
      size,
      "%02lu:%02lu",
      static_cast<unsigned long>(minutes),
      static_cast<unsigned long>(seconds)
    );
  } else {
    uint32_t hours = totalSeconds / 3600U;
    uint32_t minutes = (totalSeconds / 60U) % 60U;
    snprintf(
      buffer,
      size,
      "%lu:%02lu",
      static_cast<unsigned long>(hours),
      static_cast<unsigned long>(minutes)
    );
  }
}

void drawCalibrationScreen(const UiSnapshot &snapshot) {
  int percent = static_cast<int>(
    snapshot.calibrationProgress * 100.0f + 0.5f
  );

  char percentText[8];
  snprintf(percentText, sizeof(percentText), "%d%%", percent);
  drawTopLine("CALIBRATE", percentText);

  drawBar(8, 17, 112, 10, snapshot.calibrationProgress);

  display.setCursor(20, 34);
  display.print("HOLD COMPLETELY");
  display.setCursor(50, 44);
  display.print("STILL");

  display.setCursor(snapshot.calibrationMoving ? 8 : 31, 56);
  display.print(
    snapshot.calibrationMoving ?
      "MOTION - RESTARTING" :
      "AUTO START"
  );
}

void drawTrackingScreen(const UiSnapshot &snapshot) {
  char timeText[12];
  uint32_t elapsedSeconds =
    (millis() - snapshot.sessionStartedAt) / 1000U;
  formatTime(elapsedSeconds, timeText, sizeof(timeText));

  drawTopLine(
    snapshot.rhythmLocked ? "AVG CAD" : "AVG SEEK",
    timeText
  );

  display.setTextSize(2);
  display.setCursor(1, 14);
  if (snapshot.averageCadence > 0.0f) {
    display.print(static_cast<int>(snapshot.averageCadence + 0.5f));
  } else {
    display.print("---");
  }

  display.setTextSize(1);
  display.setCursor(40, 18);
  display.print("AVG");
  display.setCursor(40, 27);
  display.print("SPM");

  display.setCursor(79, 12);
  display.print("STEPS");

  if (snapshot.stepCount <= 9999U) {
    display.setTextSize(2);
    display.setCursor(79, 22);
  } else {
    display.setTextSize(1);
    display.setCursor(79, 27);
  }
  display.print(snapshot.stepCount);

  display.drawLine(0, 40, SCREEN_WIDTH - 1, 40, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(1, 44);
  display.print("MOT");
  drawBar(24, 44, 103, 8, snapshot.motionScore / 2.4f);

  display.setCursor(1, 55);
  display.print("NOW ");
  if (snapshot.currentCadence > 0.0f) {
    display.print(static_cast<int>(snapshot.currentCadence + 0.5f));
  } else {
    display.print("---");
  }
  display.print(" SPM  LED=STEP");
}

void drawErrorScreen(const UiSnapshot &snapshot) {
  drawTopLine("ERROR", "STOP");
  display.setCursor(5, 18);
  display.print(snapshot.errorMessage);
  display.setCursor(5, 33);
  display.print("CHECK MPU + RESET");
  display.setCursor(5, 48);
  display.print("I2C FAIL ");
  display.print(snapshot.sensorFailures);
  display.setCursor(5, 57);
  display.print("LATE ");
  display.print(snapshot.missedDeadlines);
}

// ============================================================
// Error handling
// ============================================================

void enterError(const char *message) {
  strncpy(errorMessage, message, sizeof(errorMessage) - 1);
  errorMessage[sizeof(errorMessage) - 1] = '\0';
  appState = STATE_ERROR;
  setLed(false);

  Serial.print("ERROR: ");
  Serial.println(errorMessage);
  publishUiSnapshot();
}
