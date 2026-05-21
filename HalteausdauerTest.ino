#include <M5StickCPlus.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>

// ================================================================
// Anpassbare Parameter
// ================================================================

const float START_THRESHOLD_CM = 1.0f;
const float STOP_THRESHOLD_CM = 1.0f;
const uint32_t STOP_HOLD_MS = 400;

const uint8_t CALIBRATION_SAMPLES = 20;

const uint32_t SENSOR_UPDATE_MS = 40;   // 25 Hz
const uint32_t DISPLAY_UPDATE_MS = 200; // 5 Hz

const uint32_t BEEP_INTERVAL_MS = 1000;
const uint32_t BEEP_DURATION_MS = 50;

const uint32_t CONFIRM_BEEP_DURATION_MS = 80;
const uint32_t CONFIRM_BEEP_GAP_MS = 100;

const uint16_t RUN_BEEP_FREQ_HZ = 2200;
const uint16_t CONFIRM_BEEP_FREQ_HZ = 3000;

const uint8_t SMOOTHING_WINDOW = 5;

// Typischer I2C-Anschluss des M5StickC PLUS HAT-Ports
const int I2C_SDA_PIN = 0;
const int I2C_SCL_PIN = 26;

// VL53L0X gültiger Bereich, für deine Geometrie ggf. anpassen
const uint16_t MIN_VALID_MM = 20;
const uint16_t MAX_VALID_MM = 2000;

// ================================================================
// Zustandsmaschine
// ================================================================

enum AppState {
  IDLE,
  CALIBRATING,
  WAIT_FOR_LIFT,
  RUNNING,
  FINISHED
};

AppState state = IDLE;

// ================================================================
// ToF-Sensor
// ================================================================

VL53L0X tof;
bool tofOk = false;

uint32_t lastSensorUpdateMs = 0;
uint32_t lastDisplayUpdateMs = 0;

bool hasDistance = false;
float currentDistanceCm = 0.0f;

bool hasZero = false;
float zeroDistanceCm = 0.0f;
float currentHeightCm = 0.0f;

// ================================================================
// Glättung: Median aus den letzten 5 gültigen Werten
// ================================================================

float smoothBuffer[SMOOTHING_WINDOW];
uint8_t smoothCount = 0;
uint8_t smoothIndex = 0;

void resetSmoothing() {
  smoothCount = 0;
  smoothIndex = 0;
}

float medianFilter(float value) {
  smoothBuffer[smoothIndex] = value;
  smoothIndex = (smoothIndex + 1) % SMOOTHING_WINDOW;

  if (smoothCount < SMOOTHING_WINDOW) {
    smoothCount++;
  }

  float values[SMOOTHING_WINDOW];

  for (uint8_t i = 0; i < smoothCount; i++) {
    values[i] = smoothBuffer[i];
  }

  for (uint8_t i = 0; i < smoothCount; i++) {
    for (uint8_t j = i + 1; j < smoothCount; j++) {
      if (values[j] < values[i]) {
        float tmp = values[i];
        values[i] = values[j];
        values[j] = tmp;
      }
    }
  }

  if (smoothCount % 2 == 1) {
    return values[smoothCount / 2];
  }

  uint8_t upper = smoothCount / 2;
  uint8_t lower = upper - 1;
  return 0.5f * (values[lower] + values[upper]);
}

// ================================================================
// Kalibrierung
// ================================================================

uint8_t calibrationCount = 0;
float calibrationSumCm = 0.0f;

// ================================================================
// Messdaten
// ================================================================

uint32_t startTimeMs = 0;
uint32_t endTimeMs = 0;
uint32_t lastAucUpdateMs = 0;
uint32_t stopZoneSinceMs = 0;

float maxHeightCm = 0.0f;
float aucCmSec = 0.0f;

float finalDurationSec = 0.0f;
float finalMaxHeightCm = 0.0f;
float finalAucCmSec = 0.0f;
float finalAverageHeightCm = 0.0f;

// ================================================================
// Nicht-blockierender Buzzer
// ================================================================

bool beepOn = false;
uint32_t beepOffAtMs = 0;

bool confirmSequenceActive = false;
uint8_t confirmBeepsLeft = 0;
uint32_t confirmNextStartMs = 0;

uint32_t lastRunBeepMs = 0;

void stopBeepNow() {
  M5.Beep.mute();
  beepOn = false;
}

void startSingleBeep(uint16_t freqHz, uint32_t durationMs) {
  M5.Beep.tone(freqHz);
  beepOn = true;
  beepOffAtMs = millis() + durationMs;
}

void startConfirmBeeps() {
  stopBeepNow();

  confirmSequenceActive = true;
  confirmBeepsLeft = 2;
  confirmNextStartMs = millis();
}

void updateBeeper() {
  uint32_t now = millis();

  if (beepOn && (int32_t)(now - beepOffAtMs) >= 0) {
    stopBeepNow();

    if (confirmSequenceActive) {
      if (confirmBeepsLeft > 0) {
        confirmBeepsLeft--;
      }
      confirmNextStartMs = now + CONFIRM_BEEP_GAP_MS;
    }
  }

  if (confirmSequenceActive && !beepOn) {
    if (confirmBeepsLeft == 0) {
      confirmSequenceActive = false;
    } else if ((int32_t)(now - confirmNextStartMs) >= 0) {
      startSingleBeep(CONFIRM_BEEP_FREQ_HZ, CONFIRM_BEEP_DURATION_MS);
    }

    return;
  }

  if (state == RUNNING && !beepOn) {
    if ((int32_t)(now - lastRunBeepMs) >= (int32_t)BEEP_INTERVAL_MS) {
      lastRunBeepMs = now;
      startSingleBeep(RUN_BEEP_FREQ_HZ, BEEP_DURATION_MS);
    }
  }
}

// ================================================================
// Hilfsfunktionen
// ================================================================

void resetMeasurementData() {
  startTimeMs = 0;
  endTimeMs = 0;
  lastAucUpdateMs = 0;
  stopZoneSinceMs = 0;

  maxHeightCm = 0.0f;
  aucCmSec = 0.0f;

  finalDurationSec = 0.0f;
  finalMaxHeightCm = 0.0f;
  finalAucCmSec = 0.0f;
  finalAverageHeightCm = 0.0f;
}

void startCalibration() {
  stopBeepNow();
  confirmSequenceActive = false;

  resetMeasurementData();
  resetSmoothing();

  hasZero = false;
  calibrationCount = 0;
  calibrationSumCm = 0.0f;

  state = CALIBRATING;
}

void startMeasurement(uint32_t now) {
  startTimeMs = now;
  lastAucUpdateMs = now;
  stopZoneSinceMs = 0;

  aucCmSec = 0.0f;
  maxHeightCm = currentHeightCm;

  if (maxHeightCm < 0.0f) {
    maxHeightCm = 0.0f;
  }

  lastRunBeepMs = now - BEEP_INTERVAL_MS;
  state = RUNNING;
}

void finishMeasurement(uint32_t now) {
  endTimeMs = now;

  finalDurationSec = (endTimeMs - startTimeMs) / 1000.0f;
  finalMaxHeightCm = maxHeightCm;
  finalAucCmSec = aucCmSec;

  if (finalDurationSec > 0.0f) {
    finalAverageHeightCm = finalAucCmSec / finalDurationSec;
  } else {
    finalAverageHeightCm = 0.0f;
  }

  stopBeepNow();
  confirmSequenceActive = false;

  state = FINISHED;
}

bool readTofDistanceCm(float &distanceCm) {
  if (!tofOk) {
    return false;
  }

  uint16_t distanceMm = tof.readRangeContinuousMillimeters();

  if (tof.timeoutOccurred()) {
    return false;
  }

  if (distanceMm < MIN_VALID_MM || distanceMm > MAX_VALID_MM) {
    return false;
  }

  distanceCm = distanceMm / 10.0f;
  return true;
}

void processNewDistance(float rawDistanceCm, uint32_t now) {
  currentDistanceCm = medianFilter(rawDistanceCm);
  hasDistance = true;

  if (hasZero) {
    currentHeightCm = currentDistanceCm - zeroDistanceCm;
  } else {
    currentHeightCm = 0.0f;
  }

  if (state == CALIBRATING) {
    calibrationSumCm += currentDistanceCm;
    calibrationCount++;

    if (calibrationCount >= CALIBRATION_SAMPLES) {
      zeroDistanceCm = calibrationSumCm / calibrationCount;
      hasZero = true;

      resetMeasurementData();

      currentHeightCm = currentDistanceCm - zeroDistanceCm;
      state = WAIT_FOR_LIFT;

      startConfirmBeeps();
    }

    return;
  }

  if (!hasZero) {
    return;
  }

  if (state == WAIT_FOR_LIFT) {
    if (currentHeightCm >= START_THRESHOLD_CM) {
      startMeasurement(now);
    }

    return;
  }

  if (state == RUNNING) {
    uint32_t dtMs = now - lastAucUpdateMs;
    lastAucUpdateMs = now;

    float dtSec = dtMs / 1000.0f;
    float positiveHeightCm = currentHeightCm;

    if (positiveHeightCm < 0.0f) {
      positiveHeightCm = 0.0f;
    }

    aucCmSec += positiveHeightCm * dtSec;

    if (currentHeightCm > maxHeightCm) {
      maxHeightCm = currentHeightCm;
    }

    if (currentHeightCm <= STOP_THRESHOLD_CM) {
      if (stopZoneSinceMs == 0) {
        stopZoneSinceMs = now;
      } else if ((now - stopZoneSinceMs) >= STOP_HOLD_MS) {
        finishMeasurement(now);
      }
    } else {
      stopZoneSinceMs = 0;
    }

    return;
  }
}

// ================================================================
// Display
// ================================================================

void printLine(const char *label, float value, const char *unit) {
  M5.Lcd.print(label);
  M5.Lcd.print(": ");

  if (isnan(value)) {
    M5.Lcd.println("--");
  } else {
    M5.Lcd.print(value, 1);
    M5.Lcd.print(" ");
    M5.Lcd.println(unit);
  }
}

void drawIdleLikeScreen(const char *title) {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setTextSize(2);

  M5.Lcd.println(title);
  M5.Lcd.println();

  if (hasDistance) {
    printLine("Dist", currentDistanceCm, "cm");
  } else {
    printLine("Dist", NAN, "cm");
  }

  if (hasZero) {
    printLine("Null", zeroDistanceCm, "cm");
    printLine("Hoehe", currentHeightCm, "cm");
  } else {
    printLine("Null", NAN, "cm");
    printLine("Hoehe", NAN, "cm");
  }

  M5.Lcd.println();
  M5.Lcd.setTextSize(1);
  M5.Lcd.println("Btn A = Nullung / neue Messung");

  if (!tofOk) {
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.println("ToF Sensor nicht gefunden!");
  }
}

void drawCalibrationScreen() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.setTextSize(2);

  M5.Lcd.println("NULLUNG");
  M5.Lcd.println();

  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.print("Samples: ");
  M5.Lcd.print(calibrationCount);
  M5.Lcd.print("/");
  M5.Lcd.println(CALIBRATION_SAMPLES);

  if (hasDistance) {
    printLine("Dist", currentDistanceCm, "cm");
  } else {
    printLine("Dist", NAN, "cm");
  }

  M5.Lcd.println();
  M5.Lcd.setTextSize(1);
  M5.Lcd.println("Gewicht ruhig in Startposition halten");
}

void drawRunningScreen() {
  uint32_t now = millis();
  float durationSec = (now - startTimeMs) / 1000.0f;

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setTextSize(2);

  M5.Lcd.println("RUNNING");
  M5.Lcd.println();

  M5.Lcd.setTextColor(WHITE, BLACK);
  printLine("Hoehe", currentHeightCm, "cm");
  printLine("Max", maxHeightCm, "cm");
  printLine("Zeit", durationSec, "s");
  printLine("AUC", aucCmSec, "cm*s");
}

void drawFinishedScreen() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.setTextSize(2);

  M5.Lcd.println("DONE");
  M5.Lcd.println();

  M5.Lcd.setTextColor(WHITE, BLACK);
  printLine("Dauer", finalDurationSec, "s");
  printLine("Max", finalMaxHeightCm, "cm");
  printLine("AUC", finalAucCmSec, "cm*s");
  printLine("Mittel", finalAverageHeightCm, "cm");

  M5.Lcd.println();
  M5.Lcd.setTextSize(1);
  M5.Lcd.println("Btn A = neue Nullung / Messung");
}

void updateDisplay() {
  switch (state) {
    case IDLE:
      drawIdleLikeScreen("IDLE");
      break;

    case CALIBRATING:
      drawCalibrationScreen();
      break;

    case WAIT_FOR_LIFT:
      drawIdleLikeScreen("WAIT LIFT");
      break;

    case RUNNING:
      drawRunningScreen();
      break;

    case FINISHED:
      drawFinishedScreen();
      break;
  }
}

// ================================================================
// Arduino setup / loop
// ================================================================

void setup() {
  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setTextSize(2);

  M5.Beep.setVolume(8);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  tof.setTimeout(50);
  tofOk = tof.init();

  if (tofOk) {
    tof.setMeasurementTimingBudget(33000);
    tof.startContinuous(SENSOR_UPDATE_MS);
  }

  resetMeasurementData();

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("ToF Hold Test");
  M5.Lcd.println();
  M5.Lcd.println("Btn A:");
  M5.Lcd.println("Nullung");
}

void loop() {
  uint32_t now = millis();

  M5.update();

  if (M5.BtnA.wasPressed()) {
    startCalibration();
  }

  if ((now - lastSensorUpdateMs) >= SENSOR_UPDATE_MS) {
    lastSensorUpdateMs = now;

    float rawDistanceCm = 0.0f;

    if (readTofDistanceCm(rawDistanceCm)) {
      processNewDistance(rawDistanceCm, now);
    }
  }

  updateBeeper();

  if ((now - lastDisplayUpdateMs) >= DISPLAY_UPDATE_MS) {
    lastDisplayUpdateMs = now;
    updateDisplay();
  }
}
