#include <M5StickCPlus.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <math.h>

// ================================================================
// Anpassbare Parameter
// ================================================================

// Strecken jetzt immer in mm
const float START_THRESHOLD_MM = 10.0f;
const float STOP_THRESHOLD_MM = 10.0f;
const uint32_t STOP_HOLD_MS = 400;

const uint8_t CALIBRATION_SAMPLES = 20;

// Sensor nur dort schnell, wo es wichtig ist
const uint32_t SENSOR_UPDATE_ACTIVE_MS = 40; // Kalibrierung + Messung, 25 Hz
const uint32_t SENSOR_UPDATE_WAIT_MS = 80;   // Warten auf Lift, 12.5 Hz

// Display langsamer spart Strom
const uint32_t DISPLAY_UPDATE_ACTIVE_MS = 250; // RUNNING / CALIBRATING
const uint32_t DISPLAY_UPDATE_IDLE_MS = 1000;  // IDLE / WAIT / DONE

// Akku nicht dauernd per I2C abfragen
const uint32_t BATTERY_UPDATE_MS = 5000;

const uint32_t BEEP_INTERVAL_MS = 1000;
const uint32_t BEEP_DURATION_MS = 50;

const uint32_t CONFIRM_BEEP_DURATION_MS = 80;
const uint32_t CONFIRM_BEEP_GAP_MS = 100;

const uint16_t RUN_BEEP_FREQ_HZ = 2200;
const uint16_t CONFIRM_BEEP_FREQ_HZ = 3000;

// Etwas aggressiver für mehr Akkulaufzeit
const uint32_t AUTO_SHUTOFF_MS = 90000;

// Display-Helligkeit: M5StickC PLUS meist ca. 7 bis 12
// 7 = recht dunkel, spart Akku
const uint8_t LCD_BRIGHTNESS = 15;

// ESP32 niedriger takten
const uint32_t CPU_FREQ_MHZ = 80;

// Spannungsbasierte Akku-%-Schätzung
const float BATTERY_EMPTY_V = 3.20f;
const float BATTERY_LOW_V = 3.50f;
const float BATTERY_MID_V = 3.80f;
const float BATTERY_HIGH_V = 4.00f;
const float BATTERY_FULL_V = 4.20f;

// Interner Buzzer beim M5StickC PLUS normalerweise GPIO 2
const bool BUZZER_ENABLED = true;
const int BUZZER_PIN = 2;

const uint8_t SMOOTHING_WINDOW = 5;

// Display-Rand
const int16_t DISPLAY_MARGIN_X = 8;
const int16_t DISPLAY_MARGIN_Y = 6;
const int16_t DISPLAY_LINE_SPACING = 2;

// Offscreen-Framebuffer gegen Display-Flackern
TFT_eSprite displayBuffer = TFT_eSprite(&M5.Lcd);

// Typischer I2C-Anschluss des M5StickC PLUS HAT-Ports
const int I2C_SDA_PIN = 0;
const int I2C_SCL_PIN = 26;

// Gültiger ToF-Messbereich in mm
const uint16_t MIN_VALID_DISTANCE_MM = 20;
const uint16_t MAX_VALID_DISTANCE_MM = 2000;

// ================================================================
// Zustandsmaschine
// ================================================================

enum AppState
{
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
bool tofRunning = false;
uint32_t currentTofPeriodMs = 0;

uint32_t lastSensorUpdateMs = 0;
uint32_t lastDisplayUpdateMs = 0;

bool hasDistance = false;
float currentDistanceMm = 0.0f;

bool hasZero = false;
float zeroDistanceMm = 0.0f;
float currentHeightMm = 0.0f;

// ================================================================
// Glättung: Median aus den letzten 5 gültigen Werten
// ================================================================

float smoothBuffer[SMOOTHING_WINDOW];
uint8_t smoothCount = 0;
uint8_t smoothIndex = 0;

void resetSmoothing()
{
  smoothCount = 0;
  smoothIndex = 0;
}

float medianFilter(float value)
{
  smoothBuffer[smoothIndex] = value;
  smoothIndex = (smoothIndex + 1) % SMOOTHING_WINDOW;

  if (smoothCount < SMOOTHING_WINDOW)
  {
    smoothCount++;
  }

  float values[SMOOTHING_WINDOW];

  for (uint8_t i = 0; i < smoothCount; i++)
  {
    values[i] = smoothBuffer[i];
  }

  for (uint8_t i = 0; i < smoothCount; i++)
  {
    for (uint8_t j = i + 1; j < smoothCount; j++)
    {
      if (values[j] < values[i])
      {
        float tmp = values[i];
        values[i] = values[j];
        values[j] = tmp;
      }
    }
  }

  if (smoothCount % 2 == 1)
  {
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
float calibrationSumMm = 0.0f;

// ================================================================
// Messdaten
// ================================================================

uint32_t startTimeMs = 0;
uint32_t endTimeMs = 0;
uint32_t lastAucUpdateMs = 0;
uint32_t stopZoneSinceMs = 0;

float maxHeightMm = 0.0f;
float aucMmSec = 0.0f;

float finalDurationSec = 0.0f;
float finalMaxHeightMm = 0.0f;
float finalAucMmSec = 0.0f;

// ================================================================
// Systemstatus
// ================================================================

uint32_t autoShutdownBaseMs = 0;
bool hasBatteryInfo = false;
uint8_t batteryPercent = 0;
uint32_t lastBatteryUpdateMs = 0;

// ================================================================
// Nicht-blockierender Buzzer über tone() / noTone()
// ================================================================

bool beepOn = false;
uint32_t beepOffAtMs = 0;

bool confirmSequenceActive = false;
uint8_t confirmBeepsLeft = 0;
uint32_t confirmNextStartMs = 0;

uint32_t lastRunBeepMs = 0;

void stopBeepNow()
{
  if (BUZZER_ENABLED)
  {
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
  }

  beepOn = false;
}

void startSingleBeep(uint16_t freqHz, uint32_t durationMs)
{
  if (!BUZZER_ENABLED)
  {
    return;
  }

  tone(BUZZER_PIN, freqHz);
  beepOn = true;
  beepOffAtMs = millis() + durationMs;
}

void startConfirmBeeps()
{
  stopBeepNow();

  confirmSequenceActive = true;
  confirmBeepsLeft = 2;
  confirmNextStartMs = millis();
}

void updateBeeper()
{
  uint32_t now = millis();

  if (beepOn && (int32_t)(now - beepOffAtMs) >= 0)
  {
    stopBeepNow();

    if (confirmSequenceActive)
    {
      if (confirmBeepsLeft > 0)
      {
        confirmBeepsLeft--;
      }

      confirmNextStartMs = now + CONFIRM_BEEP_GAP_MS;
    }
  }

  if (confirmSequenceActive && !beepOn)
  {
    if (confirmBeepsLeft == 0)
    {
      confirmSequenceActive = false;
    }
    else if ((int32_t)(now - confirmNextStartMs) >= 0)
    {
      startSingleBeep(CONFIRM_BEEP_FREQ_HZ, CONFIRM_BEEP_DURATION_MS);
    }

    return;
  }

  if (state == RUNNING && !beepOn)
  {
    if ((int32_t)(now - lastRunBeepMs) >= (int32_t)BEEP_INTERVAL_MS)
    {
      lastRunBeepMs = now;
      startSingleBeep(RUN_BEEP_FREQ_HZ, BEEP_DURATION_MS);
    }
  }
}

// ================================================================
// ToF-Stromsparlogik
// ================================================================

uint32_t desiredTofPeriodMs()
{
  switch (state)
  {
  case CALIBRATING:
  case RUNNING:
    return SENSOR_UPDATE_ACTIVE_MS;

  case WAIT_FOR_LIFT:
    return SENSOR_UPDATE_WAIT_MS;

  case IDLE:
  case FINISHED:
  default:
    return 0;
  }
}

void stopTofContinuous()
{
  if (!tofOk || !tofRunning)
  {
    return;
  }

  tof.stopContinuous();
  tofRunning = false;
  currentTofPeriodMs = 0;
}

void startTofContinuous(uint32_t periodMs)
{
  if (!tofOk)
  {
    return;
  }

  if (tofRunning && currentTofPeriodMs == periodMs)
  {
    return;
  }

  if (tofRunning)
  {
    tof.stopContinuous();
  }

  tof.startContinuous(periodMs);
  tofRunning = true;
  currentTofPeriodMs = periodMs;
  lastSensorUpdateMs = millis();
}

void updateTofPower()
{
  uint32_t desiredPeriodMs = desiredTofPeriodMs();

  if (desiredPeriodMs == 0)
  {
    stopTofContinuous();
    return;
  }

  startTofContinuous(desiredPeriodMs);
}

uint32_t desiredDisplayUpdateMs()
{
  switch (state)
  {
  case CALIBRATING:
  case RUNNING:
    return DISPLAY_UPDATE_ACTIVE_MS;

  case IDLE:
  case WAIT_FOR_LIFT:
  case FINISHED:
  default:
    return DISPLAY_UPDATE_IDLE_MS;
  }
}

// ================================================================
// Hilfsfunktionen
// ================================================================

void resetAutoShutdownTimer(uint32_t now)
{
  autoShutdownBaseMs = now;
}

bool isAutoShutdownArmed()
{
  return state == IDLE || state == WAIT_FOR_LIFT || state == FINISHED;
}

uint32_t getAutoShutdownRemainingMs(uint32_t now)
{
  if (!isAutoShutdownArmed())
  {
    return AUTO_SHUTOFF_MS;
  }

  uint32_t elapsed = now - autoShutdownBaseMs;

  if (elapsed >= AUTO_SHUTOFF_MS)
  {
    return 0;
  }

  return AUTO_SHUTOFF_MS - elapsed;
}

uint8_t batteryPercentFromVoltage(float voltageV)
{
  float percent = 0.0f;

  if (voltageV >= BATTERY_FULL_V)
  {
    percent = 100.0f;
  }
  else if (voltageV >= BATTERY_HIGH_V)
  {
    percent = 80.0f +
              ((voltageV - BATTERY_HIGH_V) /
               (BATTERY_FULL_V - BATTERY_HIGH_V)) *
                  20.0f;
  }
  else if (voltageV >= BATTERY_MID_V)
  {
    percent = 40.0f +
              ((voltageV - BATTERY_MID_V) /
               (BATTERY_HIGH_V - BATTERY_MID_V)) *
                  40.0f;
  }
  else if (voltageV >= BATTERY_LOW_V)
  {
    percent = 10.0f +
              ((voltageV - BATTERY_LOW_V) /
               (BATTERY_MID_V - BATTERY_LOW_V)) *
                  30.0f;
  }
  else if (voltageV >= BATTERY_EMPTY_V)
  {
    percent = ((voltageV - BATTERY_EMPTY_V) /
               (BATTERY_LOW_V - BATTERY_EMPTY_V)) *
              10.0f;
  }
  else
  {
    percent = 0.0f;
  }

  if (percent < 0.0f)
  {
    percent = 0.0f;
  }

  if (percent > 100.0f)
  {
    percent = 100.0f;
  }

  return (uint8_t)(percent + 0.5f);
}

void updateBatteryStatus(bool force = false)
{
  uint32_t now = millis();

  if (!force && (now - lastBatteryUpdateMs) < BATTERY_UPDATE_MS)
  {
    return;
  }

  lastBatteryUpdateMs = now;

  float voltageV = M5.Axp.GetBatVoltage();

  if (voltageV > 3.0f && voltageV < 5.0f)
  {
    hasBatteryInfo = true;
    batteryPercent = batteryPercentFromVoltage(voltageV);
  }
  else
  {
    hasBatteryInfo = false;
    batteryPercent = 0;
  }
}

void updateAutoShutdown(uint32_t now)
{
  if (!isAutoShutdownArmed())
  {
    return;
  }

  if ((now - autoShutdownBaseMs) < AUTO_SHUTOFF_MS)
  {
    return;
  }

  stopBeepNow();
  stopTofContinuous();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(DISPLAY_MARGIN_X, DISPLAY_MARGIN_Y);
  M5.Lcd.print("AUTO OFF");

  delay(150);
  M5.Axp.PowerOff();
}

void resetMeasurementData()
{
  startTimeMs = 0;
  endTimeMs = 0;
  lastAucUpdateMs = 0;
  stopZoneSinceMs = 0;

  maxHeightMm = 0.0f;
  aucMmSec = 0.0f;

  finalDurationSec = 0.0f;
  finalMaxHeightMm = 0.0f;
  finalAucMmSec = 0.0f;
}

void startCalibration()
{
  stopBeepNow();
  confirmSequenceActive = false;

  resetMeasurementData();
  resetSmoothing();

  hasZero = false;
  calibrationCount = 0;
  calibrationSumMm = 0.0f;

  state = CALIBRATING;
}

void startMeasurement(uint32_t now)
{
  resetAutoShutdownTimer(now);

  startTimeMs = now;
  lastAucUpdateMs = now;
  stopZoneSinceMs = 0;

  aucMmSec = 0.0f;
  maxHeightMm = currentHeightMm;

  if (maxHeightMm < 0.0f)
  {
    maxHeightMm = 0.0f;
  }

  lastRunBeepMs = now - BEEP_INTERVAL_MS;
  state = RUNNING;
}

void finishMeasurement(uint32_t now)
{
  endTimeMs = now;

  finalDurationSec = (endTimeMs - startTimeMs) / 1000.0f;
  finalMaxHeightMm = maxHeightMm;
  finalAucMmSec = aucMmSec;

  stopBeepNow();
  confirmSequenceActive = false;

  state = FINISHED;
  resetAutoShutdownTimer(now);
}

bool readTofDistanceMm(float &distanceMmOut)
{
  if (!tofOk || !tofRunning)
  {
    return false;
  }

  uint16_t distanceMm = tof.readRangeContinuousMillimeters();

  if (tof.timeoutOccurred())
  {
    return false;
  }

  if (distanceMm < MIN_VALID_DISTANCE_MM ||
      distanceMm > MAX_VALID_DISTANCE_MM)
  {
    return false;
  }

  distanceMmOut = (float)distanceMm;
  return true;
}

void processNewDistance(float rawDistanceMm, uint32_t now)
{
  currentDistanceMm = medianFilter(rawDistanceMm);
  hasDistance = true;

  if (hasZero)
  {
    currentHeightMm = currentDistanceMm - zeroDistanceMm;
  }
  else
  {
    currentHeightMm = 0.0f;
  }

  if (state == CALIBRATING)
  {
    calibrationSumMm += currentDistanceMm;
    calibrationCount++;

    if (calibrationCount >= CALIBRATION_SAMPLES)
    {
      zeroDistanceMm = calibrationSumMm / calibrationCount;
      hasZero = true;

      resetMeasurementData();

      currentHeightMm = currentDistanceMm - zeroDistanceMm;
      state = WAIT_FOR_LIFT;
      resetAutoShutdownTimer(now);

      startConfirmBeeps();
    }

    return;
  }

  if (!hasZero)
  {
    return;
  }

  if (state == WAIT_FOR_LIFT)
  {
    if (currentHeightMm >= START_THRESHOLD_MM)
    {
      startMeasurement(now);
    }

    return;
  }

  if (state == RUNNING)
  {
    uint32_t dtMs = now - lastAucUpdateMs;
    lastAucUpdateMs = now;

    float dtSec = dtMs / 1000.0f;
    float positiveHeightMm = currentHeightMm;

    if (positiveHeightMm < 0.0f)
    {
      positiveHeightMm = 0.0f;
    }

    aucMmSec += positiveHeightMm * dtSec;

    if (currentHeightMm > maxHeightMm)
    {
      maxHeightMm = currentHeightMm;
    }

    if (currentHeightMm <= STOP_THRESHOLD_MM)
    {
      if (stopZoneSinceMs == 0)
      {
        stopZoneSinceMs = now;
      }
      else if ((now - stopZoneSinceMs) >= STOP_HOLD_MS)
      {
        finishMeasurement(now);
      }
    }
    else
    {
      stopZoneSinceMs = 0;
    }

    return;
  }
}

// ================================================================
// Display mit Rand
// ================================================================

int16_t displayY = DISPLAY_MARGIN_Y;

void beginScreen()
{
  displayBuffer.fillSprite(BLACK);
  displayY = DISPLAY_MARGIN_Y;
}

void finishScreen()
{
  displayBuffer.pushSprite(0, 0);
}

void drawLine(const String &text, uint16_t color = WHITE, uint8_t size = 2)
{
  displayBuffer.setTextColor(color, BLACK);
  displayBuffer.setTextSize(size);
  displayBuffer.setCursor(DISPLAY_MARGIN_X, displayY);
  displayBuffer.print(text);

  displayY += (8 * size) + DISPLAY_LINE_SPACING;
}

void drawSpacer(uint8_t pixels = 6)
{
  displayY += pixels;
}

String valueLine(const char *label, float value, const char *unit,
                 unsigned int decimals)
{
  String line = String(label) + ": ";

  if (isnan(value))
  {
    line += "--";
  }
  else
  {
    line += String(value, decimals);
    line += " ";
    line += unit;
  }

  return line;
}

String batteryStatusText()
{
  if (!hasBatteryInfo)
  {
    return "Akku: --";
  }

  return "Akku: " + String(batteryPercent) + "%";
}

String autoOffStatusText(uint32_t now)
{
  if (!isAutoShutdownArmed())
  {
    return "Aus: pause";
  }

  uint32_t remainingSec = (getAutoShutdownRemainingMs(now) + 999) / 1000;
  return "Aus: " + String(remainingSec) + "s";
}

void drawStatusLine()
{
  uint32_t now = millis();
  updateBatteryStatus();
  drawLine(batteryStatusText() + "  " + autoOffStatusText(now), WHITE, 1);
}

void drawIdleLikeScreen(const char *title)
{
  beginScreen();

  drawLine(title, WHITE, 2);
  drawStatusLine();
  drawSpacer(4);

  if (hasDistance)
  {
    drawLine(valueLine("Distanz", currentDistanceMm, "mm", 0));
  }
  else
  {
    drawLine(valueLine("Distanz", NAN, "mm", 0));
  }

  if (hasZero)
  {
    drawLine(valueLine("Null", zeroDistanceMm, "mm", 0));
    drawLine(valueLine("Hoehe", currentHeightMm, "mm", 1));
  }
  else
  {
    drawLine(valueLine("Null", NAN, "mm", 0));
    drawLine(valueLine("Hoehe", NAN, "mm", 1));
  }

  drawSpacer(4);
  drawLine("Btn A = Nullung / neue Messung", WHITE, 1);

  if (!tofOk)
  {
    drawLine("ToF Sensor nicht gefunden!", RED, 1);
  }

  finishScreen();
}

void drawCalibrationScreen()
{
  beginScreen();

  drawLine("NULLUNG", YELLOW, 2);
  drawStatusLine();
  drawSpacer(4);

  drawLine(
      "Samples: " + String(calibrationCount) + "/" +
          String(CALIBRATION_SAMPLES),
      WHITE, 2);

  if (hasDistance)
  {
    drawLine(valueLine("Distanz", currentDistanceMm, "mm", 0));
  }
  else
  {
    drawLine(valueLine("Distanz", NAN, "mm", 0));
  }

  drawSpacer(4);
  drawLine("Gewicht ruhig halten", WHITE, 1);

  finishScreen();
}

void drawRunningScreen()
{
  uint32_t now = millis();
  float durationSec = (now - startTimeMs) / 1000.0f;

  beginScreen();

  drawLine("RUNNING", GREEN, 2);
  drawStatusLine();
  drawSpacer(4);

  drawLine(valueLine("Hoehe", currentHeightMm, "mm", 1));
  drawLine(valueLine("Max", maxHeightMm, "mm", 1));
  drawLine(valueLine("Zeit", durationSec, "s", 1));
  drawLine(valueLine("AUC", aucMmSec, "mm*s", 1));

  finishScreen();
}

void drawFinishedScreen()
{
  beginScreen();

  drawLine("DONE", CYAN, 2);
  drawStatusLine();
  drawSpacer(4);

  drawLine(valueLine("Dauer", finalDurationSec, "s", 1));
  drawLine(valueLine("Max", finalMaxHeightMm, "mm", 1));
  drawLine(valueLine("AUC", finalAucMmSec, "mm*s", 1));

  drawSpacer(4);
  drawLine("Btn A = neue Nullung / Messung", WHITE, 1);

  finishScreen();
}

void updateDisplay()
{
  switch (state)
  {
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

void setup()
{
  M5.begin();

  setCpuFrequencyMhz(CPU_FREQ_MHZ);
  M5.Axp.ScreenBreath(LCD_BRIGHTNESS);

  M5.Lcd.setRotation(3);
  displayBuffer.setColorDepth(8); // 8 Bit spart RAM und reicht fuer Text/Farben hier vollkommen aus.
  displayBuffer.createSprite(M5.Lcd.width(), M5.Lcd.height());
  displayBuffer.setTextWrap(false);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setTextSize(2);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  noTone(BUZZER_PIN);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  tof.setTimeout(50);
  tofOk = tof.init();

  if (tofOk)
  {
    tof.setMeasurementTimingBudget(33000);
    // Sensor wird jetzt nicht mehr dauerhaft gestartet.
    // Er läuft nur bei Kalibrierung, Warten auf Lift und Messung.
  }

  resetMeasurementData();
  updateBatteryStatus(true);
  resetAutoShutdownTimer(millis());

  beginScreen();
  drawLine("ToF Hold Test", WHITE, 2);
  drawStatusLine();
  drawSpacer(4);
  drawLine("Btn A:", WHITE, 2);
  drawLine("Nullung", WHITE, 2);

  if (!tofOk)
  {
    drawSpacer(4);
    drawLine("ToF Sensor nicht gefunden!", RED, 1);
  }

  finishScreen();
}

void loop()
{
  uint32_t now = millis();

  M5.update();

  if (M5.BtnA.wasPressed())
  {
    resetAutoShutdownTimer(now);
    startCalibration();
  }

  updateTofPower();

  uint32_t sensorUpdateMs = desiredTofPeriodMs();

  if (sensorUpdateMs > 0 && (now - lastSensorUpdateMs) >= sensorUpdateMs)
  {
    lastSensorUpdateMs = now;

    float rawDistanceMm = 0.0f;

    if (readTofDistanceMm(rawDistanceMm))
    {
      processNewDistance(rawDistanceMm, now);
    }
  }

  updateBeeper();
  updateAutoShutdown(now);

  if ((now - lastDisplayUpdateMs) >= desiredDisplayUpdateMs())
  {
    lastDisplayUpdateMs = now;
    updateDisplay();
  }
}
