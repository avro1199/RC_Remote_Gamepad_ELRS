#include <Arduino.h>
#include <BleGamepad.h>
#include "USB.h"
#include "USBHIDGamepad.h"
#include <Preferences.h>

// =====================================================
//                 RJ AVRO CONTROLLER
// =====================================================
// Default  : USB HID Game Controller
// Special  : BLE Wireless Controller
// Calibrate: Hold SW_A + SW_D during boot
// BLE Boot : Both sticks DOWN-INWARD during boot
// =====================================================
// AXIS MAPPING FOR FPV SIMULATORS (MODE 2):
// - Left Stick X = Yaw
// - Left Stick Y = Throttle
// - Right Stick X = Roll
// - Right Stick Y = Pitch
// =====================================================

// =========================
// Analog Pins
// =========================

#define ROLL_PIN 7
#define PITCH_PIN 4
#define THROTTLE_PIN 2
#define YAW_PIN 10

// =========================
// Switch Pins
// =========================
#define SW_A 41
#define SW_B 42
#define SW_C_DOWN 39
#define SW_C_UP 40
#define SW_D 37

#define btn_lu 36
#define btn_ld 35
#define btn_ru 45
#define btn_rd 47

// =========================
// ADC Settings
// =========================

#define ADC_SAMPLES   10
#define DEADZONE      70

// =========================
// Calibration Structure
// =========================

struct AxisCalibration
{
  int minimum;
  int center;
  int maximum;
};

// =====================================================
// ADC Directions
// UP / RIGHT    -> ADC INCREASES
// DOWN / LEFT   -> ADC DECREASES
// =====================================================

AxisCalibration rollCal     = {276, 1983, 4095};
AxisCalibration pitchCal    = {343, 1998, 4095};
AxisCalibration throttleCal = {77, 1940, 3810};
AxisCalibration yawCal      = {155, 1964, 4085};

Preferences prefs;

int16_t axis_min = 0;
int16_t axis_max = 32767;

// =========================
// BLE Gamepad
// =========================

BleGamepad bleGamepad(
  "Rj Avro Controller",
  "Rj Avro",
  100
);

// =========================
// USB HID Gamepad
// =========================

USBHIDGamepad usbGamepad;

bool bleMode = false;

// =====================================================
// Smooth ADC Read
// =====================================================

int readSmoothADC(int pin)
{
  long total = 0;

  for (int i = 0; i < ADC_SAMPLES; i++)
  {
    total += analogRead(pin);
    delayMicroseconds(120);
  }

  return total / ADC_SAMPLES;
}

// =====================================================
// Axis Mapping
// =====================================================

int mapJoystick(int raw, AxisCalibration cal, bool throttle = false)
{
  raw = constrain(raw, cal.minimum, cal.maximum);

  // Deadzone for non-throttle axes
  if (!throttle && (abs(raw - cal.center) < DEADZONE))
  {
    return (axis_min + (axis_max - axis_min) / 2); // return center value
  }
  return map(raw, cal.minimum, cal.maximum, axis_min, axis_max); // return map(raw, cal.minimum, cal.maximum, -32767, 32767);
}

// =====================================================
// BLE Startup Gesture
// =====================================================
// Left Stick  : Bottom Right
// Right Stick : Bottom Left
// =====================================================

bool shouldEnterBLEMode()
{
  int throttle = readSmoothADC(THROTTLE_PIN);
  int yaw      = readSmoothADC(YAW_PIN);
  int pitch    = readSmoothADC(PITCH_PIN);
  int roll     = readSmoothADC(ROLL_PIN);

  bool leftStickDownRight =
    (throttle < 1000) &&   // Throttle down
    (yaw > 3200);         // Yaw right

  bool rightStickDownLeft =
    (pitch < 700) &&      // Pitch down
    (roll < 700);         // Roll left

  return leftStickDownRight && rightStickDownLeft;
}

// =====================================================
// Save Calibration
// =====================================================

void saveCalibration()
{
  prefs.begin("rjavro", false);

  prefs.putInt("rmin", rollCal.minimum);
  prefs.putInt("rctr", rollCal.center);
  prefs.putInt("rmax", rollCal.maximum);

  prefs.putInt("pmin", pitchCal.minimum);
  prefs.putInt("pctr", pitchCal.center);
  prefs.putInt("pmax", pitchCal.maximum);

  prefs.putInt("tmin", throttleCal.minimum);
  prefs.putInt("tctr", throttleCal.center);
  prefs.putInt("tmax", throttleCal.maximum);

  prefs.putInt("ymin", yawCal.minimum);
  prefs.putInt("yctr", yawCal.center);
  prefs.putInt("ymax", yawCal.maximum);

  prefs.end();
}

// =====================================================
// Load Calibration
// =====================================================

void loadCalibration()
{
  prefs.begin("rjavro", true);

  rollCal.minimum = prefs.getInt("rmin", 276);
  rollCal.center  = prefs.getInt("rctr", 1983);
  rollCal.maximum = prefs.getInt("rmax", 4095);

  pitchCal.minimum = prefs.getInt("pmin", 343);
  pitchCal.center  = prefs.getInt("pctr", 1998);
  pitchCal.maximum = prefs.getInt("pmax", 4095);

  throttleCal.minimum = prefs.getInt("tmin", 77);
  throttleCal.center  = prefs.getInt("tctr", 1940);
  throttleCal.maximum = prefs.getInt("tmax", 3810);

  yawCal.minimum = prefs.getInt("ymin", 155);
  yawCal.center  = prefs.getInt("yctr", 1964);
  yawCal.maximum = prefs.getInt("ymax", 4085);

  prefs.end();
}

// =====================================================
// Calibration Mode
// =====================================================

void runCalibration()
{
  Serial.println("================================");
  Serial.println("CALIBRATION STARTED");
  Serial.println("MOVE ALL STICKS FULLY");
  Serial.println("================================");

  rollCal.minimum = 4095;
  rollCal.maximum = 0;

  pitchCal.minimum = 4095;
  pitchCal.maximum = 0;

  throttleCal.minimum = 4095;
  throttleCal.maximum = 0;

  yawCal.minimum = 4095;
  yawCal.maximum = 0;

  unsigned long startTime = millis();

  while (millis() - startTime < 10000)
  {
    int roll     = readSmoothADC(ROLL_PIN);
    int pitch    = readSmoothADC(PITCH_PIN);
    int throttle = readSmoothADC(THROTTLE_PIN);
    int yaw      = readSmoothADC(YAW_PIN);

    rollCal.minimum = min(rollCal.minimum, roll);
    rollCal.maximum = max(rollCal.maximum, roll);

    pitchCal.minimum = min(pitchCal.minimum, pitch);
    pitchCal.maximum = max(pitchCal.maximum, pitch);

    throttleCal.minimum = min(throttleCal.minimum, throttle);
    throttleCal.maximum = max(throttleCal.maximum, throttle);

    yawCal.minimum = min(yawCal.minimum, yaw);
    yawCal.maximum = max(yawCal.maximum, yaw);
  }

  Serial.println("......Almost done!......");
  Serial.println("Center all the sticks and wait...");

  delay(3000);

  rollCal.center = readSmoothADC(ROLL_PIN);
  pitchCal.center = readSmoothADC(PITCH_PIN);
  throttleCal.center = readSmoothADC(THROTTLE_PIN);
  yawCal.center = readSmoothADC(YAW_PIN);

  saveCalibration();

  Serial.println("CALIBRATION SAVED");
}

// =====================================================
// Setup
// =====================================================

void setup()
{
  delay(1500);
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(SW_A, INPUT_PULLUP);
  pinMode(SW_B, INPUT_PULLUP);
  pinMode(SW_C_DOWN, INPUT_PULLUP);
  pinMode(SW_C_UP, INPUT_PULLUP);
  pinMode(SW_D, INPUT_PULLUP);

  pinMode(btn_lu, INPUT_PULLUP);
  pinMode(btn_ld, INPUT_PULLUP);
  pinMode(btn_ru, INPUT_PULLUP);
  pinMode(btn_rd, INPUT_PULLUP);

  loadCalibration();

  delay(400);

  // Calibration Mode (hold SW_A + SW_D)
  if (!digitalRead(SW_A) && !digitalRead(SW_D))
  {
    runCalibration();
  }

  // BLE Mode Check
  bleMode = shouldEnterBLEMode();

  if (bleMode)
  {
    Serial.println("BLE MODE ENABLED");;
    bleGamepad.begin();
  }
  else
  {
    Serial.println("USB MODE ENABLED");
    // axis_min = -127;
    axis_max = 127;

    USB.productName("Rj Avro Controller");
    USB.manufacturerName("Rj Avro");
    USB.serialNumber("001");
    USB.begin();
    usbGamepad.begin();
  }
}

// =====================================================
// Main Loop
// =====================================================

void loop()
{
  // Read ADC
  int rollRaw     = readSmoothADC(ROLL_PIN);
  int pitchRaw    = readSmoothADC(PITCH_PIN);
  int throttleRaw = readSmoothADC(THROTTLE_PIN);
  int yawRaw      = readSmoothADC(YAW_PIN);

  // Convert to HID values (signed 16-bit)
  int roll     = mapJoystick(rollRaw, rollCal);
  int pitch    = mapJoystick(pitchRaw, pitchCal);
  int throttle = mapJoystick(throttleRaw, throttleCal, true);
  int yaw      = mapJoystick(yawRaw, yawCal);

  // Read switches (inverted because INPUT_PULLUP)
  bool swA      = !digitalRead(SW_A);
  bool swB      = !digitalRead(SW_B);
  bool swCDown  = !digitalRead(SW_C_DOWN);
  bool swCUp    = !digitalRead(SW_C_UP);
  bool swD      = !digitalRead(SW_D);

  // Three-position switch SW_C: up/down mutually exclusive, center = both off
  // (Already handled by hardware, but we ensure consistent behavior)
  if (swCDown && swCUp) {
    // If both active (should not happen), default to center
    swCDown = false;
    swCUp = false;
  }

  // ========== BLE MODE ==========
  if (bleMode)
  {
    if (bleGamepad.isConnected())
    {
      // Axis order for BleGamepad: setAxes(x, y, z, rx, ry, rz, slider1, slider2)
      // Mode 2 mapping:
      //   x (left X) = yaw
      //   y (left Y) = throttle
      //   rx (right X) = roll
      //   ry (right Y) = pitch
      bleGamepad.setAxes(roll, pitch, 0, yaw, throttle, 0, 0, 0);

      // Buttons
      if (swA) bleGamepad.press(BUTTON_1);
      else     bleGamepad.release(BUTTON_1);

      if (swB) bleGamepad.press(BUTTON_2);
      else     bleGamepad.release(BUTTON_2);

      if (swCDown) bleGamepad.press(BUTTON_3);
      else         bleGamepad.release(BUTTON_3);

      if (swCUp) bleGamepad.press(BUTTON_4);
      else       bleGamepad.release(BUTTON_4);

      if (swD) bleGamepad.press(BUTTON_5);
      else     bleGamepad.release(BUTTON_5);
    }
  }

  // ========== USB MODE ==========
  else
  {
    // USBHIDGamepad: leftStick(x, y), rightStick(x, y)
    // Mode 2 mapping:
    //   leftStick = (yaw, throttle)
    //   rightStick = (roll, pitch)
    usbGamepad.leftStick(roll, pitch);
    usbGamepad.rightStick(yaw, throttle);

    // Buttons (0-indexed)
    if (swA) usbGamepad.pressButton(0);
    else     usbGamepad.releaseButton(0);

    if (swB) usbGamepad.pressButton(1);
    else     usbGamepad.releaseButton(1);

    if (swCDown) usbGamepad.pressButton(2);
    else         usbGamepad.releaseButton(2);

    if (swCUp) usbGamepad.pressButton(3);
    else       usbGamepad.releaseButton(3);

    if (swD) usbGamepad.pressButton(4);
    else     usbGamepad.releaseButton(4);
  }

  delay(3);  // Small delay for stability
}