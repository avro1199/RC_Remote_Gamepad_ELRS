#include <Arduino.h>
#include <BleGamepad.h>
#include "USB.h"
#include "USBHIDGamepad.h"
#include <Preferences.h>
#include <Wire.h>
#include <U8g2lib.h>

// =====================================================
//                 RJ AVRO CONTROLLER
// =====================================================
// Default  : USB HID Game Controller
// Special  : BLE Wireless Controller
// Calibrate: Hold SW_A + SW_D during boot
//            OR via OLED Menu → Calibration
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
// Switch Pins (original)
// =========================

#define SW_A 41
#define SW_B 42
#define SW_C_DOWN 39
#define SW_C_UP 40
#define SW_D 37

// =========================
// New OLED Nav Buttons
// btn_lu = Up
// btn_ld = Down
// btn_ru = Enter
// btn_rd = Back
// =========================

#define BTN_UP 36
#define BTN_DOWN 35
#define BTN_ENTER 45
#define BTN_BACK 47

// =========================
// ADC Settings
// =========================

#define ADC_SAMPLES 10
#define DEADZONE_DEFAULT 70
#define DEADZONE_MIN 0
#define DEADZONE_MAX 200
#define DEADZONE_STEP 5

// =========================
// Trim Settings
// =========================

#define TRIM_MIN -300
#define TRIM_MAX 300
#define TRIM_STEP 10

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
// UP / RIGHT  -> ADC INCREASES
// DOWN / LEFT -> ADC DECREASES
// =====================================================

AxisCalibration rollCal = {276, 1983, 4095};
AxisCalibration pitchCal = {343, 1998, 4095};
AxisCalibration throttleCal = {77, 1940, 3810};
AxisCalibration yawCal = {155, 1964, 4085};

Preferences prefs;

int16_t axis_min = 0;
int16_t axis_max = 32767;

// =========================
// User-adjustable settings
// =========================

int deadzone = DEADZONE_DEFAULT;
int trimRoll = 0;
int trimPitch = 0;
int trimThrottle = 0;
int trimYaw = 0;
bool invertRoll = false;
bool invertPitch = false;
bool invertThrottle = false;
bool invertYaw = false;

// =========================
// BLE / USB
// =========================

BleGamepad bleGamepad("Rj Avro Controller", "Rj Avro", 100);
USBHIDGamepad usbGamepad;
bool bleMode = false;

// =========================
// OLED — SH1106 128x64 I2C
// =========================

U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// =====================================================
//  MENU SYSTEM
// =====================================================

enum Screen
{
  SCREEN_STATUS,    // Default: status/live view
  SCREEN_MENU,      // Main menu
  SCREEN_TRIM,      // Trim sub-menu
  SCREEN_INVERT,    // Invert sub-menu
  SCREEN_DEADZONE,  // Deadzone adjust
  SCREEN_CALIBRATE, // Calibration flow
};

Screen currentScreen = SCREEN_STATUS;

// Main menu entries
const char *mainMenuItems[] = {
    "Trim Adjust",
    "Axis Invert",
    "Deadzone",
    "Calibrate",
    "< Back"};
const int MAIN_MENU_COUNT = 5;
int mainMenuSel = 0;

// Trim menu entries
const char *trimMenuItems[] = {
    "Roll",
    "Pitch",
    "Throttle",
    "Yaw",
    "< Back"};
const int TRIM_MENU_COUNT = 5;
int trimMenuSel = 0;
bool trimEditing = false;

// Invert menu entries
const char *invertMenuItems[] = {
    "Roll",
    "Pitch",
    "Throttle",
    "Yaw",
    "< Back"};
const int INVERT_MENU_COUNT = 5;
int invertMenuSel = 0;

// Status screen: cycle pages with Up/Down
// Page 0: mode + connection + sticks overview
// Page 1: raw axis values
int statusPage = 0;
#define STATUS_PAGES 2

// =========================
// Button debounce
// =========================

struct DebouncedBtn
{
  int pin;
  bool lastState;
  bool pressed; // true for ONE cycle on press
  unsigned long lastTime;
};

DebouncedBtn btnUp = {BTN_UP, false, false, 0};
DebouncedBtn btnDown = {BTN_DOWN, false, false, 0};
DebouncedBtn btnEnter = {BTN_ENTER, false, false, 0};
DebouncedBtn btnBack = {BTN_BACK, false, false, 0};

#define DEBOUNCE_MS 40

void updateBtn(DebouncedBtn &b)
{
  bool reading = !digitalRead(b.pin); // INPUT_PULLUP: LOW = pressed
  b.pressed = false;
  if (reading != b.lastState)
  {
    if (millis() - b.lastTime > DEBOUNCE_MS)
    {
      b.lastState = reading;
      b.lastTime = millis();
      if (reading)
        b.pressed = true; // rising edge
    }
  }
}

void updateAllBtns()
{
  updateBtn(btnUp);
  updateBtn(btnDown);
  updateBtn(btnEnter);
  updateBtn(btnBack);
}

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
// Axis Mapping (with trim + invert)
// =====================================================

int mapJoystick(int raw, AxisCalibration cal, int trim = 0, bool invert = false, bool throttle = false)
{
  raw += trim;

  int center = (axis_min + (axis_max - axis_min) / 2);

  if (!throttle && (abs(raw - cal.center) < deadzone))
    return center;

  int mapped;

  if (throttle)
    mapped = map(raw, cal.minimum + trim, cal.maximum + trim, axis_min, axis_max);
  else if (raw < cal.center)
    mapped = map(raw, cal.minimum + trim, cal.center - deadzone, axis_min, center);
  else
    mapped = map(raw, cal.center + deadzone, cal.maximum + trim, center, axis_max);

  if (invert)
    mapped = axis_max - (mapped - axis_min);

  return mapped;
}

// =====================================================
// BLE Startup Gesture
// =====================================================

bool shouldEnterBLEMode()
{
  int throttle = readSmoothADC(THROTTLE_PIN);
  int yaw = readSmoothADC(YAW_PIN);
  int pitch = readSmoothADC(PITCH_PIN);
  int roll = readSmoothADC(ROLL_PIN);

  bool leftStickDownRight = (throttle < 1000) && (yaw > 3200);
  bool rightStickDownLeft = (pitch < 700) && (roll < 700);

  return leftStickDownRight && rightStickDownLeft;
}

// =====================================================
// Save / Load Calibration
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

void loadCalibration()
{
  prefs.begin("rjavro", true);

  rollCal.minimum = prefs.getInt("rmin", 276);
  rollCal.center = prefs.getInt("rctr", 1983);
  rollCal.maximum = prefs.getInt("rmax", 4095);

  pitchCal.minimum = prefs.getInt("pmin", 343);
  pitchCal.center = prefs.getInt("pctr", 1998);
  pitchCal.maximum = prefs.getInt("pmax", 4095);

  throttleCal.minimum = prefs.getInt("tmin", 77);
  throttleCal.center = prefs.getInt("tctr", 1940);
  throttleCal.maximum = prefs.getInt("tmax", 3810);

  yawCal.minimum = prefs.getInt("ymin", 155);
  yawCal.center = prefs.getInt("yctr", 1964);
  yawCal.maximum = prefs.getInt("ymax", 4085);

  prefs.end();
}

// =====================================================
// Save / Load User Settings
// =====================================================

void saveSettings()
{
  prefs.begin("rjavro_s", false);

  prefs.putInt("dz", deadzone);
  prefs.putInt("tr_r", trimRoll);
  prefs.putInt("tr_p", trimPitch);
  prefs.putInt("tr_t", trimThrottle);
  prefs.putInt("tr_y", trimYaw);
  prefs.putBool("inv_r", invertRoll);
  prefs.putBool("inv_p", invertPitch);
  prefs.putBool("inv_t", invertThrottle);
  prefs.putBool("inv_y", invertYaw);

  prefs.end();
}

void loadSettings()
{
  prefs.begin("rjavro_s", true);

  deadzone = prefs.getInt("dz", DEADZONE_DEFAULT);
  trimRoll = prefs.getInt("tr_r", 0);
  trimPitch = prefs.getInt("tr_p", 0);
  trimThrottle = prefs.getInt("tr_t", 0);
  trimYaw = prefs.getInt("tr_y", 0);
  invertRoll = prefs.getBool("inv_r", false);
  invertPitch = prefs.getBool("inv_p", false);
  invertThrottle = prefs.getBool("inv_t", false);
  invertYaw = prefs.getBool("inv_y", false);

  prefs.end();
}

// =====================================================
// Calibration — with OLED feedback
// =====================================================

void runCalibration()
{
  Serial.println("================================");
  Serial.println("CALIBRATION STARTED");
  Serial.println("MOVE ALL STICKS FULLY");
  Serial.println("================================");

  // Phase 1 — move all sticks
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 10, "CALIBRATION");
  display.drawStr(0, 24, "Move ALL sticks");
  display.drawStr(0, 36, "to extremes");
  display.drawStr(0, 54, "10 seconds...");
  display.sendBuffer();

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
    int roll = readSmoothADC(ROLL_PIN);
    int pitch = readSmoothADC(PITCH_PIN);
    int throttle = readSmoothADC(THROTTLE_PIN);
    int yaw = readSmoothADC(YAW_PIN);

    rollCal.minimum = min(rollCal.minimum, roll);
    rollCal.maximum = max(rollCal.maximum, roll);
    pitchCal.minimum = min(pitchCal.minimum, pitch);
    pitchCal.maximum = max(pitchCal.maximum, pitch);
    throttleCal.minimum = min(throttleCal.minimum, throttle);
    throttleCal.maximum = max(throttleCal.maximum, throttle);
    yawCal.minimum = min(yawCal.minimum, yaw);
    yawCal.maximum = max(yawCal.maximum, yaw);

    // Countdown
    int remaining = 10 - (int)((millis() - startTime) / 1000);
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tr);
    display.drawStr(0, 10, "CALIBRATION");
    display.drawStr(0, 24, "Move ALL sticks");
    display.drawStr(0, 36, "to extremes");

    char buf[20];
    snprintf(buf, sizeof(buf), "Time left: %ds", remaining);
    display.drawStr(0, 54, buf);
    display.sendBuffer();
  }

  // Phase 2 — center sticks
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 10, "CALIBRATION");
  display.drawStr(0, 24, "Center ALL sticks");
  display.drawStr(0, 36, "then wait...");
  display.sendBuffer();

  Serial.println("......Almost done!......");
  Serial.println("Center all the sticks and wait...");
  delay(3000);

  rollCal.center = readSmoothADC(ROLL_PIN);
  pitchCal.center = readSmoothADC(PITCH_PIN);
  throttleCal.center = readSmoothADC(THROTTLE_PIN);
  yawCal.center = readSmoothADC(YAW_PIN);

  saveCalibration();
  Serial.println("CALIBRATION SAVED");

  // Done screen
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 10, "CALIBRATION");
  display.drawStr(0, 26, "COMPLETE!");
  display.drawStr(0, 42, "Values saved.");
  display.sendBuffer();
  delay(2000);

  currentScreen = SCREEN_STATUS;
}

// =====================================================
// OLED Draw Helpers
// =====================================================

// Draw a generic vertical list menu
void drawListMenu(const char *title, const char **items, int count, int selected)
{
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);

  // Title bar
  display.drawBox(0, 0, 128, 12);
  display.setDrawColor(0);
  display.drawStr(2, 10, title);
  display.setDrawColor(1);

  // Show up to 4 items, scroll so selected is visible
  int startIdx = 0;
  if (selected >= 4)
    startIdx = selected - 3;

  for (int i = 0; i < 4 && (startIdx + i) < count; i++)
  {
    int idx = startIdx + i;
    int y = 14 + i * 13;

    if (idx == selected)
    {
      display.drawBox(0, y - 1, 128, 12);
      display.setDrawColor(0);
    }

    display.drawStr(4, y + 9, items[idx]);
    display.setDrawColor(1);
  }

  display.sendBuffer();
}

// =====================================================
// Status Screen — Page 0: Mode & sticks
// =====================================================

void drawStatusPage0(int roll, int pitch, int throttle, int yaw, bool connected)
{
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);

  // Header bar
  display.drawBox(0, 0, 128, 10);
  display.setDrawColor(0);
  if (bleMode)
  {
    display.drawStr(33, 8, connected ? "BLE CONNECTED" : "BLE SEARCHING...");
  }
  else
  {
    display.drawStr(46, 8, "USB Mode");
  }
  display.setDrawColor(1);

  // Stick visual: left
  // Box 20x20 at (4, 14)
  display.drawFrame(4, 14, 33, 33);
  // yaw = x, throttle = y for left stick
  int lx = map(yaw, axis_min, axis_max, 4, 36);
  int ly = map(throttle, axis_max, axis_min, 14, 46); // invert Y for display
  lx = constrain(lx, 4, 36);
  ly = constrain(ly, 14, 46);
  display.drawDisc(lx, ly, 3);

  // Stick visual: right
  display.drawFrame(91, 14, 33, 33);
  int rx = map(roll, axis_min, axis_max, 91, 123);
  int ry = map(pitch, axis_max, axis_min, 14, 46);
  rx = constrain(rx, 91, 123);
  ry = constrain(ry, 14, 46);
  display.drawDisc(rx, ry, 3);

  // Labels
  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(3, 59, "YAW/THR");
  display.drawStr(91, 59, "ROL/PCH");

  // Center info
  display.setFont(u8g2_font_5x7_tr);
  char buf[16];
  snprintf(buf, sizeof(buf), "DZ:%d", deadzone);
  display.drawStr(53, 22, buf);
  display.drawStr(53, 34, invertRoll ? "R:INV" : "R:NRM");
  display.drawStr(53, 46, invertThrottle ? "T:INV" : "T:NRM");

  // Page indicator
  display.drawStr(58, 59, "1/2");

  display.sendBuffer();
}

// =====================================================
// Status Screen — Page 1: Raw axis values + trim
// =====================================================

void drawStatusPage1(int rollRaw, int pitchRaw, int throttleRaw, int yawRaw)
{
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);

  // Header
  display.drawBox(0, 0, 128, 10);
  display.setDrawColor(0);
  display.drawStr(2, 8, "AXIS VALUES  RAW");
  display.setDrawColor(1);

  char buf[32];
  snprintf(buf, sizeof(buf), "ROL:%4d  Tr:%+d", rollRaw, trimRoll);
  display.drawStr(0, 21, buf);
  snprintf(buf, sizeof(buf), "PCH:%4d  Tr:%+d", pitchRaw, trimPitch);
  display.drawStr(0, 32, buf);
  snprintf(buf, sizeof(buf), "THR:%4d  Tr:%+d", throttleRaw, trimThrottle);
  display.drawStr(0, 43, buf);
  snprintf(buf, sizeof(buf), "YAW:%4d  Tr:%+d", yawRaw, trimYaw);
  display.drawStr(0, 54, buf);

  // Page indicator
  display.drawStr(58, 63, "2/2");

  display.sendBuffer();
}

// =====================================================
// Trim editing screen
// =====================================================

void drawTrimEdit(int axisIndex)
{
  const char *names[] = {"Roll", "Pitch", "Throttle", "Yaw"};
  int *trims[] = {&trimRoll, &trimPitch, &trimThrottle, &trimYaw};

  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);

  display.drawBox(0, 0, 128, 12);
  display.setDrawColor(0);
  display.drawStr(2, 10, "TRIM ADJUST");
  display.setDrawColor(1);

  display.drawStr(4, 28, names[axisIndex]);

  char buf[20];
  snprintf(buf, sizeof(buf), "Value: %+d", *trims[axisIndex]);
  display.drawStr(4, 44, buf);

  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(0, 58, "UP/DN:adjust  ENT:save");

  display.sendBuffer();
}

// =====================================================
// Deadzone editing screen
// =====================================================

void drawDeadzoneEdit()
{
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);

  display.drawBox(0, 0, 128, 12);
  display.setDrawColor(0);
  display.drawStr(2, 10, "DEADZONE");
  display.setDrawColor(1);

  char buf[20];
  snprintf(buf, sizeof(buf), "Value: %d", deadzone);
  display.drawStr(4, 34, buf);

  // Visual bar
  int barW = map(deadzone, DEADZONE_MIN, DEADZONE_MAX, 0, 118);
  display.drawFrame(4, 40, 120, 8);
  display.drawBox(4, 40, barW, 8);

  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(0, 58, "UP/DN:adjust  ENT:save");

  display.sendBuffer();
}

// =====================================================
// Invert screen — show toggles inline
// =====================================================

void drawInvertMenu()
{
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);

  display.drawBox(0, 0, 128, 12);
  display.setDrawColor(0);
  display.drawStr(2, 10, "AXIS INVERT");
  display.setDrawColor(1);

  const char *labels[] = {"Roll", "Pitch", "Throttle", "Yaw", "< Back"};
  bool *inverts[] = {&invertRoll, &invertPitch, &invertThrottle, &invertYaw, nullptr};

  int startIdx = 0;
  if (invertMenuSel >= 4)
    startIdx = invertMenuSel - 3;

  for (int i = 0; i < 4 && (startIdx + i) < INVERT_MENU_COUNT; i++)
  {
    int idx = startIdx + i;
    int y = 14 + i * 13;

    if (idx == invertMenuSel)
    {
      display.drawBox(0, y - 1, 128, 12);
      display.setDrawColor(0);
    }

    display.drawStr(4, y + 9, labels[idx]);

    if (inverts[idx] != nullptr)
    {
      display.drawStr(90, y + 9, *inverts[idx] ? "[ON]" : "[OFF]");
    }

    display.setDrawColor(1);
  }

  display.sendBuffer();
}

// =====================================================
// Setup
// =====================================================

void setup()
{
  // delay(500);
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Original buttons
  pinMode(SW_A, INPUT_PULLUP);
  pinMode(SW_B, INPUT_PULLUP);
  pinMode(SW_C_DOWN, INPUT_PULLUP);
  pinMode(SW_C_UP, INPUT_PULLUP);
  pinMode(SW_D, INPUT_PULLUP);

  // New nav buttons
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);

  // OLED init
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_10x20_tr);
  display.drawStr(30, 28, "RJ AVRO");
  display.drawStr(18, 44, "Controller");
  display.sendBuffer();
  delay(1000);

  loadCalibration();
  loadSettings();

  delay(400);

  // Calibration Mode (hold SW_A + SW_D at boot)
  if (!digitalRead(SW_A) && !digitalRead(SW_D))
  {
    runCalibration();
  }

  // BLE Mode Check
  bleMode = shouldEnterBLEMode();

  if (bleMode)
  {
    Serial.println("BLE MODE ENABLED");
    bleGamepad.begin();

    display.clearBuffer();
    display.setFont(u8g2_font_10x20_tr);
    display.drawStr(30, 28, "BLE MODE");
    display.setFont(u8g2_font_6x10_tr);
    display.drawStr(30, 45, "Searching...");
    display.sendBuffer();
  }
  else
  {
    Serial.println("USB MODE ENABLED");
    axis_max = 127;

    USB.productName("Rj Avro Controller");
    USB.manufacturerName("Rj Avro");
    USB.serialNumber("001");
    USB.begin();
    usbGamepad.begin();

    display.clearBuffer();
    display.setFont(u8g2_font_10x20_tr);
    display.drawStr(30, 28, "USB MODE");
    display.setFont(u8g2_font_6x10_tr);
    display.drawStr(30, 45, "(: Ready! :)");
    display.sendBuffer();
  }

  delay(1000);
}

// =====================================================
// Menu Logic
// =====================================================

bool deadzoneEditing = false;

void handleMenu()
{
  // ---- STATUS SCREEN ----
  if (currentScreen == SCREEN_STATUS)
  {
    if (btnEnter.pressed)
    {
      currentScreen = SCREEN_MENU;
      mainMenuSel = 0;
    }
    else if (btnUp.pressed)
    {
      statusPage = (statusPage - 1 + STATUS_PAGES) % STATUS_PAGES;
    }
    else if (btnDown.pressed)
    {
      statusPage = (statusPage + 1) % STATUS_PAGES;
    }
    return;
  }

  // ---- MAIN MENU ----
  if (currentScreen == SCREEN_MENU)
  {
    if (btnUp.pressed)
      mainMenuSel = (mainMenuSel - 1 + MAIN_MENU_COUNT) % MAIN_MENU_COUNT;
    if (btnDown.pressed)
      mainMenuSel = (mainMenuSel + 1) % MAIN_MENU_COUNT;
    if (btnBack.pressed)
    {
      currentScreen = SCREEN_STATUS;
      return;
    }

    if (btnEnter.pressed)
    {
      switch (mainMenuSel)
      {
      case 0:
        currentScreen = SCREEN_TRIM;
        trimMenuSel = 0;
        trimEditing = false;
        break;
      case 1:
        currentScreen = SCREEN_INVERT;
        invertMenuSel = 0;
        break;
      case 2:
        currentScreen = SCREEN_DEADZONE;
        deadzoneEditing = false;
        break;
      case 3:
        runCalibration();
        break;
      case 4:
        currentScreen = SCREEN_STATUS;
        break;
      }
    }
    return;
  }

  // ---- TRIM MENU ----
  if (currentScreen == SCREEN_TRIM)
  {
    if (!trimEditing)
    {
      if (btnUp.pressed)
        trimMenuSel = (trimMenuSel - 1 + TRIM_MENU_COUNT) % TRIM_MENU_COUNT;
      if (btnDown.pressed)
        trimMenuSel = (trimMenuSel + 1) % TRIM_MENU_COUNT;
      if (btnBack.pressed || (trimMenuSel == 4 && btnEnter.pressed))
      {
        currentScreen = SCREEN_MENU;
        return;
      }
      if (btnEnter.pressed && trimMenuSel < 4)
      {
        trimEditing = true;
      }
    }
    else
    {
      // Editing a trim value
      int *trims[] = {&trimRoll, &trimPitch, &trimThrottle, &trimYaw};
      if (btnUp.pressed)
        *trims[trimMenuSel] = constrain(*trims[trimMenuSel] + TRIM_STEP, TRIM_MIN, TRIM_MAX);
      if (btnDown.pressed)
        *trims[trimMenuSel] = constrain(*trims[trimMenuSel] - TRIM_STEP, TRIM_MIN, TRIM_MAX);
      if (btnEnter.pressed || btnBack.pressed)
      {
        saveSettings();
        trimEditing = false;
      }
    }
    return;
  }

  // ---- INVERT MENU ----
  if (currentScreen == SCREEN_INVERT)
  {
    if (btnUp.pressed)
      invertMenuSel = (invertMenuSel - 1 + INVERT_MENU_COUNT) % INVERT_MENU_COUNT;
    if (btnDown.pressed)
      invertMenuSel = (invertMenuSel + 1) % INVERT_MENU_COUNT;
    if (btnBack.pressed || (invertMenuSel == 4 && btnEnter.pressed))
    {
      saveSettings();
      currentScreen = SCREEN_MENU;
      return;
    }
    if (btnEnter.pressed && invertMenuSel < 4)
    {
      bool *inverts[] = {&invertRoll, &invertPitch, &invertThrottle, &invertYaw};
      *inverts[invertMenuSel] = !*inverts[invertMenuSel];
      saveSettings();
    }
    return;
  }

  // ---- DEADZONE ----
  if (currentScreen == SCREEN_DEADZONE)
  {
    if (btnUp.pressed)
      deadzone = constrain(deadzone + DEADZONE_STEP, DEADZONE_MIN, DEADZONE_MAX);
    if (btnDown.pressed)
      deadzone = constrain(deadzone - DEADZONE_STEP, DEADZONE_MIN, DEADZONE_MAX);
    if (btnEnter.pressed || btnBack.pressed)
    {
      saveSettings();
      currentScreen = SCREEN_MENU;
    }
    return;
  }
}

// =====================================================
// OLED Draw Dispatch
// =====================================================

void drawOLED(int roll, int pitch, int throttle, int yaw,
              int rollRaw, int pitchRaw, int throttleRaw, int yawRaw,
              bool connected)
{
  switch (currentScreen)
  {
  case SCREEN_STATUS:
    if (statusPage == 0)
      drawStatusPage0(roll, pitch, throttle, yaw, connected);
    else
      drawStatusPage1(rollRaw, pitchRaw, throttleRaw, yawRaw);
    break;

  case SCREEN_MENU:
    drawListMenu("  MAIN MENU", mainMenuItems, MAIN_MENU_COUNT, mainMenuSel);
    break;

  case SCREEN_TRIM:
    if (trimEditing)
      drawTrimEdit(trimMenuSel);
    else
      drawListMenu("  TRIM ADJUST", trimMenuItems, TRIM_MENU_COUNT, trimMenuSel);
    break;

  case SCREEN_INVERT:
    drawInvertMenu();
    break;

  case SCREEN_DEADZONE:
    drawDeadzoneEdit();
    break;

  case SCREEN_CALIBRATE:
    // handled inside runCalibration()
    break;
  }
}

// =====================================================
// Main Loop
// =====================================================

void loop()
{
  // Update nav buttons
  updateAllBtns();

  // Handle menu input
  handleMenu();

  // Read ADC
  int rollRaw = readSmoothADC(ROLL_PIN);
  int pitchRaw = readSmoothADC(PITCH_PIN);
  int throttleRaw = readSmoothADC(THROTTLE_PIN);
  int yawRaw = readSmoothADC(YAW_PIN);

  // Convert to HID values
  int roll = mapJoystick(rollRaw, rollCal, trimRoll, invertRoll, false);
  int pitch = mapJoystick(pitchRaw, pitchCal, trimPitch, invertPitch, false);
  int throttle = mapJoystick(throttleRaw, throttleCal, trimThrottle, invertThrottle, true);
  int yaw = mapJoystick(yawRaw, yawCal, trimYaw, invertYaw, false);

  // Read original switches
  bool swA = !digitalRead(SW_A);
  bool swB = !digitalRead(SW_B);
  bool swCDown = !digitalRead(SW_C_DOWN);
  bool swCUp = !digitalRead(SW_C_UP);
  bool swD = !digitalRead(SW_D);

  if (swCDown && swCUp)
  {
    swCDown = false;
    swCUp = false;
  }

  bool connected = false;

  // ========== BLE MODE ==========
  if (bleMode)
  {
    connected = bleGamepad.isConnected();
    if (connected)
    {
      bleGamepad.setAxes(roll, pitch, 0, yaw, throttle, 0, 0, 0);

      if (swA)
        bleGamepad.press(BUTTON_1);
      else
        bleGamepad.release(BUTTON_1);
      if (swB)
        bleGamepad.press(BUTTON_2);
      else
        bleGamepad.release(BUTTON_2);
      if (swCDown)
        bleGamepad.press(BUTTON_3);
      else
        bleGamepad.release(BUTTON_3);
      if (swCUp)
        bleGamepad.press(BUTTON_4);
      else
        bleGamepad.release(BUTTON_4);
      if (swD)
        bleGamepad.press(BUTTON_5);
      else
        bleGamepad.release(BUTTON_5);
    }
  }

  // ========== USB MODE ==========
  else
  {
    connected = true;
    usbGamepad.leftStick(roll, pitch);
    usbGamepad.rightStick(yaw, throttle);

    if (swA)
      usbGamepad.pressButton(0);
    else
      usbGamepad.releaseButton(0);
    if (swB)
      usbGamepad.pressButton(1);
    else
      usbGamepad.releaseButton(1);
    if (swCDown)
      usbGamepad.pressButton(2);
    else
      usbGamepad.releaseButton(2);
    if (swCUp)
      usbGamepad.pressButton(3);
    else
      usbGamepad.releaseButton(3);
    if (swD)
      usbGamepad.pressButton(4);
    else
      usbGamepad.releaseButton(4);
  }

  // Update OLED
  drawOLED(roll, pitch, throttle, yaw,
           rollRaw, pitchRaw, throttleRaw, yawRaw,
           connected);

  delay(3);
}
