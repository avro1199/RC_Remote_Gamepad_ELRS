#include <Arduino.h>

#ifndef ARDUINO_USB_MODE
#error "This ESP32 SoC has no Native USB interface"
#elif ARDUINO_USB_MODE == 1
#warning "Select USB Device mode (TinyUSB), not Host mode."
void setup() {}
void loop() {}
#else

#include "USB.h"
#include "USBHIDGamepad.h"

// ================= USER CONFIG =================
static const int IBUS_RX_PIN = 14;
static const int IBUS_TX_PIN = -1;
static const uint32_t IBUS_BAUD = 115200;
static const uint32_t FAILSAFE_MS = 200;
// ===============================================

HardwareSerial IBusSerial(1);
USBHIDGamepad Gamepad;

static uint16_t ch[14];
static uint32_t lastGoodFrameMs = 0;

// ---------------- iBUS PARSER ----------------
static bool readIbusFrame(uint16_t outCh[14]) {
  static uint8_t buf[32];

  while (IBusSerial.available() > 0) {
    int b = IBusSerial.peek();
    if (b < 0) return false;
    if ((uint8_t)b == 0x20) break;
    IBusSerial.read();
  }

  if (IBusSerial.available() < 32) return false;

  for (int i = 0; i < 32; i++) {
    int v = IBusSerial.read();
    if (v < 0) return false;
    buf[i] = (uint8_t)v;
  }

  if (buf[0] != 0x20 || buf[1] != 0x40) return false;

  uint16_t sum = 0;
  for (int i = 0; i < 30; i++) sum += buf[i];

  uint16_t rxCk = (uint16_t)buf[30] | ((uint16_t)buf[31] << 8);
  uint16_t calcCk = (uint16_t)(0xFFFF - sum);

  if (rxCk != calcCk) return false;

  for (int c = 0; c < 14; c++) {
    int idx = 2 + c * 2;
    outCh[c] = (uint16_t)buf[idx] | ((uint16_t)buf[idx + 1] << 8);
  }

  return true;
}

static inline uint16_t clampU16(uint16_t v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// --------- AXIS MAPPING: 0 → 127 ONLY ---------
static int8_t mapAxis0_127(int channel) {
  uint16_t us = clampU16(ch[channel], 1000, 2000);
  uint32_t val = (uint32_t)(us - 1000) * 127u / 1000u;
  if (val > 127) val = 127;
  return (int8_t)val;  // stays 0..127
}

// Switch helper (kept)
static bool swHigh(uint16_t us, uint16_t thr = 1600) { return us >= thr; }

void setup() {
  IBusSerial.begin(IBUS_BAUD, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);

  Gamepad.begin();
  USB.begin();

  lastGoodFrameMs = millis();
}

void loop() {
  uint16_t temp[14];

  if (readIbusFrame(temp)) {
    for (int i = 0; i < 14; i++) ch[i] = temp[i];
    lastGoodFrameMs = millis();
  }

  bool failsafe = (millis() - lastGoodFrameMs) > FAILSAFE_MS;

  int8_t roll     = failsafe ? 0 : mapAxis0_127(0); // CH1
  int8_t pitch    = failsafe ? 0 : mapAxis0_127(1); // CH2
  int8_t throttle = failsafe ? 0 : mapAxis0_127(2); // CH3 (NOT reversed)
  int8_t yaw      = failsafe ? 0 : mapAxis0_127(3); // CH4

  Gamepad.leftStick(roll, pitch);
  Gamepad.rightStick(yaw, throttle);

  // Buttons from switches (same intent as before)
  // CH5..CH8 => A,B,X,Y
  if (!failsafe && swHigh(ch[4])) Gamepad.pressButton(BUTTON_A); else Gamepad.releaseButton(BUTTON_A);
  if (!failsafe && swHigh(ch[5])) Gamepad.pressButton(BUTTON_B); else Gamepad.releaseButton(BUTTON_B);
  if (!failsafe && swHigh(ch[6])) Gamepad.pressButton(BUTTON_X); else Gamepad.releaseButton(BUTTON_X);
  if (!failsafe && swHigh(ch[7])) Gamepad.pressButton(BUTTON_Y); else Gamepad.releaseButton(BUTTON_Y);

  delay(5);
}

#endif