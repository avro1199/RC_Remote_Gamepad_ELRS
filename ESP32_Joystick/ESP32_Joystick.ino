#include <Arduino.h>

#ifndef ARDUINO_USB_MODE
#error "Rashed gave me a copy ESP32S3"
#elif ARDUINO_USB_MODE == 1
#warning "Nissha settings diye game khela jabe na, just a dummy sketch"
void setup() {}
void loop() {}
#else

#include "USB.h"
#include "USBHIDGamepad.h"

static const int IBUS_RX_PIN = 14;
static const int IBUS_TX_PIN = -1; // not used
static const uint32_t IBUS_BAUD = 115200;

HardwareSerial IBusSerial(1);
USBHIDGamepad Gamepad;

static uint16_t ch[14] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
static uint32_t lastGoodFrameMs = 0;
static const uint32_t FAILSAFE_MS = 200;

// ---- iBUS frame parser (32 bytes) ----
static bool readIbusFrame(uint16_t outCh[14])
{
  static uint8_t buf[32];

  // Sync to 0x20 (length byte)
  while (IBusSerial.available() > 0)
  {
    int b = IBusSerial.peek();
    if (b < 0)
      return false;
    if ((uint8_t)b == 0x20)
      break;
    IBusSerial.read();
  }

  if (IBusSerial.available() < 32)
    return false;

  for (int i = 0; i < 32; i++)
  {
    int v = IBusSerial.read();
    if (v < 0)
      return false;
    buf[i] = (uint8_t)v;
  }

  if (buf[0] != 0x20 || buf[1] != 0x40)
    return false;

  // checksum: 0xFFFF - sum(first 30 bytes)
  uint16_t sum = 0;
  for (int i = 0; i < 30; i++)
    sum += buf[i];
  uint16_t rxCk = (uint16_t)buf[30] | ((uint16_t)buf[31] << 8);
  uint16_t calcCk = (uint16_t)(0xFFFF - sum);
  if (rxCk != calcCk)
    return false;

  // decode 14 channels from byte 2 (little-endian)
  for (int c = 0; c < 14; c++)
  {
    int idx = 2 + c * 2;
    outCh[c] = (uint16_t)buf[idx] | ((uint16_t)buf[idx + 1] << 8);
  }
  return true;
}

static inline uint16_t clampU16(uint16_t v, uint16_t lo, uint16_t hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

// Map 1000..2000 to -127..127 (int8 axis)
static int8_t mapAxis(uint16_t us, bool invert = false, uint16_t deadband_us = 12)
{
  us = clampU16(us, 1000, 2000);
  const int32_t center = 1500;
  int32_t d = (int32_t)us - center;

  if (d > -(int32_t)deadband_us && d < (int32_t)deadband_us)
    d = 0;

  int32_t val = (d * 127) / 500;
  if (val < -127)
    val = -127;
  if (val > 127)
    val = 127;

  if (invert)
    val = -val;
  return (int8_t)val;
}

static bool swHigh(uint16_t us, uint16_t thr = 1600) { return us >= thr; }

void setup()
{
  // Don’t start Serial unless you really need it (keeps USB HID cleaner on some setups)

  // iBus UART
  IBusSerial.begin(IBUS_BAUD, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);

  // USB HID Gamepad
  Gamepad.begin();
  USB.begin();

  lastGoodFrameMs = millis();
}

void loop()
{
  uint16_t temp[14];

  if (readIbusFrame(temp))
  {
    for (int i = 0; i < 14; i++)
      ch[i] = temp[i];
    lastGoodFrameMs = millis();
  }

  bool failsafe = (millis() - lastGoodFrameMs) > FAILSAFE_MS;

  // FlySky typical: CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw
  int8_t roll = failsafe ? 0 : mapAxis(ch[0], false);
  int8_t pitch = failsafe ? 0 : mapAxis(ch[1], true);       // invert for sims
  int8_t throttle = failsafe ? -127 : mapAxis(ch[2], true); // invert so stick up -> +127; failsafe low
  int8_t yaw = failsafe ? 0 : mapAxis(ch[3], false);

  // Sticks
  Gamepad.leftStick(roll, pitch);
  Gamepad.rightStick(yaw, throttle);

  // Buttons from switches (edit which channels you use)
  // CH5..CH8 => A,B,X,Y
  if (!failsafe && swHigh(ch[4]))
    Gamepad.pressButton(BUTTON_A);
  else
    Gamepad.releaseButton(BUTTON_A);
  if (!failsafe && swHigh(ch[5]))
    Gamepad.pressButton(BUTTON_B);
  else
    Gamepad.releaseButton(BUTTON_B);
  if (!failsafe && swHigh(ch[6]))
    Gamepad.pressButton(BUTTON_X);
  else
    Gamepad.releaseButton(BUTTON_X);
  if (!failsafe && swHigh(ch[7]))
    Gamepad.pressButton(BUTTON_Y);
  else
    Gamepad.releaseButton(BUTTON_Y);

  // Update rate ~125 Hz
  delay(8);
}
#endif