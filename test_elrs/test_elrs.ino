#define ELRS_RX_PIN 15  
#define ELRS_TX_PIN 16  
#define ELRS_BAUD 420000 

// Array to store 16 channels (Standard RC values: 1000 to 2000)
uint16_t channels[16]; 

// CRSF specific constants
#define CRSF_ADDRESS_TX           0xEE
#define CRSF_FRAMELEN_RC_CHANNELS 24   // 1 byte type + 22 bytes data + 1 byte CRC
#define CRSF_TYPE_RC_CHANNELS     0x16

// Fast CRC8 implementation optimized for microcontrollers
uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= ptr[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0xD5; // CRSF uses 0xD5 polynomial
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void sendCrsfChannels() {
  uint8_t frame[26];
  
  // 1. Set headers
  frame[0] = CRSF_ADDRESS_TX;
  frame[1] = CRSF_FRAMELEN_RC_CHANNELS;
  frame[2] = CRSF_TYPE_RC_CHANNELS;

  // 2. Map standard 1000-2000us PWM values to CRSF 172-1811 range
  uint16_t crsfChannels[16];
  for (int i = 0; i < 16; i++) {
    // Basic scaling formula: (pwm - 1000) * (1639 / 1000) + 172
    crsfChannels[i] = map(channels[i], 1000, 2000, 172, 1811);
  }

  // 3. Pack 16 channels into 22 payload bytes (11 bits each)
  frame[3]  = (uint8_t)(crsfChannels[0]);
  frame[4]  = (uint8_t)((crsfChannels[0] >> 8)  | (crsfChannels[1] << 3));
  frame[5]  = (uint8_t)((crsfChannels[1] >> 5)  | (crsfChannels[2] << 6));
  frame[6]  = (uint8_t)(crsfChannels[2] >> 2);
  frame[7]  = (uint8_t)((crsfChannels[2] >> 10) | (crsfChannels[3] << 1));
  frame[8]  = (uint8_t)((crsfChannels[3] >> 7)  | (crsfChannels[4] << 4));
  frame[9]  = (uint8_t)((crsfChannels[4] >> 4)  | (crsfChannels[5] << 7));
  frame[10] = (uint8_t)(crsfChannels[5] >> 1);
  frame[11] = (uint8_t)((crsfChannels[5] >> 9)  | (uint16_t)crsfChannels[6] << 2);
  frame[12] = (uint8_t)((crsfChannels[6] >> 6)  | (uint16_t)crsfChannels[7] << 5);
  frame[13] = (uint8_t)(crsfChannels[7] >> 3);
  frame[14] = (uint8_t)(crsfChannels[8]);
  frame[15] = (uint8_t)((crsfChannels[8] >> 8)  | (crsfChannels[9] << 3));
  frame[16] = (uint8_t)((crsfChannels[9] >> 5)  | (crsfChannels[10] << 6));
  frame[17] = (uint8_t)(crsfChannels[10] >> 2);
  frame[18] = (uint8_t)((crsfChannels[10] >> 10) | (crsfChannels[11] << 1));
  frame[19] = (uint8_t)((crsfChannels[11] >> 7)  | (crsfChannels[12] << 4));
  frame[20] = (uint8_t)((crsfChannels[12] >> 4)  | (crsfChannels[13] << 7));
  frame[21] = (uint8_t)(crsfChannels[13] >> 1);
  frame[22] = (uint8_t)((crsfChannels[13] >> 9)  | (uint16_t)crsfChannels[14] << 2);
  frame[23] = (uint8_t)((crsfChannels[14] >> 6)  | (uint16_t)crsfChannels[15] << 5);
  frame[24] = (uint8_t)(crsfChannels[15] >> 3);

  // 4. Calculate CRC over Type (frame[2]) up to end of data (frame[24])
  frame[25] = crsf_crc8(&frame[2], 23);

  // 5. Blast frame to ELRS TX
  Serial1.write(frame, 26);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(ELRS_BAUD, SERIAL_8N1, ELRS_RX_PIN, ELRS_TX_PIN);

  // Initialize test channel array values to center stick defaults (1500us)
  for (int i = 0; i < 16; i++) {
    channels[i] = 1500; 
  }
}

void loop() {
  // Update your channels array here with joystick/sensor data
  // Example: channels[0] = analogRead(A0 mapped to 1000-2000);

  // Send updates non-blocking every 10ms (100Hz transmission frequency)
  static unsigned long lastTxTime = 0;
  if (millis() - lastTxTime >= 10) {
    lastTxTime = millis();
    sendCrsfChannels();
  }
}