#include <Wire.h>

#define PCA9548A_ADDR  0x70  // Default address of PCA9548A
#define AS5600_ADDR    0x36  // Fixed address of AS5600 encoder
#define TOTAL_CHANNELS 8     // Channels 0 to 7

#define SDA_PIN 21
#define SCL_PIN 22

// Select a single channel (0 to 7) on the PCA9548A
void selectI2CChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Read 12-bit raw angle from the active channel's AS5600 sensor
uint16_t readAS5600Angle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C); // RAW ANGLE register high byte
  if (Wire.endTransmission(true) != 0) {
    return 0xFFFF; // Communication error or sensor disconnected
  }

  if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) == 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte  = Wire.read();
    return ((highByte & 0x0F) << 8) | lowByte;
  }

  return 0xFFFF; // Timeout or read failure
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Set a 50ms timeout so disconnected channels do not block execution
  Wire.setTimeOut(50);

  Serial.println("PCA9548A — 8-Channel AS5600 Reader Initialized.");
  Serial.println("===============================================");
}

void loop() {
  for (uint8_t ch = 0; ch < TOTAL_CHANNELS; ch++) {
    selectI2CChannel(ch);

    uint16_t rawAngle = readAS5600Angle();

    Serial.print("Ch ");
    Serial.print(ch);
    Serial.print(": ");

    if (rawAngle == 0xFFFF) {
      Serial.println("N/C");
    } else {
      float degrees = (rawAngle * 360.0) / 4096.0;
      Serial.print("Raw: ");
      Serial.print(rawAngle);
      Serial.print("\t| Angle: ");
      Serial.print(degrees, 2);
      Serial.println("°");
    }
  }

  Serial.println("-----------------------------------------------");
  delay(500); // Reading interval
}