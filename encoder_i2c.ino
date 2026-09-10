#include <Wire.h>

#define I2C_SDA 21
#define I2C_SCL 22

const uint8_t encoderAddress = 0x36;
const uint8_t angleRegister = 0x0C;  // AS5600 RAW ANGLE register

void setup() {
  Serial.begin(115200);

  if (!Wire.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("I2C initialization failed!");
    while (true) {}
  }
}

void loop() {
  // Point the AS5600 at its RAW ANGLE register
  Wire.beginTransmission(encoderAddress);
  Wire.write(angleRegister);

  if (Wire.endTransmission(false) != 0) {
    Serial.println("Encoder not found!");
    delay(100);
    return;
  }

  Wire.requestFrom(encoderAddress, (uint8_t)2);

  if (Wire.available() == 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();

    uint16_t rawAngle = ((uint16_t)highByte << 8) | lowByte;
    rawAngle &= 0x0FFF; // AS5600 uses 12 bits

    float degrees = rawAngle * 360.0f / 4096.0f;

    Serial.printf("Raw: %u | Angle: %.2f deg\n", rawAngle, degrees);
  } else {
    Serial.println("Failed to read from encoder!");
  }

  delay(100);
}