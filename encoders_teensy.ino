#include <Wire.h>

#define PCA9548A_ADDR  0x70   // A0 = A1 = A2 = GND
#define AS5600_ADDR    0x36
#define TOTAL_CHANNELS 8

// Teensy 4.1 Wire2: SDA2 = pin 25, SCL2 = pin 24
// Note: these pins are also Serial6 (TX6/RX6). Do not use Serial6 in this sketch.

// Returns the I2C error code (0 = OK)
uint8_t selectI2CChannel(uint8_t channel) {
  Wire2.beginTransmission(PCA9548A_ADDR);
  Wire2.write(1 << channel);
  return Wire2.endTransmission();
}

// Returns raw 12-bit angle, or 0xFFFF on failure
uint16_t readAS5600Angle() {
  Wire2.beginTransmission(AS5600_ADDR);
  Wire2.write(0x0C);  // RAW ANGLE high byte register
  if (Wire2.endTransmission(true) != 0) return 0xFFFF;

  if (Wire2.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2) return 0xFFFF;

  uint8_t high = Wire2.read();
  uint8_t low  = Wire2.read();
  return ((high & 0x0F) << 8) | low;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // wait for USB serial

  Wire2.begin();
  Wire2.setClock(100000);  // 100 kHz for reliable start-up

  Serial.println("PCA9548A + AS5600 reader (Teensy 4.1, Wire2)");

  // Startup check: is the mux there at all?
  Wire2.beginTransmission(PCA9548A_ADDR);
  uint8_t err = Wire2.endTransmission();
  Serial.print("Mux 0x70 check, code ");
  Serial.print(err);
  Serial.println(err == 0 ? " -> mux OK" : " -> MUX NOT RESPONDING");
  Serial.println("===============================================");
}

void loop() {
  for (uint8_t ch = 0; ch < TOTAL_CHANNELS; ch++) {
    Serial.print("Ch ");
    Serial.print(ch);
    Serial.print(": ");

    uint8_t muxErr = selectI2CChannel(ch);
    if (muxErr != 0) {
      Serial.print("MUX ERROR (code ");
      Serial.print(muxErr);
      Serial.println(")");
      continue;
    }

    uint16_t raw = readAS5600Angle();
    if (raw == 0xFFFF) {
      Serial.println("N/C");
    } else {
      float degrees = (raw * 360.0f) / 4096.0f;
      Serial.print("Raw: ");
      Serial.print(raw);
      Serial.print("\t| Angle: ");
      Serial.print(degrees, 2);
      Serial.println("°");
    }
  }

  Serial.println("-----------------------------------------------");
  delay(500);
}