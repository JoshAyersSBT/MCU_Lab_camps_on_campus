#include <Wire.h>
#include "MaxMatrix.h"
#include <math.h>

// LED Matrix Pins
#define LEDMATRIX_PIN_DIN  50
#define LEDMATRIX_PIN_CLK  51
#define LEDMATRIX_PIN_CS   53
MaxMatrix ledMatrix(LEDMATRIX_PIN_DIN, LEDMATRIX_PIN_CS, LEDMATRIX_PIN_CLK);

// MPU-9250 Constants
#define MPU9250_ADDR 0x68  // AD0 = LOW
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

// Matrix center
const int MATRIX_SIZE = 8;
const int CENTER_X = 3;
const int CENTER_Y = 3;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize LED Matrix
  ledMatrix.init(1);
  ledMatrix.setIntensity(5);
  ledMatrix.clear();

  // Initialize MPU-9250 (wake from sleep)
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  Serial.println("System Initialized. Reading YZ Tilt Angle...");
}

void loop() {
  int16_t ay, az;
  readAccelYZ(ay, az);

  // Calculate pitch from YZ plane
  float angle = atan2((float)ay, (float)az) * 180.0 / PI;
  if (angle < 0) angle += 360;

  Serial.print("Tilt angle (YZ plane): ");
  Serial.print(angle);
  Serial.println("°");

  drawTiltLine(angle);
  delay(100);
}

void readAccelYZ(int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 6, true);

  Wire.read(); Wire.read(); // Skip ax
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
}

void drawTiltLine(float angle) {
  ledMatrix.clear();

  float rad = angle * PI / 180.0;
  float dx = cos(rad);
  float dy = sin(rad);

  float x = CENTER_X + 0.5;
  float y = CENTER_Y + 0.5;

  // Draw forward direction
  for (float i = 0; i < MATRIX_SIZE; i += 0.5) {
    int xi = (int)(x + dx * i);
    int yi = (int)(y + dy * i);
    if (xi >= 0 && xi < MATRIX_SIZE && yi >= 0 && yi < MATRIX_SIZE)
      ledMatrix.setDot(xi, yi, true);
  }

  // Draw backward direction
  for (float i = 0; i < MATRIX_SIZE; i += 0.5) {
    int xi = (int)(x - dx * i);
    int yi = (int)(y - dy * i);
    if (xi >= 0 && xi < MATRIX_SIZE && yi >= 0 && yi < MATRIX_SIZE)
      ledMatrix.setDot(xi, yi, true);
  }
}
