// Include Libraries
#include "Arduino.h"
#include "MaxMatrix.h"
#include <Wire.h>

#include <math.h>

// Pin Definitions
#define LEDMATRIX_PIN_DIN  50
#define LEDMATRIX_PIN_CLK  51
#define LEDMATRIX_PIN_CS   53

// LED Matrix and MPU setup
MaxMatrix ledMatrix(LEDMATRIX_PIN_DIN, LEDMATRIX_PIN_CS, LEDMATRIX_PIN_CLK);
MPU9250_DMP imu;

// LED Matrix Settings
const int MATRIX_SIZE = 8;
const int CENTER_X = 3;
const int CENTER_Y = 3;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize LED Matrix
  ledMatrix.init(1); // One 8x8 matrix
  ledMatrix.setIntensity(5);
  ledMatrix.clear();

  // Initialize MPU-9250
  if (imu.begin() != INV_SUCCESS) {
    Serial.println("Failed to initialize MPU-9250!");
    while (1);
  }

  imu.setSensors(INV_XYZ_ACCEL | INV_XYZ_GYRO);
  imu.setGyroFSR(2000);  // degrees/s
  imu.setAccelFSR(2);    // g
  imu.setLPF(5);         // Hz
  imu.setSampleRate(10); // Hz
  imu.enableFIFO(true);

  Serial.println("MPU-9250 Initialized");
}

void loop() {
  if (imu.fifoAvailable()) {
    if (imu.dmpUpdateFifo() == INV_SUCCESS) {
      float ax = imu.calcAccel(imu.ax);
      float ay = imu.calcAccel(imu.ay);

      float angle = atan2(ay, ax) * 180.0 / PI;
      if (angle < 0) angle += 360;

      Serial.print("Tilt Angle: ");
      Serial.println(angle);

      drawTiltLine(angle);
      delay(100);
    }
  }
}

void drawTiltLine(float angle) {
  ledMatrix.clear();

  // Convert angle to radians
  float rad = angle * PI / 180.0;

  // Direction vector
  float dx = cos(rad);
  float dy = sin(rad);

  // Draw outward from center
  for (int i = 0; i < MATRIX_SIZE; i++) {
    int x = round(CENTER_X + dx * i);
    int y = round(CENTER_Y + dy * i);

    if (x >= 0 && x < MATRIX_SIZE && y >= 0 && y < MATRIX_SIZE) {
      ledMatrix.setDot(y, x, true); // setDot(row, col)
    }
  }
}
