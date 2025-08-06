#include "Arduino.h"
#include "MaxMatrix.h"
#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>

// LED Matrix Pins
#define LEDMATRIX_PIN_DIN  50
#define LEDMATRIX_PIN_CLK  51
#define LEDMATRIX_PIN_CS   53

MaxMatrix ledMatrix(LEDMATRIX_PIN_DIN, LEDMATRIX_PIN_CS, LEDMATRIX_PIN_CLK);
MPU9250_DMP imu;

// Matrix center
const int MATRIX_SIZE = 8;
const int cx = 3;
const int cy = 3;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  ledMatrix.init(1); // One 8x8 matrix
  ledMatrix.setIntensity(5);
  ledMatrix.clear();

  if (imu.begin() != INV_SUCCESS) {
    Serial.println("Failed to initialize MPU-9250!");
    while (1);
  }

  imu.setSensors(INV_XYZ_ACCEL | INV_XYZ_GYRO);
  imu.setGyroFSR(2000); // degrees/s
  imu.setAccelFSR(2);   // g
  imu.setLPF(5);        // Hz
  imu.setSampleRate(10); // Hz
  imu.enableFIFO(true);
}

void loop() {
  if (imu.fifoAvailable()) {
    if (imu.dmpUpdateFifo() == INV_SUCCESS) {
      float ax = imu.calcAccel(imu.ax);
      float ay = imu.calcAccel(imu.ay);

      float angle = atan2(ay, ax) * 180.0 / PI;
      if (angle < 0) angle += 360;

      Serial.print("Angle: ");
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

  // Compute vector
  float dx = cos(rad);
  float dy = sin(rad);

  // Draw from center outward
  for (int i = 0; i < MATRIX_SIZE; i++) {
    int x = round(cx + dx * i);
    int y = round(cy + dy * i);
    if (x >= 0 && x < MATRIX_SIZE && y >= 0 && y < MATRIX_SIZE) {
      ledMatrix.setDot(y, x, true); // setDot(row, col)
    }
  }
}
