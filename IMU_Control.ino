#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t angleX, angleY;
const float alpha = 0.98;  // Complementary filter constant

void setup() {
  Wire.begin();
  Serial.begin(9600);
  mpu.initialize();
  mpu.CalibrateAccel();
  mpu.CalibrateGyro();
  mpu.setDLPFMode(1);  // Enable low-pass filter
}

void loop() {
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

  // Convert accelerometer values to angles
  angleX = atan2(accY, accZ) * (180 / PI);
  angleY = atan2(accX, accZ) * (180 / PI);

  // Filter gyro values using the complementary filter
  angleX = alpha * (angleX + gyroX * 0.01) + (1 - alpha) * angleX;
  angleY = alpha * (angleY + gyroY * 0.01) + (1 - alpha) * angleY;

  Serial.print("Filtered Angle X: ");
  Serial.println(angleX);
  Serial.print("   Filtered Angle Y: ");
  Serial.println(angleY);

  delay(50);  // Adjust the delay as needed

}
