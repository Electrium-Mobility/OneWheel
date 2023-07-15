#include "IMU.hpp"

IMU::IMU(TwoWire *wire, MPU6050 *mpu)
{
  (*this).Wire = wire;
  (*this).Mpu = mpu;

  init();
}

IMU::~IMU()
{
  delete currentData;
}

void IMU::init()
{   
  currentData = new IMU_data();
  Mpu.initialize();
  Mpu.CalibrateAccel();
  Mpu.CalibrateGyro();
  Mpu.setDLPFMode(1); // Enable Low Pass Filter
}

IMU_Data *IMU::getData()
{
  (*this).updateResult();
  return currentData;
}

void IMU::updateResult()
{
  Mpu.getMotion6(
    &currentData->accx,
    &currentData->accY,
    &currentData->accZ,
    &currentData->gyroX,
    &currentData->gyroY,
    &currentData->gyroZ
  );

  // Convert accelerometer values to angles
  currentData->angleX = atan2(accY, accZ) * (180 / PI);
  currentData->angleY = atan2(accX, accZ) * (180 / PI);

  // Filter gyro values using the complementary filter
  currentData->angleX = alpha * (angleX + gyroX * 0.01) 
                          + (1 - alpha) * angleX;
  currentData->angleY = alpha * (angleY + gyroY * 0.01) 
                          + (1 - alpha) * angleY;

}
