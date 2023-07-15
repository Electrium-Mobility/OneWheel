#include "IMU.hpp"

IMU::IMU(TwoWire wire, MPU6050 *mpu)
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
  Wire.begin();
  Mpu->initialize();
  Mpu->CalibrateAccel();
  Mpu->CalibrateGyro();
  Mpu->setDLPFMode(1); // Enable Low Pass Filter
}

IMU_data *IMU::getData()
{
  (*this).updateResult();
  return currentData;
}

void IMU::updateResult()
{
  Mpu->getMotion6(
    &currentData->accX,
    &currentData->accY,
    &currentData->accZ,
    &currentData->gyroX,
    &currentData->gyroY,
    &currentData->gyroZ
  );

  // Convert accelerometer values to angles
  currentData->angleX = atan2(currentData->accY, currentData->accZ)
                        * (180 / PI);
  currentData->angleY = atan2(currentData->accX, currentData->accZ)
                        * (180 / PI);

  // Filter gyro values using the complementary filter
  currentData->angleX = alpha * 
                        (currentData->angleX + currentData->gyroX * 0.01) 
                        + (1 - alpha) * currentData->angleX;
  currentData->angleY = alpha *
                        (currentData->angleY + currentData->gyroY * 0.01) 
                        + (1 - alpha) * currentData->angleY;

}
