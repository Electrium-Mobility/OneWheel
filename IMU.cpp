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

  MahonyAHRSupdateIMU(currentData->gyroX, currentData->gyroY,
    currentData->gyroZ, currentData->accX, currentData->accY,
    currentData->accZ);
  
  // https://nescacademy.nasa.gov/review/downloadfile.php?file=IntroductiontoQuaternionsPart2of2V2.pptx&id=58&distr=Public

  const float RAD2DEG = 180 / M_PI;

  // yaw (z axis)
  // pitch (y axis)
  // roll (x axis)

  currentData->angleZ = -atan2(2.0f * (q1 * q2 + q0 * q3),
    q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * RAD2DEG;
  
  currentData->angleY = asin(2.0f * (q1 * q3 - q0 * q2)) * RAD2DEG;

  currentData->angleX = atan2(2.0f * (q0 * q1 + q2 * q3),
    q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * RAD2DEG;
  

}
