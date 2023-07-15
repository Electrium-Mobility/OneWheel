#include "Arduino.h"
#include <Wire.h>
#include <MPU6050.h>

struct IMU_data{
    int16_t accX{0}, accY{0}, accZ{0};
    int16_t gyroX{0}, gyroY{0}, gyroZ{0};
    int16_t angleX{0}, angleY{0};
};

class IMU
{
  private:
    const float alpha = 0.98;
    TwoWire *Wire;
    MPU6050 *Mpu;

    IMU_data* currentData;

    void init();
    
  public:
    IMU(TwoWire *wire, MPU6050 *mpu);
    ~IMU();

    IMU_data *getData();
    void updateResult();
};