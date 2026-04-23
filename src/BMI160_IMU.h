#ifndef BMI160_IMU_H
#define BMI160_IMU_H

#include <Arduino.h>
#include <SPI.h>

class BMI160_IMU {
public:
    BMI160_IMU(uint8_t csPin);

    void begin();
    void calibrate(int samples = 200);
    void update();

    float getRoll();
    float getPitch();
    float getYaw(); // yaw ยัง drift

private:
    uint8_t _csPin;

    // offsets
    float gx_off=0, gy_off=0, gz_off=0;
    float ax_off=0, ay_off=0, az_off=0;

    // angles
    float roll=0, pitch=0, yaw=0;

    unsigned long lastTime;

    void writeReg(uint8_t reg, uint8_t data);
    int16_t read16(uint8_t reg);
};

#endif