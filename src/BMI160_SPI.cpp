#include "BMI160_IMU.h"
#include <math.h>

BMI160_IMU::BMI160_IMU(uint8_t csPin) {
    _csPin = csPin;
}

void BMI160_IMU::begin() {
    SPI.begin(18,19,23,_csPin);
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    delay(100);

    writeReg(0x7E, 0x11); // accel
    delay(50);
    writeReg(0x7E, 0x15); // gyro
    delay(50);

    lastTime = millis();
}

void BMI160_IMU::calibrate(int samples) {
    for(int i=0;i<samples;i++){
        gx_off += read16(0x0C);
        gy_off += read16(0x0E);
        gz_off += read16(0x10);

        ax_off += read16(0x12);
        ay_off += read16(0x14);
        az_off += read16(0x16);

        delay(5);
    }

    gx_off/=samples; gy_off/=samples; gz_off/=samples;
    ax_off/=samples; ay_off/=samples; az_off/=samples;
}

void BMI160_IMU::update() {
    unsigned long now = millis();
    float dt = (now - lastTime)/1000.0;
    lastTime = now;

    // ---- read raw ----
    float gx = (read16(0x0C)-gx_off)/16.4;
    float gy = (read16(0x0E)-gy_off)/16.4;
    float gz = (read16(0x10)-gz_off)/16.4;

    float ax = (read16(0x12)-ax_off)/16384.0;
    float ay = (read16(0x14)-ay_off)/16384.0;
    float az = (read16(0x16)-az_off)/16384.0;

    // ---- accel angle ----
    float roll_acc  = atan2(ay, az) * 180 / PI;
    float pitch_acc = atan2(-ax, sqrt(ay*ay + az*az)) * 180 / PI;

    // ---- complementary filter ----
    float alpha = 0.98;

    roll  = alpha*(roll  + gx*dt) + (1-alpha)*roll_acc;
    pitch = alpha*(pitch + gy*dt) + (1-alpha)*pitch_acc;

    // yaw ไม่มี accel ช่วย → drift
    yaw += gz * dt;
}

float BMI160_IMU::getRoll(){ return roll; }
float BMI160_IMU::getPitch(){ return pitch; }
float BMI160_IMU::getYaw(){ return yaw; }

// low level
void BMI160_IMU::writeReg(uint8_t reg, uint8_t data){
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(data);
    digitalWrite(_csPin, HIGH);
}

int16_t BMI160_IMU::read16(uint8_t reg){
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80);
    uint8_t l = SPI.transfer(0x00);
    uint8_t h = SPI.transfer(0x00);
    digitalWrite(_csPin, HIGH);
    return (int16_t)(h<<8 | l);
}