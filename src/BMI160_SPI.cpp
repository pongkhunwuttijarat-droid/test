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

// void BMI160_IMU::update() {
//     unsigned long now = millis();
//     float dt = (now - lastTime)/1000.0;
//     lastTime = now;

//     // ---- read raw และเก็บลงตัวแปรของ Class โดยตรง (ไม่ต้องประกาศ float ซ้ำ) ----
//     gx = (read16(0x0C)-gx_off)/16.4;
//     gy = (read16(0x0E)-gy_off)/16.4;
//     gz = (read16(0x10)-gz_off)/16.4;

//     ax = (read16(0x12)-ax_off)/16384.0;
//     ay = (read16(0x14)-ay_off)/16384.0;
//     az = (read16(0x16)-az_off)/16384.0;

//     // ---- accel angle ----
//     float roll_acc  = atan2(ay, az) * 180 / PI;
//     float pitch_acc = atan2(-ax, sqrt(ay*ay + az*az)) * 180 / PI;

//     // ---- complementary filter ----
//     float alpha = 0.98;

//     roll  = alpha*(roll  + gx*dt) + (1-alpha)*roll_acc;
//     pitch = alpha*(pitch + gy*dt) + (1-alpha)*pitch_acc;

//     // yaw ไม่มี accel ช่วย → drift
//     yaw += gz * dt;
// }
struct Quaternion {
    float w = 1, x = 0, y = 0, z = 0;
} q;

void BMI160_IMU::update() {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    // ---- Read raw ----
    gx = (read16(0x0C) - gx_off) / 16.4f * DEG_TO_RAD;
    gy = (read16(0x0E) - gy_off) / 16.4f * DEG_TO_RAD;
    gz = (read16(0x10) - gz_off) / 16.4f * DEG_TO_RAD;

    ax = (read16(0x12) - ax_off) / 16384.0f;
    ay = (read16(0x14) - ay_off) / 16384.0f;
    az = (read16(0x16) - az_off) / 16384.0f;

    // ---- Normalize accel (gravity) ----
    float norm = sqrt(ax*ax + ay*ay + az*az);
    if (norm == 0) return;
    ax /= norm;
    ay /= norm;
    az /= norm;

    // ---- Madgwick filter ----
    float beta = 0.1f; // tuning parameter

    float q1 = q.w, q2 = q.x, q3 = q.y, q4 = q.z;

    // Gradient descent step
    float f1 = 2*(q2*q4 - q1*q3) - ax;
    float f2 = 2*(q1*q2 + q3*q4) - ay;
    float f3 = 2*(0.5f - q2*q2 - q3*q3) - az;

    float J_11or24 = 2*q3;
    float J_12or23 = 2*q4;
    float J_13or22 = 2*q1;
    float J_14or21 = 2*q2;
    float J_32 = 2*J_14or21;
    float J_33 = 2*J_11or24;

    float grad1 = J_14or21*f2 - J_11or24*f1;
    float grad2 = J_12or23*f1 + J_13or22*f2 - J_32*f3;
    float grad3 = J_12or23*f2 - J_33*f3 - J_13or22*f1;
    float grad4 = J_14or21*f1 + J_11or24*f2;

    norm = sqrt(grad1*grad1 + grad2*grad2 + grad3*grad3 + grad4*grad4);
    grad1 /= norm;
    grad2 /= norm;
    grad3 /= norm;
    grad4 /= norm;

    // Quaternion derivative
    float qDot1 = 0.5f * (-q2*gx - q3*gy - q4*gz) - beta * grad1;
    float qDot2 = 0.5f * ( q1*gx + q3*gz - q4*gy) - beta * grad2;
    float qDot3 = 0.5f * ( q1*gy - q2*gz + q4*gx) - beta * grad3;
    float qDot4 = 0.5f * ( q1*gz + q2*gy - q3*gx) - beta * grad4;

    // Integrate
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    q4 += qDot4 * dt;

    // Normalize quaternion
    norm = sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    q.w = q1 / norm;
    q.x = q2 / norm;
    q.y = q3 / norm;
    q.z = q4 / norm;

    // ---- Convert to Euler ----
    roll  = atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y)) * RAD_TO_DEG;
    pitch = asin(2*(q.w*q.y - q.z*q.x)) * RAD_TO_DEG;
    yaw   = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)) * RAD_TO_DEG;

    // ---- Runtime gyro bias correction (only when still) ----
    float acc_mag = sqrt(ax*ax + ay*ay + az*az);

    if (fabs(acc_mag - 1.0f) < 0.05f &&
        fabs(gx) < 0.05f &&
        fabs(gy) < 0.05f &&
        fabs(gz) < 0.05f) {

        float alpha = 0.001f;
        gx_off = (1 - alpha)*gx_off + alpha*(read16(0x0C));
        gy_off = (1 - alpha)*gy_off + alpha*(read16(0x0E));
        gz_off = (1 - alpha)*gz_off + alpha*(read16(0x10));
    }
}
unsigned long BMI160_IMU::getLastTimestamp() {
    return lastTime/1000;
}

// ไม่จำเป็นต้องเพิ่ม getter ใน .cpp ถ้าใส่ไว้ใน .h แล้ว (Inline)
// แต่ถ้าอยากเอาไว้ที่นี่ ก็สามารถทำได้ครับ

float BMI160_IMU::getRoll(){ return roll; }
float BMI160_IMU::getPitch(){ return pitch; }
float BMI160_IMU::getYaw(){ return yaw; }
float BMI160_IMU::getAccX() { return ax; }
float BMI160_IMU::getAccY() { return ay; }
float BMI160_IMU::getAccZ() { return az; }
float BMI160_IMU::getGyroX() { return gx; }
float BMI160_IMU::getGyroY() { return gy; }
float BMI160_IMU::getGyroZ() { return gz; }


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