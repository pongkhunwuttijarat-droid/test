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

    // ---- read raw และเก็บลงตัวแปรของ Class โดยตรง (ไม่ต้องประกาศ float ซ้ำ) ----
    gx = (read16(0x0C)-gx_off)/16.4;
    gy = (read16(0x0E)-gy_off)/16.4;
    gz = (read16(0x10)-gz_off)/16.4;

    ax = (read16(0x12)-ax_off)/16384.0;
    ay = (read16(0x14)-ay_off)/16384.0;
    az = (read16(0x16)-az_off)/16384.0;

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

// void BMI160_IMU::calibrate(int samples) {
//     for(int i=0;i<samples;i++){
//         gx_off += read16(0x0C);
//         gy_off += read16(0x0E);
//         gz_off += read16(0x10);

//         ax_off += read16(0x12);
//         ay_off += read16(0x14);
//         az_off += read16(0x16);

//         delay(5);
//     }

//     gx_off/=samples; gy_off/=samples; gz_off/=samples;
//     ax_off/=samples; ay_off/=samples; az_off/=samples;
// }

// void BMI160_IMU::update() {
//     unsigned long now = micros(); // Use micros for higher precision
//     float dt = (now - lastTime) / 1000000.0;
//     lastTime = now;

//     // 2. Burst Read: Read 12 bytes (6 axes * 2 bytes) in one go
//     int16_t rawData[6];
//     digitalWrite(_csPin, LOW);
//     SPI.transfer(0x0C | 0x80); // Start reading from 0x0C
//     for(int i = 0; i < 6; i++) {
//         uint8_t l = SPI.transfer(0x00);
//         uint8_t h = SPI.transfer(0x00);
//         rawData[i] = (int16_t)(h << 8 | l);
//     }
//     digitalWrite(_csPin, HIGH);

//     // 3. Process with scaling
//     gx = (rawData[0] - gx_off) / GYRO_SCALE;
//     gy = (rawData[1] - gy_off) 
//     / GYRO_SCALE;
//     gz = (rawData[2] - gz_off) / GYRO_SCALE;

//     ax = (rawData[3] - ax_off) / ACCEL_SCALE;
//     ay = (rawData[4] - ay_off) / ACCEL_SCALE;
//     az = (rawData[5] - az_off) / ACCEL_SCALE;

//     // 4. Complementary Filter (Simplified)
//     float roll_acc  = atan2(ay, az) * RAD_TO_DEG;
//     float pitch_acc = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;

//     float alpha = 0.98; // Adjust based on your vibration environment
    
//     // Use the filter on the integrated value
//     roll  = alpha * (roll + gx * dt) + (1.0 - alpha) * roll_acc;
//     pitch = alpha * (pitch + gy * dt) + (1.0 - alpha) * pitch_acc;
    
//     // Yaw remains relative; recognize this will drift
//     yaw += gz * dt;
// }

void BMI160_IMU::calibrate(int samples) {
    // 1. Reset variables to ensure a clean slate
    int64_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int64_t ax_sum = 0, ay_sum = 0, az_sum = 0;

    // 2. Stabilization delay: Give the sensor time to settle 
    // after the user lets go of the device.
    delay(1000); 

    for(int i = 0; i < samples; i++) {
        gx_sum += read16(0x0C);
        gy_sum += read16(0x0E);
        gz_sum += read16(0x10);

        ax_sum += read16(0x12);
        ay_sum += read16(0x14);
        az_sum += read16(0x16);

        delay(2); // Small delay to prevent reading the exact same data point
    }

    // 3. Calculate averages using 64-bit precision
    gx_off = (float)gx_sum / samples;
    gy_off = (float)gy_sum / samples;
    gz_off = (float)gz_sum / samples;

    ax_off = (float)ax_sum / samples;
    ay_off = (float)ay_sum / samples;

    // 4. Critical Z-Axis handling:
    // If the sensor is flat, it expects 16384 (1g). 
    // We only want to remove the *bias*, not the gravity component.
    float az_avg = (float)az_sum / samples;
    az_off = az_avg - 16384.0; 
}

// 1. Define configuration constants to improve readability and reliability
const float GYRO_SCALE = 16.4;  // For +/- 2000 deg/s (ensure you set this in init!)
const float ACCEL_SCALE = 16384.0; // For +/- 2g

void BMI160_IMU::update() {
    unsigned long now = micros(); // Use micros for higher precision
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;

    // 2. Burst Read: Read 12 bytes (6 axes * 2 bytes) in one go
    int16_t rawData[6];
    digitalWrite(_csPin, LOW);
    SPI.transfer(0x0C | 0x80); // Start reading from 0x0C
    for(int i = 0; i < 6; i++) {
        uint8_t l = SPI.transfer(0x00);
        uint8_t h = SPI.transfer(0x00);
        rawData[i] = (int16_t)(h << 8 | l);
    }
    digitalWrite(_csPin, HIGH);

    // 3. Process with scaling
    gx = (rawData[0] - gx_off) / GYRO_SCALE;
    gy = (rawData[1] - gy_off) 
    / GYRO_SCALE;
    gz = (rawData[2] - gz_off) / GYRO_SCALE;

    ax = (rawData[3] - ax_off) / ACCEL_SCALE;
    ay = (rawData[4] - ay_off) / ACCEL_SCALE;
    az = (rawData[5] - az_off) / ACCEL_SCALE;

    // 4. Complementary Filter (Simplified)
    float roll_acc  = atan2(ay, az) * RAD_TO_DEG;
    float pitch_acc = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;

    float alpha = 0.98; // Adjust based on your vibration environment
    
    // Use the filter on the integrated value
    roll  = alpha * (roll + gx * dt) + (1.0 - alpha) * roll_acc;
    pitch = alpha * (pitch + gy * dt) + (1.0 - alpha) * pitch_acc;
    
    // Yaw remains relative; recognize this will drift
    yaw += gz * dt;
}

unsigned long BMI160_IMU::getLastTimestamp() {
    return lastTime;
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