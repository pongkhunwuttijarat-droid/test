
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "BMI160_IMU.h"

// กำหนดขา CS
#define BMI_CS 5
#define SD_CS  4

BMI160_IMU imu(BMI_CS);
String fileName ;
// ฟังก์ชันสำหรับบันทึกข้อมูลลง SD Card
// เพิ่มการเช็คใน Setup ให้แน่ใจว่า SD ทำงานได้จริง
void setup() {
    Serial.begin(115200);
    
    // 1. เริ่มต้น BMI160
    imu.begin();
    imu.calibrate();
    // 2. เริ่มต้น SD Card
    if (!SD.begin(SD_CS)) {
        Serial.println("Error: SD Card Mount Failed!");
        // หยุดโปรแกรมไว้ตรงนี้ถ้า SD Card มีปัญหา
        return;
    }
    Serial.println("SD Card Initialized.");
    
    // สร้าง Header ถ้าไฟล์ยังไม่มี
    if (!SD.exists("/data.csv")) {
        File file = SD.open("/data.csv", FILE_WRITE);
        if(file) {
            file.println("Time,Roll,Pitch,Yaw,Ax,Ay,Az,Gx,Gy,Gz");
            file.close();
        }
    } else {
        // สร้างไฟล์ใหม่หากไฟล์ data.csv มีอยู่แล้ว
        int fileNumber = 1;
        while (SD.exists(String("/data") + fileNumber + ".csv")) {
            fileNumber++;
        }
         fileName = String("/data") + fileNumber + ".csv";
        File file = SD.open(fileName, FILE_WRITE);
        if(file) {
            file.println("Time,Ax,Ay,Az,Gx,Gy,Gz");
            file.close();
        }
    }
}

// ปรับปรุง appendFile ให้ปลอดภัยขึ้น
void appendFile(fs::FS &fs, const char * path, const char * message) {
    // ใช้ FILE_APPEND เพื่อเขียนต่อท้ายไฟล์เดิม
    File file = fs.open(path, FILE_APPEND);
    
    if(!file) {
        Serial.println("Failed to open file for appending");
        return;
    }
    
    // เขียนข้อความและขึ้นบรรทัดใหม่
    if(file.println(message)) {
        // บันทึกสำเร็จ
    } else {
        Serial.println("Write failed");
    }
    
    // สำคัญที่สุด: ต้องปิดไฟล์เสมอ เพื่อให้ข้อมูลถูกเขียนลง SD Card จริงๆ
    file.close(); 
} 

void loop() {
    imu.update();
    // สร้าง String ข้อมูลในรูปแบบ CSV
    String dataString = String(imu.getLastTimestamp()) + "," +
                        String(imu.getAccX()) + "," +
                        String(imu.getAccY()) + "," +
                        String(imu.getAccZ()) + "," +
                        String(imu.getGyroX()) + "," +
                        String(imu.getGyroY()) + "," +
                        String(imu.getGyroZ());

    // บันทึกลง SD Card 
    appendFile(SD, fileName.c_str(), dataString.c_str());

    // แสดงผลใน Serial เพื่อตรวจสอบ
    Serial.println(dataString);

    delay(150); // เก็บข้อมูลทุก 100ms
}
