// #include <Arduino.h>
// #include <BMI160Gen.h>
// #include <SPI.h>

// // กำหนดขา CS (Chip Select)
// const int select_pin = 5; 

// void setup() {
//     Serial.begin(115200);
//     Serial.println("Initializing BMI160...");
//     // สำหรับ ESP32 แนะนำให้เริ่มต้น SPI เองก่อนเพื่อความแม่นยำของขา
//     // VSPI Default: SCK=18, MISO=19, MOSI=23, SS=5
//     SPI.begin(); 
//     BMI160.begin();
//     Serial.println("SPI initialized!");
//     // เริ่มต้นเซนเซอร์ในโหมด SPI
//     if (!BMI160.begin(BMI160GenClass::SPI_MODE, select_pin)) {
//         Serial.println("BMI160 connection failed!");
//         while (1);
//     }
//     Serial.println("BMI160 connected via SPI!");
// }
// short * ax,*ay,*az;
// void loop() {
    
//     BMI160.getAcceleration(ax,ay,az);
//     Serial.print("X: ");
//     Serial.print(*ax);
//     Serial.print(" Y: ");
//     Serial.print(*ay);
//     Serial.print(" Z: ");
//     Serial.println(*az);
//     delay(1000);
// }

#include <Arduino.h>
#include "BMI160_IMU.h"

BMI160_IMU imu(5);

void setup(){
  Serial.begin(115200);
  imu.begin();

  Serial.println("Calibrating...");
  imu.calibrate();
  Serial.println("Done");
}

void loop(){
  imu.update();

  Serial.print("Roll: "); Serial.print(imu.getRoll());
  Serial.print(" Pitch: "); Serial.print(imu.getPitch());
  Serial.print(" Yaw: "); Serial.println(imu.getYaw());

  delay(10);
}
// uint8_t readRegister(uint8_t reg) {
//   digitalWrite(CS_PIN, LOW);
//   SPI.transfer(reg | 0x80); // read
//   uint8_t data = SPI.transfer(0x00);
//   digitalWrite(CS_PIN, HIGH);
//   return data;
// }

// void loop() {
//   uint8_t chip_id = readRegister(0x00);
//   Serial.println(chip_id, HEX);
//   delay(1000);
// }