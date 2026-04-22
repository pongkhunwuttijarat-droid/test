#include <Arduino.h>
#include <BMI160Gen.h>
#include <SPI.h>

// กำหนดขา CS (Chip Select)
const int select_pin = 5; 

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing BMI160...");
    // สำหรับ ESP32 แนะนำให้เริ่มต้น SPI เองก่อนเพื่อความแม่นยำของขา
    // VSPI Default: SCK=18, MISO=19, MOSI=23, SS=5
    SPI.begin(); 

    // เริ่มต้นเซนเซอร์ในโหมด SPI
    if (!BMI160.begin(BMI160GenClass::SPI_MODE, select_pin)) {
        Serial.println("BMI160 connection failed!");
        while (1);
    }
    Serial.println("BMI160 connected via SPI!");
}
void loop() {

}