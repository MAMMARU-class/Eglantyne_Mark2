#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define SDA 5
#define SCL 4

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

imu::Quaternion q_ref;
bool ref_set = false;
unsigned long start_time;

void setup() {

    neopixelWrite(RGB_BUILTIN, 255, 0, 0);  // Red

    Serial.begin(115200);
    Wire.begin(SDA, SCL);

    if (!bno.begin()) {
        Serial.println("BNO055 not detected.");
        while (1);
    }

    delay(1000);
    bno.setExtCrystalUse(true);

    start_time = millis();

    Serial.println("Initialized");
    neopixelWrite(RGB_BUILTIN, 0, 255, 0);  // Green
}

void loop() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // --- 出力 ---
    Serial.print("angle ");
    Serial.print("Roll: ");  Serial.print(euler.z());
    Serial.print(" Pitch: "); Serial.print(-euler.y());
    Serial.print(" Yaw: ");   Serial.print(euler.x());

    Serial.print(" | accel ");
    Serial.print("X: "); Serial.print(accel.x());
    Serial.print(" Y: "); Serial.print(accel.y());
    Serial.print(" Z: "); Serial.print(accel.z());

    Serial.println();

    delay(50);
}
