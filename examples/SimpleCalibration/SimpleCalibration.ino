/*
  Arduino BMI270 - Gyroscope Calibration

  This example calibrates the gyro and the accelerometer of the BMI270
  sensor.

  The circuit:
  - Arduino Nano 33 BLE Sense Rev2

  created 3 Apr 2023
  by Lucio Rossi

  This example code is in the public domain.
*/

#include "Arduino_BMI270_BMM150.h"

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  IMU.debug(Serial);

  Serial.println("Sensors calibration ");
}

void loop() {

  Serial.println("Do you want to perform BMI270 calibration? (acc|gyro|all)");
  String resp;

  while (Serial.available() == 0) {
  }

  resp = Serial.readString();

  if (resp == "acc\n") {
    Serial.println("calibrating acc...");
    IMU.calibrate_accel();
    Serial.println("done");
  }

  if (resp == "gyro\n") {
    Serial.println("calibrating gyro...");
    IMU.calibrate_gyro();
    Serial.println("done");
  }

  if (resp == "all\n") {
    Serial.println("calibrating all sensors...");
    IMU.calibrate_all();
    Serial.println("done");
  }

}
