#include <Servo.h>
#include <mpu6050.h> /* My library*/

#define ADDRESS  0x68
float gyroRawX, gyroRawY, gyroRawZ;
float accelRawX, accelRawY, accelRawZ;
float gyroDPSX, gyroDPSY, gyroDPSZ;

Servo servoPitch;
Servo servoRoll;

void setup() {
  Serial.begin(115200);
  servoPitch.attach(3);
  servoRoll.attach(5);
  wakeSensor(ADDRESS);
}

void loop() {
  readGyroData(ADDRESS, gyroRawX, gyroRawY, gyroRawZ);
  readAccelData(ADDRESS, accelRawX, accelRawY, accelRawZ);
  rawGyroToDPS(gyroRawX, gyroRawY, gyroRawZ, gyroDPSX, gyroDPSY,gyroDPSZ);
  Serial.println(gyroDPSX);
  Serial.println(gyroDPSY);
  Serial.println(gyroDPSZ);
}
