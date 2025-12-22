#include <Servo.h>
#include <mpu6050.h> /* My library*/

#define ADDRESS  0x68
float gyroRawX, gyroRawY, gyroRawZ;

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
  Serial.println(gyroRawX);
  Serial.println(gyroRawY);
  Serial.println(gyroRawZ);
}
