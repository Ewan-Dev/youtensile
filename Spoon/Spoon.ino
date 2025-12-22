#include <Servo.h>

Servo servoPitch;
Servo servoRoll;

void setup() {
  servoPitch.attach(3);
  servoRoll.attach(5);
}

void loop() {
  servoPitch.write(0);
  delay(1000);
  servoPitch.write(180);
  delay(1000);
  servoRoll.write(0);
  delay(1000);
  servoRoll.write(180);
  delay(1000);
}
