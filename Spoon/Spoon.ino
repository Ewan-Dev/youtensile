#include <Servo.h>
#include <mpu6050.h> /* My library*/

#define ADDRESS  0x68
float gyroRawX, gyroRawY, gyroRawZ;
float accelRawX, accelRawY, accelRawZ;
float gyroDPSX, gyroDPSY, gyroDPSZ;
float accelGForceX, accelGForceY, accelGForceZ;
double gyroOffsetX, gyroOffsetY, gyroOffsetZ;
double accelOffsetX, accelOffsetY;
float accelPitch, accelRoll;

Servo servoPitch;
Servo servoRoll;

void setup() {
  Serial.begin(115200);

  /* Servo setup */
  servoPitch.attach(5);
  servoRoll.attach(3);

  wakeSensor(ADDRESS);

  /* Find offsets */
  calculateGyroOffset(ADDRESS, gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  calculateAccelOffset(ADDRESS, accelOffsetX, accelOffsetY);
}

void loop() {

  // Working out delta time
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000; // Finds delta time
  lastTime = currentTime; // For next iteration

  /* Work out angle values*/
  readGyroData(ADDRESS, gyroRawX, gyroRawY, gyroRawZ);
  readAccelData(ADDRESS, accelRawX, accelRawY, accelRawZ);
  rawGyroToDPS(gyroRawX, gyroRawY, gyroRawZ, gyroDPSX, gyroDPSY,gyroDPSZ);
  rawAccelToGForce(accelRawX, accelRawY, accelRawZ,  accelGForceX, accelGForceY, accelGForceZ);
  calculateAnglesFromAccel(accelGForceX, accelGForceY, accelGForceZ, accelPitch, accelRoll);

  /* Put offsets to use! */
  gyroDPSX = gyroDPSX - gyroOffsetX;
  gyroDPSY = gyroDPSY - gyroOffsetY;
  gyroDPSZ = gyroDPSZ - gyroOffsetZ;

  accelPitch = accelPitch - accelOffsetX;
  accelRoll = accelRoll - accelOffsetY;


  float alpha = 0.95; // weighting
  float pitchFiltered = accelPitch;
  float rollFiltered = accelRoll;

  // Finds real values
  complementaryFilter(gyroDPSX, accelPitch, alpha, dt, pitchFiltered);
  complementaryFilter(gyroDPSY, accelRoll, alpha, dt, rollFiltered);
  float pitchFilteredMapped = map(pitchFiltered, 0, 360, 0, 180);
  float rollFilteredMapped = map(rollFiltered, 0, 360, 0, 180);
    Serial.print("filtered pitch:");
    Serial.print(pitchFiltered);
    servoPitch.write(pitchFilteredMapped);
    Serial.print("/");
    Serial.print("filtered Roll:");
    Serial.println(rollFiltered);
    servoRoll.write(rollFilteredMapped);
}
