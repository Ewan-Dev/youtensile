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

static float pitchFiltered = 0;
static float rollFiltered = 0;

Servo servoPitch;
Servo servoRoll;

void setup() {
  Serial.begin(115200);

  /* Servo setup */
  servoPitch.attach(3);
  servoRoll.attach(5);

  wakeSensor(ADDRESS);

  /* Find offsets */
  calculateGyroOffset(ADDRESS, gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  calculateAccelOffset(ADDRESS, accelOffsetX, accelOffsetY);

  servoPitch.write(90);
  servoRoll.write(90);
  delay(2000);
}

void loop() {
  float rollAccumulated = 0;
  float pitchAccumulated = 0;

static unsigned long lastTime = micros();
unsigned long currentTime = micros();
float dt = (currentTime - lastTime) / 1000000.0;
lastTime = currentTime;// For next iteration

  // Working out delta time
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



  float alpha = 0.96; // weighting
  // Finds real values
  complementaryFilter(gyroDPSX, accelPitch, alpha, dt, pitchFiltered);
  complementaryFilter(gyroDPSY, accelRoll, alpha, dt, rollFiltered);

float pitchFilteredMapped = map(pitchFiltered, -90, 90, 180, 0);
float rollFilteredMapped  = map(rollFiltered,  -90, 90, 0, 180);

static float pitchOut = 90;
static float rollOut = 90;

float smoothing = 0.25;

pitchOut += smoothing * (pitchFilteredMapped - pitchOut);
rollOut  += smoothing * (rollFilteredMapped  - rollOut);

servoPitch.write(constrain(pitchOut, 0, 180));
servoRoll.write(constrain(rollOut, 0, 180));

}
