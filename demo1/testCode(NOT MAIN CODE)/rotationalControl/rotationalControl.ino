/*
October 12, 2023
Goal: Unsure ATM
*/

#include <Encoder.h>

#define BATTERY_VOLTAGE 8.0
#define ROBOT_DIAMETER_IN_CM 36.5
#define WHEEL_DIAMETER_IN_CM 15

#define KpLEFT 0.012
#define KiLEFT 0.00019
#define KpRIGHT 0.012
#define KiRIGHT 0.00019

#define KpANGLE 0.012 // Completely arbitrary. I don't want to do simulation and don't have access to the robot
#define KdANGLE 0.0002 // See above

#define TARGET_FEET 10.0
#define TARGET_ANGLE_RADIANS 0
#define MAX_PWM 100


double voltDelta; // {Delta}V_a as listed in SteeringControl_handout.pdf -> V_right - V_left

double voltageLeft, voltageRight;
double integralLeft, integralRight;

double derivativeAngle = 0.0;
double prevErrorAngle = 0;

float currentTime; // Current time in seconds
unsigned long lastTimeMs = 0; // Time at which last loop finished running in ms
unsigned long startTimeMs; // Time when the code starts running in ms
unsigned long desiredTsMs = 5; // Desired sampling time in ms -- previously was 10, may want to change back



// Encoder setup
Encoder EncLeft(3, 6); // Encoder on left wheel is pins 3 and 6
Encoder EncRight(2, 5); // Encoder on right wheel is pins 2 and 5

void setup() {
  // Serial Setup - mostly used for debugging
  Serial.begin(115200);
  Serial.println("Ready!"); // For ReadfromArduino.mlx

  // Motor Pins
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(7, OUTPUT); // Left H-Bridge
  pinMode(8, OUTPUT); // Right H-Bridge
  pinMode(9, OUTPUT); // Left power
  pinMode(10, OUTPUT); // Right power

  // Timing
  lastTimeMs - millis();
  startTimeMs = lastTimeMs;

}

void loop() {
  currentTime = (float)(lastTimeMs - startTimeMs)/1000;

  long currentPosLeft = -EncLeft.read(); //Negative because this motor is facing "backwards"
  long currentPosRight = EncRight.read();


  // Linear Distance Control
  // PI Control
  long targetPos = 2070 * TARGET_FEET; // 3200.0 / (15.0 * PI) * 30.48

  long errorLeft = targetPos - currentPosLeft;
  long errorRight = targetPos - currentPosRight;

  integralRight = integralRight + ( double(desiredTsMs) / 1000.0 ) * errorRight;
  integralLeft = integralLeft + ( double(desiredTsMs) / 1000.0 ) * errorLeft;

  double linearVoltRight = KpRIGHT * errorRight + KiRIGHT * integralRight;
  double linearVoltLeft = KpLEFT * errorLeft + KiLEFT * integralLeft;


  // Angle Control
  long targetAngle = TARGET_ANGLE_RADIANS;
  double thetaRight = 2.0 * PI * (double)((double)currentPosRight / 3200.0); // Right Wheel Position in Radians
  double thetaLeft = 2.0 * PI * (double)((double)currentPosLeft / 3200.0); // Left Wheel Position in Radians
  long currentAngle = WHEEL_DIAMETER_IN_CM * (thetaRight - thetaLeft) / ROBOT_DIAMETER_IN_CM;

  // This is a basic PD control.
  // TO DO: tune PD, decide if I is necessary 
  //      : test this, could be completely incorrect
  double errorAngle = targetAngle - currentAngle;

  if (prevErrorAngle == 0) { // Don't do on the first loop
    derivativeAngle = 0;
  } else {
    derivativeAngle = (errorAngle - prevErrorAngle) / desiredTsMs;
  }

  prevErrorAngle = errorAngle;
  voltDelta = KpANGLE * errorAngle + KdANGLE * derivativeAngle;

  // Set voltages
  voltageRight = (linearVoltRight + voltDelta);
  voltageLeft = (linearVoltLeft - voltDelta);

  // H-Bridge Direction
  if(voltageLeft > 0) {
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(7, LOW);
  }

  if(voltageRight > 0) {
    digitalWrite(8, HIGH);
  } else {
    digitalWrite(8, LOW);
  }

  // Powering the motors
  int PWMLeft = 255 * abs(voltageLeft) / BATTERY_VOLTAGE;
  analogWrite(9, min(PWMLeft, MAX_PWM));

  int PWMRight = 255 * abs(voltageRight) / BATTERY_VOLTAGE;
  analogWrite(10, min(PWMRight, MAX_PWM));


  while(millis() < lastTimeMs + desiredTsMs);
  lastTimeMs = millis();
}
