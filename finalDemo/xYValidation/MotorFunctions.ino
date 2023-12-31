/*
Motor Functions
SEED Lab Group 9
Zander Eggers, Gideon Kukoyi, James Clark, Elijah Price
November 7th 2023

Uses H Bridge motor driver to control motors
Uses voltSum and voltDelta to fit control scheme
*/

#define RIGHT_MOTOR_PIN 9
#define LEFT_MOTOR_PIN 10
#define RIGHT_MOTOR_DIR_PIN 7
#define LEFT_MOTOR_DIR_PIN 8
#define MOTOR_ENB_PIN 4

#define MAX_PWM 255

void SetupMotors() {
  pinMode(MOTOR_ENB_PIN, OUTPUT);
  digitalWrite(MOTOR_ENB_PIN, HIGH);

  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PIN, OUTPUT);

}

void RunMotors(double voltSum, double voltDelta) {
  // Set voltages
  double voltageRight = (voltSum + voltDelta) / 2.0;
  double voltageLeft = (voltSum - voltDelta) / 2.0;

  // H-Bridge Direction
  if(voltageLeft > 0) {
    digitalWrite(LEFT_MOTOR_DIR_PIN, HIGH);
  } else {
    digitalWrite(LEFT_MOTOR_DIR_PIN, LOW);
  }

  if(voltageRight > 0) {
    digitalWrite(RIGHT_MOTOR_DIR_PIN, HIGH);
  } else {
    digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);
  }

  // Powering the motors
  int PWMLeft = min((MAX_PWM * abs(voltageLeft)) / 8.0 , MAX_PWM);
  analogWrite(LEFT_MOTOR_PIN, PWMLeft);

  int PWMRight = min((MAX_PWM * abs(voltageRight)) / 8.0, MAX_PWM);
  analogWrite(RIGHT_MOTOR_PIN, PWMRight);
}