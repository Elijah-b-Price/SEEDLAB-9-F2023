/*
Demo 2 Reworked Code
SEED Lab Group 9
Zander Eggers, Gideon Kukoyi, James Clark, Elijah Price
November 7th 2023

The purpose of this code is to complete demo 2 by doing the following items:
  Find aruco marker between 3 and 8 feet away,
  Drive up to within 1 foot of it,
  Do a circle around it when requested

This is done through PID control, utilizing a similar control structure to demo 1 in which linear voltage and
angular voltage are controlled seperately and then added or subtracted to drive each motor independently

This code will not be updated further, though know it is certainly not "robust"

To Use:
  Set robot parameters
  Determine if you want to do a circle
    Set IS_CIRCLE_TIME
*/

#include <Encoder.h>
#include <Wire.h>

// Define Constants
#define WHEEL_DIAMETER_IN_CM 15.0
#define ROBOT_DIAMETER_IN_CM 36.947125 // Determined empirically
#define DESIRED_TS_MS 10.0 

#define DESIRED_RADIUS 4.5

#define COUNTS_PER_FOOT 2070.0
#define COUNTS_PER_INCH 172.5

#define SPIN_RATE PI / 20.0 // Ensure this value is not too fast to allow camera to see marker
#define CIRCLE_LINEAR_SPEED 10.0 // cm/s

#define TURN_TO_MARKER_ERROR_BAND 0.01
#define ANGLE_ERROR_BAND 0.01

#define IS_CIRCLE_TIME false

// Define States
enum State {
  FIND_MARKER,
  TURN_TO_MARKER,
  ANGLE_CHECK,
  DRIVE_TO_MARKER,
  TURN_RIGHT,
  DO_A_CIRCLE,
  STOP
};

// Define Globals
double lastTimeMs, startTimeMs;
State currState = FIND_MARKER;
double previousThetaLeft = 0.0, previousThetaRight = 0.0;

double integralValX = 0.0, integralValAngle = 0.0, integralValRho = 0.0, integralValOmega = 0.0;
double prevErrorX = 0.0, prevErrorAngle = 0.0, prevErrorRho = 0.0, prevErrorOmega = 0.0;

double lastPiMeasuredAngle = NAN, lastPiMeasuredDistance = NAN;
int settleCountTurnRight = 0;
int settleCountTurnMark = 0;
int angleCheckCount = 0;

double desiredPos = 0.0, desiredAngle = 0.0;

// Initialize Encoders
Encoder EncLeft(3, 6); // Encoder on left wheel is pins 3 and 6 -- Could use #defines for this later
Encoder EncRight(2, 5); // Encoder on right wheel is pins 2 and 5


void setup() {
  Serial.begin(115200);
  Serial.println("Ready!"); // For ReadfromArduino.mlx

  SetupMotors();
  SetupI2C();

  // Timing
  lastTimeMs - millis();
  startTimeMs = lastTimeMs;
}

void loop() {
  double currentTime = (lastTimeMs - startTimeMs) / 1000.0; // Current Time in seconds

  long currentCountLeft = -EncLeft.read(); // Negative because this motor is facing the other direction
  long currentCountRight = EncRight.read();

  double thetaRight = (2.0 * PI * (double)currentCountRight) / 3200.0; // Right wheel position in radians
  double thetaLeft = (2.0 * PI * (double)currentCountLeft) / 3200.0; // Left wheel position in radians

  double thetaDotRight = (thetaRight - previousThetaRight) / DESIRED_TS_MS; // Right wheel angular velocity in rad/ms
  double thetaDotLeft = (thetaLeft - previousThetaLeft) / DESIRED_TS_MS; // Left wheel angular velocity in rad/ms

  double currentPos = (currentCountLeft + currentCountRight) / 2.0;
  double currentRho = WHEEL_DIAMETER_IN_CM / 2.0 * (thetaDotRight + thetaDotLeft) / 2.0; // Average speed of the wheels is linear velocity measured in cm / ms

  double currentAngle = (WHEEL_DIAMETER_IN_CM / 2.0) * (thetaRight - thetaLeft) / ROBOT_DIAMETER_IN_CM; // Current heading of the robot related to where it started
  double currentOmega = (WHEEL_DIAMETER_IN_CM / 2.0) * (thetaDotRight - thetaDotLeft) / ROBOT_DIAMETER_IN_CM; // Current change of heading in radians per second

  double kPOmega = 19.0;
  double kIOmega = 0.1;
  double kDOmega = 0.05;

  double kPRho = 0.1;
  double kIRho = 0.05;
  double kDRho = 0.0;

  double kPX = 0.01;
  double kIX = 0.00003;
  double kDX = 0.0;
  
  double kPAngle = 19.0;
  double kIAngle = 0.0;
  double kDAngle = 0.0;

  double desiredRho, desiredOmega;


  bool moveStateFlag = false;

  switch(currState) { // Tuning, transitions, targets (for desiredPos and desiredAngle)
    case FIND_MARKER:

      desiredPos = 0; 

      kPRho = 0.8;
      kIRho = 0.6;

      if( !isnan(lastPiMeasuredAngle) ) { // Exit state as soon as marker is seen
        moveStateFlag = true;
        if (currentTime < 1.0) { // Add fudge factor (0.25) if Aruco marker is not immediately seen
          desiredAngle = lastPiMeasuredAngle;
        } else {
          desiredAngle = lastPiMeasuredAngle - 0.32; // Fudge factor is determined emperically to compensate for camera delay
        }
        currState = TURN_TO_MARKER;
      }
      break;

    case TURN_TO_MARKER:

      // desired position is still zero, use previous PID control efforts

      if ( abs(currentAngle - desiredAngle ) < TURN_TO_MARKER_ERROR_BAND ) { // Counts if within errorband to allow for settling time
        settleCountTurnMark++;
      }

      if  ( settleCountTurnMark >= 50 && !isnan(lastPiMeasuredDistance)) {
        moveStateFlag = true;
        desiredPos = (lastPiMeasuredDistance * COUNTS_PER_INCH) - (COUNTS_PER_FOOT * 1.1); // Set desired position to the distance from the aruco marker minus 1 foot
        currState = DRIVE_TO_MARKER;
      };
      break;

    // tested case to try to become more accurate, found it was difficult to work with (especially considering delay);
    // case ANGLE_CHECK:
      
    //   if (++angleCheckCount >= 200) {
    //     moveStateFlag = true;
    //     if (abs(lastPiMeasuredAngle) < TURN_TO_MARKER_ERROR_BAND) {
    //       currState = DRIVE_TO_MARKER;
    //     } else {
    //       desiredAngle = lastPiMeasuredAngle;
    //       currState = TURN_TO_MARKER;
    //     }
    //   }

    case DRIVE_TO_MARKER:

      desiredAngle = 0.0;
      // desiredPos = 4.0 * COUNTS_PER_FOOT; // Test for this state

      if ( (desiredPos - currentPos < 0) ) { // Once the robot is within a foot
        moveStateFlag = true;
        if (IS_CIRCLE_TIME) {
          currState = TURN_RIGHT;
        } else {
          currState = STOP;
        }
      }
      break;

    case TURN_RIGHT:

      desiredAngle = -(PI / 2.0 - 0.3); // Negative indicates right in this control scheme
      desiredPos = 0.0;

      if ( abs(currentAngle + (PI/2.0 - 0.3) ) < ANGLE_ERROR_BAND ) { // Counts when within error band to allow for settling time
        settleCountTurnRight++;
      }

      if  ( settleCountTurnRight >= 50 ) {
        moveStateFlag = true;
        currState = DO_A_CIRCLE;
      }
      break;

    case DO_A_CIRCLE:

      kPOmega = 1.0;
      kIOmega = 0.0;
      kDOmega = 0.0;

      kPRho = 1.0;
      kIRho = 0.0;
      kDRho = 0.0;

      if (currentAngle > 2.0 * PI) { // Once the robot has turned more than 2pi radians, it is done with circle
        moveStateFlag = true;
        currState = STOP;
      }
      break;

    case STOP: // Don't do anything

      desiredPos = 0.0;
      desiredAngle = 0.0;
      break;
  }

  if( moveStateFlag ) { // When moving states, reset any counters, integral values, and encoders
    EncLeft.write(0);
    EncRight.write(0);
    integralValAngle = 0;
    integralValOmega = 0;
    integralValRho = 0;
    integralValX = 0;
    angleCheckCount = 0;
    settleCountTurnMark = 0;
  }

  switch(currState) { // Fall thru to run controllers and find desiredRho, desiredOmega
    case FIND_MARKER: //omega, x, rho
      desiredRho = PID(desiredPos, currentPos, DESIRED_TS_MS, kPX, kIX, kDX, &integralValX, &prevErrorX);
      desiredOmega = SPIN_RATE;
      break;

    case TURN_TO_MARKER: //theta, omega, x, rho
    case DRIVE_TO_MARKER:
    case TURN_RIGHT:
    case STOP:
      desiredRho = PID(desiredPos, currentPos, DESIRED_TS_MS, kPX, kIX, kDX, &integralValX, &prevErrorX);
      desiredOmega = PID(desiredAngle, currentAngle, DESIRED_TS_MS, kPAngle, kIAngle, kDAngle, &integralValAngle, &prevErrorAngle);
      break;

    case DO_A_CIRCLE: // omega, rho
    case ANGLE_CHECK:
      desiredRho = CIRCLE_LINEAR_SPEED;
      desiredOmega = CIRCLE_LINEAR_SPEED / DESIRED_RADIUS;
      break;
  }

  double voltDelta = PID(desiredOmega, currentOmega, DESIRED_TS_MS, kPOmega, kIOmega, kDOmega, &integralValOmega, &prevErrorOmega);
  double voltSum = PID(desiredRho, currentRho, DESIRED_TS_MS, kPRho, kIRho, kDRho, &integralValRho, &prevErrorRho);

  RunMotors(voltSum, voltDelta);

  previousThetaLeft = thetaLeft;
  previousThetaRight = thetaRight;

  // Debugging Statements
  // Serial.print(currentTime);
  // Serial.print("\t");
  // Serial.print(currState);
  // Serial.print("\t");
  // Serial.print(lastPiMeasuredAngle);
  // Serial.print("\t");
  // Serial.print(currentPos - desiredPos , 4);
  // Serial.print("\t");
  // Serial.print(currentAngle - desiredAngle, 4);
  // Serial.println("\t");
  // Serial.print(currentRho, 4);
  // Serial.print("\t");
  // Serial.print(currentOmega, 4);
  // Serial.print("\t");
  // Serial.print(voltSum);
  // Serial.print("\t");
  // Serial.println(voltDelta);
  // Serial.println();



  while(millis() < lastTimeMs + DESIRED_TS_MS); // Wait til end of desired sample time
  lastTimeMs = millis();
}