/* Approaching the front line
 * Rotate to the front wall and drive forward
 * Count the number of lines aencounter and stop at line number 2
 * A "line" is determined by all line sensors "lighting up"
 * Rotate the robot 90 degrees
*/

void updateIntersection() {
  RunAvg8();
  bool isIntersection = (RunSumLeft >> 2) > lightThreshold && (RunSumRight >> 2) > lightThreshold;
  if (isIntersection && !lowToHigh) {
    lowToHigh = true;
    intersectionCounter++;
    rotating = true;
    angle = 0;
    resetLeftEncoderCnt();
    resetRightEncoderCnt();
  } else if (!isIntersection && lowToHigh) {
    lowToHigh = false;
  }
}

void RunAvg8() {
  RunSumLeft = RunSumLeft - leftSensorRunning[OldestSensor] + sensorVal[0];
  RunSumRight = RunSumRight - rightSensorRunning[OldestSensor] + sensorVal[6];

  leftSensorRunning[OldestSensor] = sensorVal[0];
  rightSensorRunning[OldestSensor] = sensorVal[6];

  OldestSensor = (OldestSensor + 1) % 4; 
}

bool turnRobot(bool rightSide, int totalAngle) {
  float constant = 2.15;
  float turn = totalAngle * constant;
  if (getEncoderRightCnt() < turn && getEncoderLeftCnt() < turn) {
    angle = getEncoderRightCnt() / constant;
    if (rightSide) {
      // turn right
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    } else {
      // turn left
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    }
    return true;
  }
  return false;
}

void driveForward() {
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  setMotorSpeed(BOTH_MOTORS, normalSpeed);
}

void driveBackward() {
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
  setMotorSpeed(BOTH_MOTORS, normalSpeed);
}
