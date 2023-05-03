/* 
 * Patrolling
 * Follows the line and only drives back and forth
 * Determine if we need to drive and stop based on IR sensors
 */

void lineFollowing() {
  uint16_t fastSpeed = 27;
  bool leftSensors = sensorVal[0] > lightThreshold || sensorVal[1] > lightThreshold;
  bool rightSensors = sensorVal[6] > lightThreshold || sensorVal[6] > lightThreshold;

  if (leftSensors && rightSensors) {
    // continue
  } else if (leftSensors) {
    setMotorSpeed(LEFT_MOTOR, normalSpeed);
    setMotorSpeed(RIGHT_MOTOR, fastSpeed);
  } else if (rightSensors) {
    setMotorSpeed(LEFT_MOTOR, fastSpeed);
    setMotorSpeed(RIGHT_MOTOR, normalSpeed);

  } else {
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
  }
  uint32_t linePos = getLinePosition(sensorCalVal,DARK_LINE);
//  if(linePos > 0 && linePos < 3000) {
//    setMotorSpeed(LEFT_MOTOR,normalSpeed);
//    setMotorSpeed(RIGHT_MOTOR,fastSpeed);
//  } else if(linePos > 3500) {
//    setMotorSpeed(LEFT_MOTOR,fastSpeed);
//    setMotorSpeed(RIGHT_MOTOR,normalSpeed);
//  } else {
//    setMotorSpeed(LEFT_MOTOR,normalSpeed);
//    setMotorSpeed(RIGHT_MOTOR,normalSpeed);
//  }
}

void IRSensing() {
  int IRStateRight = digitalRead(IRright);
  int IRStateMid = digitalRead(IRmid);
  int IRStateLeft = digitalRead(IRleft);
  rightSum = rightSum - readingsRight[counter] + IRStateRight;
  midSum = midSum - readingsMid[counter] + IRStateMid;
  leftSum = leftSum - readingsLeft[counter] + IRStateLeft;
  readingsRight[counter] = IRStateRight;
  readingsMid[counter] = IRStateMid;
  readingsLeft[counter] = IRStateLeft;
  counter = (counter+1)%window_size;

  averageRight = double(rightSum)/window_size;
  averageMid = double(midSum)/window_size;
  averageLeft = double(leftSum)/window_size;
  delay(1);       // delay in between reads for stability.
}
