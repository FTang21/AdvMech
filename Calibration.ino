// TODO: Calibrate with ultrasonic
/* 
 * Calibration
 * Rotation of 360 degrees for the robot to measure the distances of walls
 * Determination of the farthest wall using inflections
 */
int wallDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  uint16_t duration = pulseIn(echoPin, HIGH);
  uint16_t distance = (duration / 2) / 29.1;
  Serial.println(distance);
  return distance;
}

bool ultrasonicCalibrate() {
  if (turnRobot(true, 360)) {
    uint16_t distance = wallDistance();
    RunCalAvg8(distance);
    if (angle == 0) {
      calVal[0] = distance;
    } else if (int(angle) / 5 > 0) {
      calVal[int(angle) / 5] = (RunSumCal >> 3);
    }
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal,sensorMinVal,sensorMaxVal);
    return false;
  } else {
    disableMotor(BOTH_MOTORS);
    int closestDist = 100;
    int closestAngle = 0;
    int middleCount = 0;
    for (int i = 0; i < 72; i++) {
      if (calVal[i] == 0) {
        calVal[i] = calVal[i-1];
      }
      if (calVal[i] < closestDist) {
        closestDist = calVal[i];
        closestAngle = 5 * i;
      } else if (abs(calVal[i] - closestDist) <= 1) {
        middleCount++;
      } else {
        closestAngle += 5 * (middleCount / 2);
        middleCount = 0;
      }
    }
    fourWallsDist[0] = closestDist;
    fourWallsAngle[0] = closestAngle;
    int left = closestAngle - 90;
    if (left < 0) {
      left += 360;
    }
    fourWallsDist[1] = calVal[left / 5];
    fourWallsAngle[1] = left;
    int right = closestAngle + 90;
    if (right >= 360) {
      right -= 360;
    }
    fourWallsDist[2] = calVal[right / 5];
    fourWallsAngle[2] = right;
    int basketWall = closestAngle - 180;
    if (basketWall < 0) {
      basketWall += 360;
    }
    fourWallsDist[3] = calVal[basketWall / 5];
    fourWallsAngle[3] = basketWall;
    resetLeftEncoderCnt();
    resetRightEncoderCnt();
//    for(int i = 0; i < 72; i++) {
//      Serial.println(calVal[i]);
//    }
//    Serial.println("-----");
//    for(int i = 0; i < 4; i++) {
//      Serial.println(fourWallsDist[i]);
//      Serial.println(fourWallsAngle[i]);
//    }
    delay(500);
    return true;
  }
}

void RunCalAvg8(uint16_t distance) {
  RunSumCal = RunSumCal - calRunning[OldestCal] + distance;

  calRunning[OldestCal] = distance;

  OldestCal = (OldestCal + 1) % 8; 
}
