#include <Servo.h>
#include "SimpleRSLK.h"
#define echoPin 2
#define trigPin 3
#define PWMspeed 4
#define PWMoutp 7
#define PWMoutn 8
#define IRright 23
#define IRmid 24
#define IRleft 25
#define servoPin 38

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS]; 
uint16_t calVal[72];
uint16_t normalSpeed;
uint16_t lightThreshold;
uint16_t leftSensorRunning[4] = {0, 0, 0, 0};
uint16_t rightSensorRunning[4] = {0, 0, 0, 0};
uint16_t calRunning[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t RunSumLeft;
uint16_t RunSumRight;
uint16_t RunSumCal;
unsigned char OldestCal; 
unsigned char OldestSensor; 
bool lowToHigh;
int intersectionCounter;
bool rotating;
float angle;
bool pressed;
bool finishedCal;
bool foundLineDir;
bool lineTurn;
bool backwards;
bool shooting;
bool shot;
bool patrolling;
bool moveRobot;
long delayTime;
long backTime;
uint16_t fourWallsDist[4];
uint16_t fourWallsAngle[4];
Servo myservo;
int rightSum;
int midSum;
int leftSum;
const int window_size = 25;
int readingsRight[window_size];
int readingsMid[window_size];
int readingsLeft[window_size];
int counter = 0;
double averageRight;
double averageMid;
double averageLeft;
int pos = 0;

void setup() {
  Serial.begin(115200);
  setupRSLK();
  pinMode(echoPin, OUTPUT);
  pinMode(trigPin, INPUT);
  pinMode(PWMspeed, OUTPUT);
  pinMode(PWMoutp, OUTPUT);
  pinMode(PWMoutn, OUTPUT);
  pinMode(IRright, INPUT_PULLUP);
  pinMode(IRmid, INPUT_PULLUP);
  pinMode(IRleft, INPUT_PULLUP);
  setupWaitBtn(LP_LEFT_BTN);
  setupLed(RED_LED);
  clearMinMax(sensorMinVal,sensorMaxVal);

//  // setup for driving to front wall
  normalSpeed = 20;
  lightThreshold = 2000;
  RunSumLeft = 0;
  RunSumRight = 0;
  RunSumCal = 0;
  OldestCal = 1; 
  OldestSensor = 1;
  lowToHigh = false;
  intersectionCounter = 0;
  rotating = false;
  angle = 0;
  pressed = false;
  finishedCal = false;
  foundLineDir = false;
  backwards = false;
  delayTime = 0;
  shooting = false;
  shot = false;
  patrolling = false;
  moveRobot = false;
  backTime = 0;
  digitalWrite(PWMoutp, HIGH);
  digitalWrite(PWMoutn, LOW);
  rightSum = 25;
  midSum = 25;
  leftSum = 25;
  counter = 0;
  for(int i=0;i<window_size;i++){
    readingsRight[i] = 1;
    readingsMid[i] = 1;
    readingsLeft[i] = 1;
  }
  myservo.attach(38);
  myservo.write(140);
}

void loop() {
  // put your main code here, to run repeatedly: 
  
  if (!pressed) {
    Serial.println("press button");
    waitBtnPressed(LP_LEFT_BTN);
    pressed = true;
    resetLeftEncoderCnt();
    resetRightEncoderCnt();
  }

//  IRSensing();
//  Serial.println(averageRight);
//  Serial.println(averageMid);
//  Serial.println(averageLeft);
//  Serial.println("-----");
//  Shoot();

  if (!finishedCal) {
    enableMotor(BOTH_MOTORS);
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
    finishedCal = ultrasonicCalibrate();
  } else if (!foundLineDir) {
    enableMotor(BOTH_MOTORS);
    setMotorSpeed(BOTH_MOTORS, normalSpeed);
    lineTurn = fourWallsDist[1] < fourWallsDist[2];
    if (fourWallsDist[1] < fourWallsDist[2]) {
      bool turnRight = fourWallsAngle[2] <= 180;
      uint16_t angle = fourWallsAngle[2];
      if (!turnRight && angle != 360) {
        angle = 360 - angle;
      }
      foundLineDir = !(turnRobot(turnRight, angle));
    } else {
      bool turnRight = fourWallsAngle[1] <= 180;
      uint16_t angle = fourWallsAngle[1];
      if (!turnRight && angle != 360) {
        angle = 360 - angle;
      }
      foundLineDir = !(turnRobot(turnRight, angle));
    }
  } else {
      enableMotor(BOTH_MOTORS);
      readLineSensor(sensorVal);
      readCalLineSensor(sensorVal, sensorCalVal, sensorMinVal, sensorMaxVal, DARK_LINE);
      if (!patrolling) {
        // finding the front line
        if (intersectionCounter == 1 && rotating) {
          rotating = turnRobot(lineTurn, 90);
        } else if (intersectionCounter == 3 && rotating) {
          rotating = turnRobot(false, 90);
          patrolling = !rotating;
        } else {
          driveForward();
          lineFollowing();
          updateIntersection();
        }
      } else {
        IRSensing();
        disableMotor(BOTH_MOTORS);
        if (averageMid == 0) {
          Shoot();
        } else if (averageRight == 0) {
          backwards = false;
          moveRobot = true;
          NoShoot();
        } else if (averageLeft == 0) {
          backwards = true;
          moveRobot = true;
          NoShoot();
        } else {
          moveRobot = false;
          NoShoot();
        }
        if (!backwards && wallDistance() <= 30) {
          disableMotor(BOTH_MOTORS);
          backwards = true;
          delay(500);
          backTime = millis();
        } else if (backwards && wallDistance() >= 140 && (millis() - backTime) > 2000) {
          disableMotor(BOTH_MOTORS);
          backwards = false;
          delay(500);
        }
        if (!shooting && moveRobot) {
          enableMotor(BOTH_MOTORS);
          if(backwards) {
            driveBackward();
            lineFollowing();
          } else {
            driveForward();
            lineFollowing();
          }
        }
      }
    }
}

void floorCalibration() {
  /* Place Robot On Floor (no line) */
  delay(2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);

  delay(1000);

  Serial.println("Running calibration on floor");
  simpleCalibrate();
  Serial.println("Reading floor values complete");

  btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  delay(1000);

  enableMotor(BOTH_MOTORS);
}

void simpleCalibrate() {
  /* Set both motors direction forward */
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  /* Enable both motors */
  enableMotor(BOTH_MOTORS);
  /* Set both motors speed 20 */
  setMotorSpeed(BOTH_MOTORS,20);

  for(int x = 0;x<100;x++){
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal,sensorMinVal,sensorMaxVal);
  }

  /* Disable both motors */
  disableMotor(BOTH_MOTORS);
}

// FUNCTIONS

/*
 * Feeding
 * Determine if the next ball needs to be fed
 */

/*
 * Shooting
 * Shoot only if front of the IR sensor that is on
 * Adjust motor power if IR sensor continues to be on
 */

 // Ultrasonic pins: input (4) and output (3)

 
