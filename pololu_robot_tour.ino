#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Motors motors;
Encoders encoders;
ButtonA buttonA;

// key:
// S = start
// F = forward
// B = backward
// R = right
// L = left
// T = stop/terminate
char paths[] = "FLFLFLFLFLFLFLFLT";
bool timeAdj = true;
bool encoderAdj = true;
// char paths[] = "S";
int index = 0;
float targetTime = 1.0;
float baseSpeed = 100.0;

float actionDelay = 0.5;

//angle pid
float kp = 1;
float error = 0.0;
float leftSpeed = baseSpeed;
float rightSpeed = baseSpeed;
float minSpeed = 15.0;
float maxSpeed = 150.0;

//adjust time PID
int maxCounts = 0;
float kpTime = 0.0005;
long errorTime = 0.0;
long startTime = 0.0;

// distances
const float wheelDiameter = 3.2; // cm
const float wheelDistance = 8.65; // distance between wheels
const float countsPerRevolution = 358.32; // Encoder counts per wheel revolution

const float cmPerCount = (wheelDiameter * 3.14159) / countsPerRevolution * 27/25 * 71.5/75;
const float countsPerForward = 50 / cmPerCount;
const float countsPerTurn = wheelDistance * 3.14159 / 4 / cmPerCount+1;

int16_t countsLeft = 0;
int16_t countsRight = 0;
int16_t countsLeftDist = 0;
int16_t countsRightDist = 0;
int16_t actionDist = 0;
int16_t countsLeftOffset = 0;
int16_t countsRightOffset = 0;
volatile int totalCount = 0;


void setup() {
  Serial.begin(9600);
  // calculate max counts
  for (int x = 0; x<strlen(paths); x++){
    if (paths[x]=='S'){
      maxCounts += countsPerForward/2;
    }else if (paths[x]=='F' || paths[x]=='B'){
      maxCounts += countsPerForward;
    } else if (paths[x]=='L' || paths[x]=='R'){
      maxCounts += countsPerTurn;
    }
  }
  targetTime -= (strlen(paths)-2)*actionDelay;
  if (!timeAdj){
    kpTime = 0.0;
  }
  if (!encoderAdj){
    kp = 0.0;
  }
  delay(1000);
  startTime = millis();
}

void adjustSpeeds() {
  error = countsLeftDist - countsRightDist;
  errorTime = (totalCount + actionDist)/(float)maxCounts*targetTime*1000 - (millis()-startTime-index*actionDelay*1000);
  
  // Serial.println(millis()-startTime-index*actionDelay);
  float adj_time = constrain(1 - kpTime*errorTime, 0.1, 2);
  leftSpeed = constrain(adj_time*(baseSpeed - kp*error), minSpeed, maxSpeed);
  rightSpeed = constrain(adj_time*(baseSpeed + kp*error), minSpeed, maxSpeed);
}

void finishDelay() {
  motors.setSpeeds(0, 0);
  long startDelayTime = millis();
  while (millis()-startDelayTime<actionDelay*1000){
    delay(10);
  }
}

void loop() {
    countsLeft = encoders.getCountsLeft() + countsLeftOffset;
    countsRight = encoders.getCountsRight() + countsRightOffset;

  if (paths[index]=='S'){
    countsLeftDist = countsLeft;
    countsRightDist = countsRight;
    actionDist = max(countsLeftDist, countsRightDist);
    if (actionDist > countsPerForward/2){

      totalCount += (int16_t)countsPerForward/2;
      countsLeftOffset -= (int16_t)countsPerForward/2;
      countsRightOffset -= (int16_t)countsPerForward/2;
      finishDelay();
      index++;

    }
    adjustSpeeds();
    motors.setSpeeds(leftSpeed, rightSpeed);
  }else if (paths[index]=='F'){
    countsLeftDist = countsLeft;
    countsRightDist = countsRight;
    actionDist = max(countsLeftDist, countsRightDist);
    if (actionDist>countsPerForward){
      totalCount += (int16_t)countsPerForward;
      countsLeftOffset -= (int16_t)countsPerForward;
      countsRightOffset -= (int16_t)countsPerForward;
      finishDelay();

      index++;
    }
    adjustSpeeds();
    motors.setSpeeds(leftSpeed, rightSpeed);
  }else if (paths[index]=='B'){
    countsLeftDist = -countsLeft;
    countsRightDist = -countsRight;
    actionDist = max(countsLeftDist, countsRightDist);
    if (actionDist>countsPerForward){
      totalCount += (int16_t)countsPerForward;
      countsLeftOffset += (int16_t)countsPerForward;
      countsRightOffset += (int16_t)countsPerForward;
      finishDelay();
      index++;
    }
    adjustSpeeds();
    motors.setSpeeds(-leftSpeed, -rightSpeed);
  }else if (paths[index] == 'R'){
    countsLeftDist = countsLeft;
    countsRightDist = -countsRight;
    actionDist = max(countsLeftDist, countsRightDist);
    if (actionDist>countsPerTurn){
      // Serial.println("hiii");
      // Serial.println(countsLeftOffset);
      totalCount += (int16_t)countsPerTurn;
      countsLeftOffset -= (int16_t)countsPerTurn;
      countsRightOffset += (int16_t)countsPerTurn;
      finishDelay();
      index++;
    }
    adjustSpeeds();
    motors.setSpeeds(leftSpeed, -rightSpeed);
  }else if (paths[index] == 'L'){
    countsLeftDist = -countsLeft;
    countsRightDist = countsRight;
    actionDist = max(countsLeftDist, countsRightDist);
    if (actionDist>countsPerTurn){

      // Serial.println("hiii");
      // Serial.println(countsLeftOffset);
      totalCount += (int16_t)countsPerTurn;
      countsLeftOffset += (int16_t)countsPerTurn;
      countsRightOffset -= (int16_t)countsPerTurn;
      finishDelay();
      index++;

    }
    adjustSpeeds();
  motors.setSpeeds(-leftSpeed, rightSpeed);
  } else{ // T
    motors.setSpeeds(0, 0);
    while(true){
      delay(1000);
    }
  }
}
