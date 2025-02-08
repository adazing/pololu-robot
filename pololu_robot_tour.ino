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
char paths[] = "SRRRRRRRRRRRRRRRRT";
bool timeAdj = true;
bool encoderAdj = true;
// char paths[] = "S";
int index = 0;
float targetTime = 5.0;
float baseSpeed = 100.0;

//angle pid
float kp = 1;
float error = 0.0;
float leftSpeed = baseSpeed;
float rightSpeed = baseSpeed;
float minSpeed = 15.0;
float maxSpeed = 255.0;

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
const float countsPerTurn = wheelDistance * 3.14159 / 4 / cmPerCount;

int16_t countsLeft = 0;
int16_t countsRight = 0;
int16_t countsLeftOffset = 0;
int16_t countsRightOffset = 0;
volatile int totalCount = 0;


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  maxCounts = countsPerForward/2; // start
  for (int x = 0; x<strlen(paths); x++){
    if (paths[x]=='F' || paths[x]=='B'){
      maxCounts += countsPerForward;
    } else if (paths[x]=='L' || paths[x]=='R'){
      maxCounts += countsPerTurn;
    }
  }
  Serial.println("HII");
  if (!timeAdj){
    kpTime = 0.0;
  }
  if (!encoderAdj){
    kp = 0.0;
  }
  delay(1000);
  startTime = millis();

}

void moveForward() {
  motors.setSpeeds(leftSpeed, rightSpeed);
}

void moveBackward() {
  motors.setSpeeds(-leftSpeed, -rightSpeed);
}

void turnRight() {
  motors.setSpeeds(leftSpeed, -rightSpeed);
}

void turnLeft() {
  motors.setSpeeds(-leftSpeed, rightSpeed);
}

void stop() {
  motors.setSpeeds(0, 0);
}

void adjustSpeeds() {
  // countsLeft = encoders.getCountsLeft();
  // countsRight = encoders.getCountsRight();
  error = abs(countsLeft) - abs(countsRight);
  errorTime = (totalCount + max(abs(countsLeft), abs(countsRight)))/(float)maxCounts*targetTime*1000 - (millis()-startTime);
  float adj_time = constrain(1 - kpTime*errorTime, 0.1, 2);
  // float adj_time = 1;
  Serial.print(" state ");
  Serial.print(index);
  Serial.print(" counts left ");
  Serial.print(countsLeft);
  Serial.print(" countsPerForward ");
  Serial.print(countsPerForward);
  Serial.print(" counts right ");
  Serial.print(countsRight);
  Serial.print(" error correction ");
  Serial.print(kp*error);
  Serial.print(" error time correction ");
  Serial.print(1 - kpTime*errorTime);
  leftSpeed = constrain(adj_time*(baseSpeed - kp*error), minSpeed, maxSpeed);
  rightSpeed = constrain(adj_time*(baseSpeed + kp*error), minSpeed, maxSpeed);
  Serial.print(" left speed ");
  Serial.print(leftSpeed);
  Serial.print(" right speed ");
  Serial.println(rightSpeed);

  // leftSpeed = 

}

void loop() {
    countsLeft = encoders.getCountsLeft() + countsLeftOffset;
    countsRight = encoders.getCountsRight() + countsRightOffset;

  if (paths[index]=='S'){
    if (max(abs(countsLeft), abs(countsRight))>countsPerForward/2){
      Serial.println("hiii");
      Serial.println(countsLeftOffset);

      totalCount += (int16_t)countsPerForward/2;
      countsLeftOffset -= (int16_t)countsPerForward/2;
      countsRightOffset -= (int16_t)countsPerForward/2;
      index++;

    }
    adjustSpeeds();
    moveForward();
  }else if (paths[index]=='F'){
    if (max(abs(countsLeft), abs(countsRight))>countsPerForward){
      Serial.println("hiii");
      Serial.println(countsLeftOffset);
      totalCount += (int16_t)countsPerForward;
      countsLeftOffset -= (int16_t)countsPerForward;
      countsRightOffset -= (int16_t)countsPerForward;
      index++;

    }
    adjustSpeeds();
    moveForward();
  }else if (paths[index]=='B'){
    if (max(abs(countsLeft), abs(countsRight))>countsPerForward){
      Serial.println("hiii");
      Serial.println(countsLeftOffset);
      totalCount += (int16_t)countsPerForward;
      countsLeftOffset += (int16_t)countsPerForward;
      countsRightOffset += (int16_t)countsPerForward;
      index++;

    }
    adjustSpeeds();
    moveBackward();
  }else if (paths[index] == 'R'){
    if (max(abs(countsLeft), abs(countsRight))>countsPerTurn){
      Serial.println("hiii");
      Serial.println(countsLeftOffset);
      totalCount += (int16_t)countsPerTurn;
      countsLeftOffset -= (int16_t)countsPerTurn;
      countsRightOffset += (int16_t)countsPerTurn;
      index++;

    }
    adjustSpeeds();
    turnRight();
  }else if (paths[index] == 'L'){
    if (max(abs(countsLeft), abs(countsRight))>countsPerTurn){

      Serial.println("hiii");
      Serial.println(countsLeftOffset);
      totalCount += (int16_t)countsPerTurn;
      countsLeftOffset += (int16_t)countsPerTurn;
      countsRightOffset -= (int16_t)countsPerTurn;
      index++;

    }
    adjustSpeeds();
    turnLeft();
  } else{ // T
    stop();
    while(true){
      delay(1000);
    }
  }
}
