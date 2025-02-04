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
// T = stop
char paths[] = "SFFT"
int index = 0;
float targetTime = 30.0;
float baseSpeed = 50.0;

//angle pid
float kp = 0.01;
float error = 0.0;
float leftSpeed = baseSpeed;
float rightSpeed = baseSpeed;
float minSpeed = 15.0;
float maxSpeed = 255.0;

//adjust time PID
int maxCounts = 0;
float kpTime = 0.001;
long errorTime = 0.0;
long startTime = 0.0;

// distances
const float wheelDiameter = 3.2; // cm
const float wheelDistance = 9.25; // distance between wheels
const float countsPerRevolution = 358.32; // Encoder counts per wheel revolution

const float cmPerCount = (wheelDiameter * 3.14159) / countsPerRevolution;
const float countsPerForward = 50 / cmPerCount;
const float countsPerTurn = wheelDistance * 3.14159 / 4 / cmPerCount;

int16_t countsLeft = 0;
int16_t countsRight = 0;


void setup() {
  // put your setup code here, to run once:
  maxCounts = countsPerForward/2; // start
  for (int x = 0; x<strlen(paths); x++){
    if (paths[x]=="F" || paths[x]=="B"){
      maxCounts += countsPerForward;
    } else if (paths[x]=="L" || paths[x]=="R"){
      maxCounts += countsPerTurn;
    }
  }
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
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();
  error = abs(countsLeft) - abs(countsRight);
  errorTime = (total_encoder_count + max(abs(encoderA_count), abs(encoderB_count)))/(float)maxCounts*targetTime*1000 - (millis()-startTime);
  // float adj_time = constrain(1 - kpTime*errorTime, 0.1, 2);
  float adj_time = 1;
  leftSpeed = constrain(adj_time*(baseSpeed - kp*error), 0, 400);
  rightSpeed = constrain(adj_time*(baseSpeed + kp*error), 0, 400);

}

void loop() {
  if (paths[index]="S"){
    countsLeft = encoders.getCountsLeft();
    countsRight = encoders.getCountsRight();
    if (max(abs(countsLeft), abs(countsRight))>countsPerForward/2){
      index++;
      total_encoder_count += countsPerForward/2;
      countsLeft -= countsPerForward/2;
      countsRight -= countsPerForward/2;
    }
    adjustSpeeds();
    moveForward();
  }else if (paths[index]="F"){
    countsLeft = encoders.getCountsLeft();
    countsRight = encoders.getCountsRight();
    if (max(abs(countsLeft), abs(countsRight))>countsPerForward){
      index++;
      total_encoder_count += countsPerForward;
      countsLeft -= countsPerForward;
      countsRight -= countsPerForward;
    }
    adjustSpeeds();
    moveForward();
  }else if (paths[index]="B"){
    countsLeft = encoders.getCountsLeft();
    countsRight = encoders.getCountsRight();
    if (max(abs(countsLeft), abs(countsRight))>countsPerForward){
      index++;
      total_encoder_count += countsPerForward;
      countsLeft += countsPerForward;
      countsRight += countsPerForward;
    }
    adjustSpeeds();
    moveBackward();
  }else if (paths[index] == "R"){
    if (max(abs(countsLeft), abs(countsRight))>countsPerTurn){
      index++;
      total_encoder_count += countsPerTurn;
      countsLeft -= countsPerTurn;
      countsRight += countsPerTurn;
    }
    adjustSpeeds();
    turnRight();
  }else if (paths[index] == "L"){
    if (max(abs(countsLeft), abs(countsRight))>countsPerTurn){
      index++;
      total_encoder_count += countsPerTurn;
      countsLeft += countsPerTurn;
      countsRight -= countsPerTurn;
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
