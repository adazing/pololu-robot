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
char paths[] = "SFFFT";
bool timeAdj = true;
bool encoderAdj = true;
// char paths[] = "S";
int index = 0;
float targetTime = 10.0;
float baseSpeed = 100.0;

//angle pid
float kp = 1;
float error = 0.0;
float leftSpeed = baseSpeed;
float rightSpeed = baseSpeed;
float minSpeed = 40.0;
float maxSpeed = 255.0;

//adjust time PID
int maxCounts = 0;
float kpTime = 0.005;
long errorTime = 0.0;
long startTime = 0.0;
// float timeOffset = 0;
long startActionTime = 0.0;
bool startedAction = true;

// distances
const float wheelDiameter = 3.2; // cm
const float wheelDistance = 8.65; // distance between wheels
const float countsPerRevolution = 358.32; // Encoder counts per wheel revolution

const float cmPerCount = (wheelDiameter * 3.14159) / countsPerRevolution * 27/25 * 71.5/75;
const float countsPerForward = 50 / cmPerCount;
const float countsPerTurn = wheelDistance * 3.14159 / 4 / cmPerCount;

int16_t countsLeft = 0;
int16_t countsRight = 0;
int16_t countsLeftDist = 0;
int16_t countsRightDist = 0;
int16_t actionDist = 0;
int16_t countsLeftOffset = 0;
int16_t countsRightOffset = 0;
volatile int totalCount = 0;
volatile int targetActionCount = 0;
float targetActionTime = 0;
float timeAccelProp = 0.2; // proportion of action spent accelerating and descelerating

// acceleration math
float kAccel = 0.001;
float VM = 0.0;

// int16_t accelDiff = 50; // speed diff between slow and medium and medium and high

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
  if (!timeAdj){
    kpTime = 0.0;
  }
  if (!encoderAdj){
    kp = 0.0;
  }
  delay(5000);
  startTime = millis();
  Serial.println("hi");
}

void calcVM(){
    if (pow(targetActionTime, 2)<4*kAccel*targetActionCount){
    VM = targetActionTime/(2*kAccel);
    Serial.println("cry1");
  }else{
    VM = (targetActionTime - sqrt(pow(targetActionTime, 2) - 4*kAccel*targetActionCount))/(2*kAccel);
  }
  if (2*kAccel*VM > targetActionTime){ // fallback --> cry and cry and cry omfg what the heck is happening
    Serial.println("cry2");
    VM = targetActionTime/(2*kAccel);
  }
}

void adjustSpeeds() {
  error = countsLeftDist - countsRightDist;
  float pred_ticks = 0.0;
  float time_since_action_started = ((float)millis()-(float)startActionTime)/1000.0;
  Serial.println(kAccel*VM);
  Serial.println(time_since_action_started);
  Serial.println(1/kAccel);
  // Serial.println(startActionTime);
  if (time_since_action_started<kAccel*VM){
    Serial.println(1);
    pred_ticks = (1/kAccel)*pow(time_since_action_started, 2)/2;
  }else if (time_since_action_started<targetActionTime-kAccel*VM){
    Serial.println(2);
    pred_ticks = VM*kAccel*VM/2+VM*(time_since_action_started-VM*kAccel);
  }else if (time_since_action_started<targetActionTime){
    Serial.println(3);
    pred_ticks = VM*kAccel*VM/2+(targetActionTime - 2*VM*kAccel)*VM + 1/kAccel*pow(time_since_action_started-(targetActionTime - kAccel*VM), 2)/2 + VM*(time_since_action_started - (targetActionTime - kAccel*VM));
  }
    Serial.print(leftSpeed);
    Serial.print(" | ");
    Serial.println(rightSpeed);
  // Serial.print(pred_ticks);
  // Serial.print(" ");
  // Serial.println(actionDist);
  errorTime = actionDist - pred_ticks;
  // errorTime = (totalCount + actionDist)/(float)maxCounts*targetTime*1000 - (millis()-startTime);
  float adj_time = constrain(1 - kpTime*errorTime, 0.1, 2);
  // Serial.println(adj_time);
  leftSpeed = constrain(adj_time*(baseSpeed) - kp*error, minSpeed, maxSpeed);
  rightSpeed = constrain(adj_time*(baseSpeed) + kp*error, minSpeed, maxSpeed);
}

void loop() {
    countsLeft = encoders.getCountsLeft() + countsLeftOffset;
    countsRight = encoders.getCountsRight() + countsRightOffset;

  if (paths[index]=='S'){
    countsLeftDist = countsLeft;
    countsRightDist = countsRight;
    actionDist = max(countsLeftDist, countsRightDist);
    targetActionCount = countsPerForward/2;
    // targetActionTime = ((startTime + targetTime*1000)-startActionTime)*(float)targetActionCount/((float)maxCounts-(float)totalCount)/1000.0;
    // targetActionTime = targetTime * (float)targetActionCount/(float)maxCounts;

    if (startedAction){
      calcVM();
      startActionTime = millis();
      targetActionTime = ((startTime + targetTime*1000)-startActionTime)*(float)targetActionCount/((float)maxCounts-(float)totalCount)/1000.0;
      startedAction = false;
    }

    if (actionDist > targetActionCount){
      // timeOffset += targetActionTime - ((float)millis()-(float)startActionTime)/1000.0;
      totalCount += (int16_t)targetActionCount;
      countsLeftOffset -= (int16_t)targetActionCount;
      countsRightOffset -= (int16_t)targetActionCount;
      index++;
      startedAction = true;
    }
    adjustSpeeds();
    motors.setSpeeds(leftSpeed, rightSpeed);
  }else if (paths[index]=='F'){
    countsLeftDist = countsLeft;
    countsRightDist = countsRight;
    actionDist = max(countsLeftDist, countsRightDist);
    targetActionCount = countsPerForward;
    // targetActionTime = targetTime * (float)targetActionCount/(float)maxCounts;
    if (startedAction){
      calcVM();
      startActionTime = millis();
      targetActionTime = ((startTime + targetTime*1000)-startActionTime)*(float)targetActionCount/((float)maxCounts-(float)totalCount)/1000.0;
      startedAction = false;
    }

    if (actionDist>targetActionCount){
      totalCount += (int16_t)targetActionCount;
      countsLeftOffset -= (int16_t)targetActionCount;
      countsRightOffset -= (int16_t)targetActionCount;
      index++;
      startedAction = true;
    }
    adjustSpeeds();
    motors.setSpeeds(leftSpeed, rightSpeed);
  }else if (paths[index]=='B'){
    countsLeftDist = -countsLeft;
    countsRightDist = -countsRight;
    actionDist = max(countsLeftDist, countsRightDist);
    targetActionCount = countsPerForward;
    // targetActionTime = targetTime * (float)targetActionCount/(float)maxCounts;
    if (startedAction){
      calcVM();
      startActionTime = millis();
      targetActionTime = ((startTime + targetTime*1000)-startActionTime)*(float)targetActionCount/((float)maxCounts-(float)totalCount)/1000.0;
      startedAction = false;
    }
    if (actionDist>targetActionCount){
      totalCount += (int16_t)targetActionCount;
      countsLeftOffset += (int16_t)targetActionCount;
      countsRightOffset += (int16_t)targetActionCount;
      index++;
      startedAction = true;
    }
    adjustSpeeds();
    motors.setSpeeds(-leftSpeed, -rightSpeed);
  }else if (paths[index] == 'R'){
    countsLeftDist = countsLeft;
    countsRightDist = -countsRight;
    actionDist = max(countsLeftDist, countsRightDist);
    targetActionCount = countsPerTurn;
    // targetActionTime = targetTime * (float)targetActionCount/(float)maxCounts;
    if (startedAction){
      calcVM();
      startActionTime = millis();
      targetActionTime = ((startTime + targetTime*1000)-startActionTime)*(float)targetActionCount/((float)maxCounts-(float)totalCount)/1000.0;
      startedAction = false;
    }
    if (actionDist>targetActionCount){
      totalCount += (int16_t)targetActionCount;
      countsLeftOffset -= (int16_t)targetActionCount;
      countsRightOffset += (int16_t)targetActionCount;
      index++;
      startedAction = true;
    }
    adjustSpeeds();
    motors.setSpeeds(leftSpeed, -rightSpeed);
  }else if (paths[index] == 'L'){
    countsLeftDist = -countsLeft;
    countsRightDist = countsRight;
    actionDist = max(countsLeftDist, countsRightDist);
    targetActionCount = countsPerTurn;
    // targetActionTime = targetTime * (float)targetActionCount/(float)maxCounts;
    if (startedAction){
      calcVM();
      startActionTime = millis();
      targetActionTime = ((startTime + targetTime*1000)-startActionTime)*(float)targetActionCount/((float)maxCounts-(float)totalCount)/1000.0;
      startedAction = false;
    }
    if (actionDist>targetActionCount){

      totalCount += (int16_t)targetActionCount;
      countsLeftOffset += (int16_t)targetActionCount;
      countsRightOffset -= (int16_t)targetActionCount;
      index++;
      startedAction = true;

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
