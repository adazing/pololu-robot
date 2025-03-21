#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Motors motors;
Encoders encoders;
ButtonA buttonA;

char paths[] = "SFFFLFLFFFRFRFFFLFLFFFRFRFFFT";
bool timeAdj = true;
bool encoderAdj = true;
bool fastTurns = false;

int index = 0;
float targetTime = 70.0;
float baseSpeed = 60.0;

float actionDelay = 1.0;

//angle PID
float kp = 1;
float error = 0.0;
float leftSpeed = baseSpeed;
float rightSpeed = baseSpeed;
float minSpeed = 15.0;
float maxSpeed = 170.0;

//time PID
int32_t maxCounts = 0;
float kpTime = 0.0005;
int32_t errorTime = 0;
uint32_t startTime = 0;  // Use uint32_t for consistency with millis()

// distances
const float wheelDiameter = 3.2;  // cm
const float wheelDistance = 8.65; 
const float countsPerRevolution = 358.32;

const float cmPerCount = (wheelDiameter * 3.14159) / countsPerRevolution * 27/25 * 71.5/75;
const float countsPerForward = 50 / cmPerCount;
const float countsPerTurn = wheelDistance * 3.14159 / 4 / cmPerCount - 1;

int32_t countsLeft = 0;
int32_t countsRight = 0;
int32_t countsLeftDist = 0;
int32_t countsRightDist = 0;
int32_t actionDist = 0;
int32_t countsLeftOffset = 0;
int32_t countsRightOffset = 0;
volatile int32_t totalCount = 0;

// Wrap encoder counts to avoid overflow (bounds check)
int32_t wrapEncoderCount(int32_t count) {
  // Handle overflow for 32-bit signed integer
  const int32_t MAX_COUNT = 32767;
  const int32_t MIN_COUNT = -32768;

  if (count > MAX_COUNT) {
    return MIN_COUNT + (count - MAX_COUNT - 1); // Wrap to negative range
  } else if (count < MIN_COUNT) {
    return MAX_COUNT - (MIN_COUNT - count - 1); // Wrap to positive range
  }
  return count;
}

void setup() {
  Serial.begin(9600);

  // Calculate maxCounts based on path
  for (int x = 0; x < strlen(paths); x++) {
    if (paths[x] == 'S') {
      maxCounts += (int32_t)(countsPerForward / 2);
    } else if (paths[x] == 'F' || paths[x] == 'B') {
      maxCounts += (int32_t)countsPerForward;
    } else if (paths[x] == 'L' || paths[x] == 'R') {
      maxCounts += (int32_t)countsPerTurn;
    }
  }

  targetTime -= (strlen(paths) - 2) * actionDelay;
  if (!timeAdj) kpTime = 0.0;
  if (!encoderAdj) kp = 0.0;

  delay(1000);
  startTime = millis();
}

void adjustSpeeds() {
  error = countsLeftDist - countsRightDist;

  // Prevent division by zero
  if (maxCounts == 0) maxCounts = 1;

  // Calculate time error
  errorTime = (int32_t)((totalCount + actionDist) / (float)maxCounts * targetTime * 1000 
              - (millis() - startTime - index * actionDelay * 1000));

  float adj_time = constrain(1 - kpTime * errorTime, 0.1, 2);

  if (!fastTurns && (paths[index] == 'R' || paths[index] == 'L')) {
    adj_time = 1.0;
  }

  leftSpeed = constrain(adj_time * (baseSpeed - kp * error), minSpeed, maxSpeed);
  rightSpeed = constrain(adj_time * (baseSpeed + kp * error), minSpeed, maxSpeed);
}

void finishDelay() {
  motors.setSpeeds(0, 0);
  uint32_t startDelayTime = millis();

  // Use non-blocking delay
  while (millis() - startDelayTime < (uint32_t)(actionDelay * 1000)) {
    // Do nothing
  }
}

void loop() {
  // Get current encoder counts and wrap them to avoid overflow
  countsLeft = wrapEncoderCount((int32_t)encoders.getCountsLeft() + countsLeftOffset);
  countsRight = wrapEncoderCount((int32_t)encoders.getCountsRight() + countsRightOffset);

  // Print the encoder counts for debugging
  Serial.print(countsLeft);
  Serial.print(" | ");
  Serial.println(countsRight);

  // Handle each action
  if (paths[index] == 'S') {
    countsLeftDist = countsLeft;
    countsRightDist = countsRight;
    actionDist = max(countsLeftDist, countsRightDist);

    if (actionDist > countsPerForward / 2) {
      totalCount += (int32_t)countsPerForward / 2;
      countsLeftOffset -= (int32_t)countsPerForward / 2;
      countsRightOffset -= (int32_t)countsPerForward / 2;
      finishDelay();
      index++;
    }

    adjustSpeeds();
    motors.setSpeeds(leftSpeed, rightSpeed);

  } else if (paths[index] == 'F') {
    countsLeftDist = countsLeft;
    countsRightDist = countsRight;
    actionDist = max(countsLeftDist, countsRightDist);

    if (actionDist > countsPerForward) {
      totalCount += (int32_t)countsPerForward;
      countsLeftOffset -= (int32_t)countsPerForward;
      countsRightOffset -= (int32_t)countsPerForward;
      finishDelay();
      index++;
    }

    adjustSpeeds();
    motors.setSpeeds(leftSpeed, rightSpeed);

  } else if (paths[index] == 'B') {
    countsLeftDist = -countsLeft;
    countsRightDist = -countsRight;
    actionDist = max(countsLeftDist, countsRightDist);

    if (actionDist > countsPerForward) {
      totalCount += (int32_t)countsPerForward;
      countsLeftOffset += (int32_t)countsPerForward;
      countsRightOffset += (int32_t)countsPerForward;
      finishDelay();
      index++;
    }

    adjustSpeeds();
    motors.setSpeeds(-leftSpeed, -rightSpeed);

  } else if (paths[index] == 'R') {
    countsLeftDist = countsLeft;
    countsRightDist = -countsRight;
    actionDist = max(countsLeftDist, countsRightDist);

    if (actionDist > countsPerTurn) {
      totalCount += (int32_t)countsPerTurn;
      countsLeftOffset -= (int32_t)countsPerTurn;
      countsRightOffset += (int32_t)countsPerTurn;
      finishDelay();
      index++;
    }

    adjustSpeeds();
    motors.setSpeeds(leftSpeed, -rightSpeed);

  } else if (paths[index] == 'L') {
    countsLeftDist = -countsLeft;
    countsRightDist = countsRight;
    actionDist = max(countsLeftDist, countsRightDist);

    if (actionDist > countsPerTurn) {
      totalCount += (int32_t)countsPerTurn;
      countsLeftOffset += (int32_t)countsPerTurn;
      countsRightOffset -= (int32_t)countsPerTurn;
      finishDelay();
      index++;
    }

    adjustSpeeds();
    motors.setSpeeds(-leftSpeed, rightSpeed);

  } else {  // 'T' - Terminate
    motors.setSpeeds(0, 0);
    while (true) {
      delay(1000);  // Infinite loop
    }
  }
}
