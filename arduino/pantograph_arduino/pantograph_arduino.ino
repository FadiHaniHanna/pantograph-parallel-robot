// ========================================================================
// Smooth Infinity Pantograph Controller
//
// MATLAB sends:
//   HOME,theta1,theta2
//   BEGIN,N
//   P,theta1,theta2
//   P,theta1,theta2
//   ...
//   END
//   RUN
//   STOP
//
// Arduino replies:
//   READY
//   OKP
//   DONE
//   STOPPED
// ========================================================================

// ---------------- PIN DEFINITIONS ----------------
#define STEP1   3
#define DIR1    2
#define MS1_1   7
#define MS2_1   6

#define STEP2   9
#define DIR2    10
#define MS1_2   5
#define MS2_2   8

#define EN_PIN  4

// ---------------- DIRECTION SETTINGS ----------------
// If a motor moves opposite, change HIGH to LOW.
#define DIR1_POSITIVE HIGH
#define DIR2_POSITIVE HIGH

// ---------------- MOTOR SETTINGS ----------------
const int MOTOR_STEPS_PER_REV = 200;
const int MICROSTEP = 16;

// If you have a gear or pulley ratio, change these.
const float GEAR_RATIO1 = 1.0;
const float GEAR_RATIO2 = 1.0;

const float STEPS_PER_DEG1 = (MOTOR_STEPS_PER_REV * MICROSTEP * GEAR_RATIO1) / 360.0;
const float STEPS_PER_DEG2 = (MOTOR_STEPS_PER_REV * MICROSTEP * GEAR_RATIO2) / 360.0;

// ---------------- TRAJECTORY STORAGE ----------------
const int MAX_POINTS = 120;

long target1[MAX_POINTS];
long target2[MAX_POINTS];

int expectedPoints = 0;
int pointCount = 0;
bool receivingTrajectory = false;

// ---------------- SMOOTHNESS SETTINGS ----------------
// Increase SEGMENT_TIME_US if motion is still jerky.
// 120000 us = 0.12 sec per point.
// 90 points -> around 10.8 seconds total.
const unsigned long SEGMENT_TIME_US = 300000;

// Pulse delay limits
// Higher MIN_PULSE_US = slower and safer.
const unsigned int MIN_PULSE_US = 2500;
const unsigned int MAX_PULSE_US = 5000;

// ---------------- STATE ----------------
long currentStep1 = 0;
long currentStep2 = 0;

bool stopFlag = false;

String inputString = "";

void setup() {
  pinMode(STEP1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(MS1_1, OUTPUT);
  pinMode(MS2_1, OUTPUT);

  pinMode(STEP2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(MS1_2, OUTPUT);
  pinMode(MS2_2, OUTPUT);

  pinMode(EN_PIN, OUTPUT);

  // TMC2209 1/16 microstepping
  digitalWrite(MS1_1, HIGH);
  digitalWrite(MS2_1, HIGH);

  digitalWrite(MS1_2, HIGH);
  digitalWrite(MS2_2, HIGH);

  // Enable drivers, active LOW
  digitalWrite(EN_PIN, LOW);

  Serial.begin(115200);
  Serial.setTimeout(20);

  Serial.println("BOOT");
}

void loop() {
  if (Serial.available()) {
    inputString = Serial.readStringUntil('\n');
    inputString.trim();

    if (inputString.length() == 0) {
      return;
    }

    if (inputString == "STOP") {
      stopFlag = true;
      Serial.println("STOPPED");
      return;
    }

    if (inputString.startsWith("HOME")) {
      if (handleHome(inputString)) {
        Serial.println("READY");
      } else {
        Serial.println("ERR_HOME");
      }
      return;
    }

    if (inputString.startsWith("BEGIN")) {
      if (handleBegin(inputString)) {
        Serial.println("READY");
      } else {
        Serial.println("ERR_BEGIN");
      }
      return;
    }

    if (inputString.startsWith("P")) {
      if (handlePoint(inputString)) {
        Serial.println("OKP");
      } else {
        Serial.println("ERR_P");
      }
      return;
    }

    if (inputString == "END") {
      receivingTrajectory = false;

      if (pointCount == expectedPoints) {
        Serial.println("READY");
      } else {
        Serial.println("ERR_COUNT");
      }

      return;
    }

    if (inputString == "RUN") {
      stopFlag = false;
      runTrajectory();
      return;
    }
  }
}

// ========================================================================
// HOME command: HOME,theta1,theta2
// Does not move motors. It only syncs current step counters.
// ========================================================================
bool handleHome(String cmd) {
  float th1, th2;

  if (!parseTwoNumbersAfterFirstComma(cmd, th1, th2)) {
    return false;
  }

  currentStep1 = angleToSteps1(th1);
  currentStep2 = angleToSteps2(th2);

  stopFlag = false;

  return true;
}

// ========================================================================
// BEGIN command: BEGIN,N
// ========================================================================
bool handleBegin(String cmd) {
  int comma = cmd.indexOf(',');

  if (comma < 0) {
    return false;
  }

  expectedPoints = cmd.substring(comma + 1).toInt();

  if (expectedPoints <= 0 || expectedPoints > MAX_POINTS) {
    return false;
  }

  pointCount = 0;
  receivingTrajectory = true;

  return true;
}

// ========================================================================
// P command: P,theta1,theta2
// ========================================================================
bool handlePoint(String cmd) {
  if (!receivingTrajectory) {
    return false;
  }

  if (pointCount >= MAX_POINTS) {
    return false;
  }

  float th1, th2;

  if (!parseTwoNumbersAfterFirstComma(cmd, th1, th2)) {
    return false;
  }

  target1[pointCount] = angleToSteps1(th1);
  target2[pointCount] = angleToSteps2(th2);

  pointCount++;

  return true;
}

// ========================================================================
// RUN full trajectory
// ========================================================================
void runTrajectory() {
  if (pointCount <= 0) {
    Serial.println("ERR_EMPTY");
    return;
  }

  for (int i = 0; i < pointCount; i++) {
    if (stopFlag) {
      Serial.println("STOPPED");
      return;
    }

    moveSegmentTimed(target1[i], target2[i], segmentTimeForIndex(i, pointCount));
  }

  if (stopFlag) {
    Serial.println("STOPPED");
  } else {
    Serial.println("DONE");
  }
}

// ========================================================================
// Segment time ramp: slower at the beginning and end
// ========================================================================
unsigned long segmentTimeForIndex(int index, int total) {
  int rampPoints = 12;

  if (total < 2 * rampPoints) {
    rampPoints = total / 2;
  }

  if (rampPoints < 1) {
    return SEGMENT_TIME_US;
  }

  float scale = 1.0;

  if (index < rampPoints) {
    scale = 1.7 - 0.7 * ((float)index / rampPoints);
  } 
  else if ((total - 1 - index) < rampPoints) {
    int phase = total - 1 - index;
    scale = 1.7 - 0.7 * ((float)phase / rampPoints);
  } 
  else {
    scale = 1.0;
  }

  return (unsigned long)(SEGMENT_TIME_US * scale);
}

// ========================================================================
// Move both motors together to target steps within approximately segmentTime
// ========================================================================
void moveSegmentTimed(long targetStep1, long targetStep2, unsigned long segmentTimeUs) {
  long delta1 = targetStep1 - currentStep1;
  long delta2 = targetStep2 - currentStep2;

  int dir1 = delta1 >= 0 ? 1 : -1;
  int dir2 = delta2 >= 0 ? 1 : -1;

  digitalWrite(DIR1, dir1 > 0 ? DIR1_POSITIVE : !DIR1_POSITIVE);
  digitalWrite(DIR2, dir2 > 0 ? DIR2_POSITIVE : !DIR2_POSITIVE);

  long steps1 = labs(delta1);
  long steps2 = labs(delta2);

  long maxSteps = max(steps1, steps2);

  if (maxSteps == 0) {
    delayMicroseconds(1000);
    return;
  }

  unsigned long pulseDelay = segmentTimeUs / (2UL * maxSteps);

  if (pulseDelay < MIN_PULSE_US) {
    pulseDelay = MIN_PULSE_US;
  }

  if (pulseDelay > MAX_PULSE_US) {
    pulseDelay = MAX_PULSE_US;
  }

  long acc1 = 0;
  long acc2 = 0;

  for (long i = 0; i < maxSteps; i++) {
    checkStop();

    if (stopFlag) {
      return;
    }

    bool doStep1 = false;
    bool doStep2 = false;

    acc1 += steps1;
    acc2 += steps2;

    if (acc1 >= maxSteps) {
      acc1 -= maxSteps;
      doStep1 = true;
    }

    if (acc2 >= maxSteps) {
      acc2 -= maxSteps;
      doStep2 = true;
    }

    if (doStep1) {
      digitalWrite(STEP1, HIGH);
    }

    if (doStep2) {
      digitalWrite(STEP2, HIGH);
    }

    delayMicroseconds(pulseDelay);

    if (doStep1) {
      digitalWrite(STEP1, LOW);
      currentStep1 += dir1;
    }

    if (doStep2) {
      digitalWrite(STEP2, LOW);
      currentStep2 += dir2;
    }

    delayMicroseconds(pulseDelay);
  }
}

// ========================================================================
// Helpers
// ========================================================================
long angleToSteps1(float angleDeg) {
  return lround(angleDeg * STEPS_PER_DEG1);
}

long angleToSteps2(float angleDeg) {
  return lround(angleDeg * STEPS_PER_DEG2);
}

bool parseTwoNumbersAfterFirstComma(String cmd, float &a, float &b) {
  int comma1 = cmd.indexOf(',');
  int comma2 = cmd.indexOf(',', comma1 + 1);

  if (comma1 < 0 || comma2 < 0) {
    return false;
  }

  a = cmd.substring(comma1 + 1, comma2).toFloat();
  b = cmd.substring(comma2 + 1).toFloat();

  return true;
}

void checkStop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "STOP") {
      stopFlag = true;
    }
  }
}