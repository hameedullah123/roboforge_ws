// ROS / RViz drag controller firmware for 4 stepper joints.
// Protocol expected by the current Python bridge:
//   MODE ROS
//   MOVEABS s0 s1 s2 s3
//   HOME s0 s1 s2 s3
// Optional commands:
//   MODE IDLE
//   ZERO
//   STATUS

#define NUM_MOTORS 4

const int stepPins[NUM_MOTORS] = {22, 24, 26, 28};
const int dirPins[NUM_MOTORS]  = {23, 25, 27, 29};
const int limitPins[NUM_MOTORS] = {-1, 30, 31, -1};
const int homeSearchDirection[NUM_MOTORS] = {0, 1, 1, 0};
const bool limitActiveState = LOW;
const unsigned long homePulseDelayUs = 700;
const long homeSearchMaxSteps = 40000;
const long homeBackoffSteps = 160;

// Tune per joint if a joint moves opposite to the commanded direction.
const bool motorDir[NUM_MOTORS] = {true, true, true, true};

String mode = "IDLE";  // "IDLE" or "ROS"

volatile long currentPos[NUM_MOTORS] = {0, 0, 0, 0};
volatile long targetPos[NUM_MOTORS]  = {0, 0, 0, 0};

unsigned long lastStepMicros[NUM_MOTORS] = {0, 0, 0, 0};
unsigned long minStepIntervalUs[NUM_MOTORS] = {500, 650, 650, 700};

String rxBuffer;

void setup() {
  Serial.begin(115200);
  rxBuffer.reserve(96);

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
    digitalWrite(stepPins[i], LOW);
    digitalWrite(dirPins[i], LOW);
    if (limitPins[i] >= 0) {
      pinMode(limitPins[i], INPUT_PULLUP);
    }
    lastStepMicros[i] = micros();
  }

  Serial.println("READY");
  Serial.println("MODE IDLE");
  Serial.println("COMMANDS: MODE ROS | MODE IDLE | MOVEABS s0 s1 s2 s3 | HOME s0 s1 s2 s3 | ZERO | STATUS");
}

void loop() {
  readSerial();

  if (mode == "ROS") {
    updateMotorsROS();
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      rxBuffer.trim();
      if (rxBuffer.length() > 0) {
        handleCommand(rxBuffer);
      }
      rxBuffer = "";
      continue;
    }

    rxBuffer += c;

    if (rxBuffer.length() > 90) {
      rxBuffer = "";
      Serial.println("ERR LONG_CMD");
    }
  }
}

void handleCommand(const String &cmd) {
  if (cmd == "MODE ROS") {
    mode = "ROS";
    Serial.println("OK ROS");
    return;
  }

  if (cmd == "MODE IDLE" || cmd == "MODE MANUAL") {
    mode = "IDLE";
    Serial.println("OK IDLE");
    return;
  }

  if (cmd == "ZERO") {
    noInterrupts();
    for (int i = 0; i < NUM_MOTORS; i++) {
      currentPos[i] = 0;
      targetPos[i] = 0;
    }
    interrupts();
    Serial.println("OK ZERO");
    return;
  }

  if (cmd == "STATUS") {
    Serial.print("MODE=");
    Serial.print(mode);
    Serial.print(" CUR=");
    Serial.print(currentPos[0]); Serial.print(',');
    Serial.print(currentPos[1]); Serial.print(',');
    Serial.print(currentPos[2]); Serial.print(',');
    Serial.print(currentPos[3]);
    Serial.print(" TAR=");
    Serial.print(targetPos[0]); Serial.print(',');
    Serial.print(targetPos[1]); Serial.print(',');
    Serial.print(targetPos[2]); Serial.print(',');
    Serial.println(targetPos[3]);
    return;
  }

  if (cmd.startsWith("HOME")) {
    long s0, s1, s2, s3;
    int parsed = sscanf(cmd.c_str(), "HOME %ld %ld %ld %ld", &s0, &s1, &s2, &s3);

    if (parsed == 4) {
      long homeOffsets[NUM_MOTORS] = {s0, s1, s2, s3};
      bool ok = performHoming(homeOffsets);
      if (ok) {
        Serial.println("OK HOME");
      }
    } else {
      Serial.println("ERR HOME_PARSE");
    }
    return;
  }

  if (cmd.startsWith("MOVEABS")) {
    long s0, s1, s2, s3;
    int parsed = sscanf(cmd.c_str(), "MOVEABS %ld %ld %ld %ld", &s0, &s1, &s2, &s3);

    if (parsed == 4) {
      noInterrupts();
      targetPos[0] = s0;
      targetPos[1] = s1;
      targetPos[2] = s2;
      targetPos[3] = s3;
      interrupts();
      Serial.println("OK MOVEABS");
    } else {
      Serial.println("ERR MOVEABS");
    }
    return;
  }

  Serial.print("ERR UNKNOWN: ");
  Serial.println(cmd);
}

bool performHoming(const long homeOffsets[NUM_MOTORS]) {
  String previousMode = mode;
  mode = "IDLE";

  for (int i = 0; i < NUM_MOTORS; i++) {
    if (limitPins[i] >= 0) {
      if (!homeJointToSwitch(i)) {
        Serial.print("ERR HOME ");
        Serial.println(i);
        mode = previousMode;
        return false;
      }
    }
  }

  noInterrupts();
  for (int i = 0; i < NUM_MOTORS; i++) {
    currentPos[i] = homeOffsets[i];
    targetPos[i] = homeOffsets[i];
    lastStepMicros[i] = micros();
  }
  interrupts();

  mode = previousMode;
  return true;
}

bool homeJointToSwitch(int index) {
  if (limitPins[index] < 0 || homeSearchDirection[index] == 0) {
    return true;
  }

  if (isLimitTriggered(index)) {
    for (long step = 0; step < homeBackoffSteps; step++) {
      if (!isLimitTriggered(index)) {
        break;
      }
      pulseMotorStep(index, -homeSearchDirection[index]);
    }
  }

  for (long step = 0; step < homeSearchMaxSteps; step++) {
    if (isLimitTriggered(index)) {
      break;
    }
    pulseMotorStep(index, homeSearchDirection[index]);
  }

  if (!isLimitTriggered(index)) {
    return false;
  }

  for (long step = 0; step < homeBackoffSteps; step++) {
    pulseMotorStep(index, -homeSearchDirection[index]);
  }

  for (long step = 0; step < homeBackoffSteps * 2; step++) {
    if (isLimitTriggered(index)) {
      break;
    }
    pulseMotorStep(index, homeSearchDirection[index]);
  }

  if (!isLimitTriggered(index)) {
    return false;
  }

  currentPos[index] = 0;
  targetPos[index] = 0;
  return true;
}

bool isLimitTriggered(int index) {
  if (limitPins[index] < 0) {
    return false;
  }
  return digitalRead(limitPins[index]) == limitActiveState;
}

void pulseMotorStep(int index, int moveDirection) {
  bool positiveMove = (moveDirection > 0);
  bool dirSignal = positiveMove ? motorDir[index] : !motorDir[index];
  digitalWrite(dirPins[index], dirSignal ? HIGH : LOW);
  pulseStep(stepPins[index]);
  currentPos[index] += positiveMove ? 1 : -1;
  delayMicroseconds(homePulseDelayUs);
}

void updateMotorsROS() {
  unsigned long now = micros();
  unsigned long moveDurationUs = 0;
  long remaining[NUM_MOTORS] = {0, 0, 0, 0};
  bool hasMotion = false;

  for (int i = 0; i < NUM_MOTORS; i++) {
    remaining[i] = targetPos[i] - currentPos[i];
    long distance = labs(remaining[i]);
    if (distance == 0) {
      continue;
    }

    hasMotion = true;
    unsigned long jointDurationUs = (unsigned long)distance * minStepIntervalUs[i];
    if (jointDurationUs > moveDurationUs) {
      moveDurationUs = jointDurationUs;
    }
  }

  if (!hasMotion) {
    return;
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    long delta = remaining[i];
    if (delta == 0) {
      continue;
    }

    unsigned long distance = (unsigned long)labs(delta);
    unsigned long coordinatedIntervalUs = moveDurationUs / distance;
    if (coordinatedIntervalUs < minStepIntervalUs[i]) {
      coordinatedIntervalUs = minStepIntervalUs[i];
    }

    if (now - lastStepMicros[i] < coordinatedIntervalUs) {
      continue;
    }

    bool positiveMove = (delta > 0);
    bool dirSignal = positiveMove ? motorDir[i] : !motorDir[i];

    digitalWrite(dirPins[i], dirSignal ? HIGH : LOW);
    pulseStep(stepPins[i]);

    currentPos[i] += positiveMove ? 1 : -1;
    lastStepMicros[i] = now;
  }
}

void pulseStep(int pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(3);
  digitalWrite(pin, LOW);
}
