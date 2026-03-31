const int STEP_PIN = 22;
const int DIR_PIN = 23;
const int ENABLE_PIN = -1;
const bool DIR_RIGHT_HIGH = true;
const unsigned long STEP_PULSE_HIGH_US = 12;
const unsigned long STATUS_INTERVAL_MS = 1000;
const unsigned long HOME_STEP_INTERVAL_US = 850;
const int MAX_SPEED_LEVEL = 6;
const unsigned long STEP_INTERVALS_US[MAX_SPEED_LEVEL + 1] = {0, 1700, 1300, 1000, 760, 600, 480};

String rxBuffer;
int motionVelocity = 0;
bool homeActive = false;
unsigned long lastStepMicros = 0;
unsigned long lastStatusMillis = 0;
long basePosition = 0;

void setup() {
  Serial.begin(115200);
  rxBuffer.reserve(64);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  if (ENABLE_PIN >= 0) {
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
  }

  Serial.println("READY BASE_MOUSE_SERIAL");
  Serial.println("COMMANDS: LEFT | RIGHT | STOP | STATUS | VEL n | HOME");
}

void loop() {
  readSerial();
  updateMotion();
  publishStatus();
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
    if (rxBuffer.length() > 60) {
      rxBuffer = "";
      Serial.println("ERR LONG_CMD");
    }
  }
}

void handleCommand(const String &cmd) {
  int velocity = 0;
  if (sscanf(cmd.c_str(), "VEL %d", &velocity) == 1) {
    if (velocity > MAX_SPEED_LEVEL) {
      velocity = MAX_SPEED_LEVEL;
    } else if (velocity < -MAX_SPEED_LEVEL) {
      velocity = -MAX_SPEED_LEVEL;
    }
    homeActive = false;
    motionVelocity = velocity;
    Serial.print("OK VEL ");
    Serial.println(motionVelocity);
    return;
  }

  if (cmd == "LEFT") {
    homeActive = false;
    motionVelocity = -2;
    Serial.println("OK LEFT");
    return;
  }

  if (cmd == "RIGHT") {
    homeActive = false;
    motionVelocity = 2;
    Serial.println("OK RIGHT");
    return;
  }

  if (cmd == "STOP") {
    homeActive = false;
    motionVelocity = 0;
    Serial.println("OK STOP");
    return;
  }

  if (cmd == "HOME") {
    motionVelocity = 0;
    homeActive = true;
    Serial.println("OK HOME");
    return;
  }

  if (cmd == "STATUS") {
    Serial.print("HOME=");
    Serial.print(homeActive ? 1 : 0);
    Serial.print(" VEL=");
    Serial.print(motionVelocity);
    Serial.print(" POS=");
    Serial.println(basePosition);
    return;
  }

  Serial.print("ERR UNKNOWN: ");
  Serial.println(cmd);
}

void stepMotor(bool moveRight, unsigned long stepIntervalUs) {
  unsigned long now = micros();
  if (now - lastStepMicros < stepIntervalUs) {
    return;
  }
  lastStepMicros = now;

  bool dirSignal = moveRight ? DIR_RIGHT_HIGH : (DIR_RIGHT_HIGH ? false : true);
  digitalWrite(DIR_PIN, dirSignal ? HIGH : LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_HIGH_US);
  digitalWrite(STEP_PIN, LOW);
  basePosition += moveRight ? 1 : -1;
}

void updateMotion() {
  if (homeActive) {
    if (basePosition == 0) {
      homeActive = false;
      motionVelocity = 0;
      Serial.println("OK HOME_DONE");
      return;
    }
    stepMotor(basePosition < 0, HOME_STEP_INTERVAL_US);
    return;
  }

  if (motionVelocity == 0) {
    return;
  }

  int speedLevel = abs(motionVelocity);
  if (speedLevel > MAX_SPEED_LEVEL) {
    speedLevel = MAX_SPEED_LEVEL;
  }
  stepMotor(motionVelocity > 0, STEP_INTERVALS_US[speedLevel]);
}

void publishStatus() {
  unsigned long now = millis();
  if (now - lastStatusMillis < STATUS_INTERVAL_MS) {
    return;
  }
  lastStatusMillis = now;

  Serial.print("STATUS HOME=");
  Serial.print(homeActive ? 1 : 0);
  Serial.print(" VEL=");
  Serial.print(motionVelocity);
  Serial.print(" POS=");
  Serial.println(basePosition);
}
