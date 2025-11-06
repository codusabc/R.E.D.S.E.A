// ===== Settings (아두이노 우노용) =====
const int ENA_PIN = 9;         // PWM 핀 (아두이노 우노 하드웨어 PWM)
const int IN1_PIN = 7;         // 방향 핀
const int IN2_PIN = 8;         // 방향 핀
const int SPEED_PERCENT = 50;   // 기본 듀티 사이클 (%)
const int STRONG_SPEED = 60;    // 강한 움직임 속도 (%)
const int WEAK_SPEED = 40;      // 약한 움직임 속도 (%)
const int PULSE_MS = 200;       // 각 스텝 유지 시간 (밀리초)
const int STRONG_PULSE_MS = 200; // 강한 움직임 시간 (밀리초)
const int WEAK_PULSE_MS = 100;   // 약한 움직임 시간 (밀리초)

// ===== 전역 변수 =====
String command = "";
bool commandReady = false;
bool isRunning = false;
unsigned long stepStartTime = 0;
int currentStep = 0;
int totalSteps = 4;
bool sequenceActive = false;

// 비대칭 시퀀스 구조: {방향, 속도, 시간}
// 방향: false=backward, true=forward
// 속도: 0=기본속도, 1=강한속도, 2=약한속도
// 시간: 0=기본시간, 1=긴시간, 2=짧은시간

struct SequenceStep {
  bool direction;  // 방향
  int speed;       // 속도 타입 (0=기본, 1=강함, 2=약함)
  int duration;    // 시간 타입 (0=기본, 1=긴, 2=짧음)
};

// RIGHT 시퀀스: 오른쪽으로 강하게 -> 왼쪽으로 약하게 -> 오른쪽으로 강하게 -> 왼쪽으로 약하게
SequenceStep rightSequence[4] = {
  {true, 1, 1},   // forward, 강한속도, 긴시간
  {false, 2, 2}, // backward, 약한속도, 짧은시간
  {true, 1, 1},   // forward, 강한속도, 긴시간
  {false, 2, 2}   // backward, 약한속도, 짧은시간
};

// LEFT 시퀀스: 왼쪽으로 강하게 -> 오른쪽으로 약하게 -> 왼쪽으로 강하게 -> 오른쪽으로 약하게
SequenceStep leftSequence[4] = {
  {false, 1, 1},  // backward, 강한속도, 긴시간
  {true, 2, 2},   // forward, 약한속도, 짧은시간
  {false, 1, 1},  // backward, 강한속도, 긴시간
  {true, 2, 2}    // forward, 약한속도, 짧은시간
};

SequenceStep* currentSequence = nullptr;

// 함수 선언
void motorForward(int speedPercent = SPEED_PERCENT);
void motorBackward(int speedPercent = SPEED_PERCENT);
int getStepSpeed(int speedType);
int getStepDuration(int durationType);

void setup() {
  Serial.begin(115200);
  
  // 핀 모드 설정
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  
  // 초기 상태: 모터 정지
  motorStop();
  
  Serial.println("Haptic Steering Node Ready (Arduino Uno)");
  Serial.println("Commands: LEFT, RIGHT, STOP, NONE");
  Serial.println("Pins: ENA=" + String(ENA_PIN) + ", IN1=" + String(IN1_PIN) + ", IN2=" + String(IN2_PIN));
  Serial.println("Speed: " + String(SPEED_PERCENT) + "%, Pulse: " + String(PULSE_MS) + "ms");
}

void loop() {
  // 시리얼 명령어 읽기
  readSerialCommand();
  
  // 명령어 처리
  if (commandReady) {
    processCommand();
    commandReady = false;
  }
  
  // 시퀀스 실행 (비블로킹)
  if (sequenceActive) {
    executeSequence();
  }

  Serial.println(analogRead(A0));
}

void readSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (command.length() > 0) {
        commandReady = true;
      }
    } else {
      command += c;
    }
  }
}

void processCommand() {
  command.trim();
  command.toUpperCase();
  
  Serial.println("Command received: " + command);
  
  if (command == "STOP" || command == "NONE" || command == "") {
    motorStop();
    sequenceActive = false;
    isRunning = false;
    command = "";
    return;
  }
  
  if (command == "LEFT") {
    if (isRunning) {
      Serial.println("Busy, command ignored");
      command = "";
      return;
    }
    startSequence(leftSequence);
    command = "";
    return;
  }
  
  if (command == "RIGHT") {
    if (isRunning) {
      Serial.println("Busy, command ignored");
      command = "";
      return;
    }
    startSequence(rightSequence);
    command = "";
    return;
  }
  
  Serial.println("Unknown command: " + command);
  command = "";
}

void startSequence(SequenceStep* sequence) {
  currentSequence = sequence;
  currentStep = 0;
  sequenceActive = true;
  isRunning = true;
  stepStartTime = millis();
  
  // 첫 번째 스텝 실행
  executeStep();
}

void executeSequence() {
  if (!sequenceActive) return;
  
  unsigned long currentTime = millis();
  
  // 현재 스텝의 시간 가져오기
  int stepDuration = getStepDuration(currentSequence[currentStep].duration);
  
  // 현재 스텝이 완료되었는지 확인
  if (currentTime - stepStartTime >= stepDuration) {
    currentStep++;
    
    if (currentStep >= totalSteps) {
      // 시퀀스 완료
      motorStop();
      sequenceActive = false;
      isRunning = false;
      Serial.println("Sequence completed");
    } else {
      // 다음 스텝 실행
      executeStep();
      stepStartTime = currentTime;
    }
  }
}

void executeStep() {
  if (!currentSequence) return;
  
  SequenceStep step = currentSequence[currentStep];
  int speed = getStepSpeed(step.speed);
  
  if (step.direction) {
    motorForward(speed);
  } else {
    motorBackward(speed);
  }
}

void motorForward(int speedPercent = SPEED_PERCENT) {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  int dutyCycle = map(speedPercent, 0, 100, 0, 255);
  analogWrite(ENA_PIN, dutyCycle);
}

void motorBackward(int speedPercent = SPEED_PERCENT) {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  int dutyCycle = map(speedPercent, 0, 100, 0, 255);
  analogWrite(ENA_PIN, dutyCycle);
}

int getStepSpeed(int speedType) {
  switch(speedType) {
    case 1: return STRONG_SPEED;  // 강한 속도
    case 2: return WEAK_SPEED;    // 약한 속도
    default: return SPEED_PERCENT; // 기본 속도
  }
}

int getStepDuration(int durationType) {
  switch(durationType) {
    case 1: return STRONG_PULSE_MS; // 긴 시간
    case 2: return WEAK_PULSE_MS;   // 짧은 시간
    default: return PULSE_MS;        // 기본 시간
  }
}

void motorStop() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(ENA_PIN, 0);
}
