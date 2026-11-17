struct Motor {
  byte enPin;
  byte in1Pin;
  byte in2Pin;
  byte forwardIn1;
  byte forwardIn2;
};

enum MotorIndex {
  MOTOR_RIGHT = 0,
  MOTOR_LEFT = 1
};

// Array ของ struct
// มอเตอร์ขวาและซ้ายใช้สัญญาณเดินหน้าต่างกัน
const Motor motors[] = {
  {9, 13, 12, HIGH, LOW},  // Motor A = ล้อขวา
  {3, 11, 10, LOW, HIGH}   // Motor B = ล้อซ้าย
};

const byte MOTOR_COUNT =
    sizeof(motors) / sizeof(motors[0]);

// ===== Struct สำหรับเซนเซอร์ =====
struct UltrasonicSensor {
  byte trigPin;
  byte echoPin;
};

enum SensorIndex {
  SENSOR_RIGHT = 0,
  SENSOR_LEFT = 1,
  SENSOR_FRONT = 2
};

const UltrasonicSensor sensors[] = {
  {5, 4},  // ขวา
  {6, 7},  // ซ้าย
  {8, 2}   // หน้า
};

const byte SENSOR_COUNT =
    sizeof(sensors) / sizeof(sensors[0]);

// ปรับเวลาหมุนขวา หน่วยมิลลิวินาที
const unsigned long TURN_RIGHT_MS = 600UL;

bool finishedTurning = false;
bool turnReady = true;
int turnStep = 0;

// ===== ค่าการเดินเดิม =====
const int BASE_SPEED = 75;
const float KP = 2.5f;
const int MAX_CORRECTION = 20;

const float WALL_CM = 15.0f;
const float OPEN_CM = 20.0f;
const float DEADBAND_CM = 1.0f;

float targetWallCM = 10.0f;

bool junctionReady = true;
bool rightOpen = false;
bool leftOpen = false;

// ===== ค่าการหยุดและหมุนอยู่กับที่ =====
const int TURN_SPEED = 70;

const unsigned long BRAKE_BEFORE_TURN_MS = 200UL;
const unsigned long BRAKE_AFTER_TURN_MS = 150UL;

const unsigned long CORRECTION_MS_PER_UNIT = 4UL;
const unsigned long CORRECTION_MIN_MS = 40UL;
const unsigned long CORRECTION_MAX_MS = 80UL;

const unsigned long ADVANCE_AFTER_TURN_MS = 250UL;

bool advanceAfterTurnPending = false;
bool advanceAfterTurnRunning = false;
unsigned long advanceAfterTurnStartMs = 0;

// ===== ตัวแปรถอยตั้งหลัก =====
enum RecoveryState {
  REC_IDLE,
  REC_BACK,
  REC_SCAN,
  REC_VERIFY,
  REC_WAIT_CLEAR
};

RecoveryState recoveryState = REC_IDLE;

byte recoveryAttempts = 0;
byte recoveryClearCount = 0;

const float REC_FRONT_TRIGGER_CM = 8.0f;
const float REC_FRONT_CLEAR_CM = 12.0f;

const int REC_BACK_PWM = BASE_SPEED;
const unsigned long REC_BACK_MS = 250UL;

const unsigned long REC_LEFT_MS = 600UL;
const unsigned long REC_UTURN_MS = 1200UL;

const byte REC_MAX_ATTEMPTS = 3;

// ===== ประกาศฟังก์ชัน =====
void setMotorDirection(const Motor* motor, bool movingForward);
void setAllMotorDirections(bool movingForward);
void setAllMotorPWM(int speed);

float read_distance(const UltrasonicSensor* sensor);

void forward(int speed);
void backward(int speed);
void backward_seconds(float seconds, int speed);
void stop(int speed);
void forwardLR(int leftSpeed, int rightSpeed);

void hardBrake();
void cancelAdvanceAfterTurn();
void pivot_ms(bool turnLeft, unsigned long turnTimeMs);
void turn_left_ms(unsigned long turnTimeMs);
void turn_right_ms(unsigned long turnTimeMs);
bool advanceAfterTurn();

void navigate(float distance_1, float distance_2);

void recoveryReport(const char* message);
float recoveryReadCM(const UltrasonicSensor* sensor);
void recoveryFinished(float rightCM, float leftCM);
bool handleRecovery(float frontCM);

// ===== ควบคุมทิศทางผ่าน pointer =====
void setMotorDirection(const Motor* motor, bool movingForward) {
  digitalWrite(
    motor->in1Pin,
    movingForward ? motor->forwardIn1 : motor->forwardIn2
  );

  digitalWrite(
    motor->in2Pin,
    movingForward ? motor->forwardIn2 : motor->forwardIn1
  );
}

void setAllMotorDirections(bool movingForward) {
  // เลื่อน pointer ไปทีละสมาชิกใน array
  for (const Motor* motor = motors;
       motor < motors + MOTOR_COUNT;
       ++motor) {
    setMotorDirection(motor, movingForward);
  }
}

void setAllMotorPWM(int speed) {
  for (const Motor* motor = motors;
       motor < motors + MOTOR_COUNT;
       ++motor) {
    analogWrite(motor->enPin, speed);
  }
}

void setup() {
  Serial.begin(9600);

  for (const Motor* motor = motors;
       motor < motors + MOTOR_COUNT;
       ++motor) {
    pinMode(motor->enPin, OUTPUT);
    pinMode(motor->in1Pin, OUTPUT);
    pinMode(motor->in2Pin, OUTPUT);
  }

  for (const UltrasonicSensor* sensor = sensors;
       sensor < sensors + SENSOR_COUNT;
       ++sensor) {
    pinMode(sensor->trigPin, OUTPUT);
    pinMode(sensor->echoPin, INPUT);
    digitalWrite(sensor->trigPin, LOW);
  }
}

// ===== อ่านระยะผ่าน pointer =====
float read_distance(const UltrasonicSensor* sensor) {
  digitalWrite(sensor->trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(sensor->trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor->trigPin, LOW);

  unsigned long duration =
      pulseIn(sensor->echoPin, HIGH, 30000UL);

  if (duration == 0) {
    return 0;
  }

  return duration * 0.0343 / 2.0;
}

void loop() {
  float distance_1 =
      read_distance(sensors + SENSOR_RIGHT);
  delay(2);

  float distance_2 =
      read_distance(sensors + SENSOR_LEFT);
  delay(2);

  float distance_3 =
      read_distance(sensors + SENSOR_FRONT);
  delay(2);

  // ควบคุมมอเตอร์ก่อนพิมพ์ข้อความ
  if (!handleRecovery(distance_3)) {
    navigate(distance_1, distance_2);
  }

  Serial.print("Ultrasonic_1 : ");
  Serial.print(distance_1);
  Serial.print("cm  ");

  Serial.print("Ultrasonic_2 : ");
  Serial.print(distance_2);
  Serial.print("cm ");

  Serial.print("Ultrasonic_3 : ");
  Serial.print(distance_3);
  Serial.println("cm ");
}

// ===== เดินหน้าตรง =====
void forward(int speed) {
  setAllMotorDirections(true);
  setAllMotorPWM(speed);
}

// ===== ถอยหลังตรง =====
void backward(int speed) {
  cancelAdvanceAfterTurn();

  // ตัดกำลังมอเตอร์ทั้งสองก่อนเปลี่ยนทิศ
  setAllMotorPWM(0);
  setAllMotorDirections(false);
  setAllMotorPWM(speed);
}

void backward_seconds(float seconds, int speed) {
  if (seconds <= 0) return;

  hardBrake();
  delay(100);

  backward(speed);
  delay((unsigned long)(seconds * 1000.0f));

  hardBrake();
}

// ===== หยุดมอเตอร์ =====
// stop(255) = สั่งเบรกสำหรับ L298N
void stop(int speed) {
  for (const Motor* motor = motors;
       motor < motors + MOTOR_COUNT;
       ++motor) {
    digitalWrite(motor->in1Pin, LOW);
    digitalWrite(motor->in2Pin, LOW);
  }

  setAllMotorPWM(speed);

  // เวลาที่หยุดไม่นับเป็นช่วงเดินหลังหมุน
  advanceAfterTurnRunning = false;
}

void hardBrake() {
  stop(255);
}

// ===== ควบคุมล้อซ้าย–ขวา =====
void forwardLR(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setAllMotorPWM(0);
  setAllMotorDirections(true);

  // Pointer ชี้ไปยังสมาชิกของ array
  const Motor* rightMotor = motors + MOTOR_RIGHT;
  const Motor* leftMotor = motors + MOTOR_LEFT;

  analogWrite(rightMotor->enPin, rightSpeed);
  analogWrite(leftMotor->enPin, leftSpeed);
}

// ===== ยกเลิกช่วงเดินหน้าหลังหมุน =====
void cancelAdvanceAfterTurn() {
  advanceAfterTurnPending = false;
  advanceAfterTurnRunning = false;
}

// ===== ฟังก์ชันกลางสำหรับทุกการหมุน =====
void pivot_ms(bool turnLeft, unsigned long turnTimeMs) {
  hardBrake();

  cancelAdvanceAfterTurn();
  finishedTurning = false;

  if (turnTimeMs == 0) {
    finishedTurning = true;
    return;
  }

  Serial.println(turnLeft ? "TURN: LEFT" : "TURN: RIGHT");

  delay(BRAKE_BEFORE_TURN_MS);

  // ตัดกำลังเฉพาะตอนตั้งขาทิศทาง
  setAllMotorPWM(0);

  const Motor* rightMotor = motors + MOTOR_RIGHT;
  const Motor* leftMotor = motors + MOTOR_LEFT;

  // หมุนซ้าย: ขวาเดินหน้า ซ้ายถอยหลัง
  // หมุนขวา: ขวาถอยหลัง ซ้ายเดินหน้า
  setMotorDirection(rightMotor, turnLeft);
  setMotorDirection(leftMotor, !turnLeft);

  setAllMotorPWM(TURN_SPEED);

  delay(turnTimeMs);

  hardBrake();
  delay(BRAKE_AFTER_TURN_MS);

  finishedTurning = true;

  // กลับไปอ่านเซนเซอร์ใน loop รอบถัดไปก่อนเดิน
  advanceAfterTurnPending = true;
  advanceAfterTurnRunning = false;
}

void turn_left_ms(unsigned long turnTimeMs) {
  pivot_ms(true, turnTimeMs);
}

void turn_right_ms(unsigned long turnTimeMs) {
  pivot_ms(false, turnTimeMs);
}

// ===== เดินตรงหลังหมุน =====
bool advanceAfterTurn() {
  if (!advanceAfterTurnPending) {
    return false;
  }

  unsigned long now = millis();

  if (!advanceAfterTurnRunning) {
    advanceAfterTurnStartMs = now;
    advanceAfterTurnRunning = true;
  }

  if (now - advanceAfterTurnStartMs < ADVANCE_AFTER_TURN_MS) {
    forwardLR(BASE_SPEED, BASE_SPEED);
    return true;
  }

  advanceAfterTurnPending = false;
  advanceAfterTurnRunning = false;

  return false;
}

// ===== ประคองระยะด้วยการหยุดแล้วหมุนสั้น ๆ =====
void navigate(float distance_1, float distance_2) {
  if (distance_1 <= 0 || distance_2 <= 0) {
    hardBrake();
    return;
  }

  if (distance_1 >= OPEN_CM) {
    rightOpen = true;
  } else if (distance_1 <= WALL_CM) {
    rightOpen = false;
  }

  if (distance_2 >= OPEN_CM) {
    leftOpen = true;
  } else if (distance_2 <= WALL_CM) {
    leftOpen = false;
  }

  bool bothWalls = !rightOpen && !leftOpen;

  if (bothWalls) {
    targetWallCM = (distance_1 + distance_2) / 2.0f;
    junctionReady = true;
  }

  // หลังหมุน ต้องเดินตรงก่อนแก้ทิศรอบต่อไป
  if (advanceAfterTurn()) {
    return;
  }

  // เลี้ยวเข้าทางแยก
  if (junctionReady && leftOpen && !rightOpen) {
    junctionReady = false;
    turn_left_ms(400);
    return;
  }

  if (junctionReady && rightOpen && !leftOpen) {
    junctionReady = false;
    turn_right_ms(TURN_RIGHT_MS);
    return;
  }

  float error = 0.0f;

  if (bothWalls) {
    error = distance_2 - distance_1;
  } else if (!rightOpen) {
    error = targetWallCM - distance_1;
  } else if (!leftOpen) {
    error = distance_2 - targetWallCM;
  }

  if (error >= -DEADBAND_CM && error <= DEADBAND_CM) {
    forwardLR(BASE_SPEED, BASE_SPEED);
    return;
  }

  int correction = (int)(KP * error);

  correction = constrain(
    correction,
    -MAX_CORRECTION,
    MAX_CORRECTION
  );

  if (correction == 0) {
    forwardLR(BASE_SPEED, BASE_SPEED);
    return;
  }

  unsigned long magnitude =
      (unsigned long)(correction < 0 ? -correction : correction);

  unsigned long pulseMs =
      magnitude * CORRECTION_MS_PER_UNIT;

  pulseMs = constrain(
    pulseMs,
    CORRECTION_MIN_MS,
    CORRECTION_MAX_MS
  );

  // error บวก -> หมุนซ้าย
  // error ลบ -> หมุนขวา
  pivot_ms(correction > 0, pulseMs);

  // รอบถัดไปต้องอ่านระยะใหม่ก่อน
  return;
}

// ===== แสดงสถานะขณะรอ =====
void recoveryReport(const char* message) {
  static unsigned long lastReportMs = 0;
  unsigned long now = millis();

  if (now - lastReportMs >= 500UL) {
    Serial.println(message);
    lastReportMs = now;
  }
}

// ===== อ่านระยะใหม่ขณะหยุดตั้งหลัก =====
float recoveryReadCM(const UltrasonicSensor* sensor) {
  for (byte i = 0; i < 3; i++) {
    delay(60);

    float cm = read_distance(sensor);

    if (cm > 0) {
      return cm;
    }
  }

  return 0;
}

// ===== จบการตั้งหลัก =====
void recoveryFinished(float rightCM, float leftCM) {
  rightOpen = rightCM >= OPEN_CM;
  leftOpen = leftCM >= OPEN_CM;
  junctionReady = false;

  recoveryAttempts = 0;
  recoveryClearCount = 0;
  recoveryState = REC_IDLE;

  Serial.println("Recovery: READY");
}

// ===== ถอยตั้งหลักเมื่อด้านหน้าใกล้สิ่งกีดขวาง =====
bool handleRecovery(float frontCM) {
  if (frontCM <= 0) {
    hardBrake();
    recoveryClearCount = 0;
    recoveryReport("STOP: front sensor unreadable");
    return true;
  }

  if (recoveryState == REC_IDLE) {
    if (frontCM > REC_FRONT_TRIGGER_CM) {
      return false;
    }

    hardBrake();

    float confirmCM =
        recoveryReadCM(sensors + SENSOR_FRONT);

    if (confirmCM <= 0 ||
        confirmCM > REC_FRONT_TRIGGER_CM) {
      return true;
    }

    recoveryAttempts = 0;
    recoveryClearCount = 0;
    recoveryState = REC_BACK;
    return true;
  }

  hardBrake();

  // ถอย
  if (recoveryState == REC_BACK) {
    if (recoveryAttempts >= REC_MAX_ATTEMPTS) {
      recoveryState = REC_WAIT_CLEAR;
      recoveryClearCount = 0;

      recoveryReport("STOP: waiting for front clearance");
      return true;
    }

    recoveryAttempts++;

    Serial.println("Recovery: BACK");

    backward(REC_BACK_PWM);
    delay(REC_BACK_MS);
    hardBrake();
    delay(100);

    recoveryState = REC_SCAN;
    return true;
  }

  // รอทางโล่งหลังลองครบจำนวน
  if (recoveryState == REC_WAIT_CLEAR &&
      frontCM < REC_FRONT_CLEAR_CM) {
    recoveryClearCount = 0;
    recoveryReport("STOP: waiting for front clearance");
    return true;
  }

  float rightCM =
      recoveryReadCM(sensors + SENSOR_RIGHT);
  float leftCM =
      recoveryReadCM(sensors + SENSOR_LEFT);
  float aheadCM =
      recoveryReadCM(sensors + SENSOR_FRONT);

  if (rightCM <= 0 || leftCM <= 0 || aheadCM <= 0) {
    recoveryClearCount = 0;

    recoveryReport("STOP: waiting for valid sensor readings");
    return true;
  }

  // ยืนยันทางโล่ง
  if (recoveryState == REC_WAIT_CLEAR) {
    if (aheadCM >= REC_FRONT_CLEAR_CM) {
      recoveryClearCount++;

      if (recoveryClearCount >= 2) {
        recoveryFinished(rightCM, leftCM);
        return true;
      }
    } else {
      recoveryClearCount = 0;
    }

    recoveryReport("STOP: confirming front clearance");
    return true;
  }

  // เลือกทางหลังถอย
  if (recoveryState == REC_SCAN) {
    if (aheadCM <= REC_FRONT_TRIGGER_CM) {
      recoveryState = REC_BACK;
      return true;
    }

    bool canLeft = leftCM >= OPEN_CM;
    bool canRight = rightCM >= OPEN_CM;

    if (canLeft && (!canRight || leftCM >= rightCM)) {
      Serial.println("Recovery: LEFT");
      turn_left_ms(REC_LEFT_MS);
    } else if (canRight) {
      Serial.println("Recovery: RIGHT");
      turn_right_ms(TURN_RIGHT_MS);
    } else {
      Serial.println("Recovery: U-TURN");
      turn_left_ms(REC_UTURN_MS);
    }

    hardBrake();
    delay(100);

    recoveryState = REC_VERIFY;
    return true;
  }

  // ตรวจใหม่หลังหมุน
  if (aheadCM >= REC_FRONT_CLEAR_CM) {
    recoveryFinished(rightCM, leftCM);
    return true;
  }

  recoveryState = REC_BACK;
  return true;
}