#include <NewPing.h>
#include <Servo.h>
#include <AFMotor.h>

// Pin definitions
#define RIGHT A2
#define LEFT A3
#define TRIGGER_PIN A1
#define ECHO_PIN A0
#define MAX_DISTANCE 200

#define BUZZER_PIN 8
#define LED_PIN 7

// Distance thresholds
#define MIN_SAFE_DISTANCE 12
#define MIN_FOLLOW_DISTANCE 18
#define MAX_FOLLOW_DISTANCE 100

// Speeds
#define NORMAL_SPEED 120
#define SLOW_SPEED 70
#define TURN_SPEED 130

// Timing
#define SENSOR_DELAY 60
#define TURN_TIME 150
#define BACKUP_TIME 500

// Robot states
enum RobotState {
  SCANNING,
  PERSON_DETECTED,
  FOLLOWING,
  TURNING_LEFT,
  TURNING_RIGHT,
  AVOIDING_OBSTACLE,
  LOST_PERSON
};

RobotState current_state = SCANNING;

// Sensor variables
unsigned int distance = 0;
unsigned int Right_Value = 0;
unsigned int Left_Value = 0;
unsigned long last_print_time = 0;

// Servo variables
bool servo_locked = false;
int locked_servo_position = 90;
unsigned long last_detection_time = 0;
int detection_count = 0;

// Scanning variables
int servo_pos = 90;
int scan_direction = 1;
unsigned long last_servo_move = 0;

// Direction memory
int last_seen_direction = 0; // -1 = left, 1 = right

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Motors
AF_DCMotor Motor1(1, MOTOR12_1KHZ);
AF_DCMotor Motor2(2, MOTOR12_1KHZ);
AF_DCMotor Motor3(3, MOTOR34_1KHZ);
AF_DCMotor Motor4(4, MOTOR34_1KHZ);

Servo myservo;

void setup() {
  Serial.begin(9600);
  Serial.println("=== HUMAN FOLLOWING ROBOT - FINAL VERSION ===");

  myservo.attach(10);
  myservo.write(90);
  delay(1000);

  pinMode(RIGHT, INPUT);
  pinMode(LEFT, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("Robot ready - Starting scan");
}

void loop() {
  delay(SENSOR_DELAY);
  readSensors();

  switch (current_state) {
    case SCANNING: scanForPerson(); break;
    case PERSON_DETECTED: lockAndTurnToPerson(); break;
    case FOLLOWING: followPerson(); break;
    case TURNING_LEFT: turnLeft(); current_state = FOLLOWING; break;
    case TURNING_RIGHT: turnRight(); current_state = FOLLOWING; break;
    case AVOIDING_OBSTACLE: avoidObstacle(); break;
    case LOST_PERSON: resumeScanning(); break;
  }

  printStatus();
}

void readSensors() {
  distance = sonar.ping_cm();
  if (distance == 0) distance = MAX_DISTANCE;
  Right_Value = digitalRead(RIGHT);
  Left_Value = digitalRead(LEFT);
}

void scanForPerson() {
  stopMotors();

  // IR Override: Left
  if (Left_Value == 0) {
    Serial.println("IR LEFT triggered - turning left until lost");
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    while (digitalRead(LEFT) == 0) {
      turnLeft();
    }
    stopMotors();
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
    current_state = PERSON_DETECTED;
    return;
  }

  // IR Override: Right
  if (Right_Value == 0) {
    Serial.println("IR RIGHT triggered - turning right until lost");
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    while (digitalRead(RIGHT) == 0) {
      turnRight();
    }
    stopMotors();
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
    current_state = PERSON_DETECTED;
    return;
  }

  // Regular scanning with servo
  if (millis() - last_servo_move > 400) {
    servo_pos += scan_direction * 20;

    if (servo_pos >= 150) {
      servo_pos = 150;
      scan_direction = -1;
    } else if (servo_pos <= 30) {
      servo_pos = 30;
      scan_direction = 1;
    }

    myservo.write(servo_pos);
    last_servo_move = millis();

    delay(200);
    unsigned int scan_distance = sonar.ping_cm();

    if (scan_distance >= MIN_FOLLOW_DISTANCE && scan_distance <= MAX_FOLLOW_DISTANCE) {
      detection_count++;
      Serial.print("Detection ");
      Serial.print(detection_count);
      Serial.print(" at ");
      Serial.print(scan_distance);
      Serial.print("cm, angle ");
      Serial.println(servo_pos);

      if (detection_count >= 2) {
        servo_locked = true;
        locked_servo_position = servo_pos;
        current_state = PERSON_DETECTED;
        detection_count = 0;
        Serial.println("*** PERSON CONFIRMED - SERVO LOCKED ***");
      }
    } else {
      detection_count = 0;
    }
  }
}

void lockAndTurnToPerson() {
  myservo.write(locked_servo_position);

  if (locked_servo_position > 105) {
    Serial.println("Turning RIGHT to face person");
    turnRight();
    delay(300);
    last_seen_direction = 1;
  } else if (locked_servo_position < 75) {
    Serial.println("Turning LEFT to face person");
    turnLeft();
    delay(300);
    last_seen_direction = -1;
  }

  myservo.write(90);
  servo_pos = 90;
  current_state = FOLLOWING;
  last_detection_time = millis();
  Serial.println("Starting to follow person...");
}

void followPerson() {
  if (distance < MIN_FOLLOW_DISTANCE || distance > MAX_FOLLOW_DISTANCE) {
    if (millis() - last_detection_time > 3000) {
      current_state = LOST_PERSON;
      return;
    }
  } else {
    last_detection_time = millis();
  }

  if (distance < MIN_SAFE_DISTANCE) {
    current_state = AVOIDING_OBSTACLE;
    return;
  }

  if (Left_Value == 0 && Right_Value == 1) {
    last_seen_direction = -1;
    current_state = TURNING_LEFT;
    return;
  } else if (Right_Value == 0 && Left_Value == 1) {
    last_seen_direction = 1;
    current_state = TURNING_RIGHT;
    return;
  }

  int follow_speed = map(distance, MIN_FOLLOW_DISTANCE, MAX_FOLLOW_DISTANCE, SLOW_SPEED, NORMAL_SPEED);
  follow_speed = constrain(follow_speed, SLOW_SPEED, NORMAL_SPEED);

  Motor1.setSpeed(follow_speed - 20); Motor1.run(FORWARD);
  Motor2.setSpeed(follow_speed - 20); Motor2.run(FORWARD);
  Motor3.setSpeed(follow_speed);      Motor3.run(FORWARD);
  Motor4.setSpeed(follow_speed);      Motor4.run(FORWARD);
}

void avoidObstacle() {
  Serial.println("Obstacle close — backing up");
  moveBackward();
  delay(BACKUP_TIME);
  current_state = FOLLOWING;
}

void resumeScanning() {
  Serial.println("Person lost — scanning again");
  servo_locked = false;

  if (last_seen_direction == -1) {
    turnLeft(); delay(400);
  } else if (last_seen_direction == 1) {
    turnRight(); delay(400);
  }

  current_state = SCANNING;
}

// ========== MOVEMENT FUNCTIONS ==========

void moveForward() {
  Motor1.setSpeed(NORMAL_SPEED - 20); Motor1.run(FORWARD);
  Motor2.setSpeed(NORMAL_SPEED - 20); Motor2.run(FORWARD);
  Motor3.setSpeed(NORMAL_SPEED);      Motor3.run(FORWARD);
  Motor4.setSpeed(NORMAL_SPEED);      Motor4.run(FORWARD);
}

void turnLeft() {
  Motor1.setSpeed(TURN_SPEED - 25); Motor1.run(BACKWARD);
  Motor2.setSpeed(TURN_SPEED - 25); Motor2.run(BACKWARD);
  Motor3.setSpeed(TURN_SPEED);      Motor3.run(FORWARD);
  Motor4.setSpeed(TURN_SPEED);      Motor4.run(FORWARD);
  delay(TURN_TIME);
  stopMotors();
}

void turnRight() {
  Motor1.setSpeed(TURN_SPEED - 25); Motor1.run(FORWARD);
  Motor2.setSpeed(TURN_SPEED - 25); Motor2.run(FORWARD);
  Motor3.setSpeed(TURN_SPEED);      Motor3.run(BACKWARD);
  Motor4.setSpeed(TURN_SPEED);      Motor4.run(BACKWARD);
  delay(TURN_TIME);
  stopMotors();
}

void moveBackward() {
  Motor1.setSpeed(SLOW_SPEED); Motor1.run(BACKWARD);
  Motor2.setSpeed(SLOW_SPEED); Motor2.run(BACKWARD);
  Motor3.setSpeed(SLOW_SPEED); Motor3.run(BACKWARD);
  Motor4.setSpeed(SLOW_SPEED); Motor4.run(BACKWARD);
}

void stopMotors() {
  Motor1.run(RELEASE);
  Motor2.run(RELEASE);
  Motor3.run(RELEASE);
  Motor4.run(RELEASE);
}

void printStatus() {
  if (millis() - last_print_time >= 1500) {
    Serial.println("========= ROBOT STATUS =========");
    Serial.print("State: ");
    switch (current_state) {
      case SCANNING: Serial.println("SCANNING"); break;
      case PERSON_DETECTED: Serial.println("PERSON DETECTED"); break;
      case FOLLOWING: Serial.println("FOLLOWING"); break;
      case TURNING_LEFT: Serial.println("TURNING LEFT"); break;
      case TURNING_RIGHT: Serial.println("TURNING RIGHT"); break;
      case AVOIDING_OBSTACLE: Serial.println("AVOIDING OBSTACLE"); break;
      case LOST_PERSON: Serial.println("LOST PERSON"); break;
    }

    Serial.print("Distance: "); Serial.print(distance); Serial.println("cm");
    Serial.print("IR Sensors - Left: "); Serial.print(Left_Value);
    Serial.print(", Right: "); Serial.println(Right_Value);

    if (servo_locked) {
      Serial.print("Servo: LOCKED at "); Serial.println(locked_servo_position);
    } else {
      Serial.print("Servo: Scanning at "); Serial.println(servo_pos);
    }

    Serial.println("==============================\n");
    last_print_time = millis();
  }
}
