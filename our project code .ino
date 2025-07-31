// === SMART VACUUM CLEANER WITH THREE MODES ===

#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

#define TRIG_PIN A0
#define ECHO_PIN A1
#define MAX_DISTANCE 200
#define MAX_SPEED 255
#define MAX_SPEED_OFFSET 20

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

Servo myservo;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

int distance = 100;
int spoint = 95;  // Servo center position
char command;    // Bluetooth/Voice command storage

void setup() {
  Serial.begin(9600);
  myservo.attach(10);
  myservo.write(spoint);
  delay(1000);
}

void loop() {
  // ======== Choose ONE mode ========
  //Obstacle();        // default mode
Bluetoothcontrol();
   //voicecontrol();
}

// ========== Obstacle Avoidance Mode ==========
void Obstacle() {
  distance = readPing();

  if (distance <= 15) {
    Stop();
    delay(200);

    moveBackward();
    delay(500);
    Stop();
    delay(200);

    int distanceR = lookRight();
    delay(200);
    int distanceL = lookLeft();
    delay(200);

    myservo.write(spoint);
    delay(300);

    if (distanceR >= distanceL) {
      turnRightHard();
      delay(500);
      moveForward();
    } else {
      turnLeftHard();
      delay(500);
      moveForward();
    }
  } else {
    moveForward();
  }
}

// ========== Bluetooth Control Mode ==========
void Bluetoothcontrol() {
  if (Serial.available()) {
    command = Serial.read();

    switch (command) {
      case 'F': moveForward(); break;
      case 'B': moveBackward(); break;
      case 'L': turnLeftHard(); break;
      case 'R': turnRightHard(); break;
      case 'S': Stop(); break;
    }
  }
}

// ========== Voice Control Mode ==========
void voicecontrol() {
  if (Serial.available()) {
    command = Serial.read();

    switch (command) {
      case 'f': moveForward(); break;
      case 'b': moveBackward(); break;
      case 'l': turnLeftHard(); break;
      case 'r': turnRightHard(); break;
      case 's': Stop(); break;
    }
  }
}

// ========== Motor Functions ==========
void moveForward() {
  motor1.setSpeed(MAX_SPEED);
  motor2.setSpeed(MAX_SPEED);
  motor3.setSpeed(MAX_SPEED);
  motor4.setSpeed(MAX_SPEED);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void moveBackward() {
  motor1.setSpeed(MAX_SPEED);
  motor2.setSpeed(MAX_SPEED);
  motor3.setSpeed(MAX_SPEED);
  motor4.setSpeed(MAX_SPEED);

  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void turnRightHard() {
  motor1.setSpeed(MAX_SPEED);
  motor2.setSpeed(MAX_SPEED);
  motor3.setSpeed(MAX_SPEED);
  motor4.setSpeed(MAX_SPEED);

  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}

void turnLeftHard() {
  motor1.setSpeed(MAX_SPEED);
  motor2.setSpeed(MAX_SPEED);
  motor3.setSpeed(MAX_SPEED);
  motor4.setSpeed(MAX_SPEED);

  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

void Stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// ========== Sensor and Servo Scanning ==========
int readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0) cm = 250;
  return cm;
}

int lookRight() {
  myservo.write(spoint + 50);
  delay(500);
  int distance = readPing();
  delay(100);
  return distance;
}

int lookLeft() {
  myservo.write(spoint - 50);
  delay(500);
  int distance = readPing();
  delay(100);
  return distance;
}
