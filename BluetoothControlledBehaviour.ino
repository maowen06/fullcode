#include <Arduino.h>

 

// ------------------- BLUETOOTH -------------------

#define BT_BAUD 9600

 

// ------------------- MOTOR DRIVER PINS -------------------

#define ENA_PIN 3      // Left motor speed

#define ENB_PIN 11     // Right motor speed

#define IN1_PIN A2     // Left motor direction

#define IN2_PIN A1

#define IN3_PIN 12     // Right motor direction

#define IN4_PIN 13

 

// ------------------- SETTINGS -------------------

const int SPEED = 200;

 

// ----------------------------------------------------------

// MOTOR CONTROL FUNCTIONS

// ----------------------------------------------------------

void forward(int leftSpeed, int rightSpeed) {

  analogWrite(ENA_PIN, leftSpeed);

  analogWrite(ENB_PIN, rightSpeed);

 

  digitalWrite(IN1_PIN, HIGH);

  digitalWrite(IN2_PIN, LOW);

  digitalWrite(IN3_PIN, HIGH);

  digitalWrite(IN4_PIN, LOW);

}

 

void reverse(int leftSpeed, int rightSpeed) {

  analogWrite(ENA_PIN, leftSpeed);

  analogWrite(ENB_PIN, rightSpeed);

 

  digitalWrite(IN1_PIN, LOW);

  digitalWrite(IN2_PIN, HIGH);

  digitalWrite(IN3_PIN, LOW);

  digitalWrite(IN4_PIN, HIGH);

}

 

void left(int leftSpeed, int rightSpeed) {

  analogWrite(ENA_PIN, leftSpeed);

  analogWrite(ENB_PIN, rightSpeed);

 

  digitalWrite(IN1_PIN, LOW);

  digitalWrite(IN2_PIN, HIGH);

  digitalWrite(IN3_PIN, HIGH);

  digitalWrite(IN4_PIN, LOW);

}

 

void right(int leftSpeed, int rightSpeed) {

  analogWrite(ENA_PIN, leftSpeed);

  analogWrite(ENB_PIN, rightSpeed);

 

  digitalWrite(IN1_PIN, HIGH);

  digitalWrite(IN2_PIN, LOW);

  digitalWrite(IN3_PIN, LOW);

  digitalWrite(IN4_PIN, HIGH);

}

 

void stopMotors() {

  analogWrite(ENA_PIN, 0);

  analogWrite(ENB_PIN, 0);

 

  digitalWrite(IN1_PIN, LOW);

  digitalWrite(IN2_PIN, LOW);

  digitalWrite(IN3_PIN, LOW);

  digitalWrite(IN4_PIN, LOW);

}

 

// ----------------------------------------------------------

// SETUP

// ----------------------------------------------------------

void setup() {

  Serial.begin(BT_BAUD);

 

  pinMode(ENA_PIN, OUTPUT);

  pinMode(ENB_PIN, OUTPUT);

  pinMode(IN1_PIN, OUTPUT);

  pinMode(IN2_PIN, OUTPUT);

  pinMode(IN3_PIN, OUTPUT);

  pinMode(IN4_PIN, OUTPUT);

 

  Serial.println("Bluetooth Motor Control Ready");

}

 

// ----------------------------------------------------------

// MAIN LOOP

// ----------------------------------------------------------

void loop() {

  if (Serial.available()) {

    char bt = Serial.read();

 

    if (bt == 'F') forward(SPEED, SPEED);

    if (bt == 'B') reverse(SPEED, SPEED);

    if (bt == 'L') left(SPEED, SPEED);

    if (bt == 'R') right(SPEED, SPEED);

    if (bt == 'S') stopMotors();

  }

}

