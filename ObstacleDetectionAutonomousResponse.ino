#include <Arduino.h>

 

// ------------------- ULTRASONIC SENSOR -------------------

#define TRIG_PIN A4

#define ECHO_PIN A5

 

// ------------------- MOTOR DRIVER PINS -------------------

#define ENA_PIN 3     // Left motor speed

#define ENB_PIN 11    // Right motor speed

#define IN1_PIN A2    // Left motor direction

#define IN2_PIN A1

#define IN3_PIN 12    // Right motor direction

#define IN4_PIN 13

 

// ------------------- SETTINGS -------------------

const int SPEED_FWD = 80;      

onst int SPEED_TURN = 220;    

const int OBSTACLE_DIST = 20;  

 

// ----------------------------------------------------------

// READ ULTRASONIC DISTANCE

// ----------------------------------------------------------

long getDistance() {

  long duration;

 

  digitalWrite(TRIG_PIN, LOW);

  delayMicroseconds(2);

 

  digitalWrite(TRIG_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

 

  duration = pulseIn(ECHO_PIN, HIGH, 25000);

 

  long distance = duration * 0.034 / 2;

  return distance;

}

 

// ----------------------------------------------------------

// MOTOR CONTROL

// ----------------------------------------------------------

void forward() {

  digitalWrite(IN1_PIN, HIGH);

  digitalWrite(IN2_PIN, LOW);

  analogWrite(ENA_PIN, SPEED_FWD);

 

  digitalWrite(IN3_PIN, HIGH);

  digitalWrite(IN4_PIN, LOW);

  analogWrite(ENB_PIN, SPEED_FWD);

}

 

void stopMotors() {

  analogWrite(ENA_PIN, 0);

  analogWrite(ENB_PIN, 0);

  digitalWrite(IN1_PIN, LOW);

  digitalWrite(IN2_PIN, LOW);

  digitalWrite(IN3_PIN, LOW);

  digitalWrite(IN4_PIN, LOW);

}

 

void turnRight() {

  digitalWrite(IN1_PIN, HIGH);

  digitalWrite(IN2_PIN, LOW);

  analogWrite(ENA_PIN, SPEED_TURN);

 

  digitalWrite(IN3_PIN, LOW);

  digitalWrite(IN4_PIN, HIGH);

  analogWrite(ENB_PIN, SPEED_TURN);

}

 

// ----------------------------------------------------------

// SETUP

// ----------------------------------------------------------

void setup() {

  Serial.begin(9600);

 

  pinMode(TRIG_PIN, OUTPUT);

  pinMode(ECHO_PIN, INPUT);

 

  pinMode(ENA_PIN, OUTPUT);

  pinMode(ENB_PIN, OUTPUT);

  pinMode(IN1_PIN, OUTPUT);

  pinMode(IN2_PIN, OUTPUT);

  pinMode(IN3_PIN, OUTPUT);

  pinMode(IN4_PIN, OUTPUT);

 

  Serial.println("Obstacle Avoidance Robot Ready");

}

 

// ----------------------------------------------------------

// MAIN LOOP

// ----------------------------------------------------------

void loop() {

  long dist = getDistance();

  Serial.print("Distance: ");

  Serial.print(dist);

  Serial.println(" cm");

 

  if (dist > 0 && dist < OBSTACLE_DIST) {

    // Obstacle detected

    stopMotors();

    delay(200);

 

    // Turn right until clear path

    while (dist < OBSTACLE_DIST) {

      turnRight();

      dist = getDistance();

      Serial.println("Turning...");

    }

 

    stopMotors();

    delay(200);

  }

 

  forward();

}


 