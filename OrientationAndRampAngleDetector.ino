#include <Wire.h>

#include <LiquidCrystal.h>

#include <MPU6050_light.h>

 

// -- OBJECTS --

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

MPU6050 mpu(Wire);

 

// -- MOTOR PINS --

#define ENA_PIN 3    // Left Motor Speed (PWM)

#define ENB_PIN 11   // Right Motor Speed (PWM)

#define IN1_PIN A2  

#define IN2_PIN A1

#define IN3_PIN 12  

#define IN4_PIN 13

 

// -- CONFIGURATION --

const int RAMP_ANGLE_THRESHOLD = 10;

const int FLAT_ANGLE_THRESHOLD = 5;  

const int MOVE_SPEED = 225;          

const int SPIN_SPEED = 225;          

const long SPIN_DURATION = 2300;    

 

// -- ROBOT STATES --

enum RobotState {

  GOING_UP,

  STOP_ON_TOP,

  SPINNING,

  GOING_DOWN,

  FINISHED

};

RobotState currentState = GOING_UP;

 

// -- VARIABLES --

float rampAngle = 0;

unsigned long stateChangeTimestamp = 0;

 

void setup() {

  // Initialize Serial Monitor for debugging

  Serial.begin(9600);

 

  // Initialize Motor Pins

  pinMode(ENA_PIN, OUTPUT);

  pinMode(ENB_PIN, OUTPUT);

  pinMode(IN1_PIN, OUTPUT);

  pinMode(IN2_PIN, OUTPUT);

  pinMode(IN3_PIN, OUTPUT);

  pinMode(IN4_PIN, OUTPUT);

 

  // Initialize LCD

  lcd.begin(16, 2);

  lcd.print("Ramp Robot");

  lcd.setCursor(0, 1);

  lcd.print("Initializing...");

 

  // Initialize MPU-6050

  Wire.begin();

  byte status = mpu.begin();

  Serial.print("MPU6050 status: ");

  Serial.println(status);

  if (status != 0) {

    lcd.clear();

    lcd.print("MPU6050 Error");

    while (1) {}

  }

  delay(1000);

  mpu.calcOffsets();

  Serial.println("MPU6050 Calibrated.");

 

  lcd.clear();

  delay(1000);

}

 

void loop() {

  mpu.update();

 

  float currentAngle = mpu.getAngleY();

 

  // --- Main State Machine ---

  switch (currentState) {

    case GOING_UP:

      lcd.setCursor(0, 0);

      lcd.print("State: Going Up ");

      lcd.setCursor(0, 1);

      lcd.print("Angle: " + String(currentAngle, 1) + "  ");

 

      goForward();

 

      if (currentAngle < FLAT_ANGLE_THRESHOLD && rampAngle > RAMP_ANGLE_THRESHOLD) {

        delay(0);

       

        stopRobot();

        currentState = STOP_ON_TOP;

        stateChangeTimestamp = millis();

      }

      // Store the steepest angle

      else if (currentAngle > rampAngle) {

        rampAngle = currentAngle;

      }

      break;

 

    case STOP_ON_TOP:

      lcd.setCursor(0, 0);

      lcd.print("Ramp Angle: " + String(rampAngle, 1));

      lcd.setCursor(0, 1);

      lcd.print("Stopping...     ");

     

      // Wait for 4 seconds

      if (millis() - stateChangeTimestamp > 4000) {

        currentState = SPINNING;

        stateChangeTimestamp = millis();

      }

      break;

 

    case SPINNING:

      lcd.setCursor(0, 0);

      lcd.print("State: Spinning ");

      spin360();

     

      // Spin for the predefined duration

      if (millis() - stateChangeTimestamp > SPIN_DURATION) {

        stopRobot();

        currentState = GOING_DOWN;

        delay(500);

      }

      break;

 

    case GOING_DOWN:

      lcd.setCursor(0, 0);

      lcd.print("State: Going Down");

      lcd.setCursor(0, 1);

      lcd.print("Angle: " + String(currentAngle, 1) + "  ");

     

      goForward();

      static bool hasStartedDescent = false;

 

      if (currentAngle < -RAMP_ANGLE_THRESHOLD) {

        hasStartedDescent = true;

      }

 

      if (hasStartedDescent && currentAngle > -FLAT_ANGLE_THRESHOLD) {

        stopRobot();

        currentState = FINISHED;

      }

      break;

 

    case FINISHED:

      lcd.clear();

      lcd.setCursor(0, 0);

      lcd.print("Task Complete!");

      lcd.setCursor(0, 1);

      lcd.print("Final Angle:" + String(rampAngle, 1));

      stopRobot();

      while (1) {} // End of program

      break;

  }

}

 

// --- MOTOR CONTROL FUNCTIONS ---

 

void goForward() {

  // Left Motor Forward

  digitalWrite(IN1_PIN, HIGH);

  digitalWrite(IN2_PIN, LOW);

  analogWrite(ENA_PIN, MOVE_SPEED);

 

  // Right Motor Forward

  digitalWrite(IN3_PIN, HIGH);

  digitalWrite(IN4_PIN, LOW);

  analogWrite(ENB_PIN, MOVE_SPEED);

}

 

void stopRobot() {

  analogWrite(ENA_PIN, 0);

  analogWrite(ENB_PIN, 0);

  digitalWrite(IN1_PIN, LOW);

  digitalWrite(IN2_PIN, LOW);

  digitalWrite(IN3_PIN, LOW);

  digitalWrite(IN4_PIN, LOW);

}

 

void spin360() {

  digitalWrite(IN1_PIN, HIGH);

  digitalWrite(IN2_PIN, LOW);

  analogWrite(ENA_PIN, SPIN_SPEED);

 

  digitalWrite(IN3_PIN, LOW);

  digitalWrite(IN4_PIN, HIGH);

  analogWrite(ENB_PIN, SPIN_SPEED);

}