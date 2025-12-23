#include <LiquidCrystal.h>

#include <TimerOne.h>

 

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

 

#define ENC_LEFT_PIN A4

#define ENC_RIGHT_PIN A5

 

// MOTOR PINS

#define ENA_PIN 3    // Hardware PWM (Left Motor)

#define ENB_PIN 11    // Software PWM (Right Motor)

#define IN1_PIN 13

#define IN2_PIN 14

#define IN3_PIN 12

#define IN4_PIN 2

 

// SENSOR PINS

#define SENSOR_LEFT A2

#define SENSOR_RIGHT A1

 

// ---------------- CONFIGURATION ----------------

#define BASE_SPEED 70      

int SENSOR_THRESHOLD = 400

 

// MEASUREMENTS FOR DISTANCE/SPEED

const int DISK_SLOTS = 20;        

const float WHEEL_DIA_CM = 6.5;  

const float CM_PER_PULSE = (3.14159 * WHEEL_DIA_CM) / DISK_SLOTS;

 

// PID CONSTANTS

float KP = 3.0;    

float KI = 0.002;  

float KD = 40.0;  

 

// ---------------- VARIABLES ----------------

// Software PWM variables

volatile uint8_t softDuty = 0;

volatile uint16_t softCounter = 0;

const uint16_t SOFT_MAX = 255;

const unsigned int ISR_US = 128;

 

// Navigation Memory

int lastDirection = 0;

float integral = 0;

float lastError = 0;

 

// Encoder & Speed Variables

int lastLeftEncState = LOW;

int lastRightEncState = LOW;

unsigned long leftPulses = 0;

unsigned long rightPulses = 0;

 

unsigned long lastLCDUpdate = 0;

unsigned long lastSpeedCalcTime = 0;

unsigned long lastPulsesSum = 0;

float currentSpeed = 0.0;

 

// Timer Variables

unsigned long stopTimerStart = 0;

unsigned long startTime = 0;

 

// ---------------- SETUP ----------------

void setup() {

  Serial.begin(9600);

 

  // Setup Encoders

  pinMode(ENC_LEFT_PIN, INPUT);

  pinMode(ENC_RIGHT_PIN, INPUT);

 

  // Setup Motors

  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);

  pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);

  pinMode(ENA_PIN, OUTPUT);

  pinMode(ENB_PIN, OUTPUT);

  digitalWrite(ENB_PIN, LOW);

 

  // Initialize Timer1 for Software PWM on Pin 2

  Timer1.initialize(ISR_US);

  Timer1.attachInterrupt(softPWMisr);

 

  // LCD Setup

  pinMode(10, OUTPUT);

  digitalWrite(10, HIGH);

  lcd.begin(16, 2);

  lcd.print("Ready...");

 

  stopMotors();

  delay(1500);

  lcd.clear();

 

  startTime = millis();

}

 

// ---------------- LOOP ----------------

void loop() {

  // --- 1. READ ENCODERS (Must happen every loop) ---

  readEncoders();

 

  // --- 2. READ SENSORS ---

  int leftVal  = analogRead(SENSOR_LEFT);

  int rightVal = analogRead(SENSOR_RIGHT);

 

  bool leftBlack  = (leftVal  < SENSOR_THRESHOLD);

  bool rightBlack = (rightVal < SENSOR_THRESHOLD);

 

  // --- 3. FINISH LINE CHECK ---

  if (leftBlack && rightBlack) {

    if (stopTimerStart == 0) stopTimerStart = millis();

    if (millis() - stopTimerStart > 30) {

      stopAndDisplayFinalStats();

    }

  } else {

    stopTimerStart = 0;

  }

 

  // --- 4. PID LOGIC ---

  int error = leftVal - rightVal;

 

  // History Update

  if (leftBlack) lastDirection = -1;

  else if (rightBlack) lastDirection = 1;

 

  // 90 Degree / Lost Line Logic

  if (!leftBlack && !rightBlack) {

    if (lastDirection == -1) error = -1000;

    else error = 1000;  

    integral = 0;

  }

  else {

    if (abs(error) < 15) error = 0;

    integral += error;

  }

 

  float derivative = error - lastError;

  lastError = error;

  float correction = KP * error + KI * integral + KD * derivative;

 

  // Motor Control

  correction = constrain(correction, -BASE_SPEED, BASE_SPEED);

  int leftMotor = constrain(BASE_SPEED + correction, 0, 255);

  int rightMotor = constrain(BASE_SPEED - correction, 0, 255);

 

  setMotorSpeeds(leftMotor, rightMotor);

 

  // --- 5. UPDATE LCD (RUNNING STATS) ---

  if (millis() - lastLCDUpdate > 250) {

    calculateSpeed();

   

    lcd.setCursor(0,0);

    lcd.print("T:");

    lcd.print((millis() - startTime) / 1000.0, 1);

    lcd.print("s ");

 

    lcd.setCursor(0,1);

    lcd.print("Spd:");

    lcd.print(currentSpeed, 1);

    lcd.print("cm/s ");

   

    lastLCDUpdate = millis();

  }

}

 

// ---------------- HELPERS ----------------

 

void readEncoders() {

  int currentLeft = digitalRead(ENC_LEFT_PIN);

  int currentRight = digitalRead(ENC_RIGHT_PIN);

 

  // Count every time the state CHANGES (Low->High OR High->Low)

  if (currentLeft != lastLeftEncState) {

    leftPulses++;

    lastLeftEncState = currentLeft;

  }

 

  if (currentRight != lastRightEncState) {

    rightPulses++;

    lastRightEncState = currentRight;

  }

}

 

void calculateSpeed() {

  unsigned long currentTime = millis();

  unsigned long timeDiff = currentTime - lastSpeedCalcTime;

 

  if (timeDiff > 0) {

    unsigned long currentPulsesSum = leftPulses + rightPulses;

    unsigned long pulseDiff = currentPulsesSum - lastPulsesSum;

   

    float distanceCm = (pulseDiff * CM_PER_PULSE) / 2.0;

    currentSpeed = (distanceCm / timeDiff) * 1000.0;

   

    lastPulsesSum = currentPulsesSum;

    lastSpeedCalcTime = currentTime;

  }

}

 

void stopAndDisplayFinalStats() {

  stopMotors();

 

  // Calculate Final Stats

  float finalTime = (millis() - startTime) / 1000.0;

  float totalDist = ((leftPulses + rightPulses) / 2.0) * CM_PER_PULSE;

 

  lcd.clear();

 

  // Line 1: Time

  lcd.setCursor(0,0);

  lcd.print("Time:");

  lcd.print(finalTime, 1);

  lcd.print("s");

 

  // Line 2: Distance

  lcd.setCursor(0,1);

  lcd.print("Dist:");

  lcd.print(totalDist, 1);

  lcd.print("cm");

 

  // Infinite loop to keep displaying the stats

  while(1) {

    stopMotors();

  }

}

 

void setMotorSpeeds(int leftSpeed, int rightSpeed) {

  // LEFT MOTOR (Hardware PWM)

  digitalWrite(IN1_PIN, LOW);

  digitalWrite(IN2_PIN, HIGH);

  analogWrite(ENA_PIN, leftSpeed);

 

  // RIGHT MOTOR (Software PWM)

  digitalWrite(IN3_PIN, HIGH);

  digitalWrite(IN4_PIN, LOW);

  setSoftPWM(rightSpeed);

}

 

void stopMotors() {

  analogWrite(ENA_PIN, 0);

  setSoftPWM(0);

  digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);

  digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);

}

 

void setSoftPWM(uint8_t duty) {

  noInterrupts();

  softDuty = duty;

  interrupts();

}

 

void softPWMisr() {

  softCounter++;

  if (softCounter > SOFT_MAX) softCounter = 0;

  if (softCounter < softDuty) digitalWrite(ENB_PIN, HIGH);

  else digitalWrite(ENB_PIN, LOW);

}