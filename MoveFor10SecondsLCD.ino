include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

 

// Motor driver pins

#define ENA A1

#define IN1 A2

#define IN2 A3

#define IN3 A4

#define IN4 A5

#define ENB 2

 

void setup() {

  lcd.begin(16, 2);

  lcd.print("Starting...");

 

  pinMode(ENA, OUTPUT);

  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);

  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);

  pinMode(IN4, OUTPUT);

 

  delay(1000);

  lcd.clear();

  lcd.print("Moving forward");

}

 

void loop() {

  digitalWrite(ENA, HIGH);

  digitalWrite(ENB, HIGH);

  digitalWrite(IN1, HIGH);

  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);

  digitalWrite(IN4, LOW);

 

  for (int i = 0; i <= 10; i++) {

    lcd.setCursor(0, 1);

    lcd.print("Time: ");

    lcd.print(i);

    lcd.print("s   ");

    delay(1000);

  }

 

  // stop motors

  digitalWrite(ENA, LOW);

  digitalWrite(ENB, LOW);

  digitalWrite(IN1, LOW);

  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);

  digitalWrite(IN4, LOW);