/*
  Smart Room Occupancy Counter with Auto-Lock
  Board: Arduino Uno/Nano
  Libraries: Servo (built-in), LiquidCrystal_I2C (install from Library Manager)

  WIRING
  HC-SR04 #1 (OUTSIDE sensor, "A"): TRIG->D2  ECHO->D3  VCC->5V GND->GND
  HC-SR04 #2 (INSIDE sensor,  "B"): TRIG->D4  ECHO->D5  VCC->5V GND->GND
  Servo (latch): signal->D9, VCC->5V (external 5V recommended), GND->GND
  Green LED (open)  : D7 -> 220R -> LED -> GND
  Red LED (locked)  : D8 -> 220R -> LED -> GND
  16x2 I2C LCD      : SDA->A4  SCL->A5  VCC->5V GND->GND  (address 0x27, try 0x3F if blank)

  DIRECTION LOGIC
  A blocked then B blocked  -> person ENTERED  (count + 1)
  B blocked then A blocked  -> person LEFT     (count - 1)

  SERIAL (9600 baud) -> dashboard
  Out:  S,<count>,<max>,<locked 0/1>     status (sent on every change + each second)
        E,IN / E,OUT                     event
  In :  M<number>   set occupancy limit   e.g. M5
        R           reset count to 0
*/
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const uint8_t TRIG_A = 2, ECHO_A = 3, TRIG_B = 4, ECHO_B = 5;
const uint8_t SERVO_PIN = 9, LED_GREEN = 7, LED_RED = 8;
const int DETECT_CM = 30;            // object closer than this = beam broken
const int LOCK_ANGLE = 90, OPEN_ANGLE = 0;
const unsigned long TIMEOUT_MS = 2000;

Servo latch;
LiquidCrystal_I2C lcd(0x27, 16, 2);

int count = 0, maxOcc = 5;
bool locked = false;

enum State { IDLE, WAIT_SECOND, WAIT_CLEAR };
State state = IDLE;
char dir = 0;                        // 'I' = in, 'O' = out
unsigned long tStart = 0, lastStatus = 0;

int readCm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long d = pulseIn(echo, HIGH, 20000UL);
  if (d == 0) return 999;            // no echo = nothing close
  return d / 58;
}

void sendStatus() {
  Serial.print("S,"); Serial.print(count); Serial.print(',');
  Serial.print(maxOcc); Serial.print(','); Serial.println(locked ? 1 : 0);
  lastStatus = millis();
}

void updateOutputs() {
  locked = (count >= maxOcc);
  latch.write(locked ? LOCK_ANGLE : OPEN_ANGLE);
  digitalWrite(LED_RED, locked);
  digitalWrite(LED_GREEN, !locked);
  lcd.setCursor(0, 0);
  lcd.print("In room: "); lcd.print(count); lcd.print("/"); lcd.print(maxOcc); lcd.print("   ");
  lcd.setCursor(0, 1);
  lcd.print(locked ? "ROOM FULL-LOCKED" : "Welcome - OPEN   ");
  sendStatus();
}

void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n'); s.trim();
  if (s.length() == 0) return;
  if (s[0] == 'M') { int m = s.substring(1).toInt(); if (m >= 1 && m <= 99) maxOcc = m; }
  else if (s[0] == 'R') count = 0;
  updateOutputs();
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_A, OUTPUT); pinMode(ECHO_A, INPUT);
  pinMode(TRIG_B, OUTPUT); pinMode(ECHO_B, INPUT);
  pinMode(LED_GREEN, OUTPUT); pinMode(LED_RED, OUTPUT);
  latch.attach(SERVO_PIN);
  lcd.init(); lcd.backlight();
  updateOutputs();
}

void loop() {
  handleSerial();
  bool a = readCm(TRIG_A, ECHO_A) < DETECT_CM;
  delay(15);                         // let echoes die out between sensors
  bool b = readCm(TRIG_B, ECHO_B) < DETECT_CM;

  switch (state) {
    case IDLE:
      if (a && !b)      { dir = 'I'; state = WAIT_SECOND; tStart = millis(); }
      else if (b && !a) { dir = 'O'; state = WAIT_SECOND; tStart = millis(); }
      break;

    case WAIT_SECOND:
      if ((dir == 'I' && b) || (dir == 'O' && a)) state = WAIT_CLEAR;
      else if (millis() - tStart > TIMEOUT_MS) state = IDLE;   // gave up / turned back
      break;

    case WAIT_CLEAR:
      if (!a && !b) {                // both beams clear = crossing finished
        if (dir == 'I' && !locked) { count++; Serial.println("E,IN"); }
        else if (dir == 'O' && count > 0) { count--; Serial.println("E,OUT"); }
        updateOutputs();
        state = IDLE;
        delay(300);                  // cooldown against double counts
      }
      break;
  }
  if (millis() - lastStatus > 1000) sendStatus();
}
