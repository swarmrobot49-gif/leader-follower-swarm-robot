#include <Arduino.h>

// ================= ENCODER =================
volatile long pulseCountLeft  = 0;
volatile long pulseCountRight = 0;

const byte encLeftA  = 14;  // D5
const byte encLeftB  = 12;  // D6
const byte encRightA = 10;  // D7
const byte encRightB = 13;  // SD3

// ================= MOTOR =================
const int pwmMotorA = 5;
const int pwmMotorB = 4;
const int dirMotorA = 0;
const int dirMotorB = 2;

// ================= PARAMETER =================
#define PPR 700
#define SAMPLE_TIME 500   // ms (0.5 detik)
#define PWM_MAX 255

// ================= ISR =================
void ICACHE_RAM_ATTR isrLeftA() {
  if (digitalRead(encLeftB)) pulseCountLeft++;
  else pulseCountLeft--;
}

void ICACHE_RAM_ATTR isrRightA() {
  if (digitalRead(encRightB)) pulseCountRight++;
  else pulseCountRight--;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(encLeftA, INPUT_PULLUP);
  pinMode(encLeftB, INPUT_PULLUP);
  pinMode(encRightA, INPUT_PULLUP);
  pinMode(encRightB, INPUT_PULLUP);

  pinMode(pwmMotorA, OUTPUT);
  pinMode(pwmMotorB, OUTPUT);
  pinMode(dirMotorA, OUTPUT);
  pinMode(dirMotorB, OUTPUT);

  digitalWrite(dirMotorA, HIGH);
  digitalWrite(dirMotorB, HIGH);

  // PWM awal
  analogWrite(pwmMotorA, 0);
  analogWrite(pwmMotorB, 0);

  attachInterrupt(digitalPinToInterrupt(encLeftA),  isrLeftA,  RISING);
  attachInterrupt(digitalPinToInterrupt(encRightA), isrRightA, RISING);
}

// ================= LOOP =================
unsigned long lastTime = 0;
long lastLeftPulse  = 0;
long lastRightPulse = 0;

void loop() {
  unsigned long now = millis();

  if (now - lastTime >= SAMPLE_TIME) {
    double dt = (now - lastTime) / 1000.0; // detik
    lastTime = now;

    noInterrupts();
    long leftPulse  = pulseCountLeft;
    long rightPulse = pulseCountRight;
    interrupts();

    long deltaLeft  = leftPulse  - lastLeftPulse;
    long deltaRight = rightPulse - lastRightPulse;

    // ================= HITUNG RPM =================
    double rpmLeft  = (deltaLeft  * 60.0) / (PPR * dt);
    double rpmRight = (deltaRight * 60.0) / (PPR * dt);

    Serial.print("RPM Kiri : ");
    Serial.print(rpmLeft);
    Serial.print(" | RPM Kanan : ");
    Serial.println(rpmRight);

    lastLeftPulse  = leftPulse;
    lastRightPulse = rightPulse;
  }
}
