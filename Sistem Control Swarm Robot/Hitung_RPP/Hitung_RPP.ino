#include "math.h"
volatile long pulseCountLeft = 0;
volatile long pulseCountRight = 0;

const byte encLeftA = 14;  // D5
const byte encLeftB = 12;  // D6
const byte encRightA = 10; // D7
const byte encRightB = 13; // SD3

const int pwmMotorA = 5;
const int pwmMotorB = 4;
const int dirMotorA = 0;
const int dirMotorB = 2;

const int motorSpeed = 128;

void ICACHE_RAM_ATTR isrLeftA() {
  bool b = digitalRead(encLeftB);
  if (b) pulseCountLeft++;
  else pulseCountLeft--;
}

void ICACHE_RAM_ATTR isrRightA() {
  bool b = digitalRead(encRightB);
  if (b) pulseCountRight++;
  else pulseCountRight--;
}

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


  analogWrite(pwmMotorA, 80); //kiri
  digitalWrite(dirMotorA, HIGH);

  analogWrite(pwmMotorB, 80); //kanan
  digitalWrite(dirMotorB, HIGH);

  attachInterrupt(digitalPinToInterrupt(encLeftA), isrLeftA, RISING);
  attachInterrupt(digitalPinToInterrupt(encRightA), isrRightA, RISING);
}

unsigned long lastPrintTime = 0;
unsigned long lastpulse_Right = 0;
unsigned long lastpulse_left = 0;
const unsigned long printInterval = 1000; // 1000 ms = 1 detik

void loop() {
  static long lastLeft = 0;
  static long lastRight = 0;

  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    lastPrintTime = currentTime;

    noInterrupts();
    long left = pulseCountLeft;
    long right = pulseCountRight;
    pulseCountLeft = 0;
    pulseCountRight = 0;

    interrupts();
      Serial.print(" Kiri : ");
      Serial.print((left / 700.0 * 2 *PI) / 1 );
      Serial.print(" RPS ") ;
      Serial.print(left) ;
      Serial.print(" Pulse") ;
      Serial.print(" | Kanan : ");
      Serial.print((right / 700.0 * 2 *PI) / 1 );
      Serial.print(" RPS ") ;
      Serial.print(right) ;
      Serial.println(" Pulse") ;
    
  }
}
