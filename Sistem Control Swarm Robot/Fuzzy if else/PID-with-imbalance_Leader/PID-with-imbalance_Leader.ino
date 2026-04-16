#include <Arduino.h>
#include <math.h>

#define GEAR_RATIO 100
#define PPR 7

volatile long pulseCountLeft  = 0;
volatile long pulseCountRight = 0;

const byte encLeftA  = 14;
const byte encLeftB  = 12;
const byte encRightA = 10;
const byte encRightB = 13;

const int pwmMotorA = 5;
const int pwmMotorB = 4;
const int dirMotorA = 0;
const int dirMotorB = 2;

//TARGET
float baseRPM = 50.0;
float targetRPM_L = 50.0;
float targetRPM_R = 50.0;

//PID PARAM
double KpL = 4.65, KiL = 0.206, KdL = 0.02;
double KpR = 4.90, KiR = 0.21,  KdR = 0.02;

double errL, errR;
double lastErrL = 0, lastErrR = 0;
double intL = 0, intR = 0;

double dRPM_fuzzy = 0;

const unsigned long Ts = 20;
float dt = Ts / 1000.0;
unsigned long lastTime = 0;

long lastLeft = 0, lastRight = 0;
int pwmL = 0, pwmR = 0;

float rpmPID_L = 0, rpmPID_R = 0;
float rpmFuzzy_L = 0, rpmFuzzy_R = 0;

void ICACHE_RAM_ATTR isrLeftA() {
  pulseCountLeft += digitalRead(encLeftB) ? 1 : -1;
}
void ICACHE_RAM_ATTR isrRightA() {
  pulseCountRight += digitalRead(encRightB) ? 1 : -1;
}

double fuzzyBalanceRPM(double diff) {
  double a = abs(diff);

  if (a > 15)      return 4.0;
  else if (a > 8)  return 2.5;
  else if (a > 3)  return 1.0;
  else             return 0.0;
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

  digitalWrite(dirMotorA, HIGH);
  digitalWrite(dirMotorB, HIGH);

  attachInterrupt(digitalPinToInterrupt(encLeftA),  isrLeftA,  RISING);
  attachInterrupt(digitalPinToInterrupt(encRightA), isrRightA, RISING);

  Serial.println("PID -> FUZZY RPM BALANCE");
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime < Ts) return;
  lastTime = now;

  noInterrupts();
  long L = pulseCountLeft;
  long R = pulseCountRight;
  interrupts();

  long dL = L - lastLeft;
  long dR = R - lastRight;

  float rpmL = (dL / dt) * 60.0 / (PPR * GEAR_RATIO);
  float rpmR = (dR / dt) * 60.0 / (PPR * GEAR_RATIO);

  rpmPID_L = rpmL;
  rpmPID_R = rpmR;

  double rpmDiff = rpmPID_L - rpmPID_R;
  dRPM_fuzzy = fuzzyBalanceRPM(rpmDiff);

  if (rpmDiff > 0) {
    targetRPM_L = baseRPM - dRPM_fuzzy;
    targetRPM_R = baseRPM + dRPM_fuzzy;
  } else {
    targetRPM_L = baseRPM + dRPM_fuzzy;
    targetRPM_R = baseRPM - dRPM_fuzzy;
  }

  rpmFuzzy_L = targetRPM_L;
  rpmFuzzy_R = targetRPM_R;

  errL = targetRPM_L - rpmPID_L;
  errR = targetRPM_R - rpmPID_R;

  intL += errL * dt;
  intR += errR * dt;

  double outL = KpL * errL + KiL * intL + KdL * ((errL - lastErrL) / dt);
  double outR = KpR * errR + KiR * intR + KdR * ((errR - lastErrR) / dt);

  pwmL = constrain((outL / 125.0) * 255, 40, 255);
  pwmR = constrain((outR / 125.0) * 255, 40, 255);

  analogWrite(pwmMotorA, pwmL);
  analogWrite(pwmMotorB, pwmR);

  Serial.print("RPM_PID_L=");
  Serial.print(rpmPID_L, 1);
  Serial.print(" | RPM_PID_R=");
  Serial.print(rpmPID_R, 1);
  Serial.print(" || RPM_FUZZY_L=");
  Serial.print(rpmFuzzy_L, 1);
  Serial.print(" | RPM_FUZZY_R=");
  Serial.println(rpmFuzzy_R, 1);

  lastLeft = L;
  lastRight = R;
  lastErrL = errL;
  lastErrR = errR;
}
