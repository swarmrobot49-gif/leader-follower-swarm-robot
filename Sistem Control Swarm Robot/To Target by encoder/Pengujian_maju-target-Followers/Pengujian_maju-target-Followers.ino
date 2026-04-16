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

const double wheelRadius = 0.022;
const double DIST_PER_TICK =
  (2.0 * PI * wheelRadius) / (PPR * GEAR_RATIO);

const double TARGET_DISTANCE = 1.5;
const double DIST_TOL = 0.005;
const double SLOW_DIST  = 0.05;
const double BRAKE_DIST = 0.02;

double KpL = 3.55, KiL = 0.06, KdL = 0.05;
double KpR = 3.90, KiR = 0.06, KdR = 0.05;

double errL=0, errR=0, lastErrL=0, lastErrR=0;
double intL=0, intR=0;

double K_balance = 0.3;

double targetRPM_base = 50;
double targetRPM_L = 50;
double targetRPM_R = 50;

unsigned long lastTime = 0;
const unsigned long Ts = 20;
double dt = Ts / 1000.0;

double totalDistance = 0;
long lastL = 0, lastR = 0;
bool finished = false;

void IRAM_ATTR isrLeftA() {
  pulseCountLeft += digitalRead(encLeftB) ? 1 : -1;
}
void IRAM_ATTR isrRightA() {
  pulseCountRight += digitalRead(encRightB) ? 1 : -1;
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

  Serial.println("PID MAJU + STOP + FUZZY BALANCE (SEGI-4 STYLE)");
}

void loop() {

  if (finished) {
    analogWrite(pwmMotorA, 0);
    analogWrite(pwmMotorB, 0);
    return;
  }

  unsigned long now = millis();
  if (now - lastTime < Ts) return;
  lastTime = now;

  noInterrupts();
  long L = pulseCountLeft;
  long R = pulseCountRight;
  interrupts();

  long dL = L - lastL;
  long dR = R - lastR;
  lastL = L;
  lastR = R;

  double distL = dL * DIST_PER_TICK;
  double distR = dR * DIST_PER_TICK;
  totalDistance += (distL + distR) / 2.0;

  double remaining = TARGET_DISTANCE - totalDistance;

  if (remaining <= DIST_TOL) {
    analogWrite(pwmMotorA, 0);
    analogWrite(pwmMotorB, 0);
    intL = intR = 0;
    finished = true;
    Serial.println("TARGET TERCAPAI");
    return;
  }

  if (remaining > SLOW_DIST)
    targetRPM_base = 50;
  else if (remaining > BRAKE_DIST)
    targetRPM_base = 25;
  else
    targetRPM_base = 15;

  double rpmL = (dL * 60.0) / (PPR * GEAR_RATIO * dt);
  double rpmR = (dR * 60.0) / (PPR * GEAR_RATIO * dt);


  double diff = rpmL - rpmR;
  targetRPM_L = targetRPM_base - K_balance * diff;
  targetRPM_R = targetRPM_base + K_balance * diff;

  errL = targetRPM_L - rpmL;
  intL += errL * dt;
  double outL = KpL*errL + KiL*intL + KdL*(errL-lastErrL);
  int pwmL = constrain((int)((outL / 125.0) * 255), 40, 255);
  lastErrL = errL;

  errR = targetRPM_R - rpmR;
  intR += errR * dt;
  double outR = KpR*errR + KiR*intR + KdR*(errR-lastErrR);
  int pwmR = constrain((int)((outR / 125.0) * 255), 40, 255);
  lastErrR = errR;

  analogWrite(pwmMotorA, pwmL);
  analogWrite(pwmMotorB, pwmR);

  Serial.print("Dist=");
  Serial.print(totalDistance,3);
  Serial.print(" | RPM L=");
  Serial.print(rpmL,1);
  Serial.print(" | RPM R=");
  Serial.print(rpmR,1);
  Serial.print(" | tgtL=");
  Serial.print(targetRPM_L,1);
  Serial.print(" | tgtR=");
  Serial.println(targetRPM_R,1);
}
