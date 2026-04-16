#include <Arduino.h>
#include <math.h>

// ENCODER
#define GEAR_RATIO 100
#define PPR 7

volatile long pulseCountRight = 0;

// PIN (SESUAI ROBOT KAMU)
const byte encRightA = 10;   // SD3
const byte encRightB = 13;   // D7

const int pwmMotorA = 5;
const int pwmMotorB = 4;
const int dirMotorA = 0;
const int dirMotorB = 2;

// PARAMETER ROBOT
const double wheelRadiusMeters = 0.022;
const double wheelBaseMeters   = 0.047;

const double DIST_PER_TICK =
  (2.0 * PI * wheelRadiusMeters) / (PPR * GEAR_RATIO);

// TARGET PENGUJIAN
const double TARGET_ANGLE_DEG = 90.0;   // target sudut
const double ANGLE_TOLERANCE  = 2.0;    // toleransi stop (deg)

// KALIBRASI (BOLEH DIUBAH)
double K_THETA = 0.50;   

// STATE
bool stopLock = false;

// ODOMETRI
double theta_rad = 0.0;
long lastRight = 0;

// ISR ENCODER
void IRAM_ATTR isrRightA() {
  pulseCountRight += digitalRead(encRightB) ? 1 : -1;
}

// MOTOR CONTROL
void setMotor(int leftPWM, int rightPWM) {
  if (stopLock) return;

  leftPWM  = constrain(leftPWM,  -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  digitalWrite(dirMotorA, leftPWM >= 0 ? HIGH : LOW);
  digitalWrite(dirMotorB, rightPWM >= 0 ? HIGH : LOW);

  analogWrite(pwmMotorA, abs(leftPWM));
  analogWrite(pwmMotorB, abs(rightPWM));
}

void stopTotal() {
  analogWrite(pwmMotorA, 0);
  analogWrite(pwmMotorB, 0);
  digitalWrite(dirMotorA, LOW);
  digitalWrite(dirMotorB, LOW);
}

// SETUP
void setup() {
  Serial.begin(115200);
  Serial.println("=== UJI AKURASI HADAP ROBOT (FINAL) ===");

  pinMode(encRightA, INPUT_PULLUP);
  pinMode(encRightB, INPUT_PULLUP);

  pinMode(pwmMotorA, OUTPUT);
  pinMode(pwmMotorB, OUTPUT);
  pinMode(dirMotorA, OUTPUT);
  pinMode(dirMotorB, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encRightA), isrRightA, RISING);

  stopTotal();
  delay(1000);
}

// LOOP
void loop() {
  if (stopLock) return;   // LOOP TERKUNCI TOTAL

  //BACA ENCODER 
  noInterrupts();
  long right = pulseCountRight;
  interrupts();

  long dR = right - lastRight;
  lastRight = right;

  //ODOMETRI
  double dr = dR * DIST_PER_TICK;
  double dTheta = (2.0 * dr / wheelBaseMeters) * K_THETA;
  theta_rad += dTheta;

  // BATASI SUDUT (ANTI RUNAWAY)
  theta_rad = constrain(theta_rad, 0.0, TWO_PI);

  double theta_deg = theta_rad * 180.0 / PI;

  //STOP
  if (theta_deg >= TARGET_ANGLE_DEG - ANGLE_TOLERANCE) {

    // REM AKTIF (ANTI OVERSHOOT)
    setMotor(20, -20);
    delay(80);

    stopTotal();
    stopLock = true;

    Serial.println("=================================");
    Serial.println("STOP TOTAL AKTIF");
    Serial.print("Target  : ");
    Serial.print(TARGET_ANGLE_DEG);
    Serial.println(" deg");

    Serial.print("Aktual  : ");
    Serial.print(theta_deg, 2);
    Serial.println(" deg");

    Serial.print("Error   : ");
    Serial.print(TARGET_ANGLE_DEG - theta_deg, 2);
    Serial.println(" deg");
    Serial.println("=================================");

    return;
  }

  //KONTROL PUTAR (ZONA PERLAMBATAN)
  int pwm;

  if (theta_deg < 60) {
    pwm = 90;
  }
  else if (theta_deg < 70) {
    pwm = 70;
  }
  else if (theta_deg < 85) {
    pwm = 60;
  }
  else {
    pwm = 55;   // zona halus
  }

  setMotor(-pwm, pwm);

  //DEBUG
  Serial.print("Theta=");
  Serial.print(theta_deg, 2);
  Serial.print(" deg | PWM=");
  Serial.println(pwm);

  delay(50);
}
