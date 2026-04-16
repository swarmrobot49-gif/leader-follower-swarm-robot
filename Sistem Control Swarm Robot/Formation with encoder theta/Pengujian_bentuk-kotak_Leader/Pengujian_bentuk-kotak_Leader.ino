#include <Arduino.h>
#include <math.h>

#define GEAR_RATIO 100
#define PPR 7

volatile long pulseCountLeft = 0;
volatile long pulseCountRight = 0;

const byte encLeftA  = 14;
const byte encLeftB  = 12;
const byte encRightA = 10;
const byte encRightB = 13;

const int pwmMotorA = 5;
const int pwmMotorB = 4;
const int dirMotorA = 0;
const int dirMotorB = 2;

const double wheelRadiusMeters = 0.022;
const double wheelBaseMeters   = 0.047;

const double DIST_PER_TICK =
  (2.0 * PI * wheelRadiusMeters) / (PPR * GEAR_RATIO);

const double MOVE_DISTANCE = 0.20;   // 10 cm
const double ANGLE_TOL     = 2.0;

float targetRPM_base = 50;

double KpL = 4.65, KiL = 0.206, KdL = 0.02;
double KpR = 4.90, KiR = 0.21, KdR = 0.02;

double errL=0, errR=0, lastErrL=0, lastErrR=0;
double intL=0, intR=0;

double K_balance = 0.3;

const double Kp_heading = 1.10;
const int PWM_TURN_MIN  = 80;
const int PWM_TURN_MAX  = 100;

double x_pos = 0.0;
double y_pos = 0.0;
double theta_global_deg = 0.0;   // heading absolut

double theta_deg = 0.0;
const double K_THETA = 0.5;

long lastL = 0, lastR = 0;

enum State {
  MOVE_1, TURN_90,
  MOVE_2, TURN_180,
  MOVE_3, TURN_270,
  MOVE_4, TURN_366,
  FINISH
};

State state = MOVE_1;
bool finished = false;

void IRAM_ATTR isrLeftA() {
  pulseCountLeft += digitalRead(encLeftB) ? 1 : -1;
}
void IRAM_ATTR isrRightA() {
  pulseCountRight += digitalRead(encRightB) ? 1 : -1;
}

void setMotors(int pwmL, int pwmR, bool dirL=true, bool dirR=true) {
  digitalWrite(dirMotorA, dirL ? HIGH : LOW);
  digitalWrite(dirMotorB, dirR ? HIGH : LOW);
  analogWrite(pwmMotorA, pwmL);
  analogWrite(pwmMotorB, pwmR);
}

void stopMotors() {
  analogWrite(pwmMotorA, 0);
  analogWrite(pwmMotorB, 0);
  digitalWrite(dirMotorA, HIGH);
  digitalWrite(dirMotorB, HIGH);
}

void resetRelative() {
  pulseCountLeft = pulseCountRight = 0;
  lastL = lastR = 0;
  theta_deg = 0;
  intL = intR = lastErrL = lastErrR = 0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== GLOBAL ODOMETRI SQUARE PATH ===");

  pinMode(encLeftA, INPUT_PULLUP);
  pinMode(encLeftB, INPUT_PULLUP);
  pinMode(encRightA, INPUT_PULLUP);
  pinMode(encRightB, INPUT_PULLUP);

  pinMode(pwmMotorA, OUTPUT);
  pinMode(pwmMotorB, OUTPUT);
  pinMode(dirMotorA, OUTPUT);
  pinMode(dirMotorB, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encLeftA), isrLeftA, RISING);
  attachInterrupt(digitalPinToInterrupt(encRightA), isrRightA, RISING);
}

void loop() {

  if(finished) {
    stopMotors();
    return;
  }

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

  switch(state) {

    case MOVE_1:
    case MOVE_2:
    case MOVE_3:
    case MOVE_4: {

      double dS = (distL + distR) / 2.0;

      // update posisi global
      double theta_rad = theta_global_deg * PI / 180.0;
      x_pos += dS * cos(theta_rad);
      y_pos += dS * sin(theta_rad);

      static double distAcc = 0;
      distAcc += dS;

      float rpmL = (dL * 60.0) / (PPR * GEAR_RATIO * 0.02);
      float rpmR = (dR * 60.0) / (PPR * GEAR_RATIO * 0.02);

      double diff = rpmL - rpmR;
      double tgtL = targetRPM_base - K_balance * diff;
      double tgtR = targetRPM_base + K_balance * diff;

      errL = tgtL - rpmL;
      errR = tgtR - rpmR;

      intL += errL * 0.02;
      intR += errR * 0.02;

      int pwmL = constrain(KpL*errL + KiL*intL + KdL*(errL-lastErrL), 40, 255);
      int pwmR = constrain(KpR*errR + KiR*intR + KdR*(errR-lastErrR), 40, 255);

      lastErrL = errL;
      lastErrR = errR;

      setMotors(pwmL, pwmR, true, true);

      if(distAcc >= MOVE_DISTANCE) {
        stopMotors();
        distAcc = 0;
        resetRelative();

        if(state == MOVE_1) state = TURN_90;
        else if(state == MOVE_2) state = TURN_180;
        else if(state == MOVE_3) state = TURN_270;
        else if(state == MOVE_4) state = TURN_366;
      }
      break;
    }

    case TURN_90:
    case TURN_180:
    case TURN_270:
    case TURN_366: {

      double targetAngle;

      if(state == TURN_90)       targetAngle = 90;
      else if(state == TURN_180) targetAngle = 180;
      else if(state == TURN_270) targetAngle = 270;
      else                       targetAngle = 366;

      double dTheta =
        ((distR - distL) / wheelBaseMeters) * K_THETA * 180.0 / PI;
      theta_deg += dTheta;

      double error = targetAngle - theta_global_deg - theta_deg;

      if(error > ANGLE_TOL) {
        int pwmTurn = constrain(
          (int)(Kp_heading * error),
          PWM_TURN_MIN,
          PWM_TURN_MAX
        );
        setMotors(pwmTurn, pwmTurn, false, true);
      } else {
        stopMotors();

        theta_global_deg = targetAngle;
        if(theta_global_deg >= 360) theta_global_deg -= 360;

        resetRelative();

        if(state == TURN_90) state = MOVE_2;
        else if(state == TURN_180) state = MOVE_3;
        else if(state == TURN_270) state = MOVE_4;
        else state = FINISH;
      }
      break;
    }

    case FINISH:
      stopMotors();
      Serial.println("=== SELESAI ===");
      Serial.print("X = "); Serial.print(x_pos, 3);
      Serial.print(" | Y = "); Serial.print(y_pos, 3);
      Serial.print(" | Theta = "); Serial.println(theta_global_deg);
      finished = true;
      break;
  }
}
