#define GEAR_RATIO 100
#define PPR 7

volatile long pulseCountLeft = 0;
volatile long pulseCountRight = 0;

// ENCODER
const byte encLeftA  = 14;   // D5
const byte encLeftB  = 12;   // D6
const byte encRightA = 10;   // SD3
const byte encRightB = 13;   // D7

// MOTOR
const int pwmMotorA = 5;   // kiri
const int pwmMotorB = 4;   // kanan
const int dirMotorA = 0;
const int dirMotorB = 2;

// PARAMETER FISIK
const double wheelRadiusMeters = 0.022;
const double wheelBaseMeters   = 0.047;

const double DIST_PER_TICK =
  (2.0 * PI * wheelRadiusMeters) / (PPR * GEAR_RATIO);

// TARGET & TOLERANSI (SEMUA DALAM METER)
const double TARGET_DISTANCE = 0.10;   // cm
const double DIST_TOLERANCE  = 0.002;  // mm
const double SLOW_DOWN_DIST  = 0.03;   // cm
bool motionDone = false;

// ODOMETRI
double theta_deg = 0.0;
double totalDistance = 0.0;
double K_THETA = 0.50;

long lastLeft = 0;
long lastRight = 0;

// PID
int pwmKiri = 0;
int pwmKanan = 0;

float targetRPM_kiri  = 50;
float targetRPM_kanan = 50;

// PID kanan
double error_kanan, lastError_kanan, integral_kanan;
double Kp_kanan = 3.5, Ki_kanan = 0.04, Kd_kanan = 0.06;

// PID kiri
double error_kiri, lastError_kiri, integral_kiri;
double Kp_kiri = 3.15, Ki_kiri = 0.03, Kd_kiri = 0.05;

// TIMING
unsigned long lastControlTime = 0;
const unsigned long controlInterval = 20;
float dt = controlInterval / 1000.0;

// ISR
void ICACHE_RAM_ATTR isrLeftA() {
  pulseCountLeft += digitalRead(encLeftB) ? 1 : -1;
}
void ICACHE_RAM_ATTR isrRightA() {
  pulseCountRight += digitalRead(encRightB) ? 1 : -1;
}

// NORMALISASI SUDUT
double normalizeAngleDeg(double ang) {
  while (ang < 0) ang += 360.0;
  while (ang >= 360.0) ang -= 360.0;
  return ang;
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

  attachInterrupt(digitalPinToInterrupt(encLeftA), isrLeftA, RISING);
  attachInterrupt(digitalPinToInterrupt(encRightA), isrRightA, RISING);

  Serial.println("Robot maju 10 cm | PID + Odometri + Toleransi");
}

void loop() {
  unsigned long now = millis();
  if (now - lastControlTime < controlInterval) return;
  lastControlTime = now;

  noInterrupts();
  long left  = pulseCountLeft;
  long right = pulseCountRight;
  interrupts();

  long dL = left - lastLeft;
  long dR = right - lastRight;
  lastLeft = left;
  lastRight = right;

  // ODOMETRI
  double distL = dL * DIST_PER_TICK;
  double distR = dR * DIST_PER_TICK;
  totalDistance += (distL + distR) / 2.0;

  double dTheta = ((distR - distL) / wheelBaseMeters) * K_THETA;
  theta_deg = normalizeAngleDeg(theta_deg + dTheta * 180.0 / PI);

  double remainingDist = TARGET_DISTANCE - totalDistance;

  // PERLAMBATAN
  if (remainingDist <= SLOW_DOWN_DIST && remainingDist > DIST_TOLERANCE && !motionDone) {
    targetRPM_kiri = 25;
    targetRPM_kanan = 25;
  }

  // STOP
  if (remainingDist <= DIST_TOLERANCE && !motionDone) {
    targetRPM_kiri = 0;
    targetRPM_kanan = 0;
    integral_kiri = 0;
    integral_kanan = 0;
    motionDone = true;
  }

  if (motionDone) {
    analogWrite(pwmMotorA, 0);
    analogWrite(pwmMotorB, 0);
    return;
  }

  // HITUNG RPM
  float rpmL = ((dL / dt) * 60.0) / (PPR * GEAR_RATIO);
  float rpmR = ((dR / dt) * 60.0) / (PPR * GEAR_RATIO);

  // PID
  double outL = Kp_kiri * (targetRPM_kiri - rpmL);
  double outR = Kp_kanan * (targetRPM_kanan - rpmR);

  pwmKiri  = constrain((int)((outL / 125.0) * 255), 0, 255);
  pwmKanan = constrain((int)((outR / 125.0) * 255), 0, 255);

  analogWrite(pwmMotorA, pwmKiri);
  analogWrite(pwmMotorB, pwmKanan);

  Serial.print("Dist=");
  Serial.print(totalDistance, 4);
  Serial.print(" m | Rem=");
  Serial.print(remainingDist, 4);
  Serial.print(" m | θ=");
  Serial.print(theta_deg, 2);
  Serial.println("°");
}
