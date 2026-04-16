#define GEAR_RATIO 100
#define PPR 7
volatile long pulseCountLeft = 0;
volatile long pulseCountRight = 0;

// Encoder pin
const byte encLeftA = 14;   // D5
const byte encLeftB = 12;   // D6
const byte encRightA = 10;  // SD3
const byte encRightB = 13;  // D7

// Motor driver pin
const int pwmMotorA = 5;
const int pwmMotorB = 4;
const int dirMotorA = 0;
const int dirMotorB = 2;

// Kecepatan dasar motor
int baseSpeed = 255;
int pwmKiri = baseSpeed;
int pwmKanan = baseSpeed;

// PID kanan
double error_kanan = 0;
double lastError_kanan = 0;
double integral_kanan = 0;
double derivative_kanan = 0;

// PID kiri
double error_kiri = 0;
double lastError_kiri = 0;
double integral_kiri = 0;
double derivative_kiri = 0;

// PID output smoothing (low-pass filter)
double filteredOutput = 0;
const double alpha = 0.3;  // smoothing factor

// PID Kanan
double Kp_kanan = 4.90;
double Ki_kanan = 0.21;
double Kd_kanan = 0.02;

// PID Kiri
double Kp_kiri = 4.65;
double Ki_kiri = 0.206;
double Kd_kiri = 0.02;

// Interval waktu
unsigned long lastPrintTime = 0;
unsigned long lastControlTime = 0;
const unsigned long controlInterval = 20; // 100 ms untuk PID                                   
float dt = controlInterval / 1000.0 ; 
const unsigned long printInterval = 500; // 500 ms untuk Serial

long lastLeft = 0;
long lastRight = 0;

float targetRPM_Kanan = 50; //MAX 125
float targetRPM_kiri = 50; //MAX 125

// --- ISR Encoder ---
void ICACHE_RAM_ATTR isrLeftA() {
  pulseCountLeft += digitalRead(encLeftB) ? 1 : -1;
}

void ICACHE_RAM_ATTR isrRightA() {
  pulseCountRight += digitalRead(encRightB) ? 1 : -1;
}

// --- Setup ---
void setup() {
  Serial.begin(115200);

  // Encoder
  pinMode(encLeftA, INPUT_PULLUP);
  pinMode(encLeftB, INPUT_PULLUP);
  pinMode(encRightA, INPUT_PULLUP);
  pinMode(encRightB, INPUT_PULLUP);

  // Motor
  pinMode(pwmMotorA, OUTPUT);
  pinMode(pwmMotorB, OUTPUT);
  pinMode(dirMotorA, OUTPUT);
  pinMode(dirMotorB, OUTPUT);
  digitalWrite(dirMotorA, HIGH);
  digitalWrite(dirMotorB, HIGH);

  analogWrite(pwmMotorA, pwmKiri);
  analogWrite(pwmMotorB, pwmKanan);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(encLeftA), isrLeftA, RISING);
  attachInterrupt(digitalPinToInterrupt(encRightA), isrRightA, RISING);
}

// --- Loop utama ---
void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastControlTime >= controlInterval) {
    lastControlTime = currentTime;

    noInterrupts();
    long left = pulseCountLeft;
    long right = pulseCountRight;
    interrupts();

    double tickLeft = left - lastLeft;
    double tickRight = right - lastRight;

    float pps_left = tickLeft / dt;
    float pps_right = tickRight / dt;
    float rpm_left = (pps_left * 60) / (PPR * GEAR_RATIO);
    float rpm_right = (pps_right * 60) / (PPR * GEAR_RATIO);

    // --- PID Kanan ---
    error_kanan = targetRPM_Kanan - rpm_right;
    integral_kanan += error_kanan * dt;
    derivative_kanan = (error_kanan - lastError_kanan) / dt;

    double output_kanan = (Kp_kanan * error_kanan) + (Ki_kanan * integral_kanan) + (Kd_kanan * derivative_kanan);
    output_kanan = constrain(output_kanan, -125, 125);
    pwmKanan = constrain((int)((output_kanan / 125.0) * 255), 40, 255);

    // --- PID Kiri ---
    error_kiri = targetRPM_kiri - rpm_left;
    integral_kiri += error_kiri * dt;
    derivative_kiri = (error_kiri - lastError_kiri) / dt;

    double output_kiri = (Kp_kiri * error_kiri) + (Ki_kiri * integral_kiri) + (Kd_kiri * derivative_kiri);
    output_kiri = constrain(output_kiri, -125, 125);
    pwmKiri = constrain((int)((output_kiri / 125.0) * 255), 40, 255);

    // Kirim ke motor
    analogWrite(pwmMotorA, pwmKiri);
    analogWrite(pwmMotorB, pwmKanan);

    // Simpan nilai sebelumnya
    lastLeft = left;
    lastRight = right;
    lastError_kanan = error_kanan;
    lastError_kiri = error_kiri;

    // Serial debug
    Serial.print("RPM Kiri: "); Serial.print(rpm_left);
    Serial.print(" | RPM Kanan: "); Serial.println(rpm_right);
  }
}