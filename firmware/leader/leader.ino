#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <vector> // Pustaka C++ untuk array dinamis (waypoint tanpa batas)

// --- Konfigurasi WiFi ---
const char* ssid = "Redmi";         // Ganti dengan nama WiFi Anda
const char* password = "$#454565"; // Ganti dengan password WiFi Anda
const int tcp_port = 5555;                 // Port yang akan digunakan oleh TCP Server

WiFiServer tcpServer(tcp_port); // Buat objek server
WiFiClient client;              // Objek untuk menangani koneksi klien yang masuk

// --- PIN DEFINITIONS ---
const int pwmA_pin = 5;
const int dirA_pin = 0;
const int pwmB_pin = 4;
const int dirB_pin = 2;
const int encA1 = 14;
const int encA2 = 12;
const int encB1 = 13;
const int encB2 = 10;

// --- ROBOT SPECIFICATIONS & ODOMETRY CONSTANTS ---
const double WHEEL_DIAMETER_CM = 4.0;
const double WHEEL_CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;
const int TICKS_PER_REVOLUTION = 2800;
const double DISTANCE_PER_TICK_CM = WHEEL_CIRCUMFERENCE_CM / (double)TICKS_PER_REVOLUTION;
const double WHEEL_BASE_CM = 10.0;
const double GRID_SCALE_CM = 15.0;

// --- Waypoint Management ---
struct Waypoint {
    float x_grid;
    float y_grid;
};
std::vector<Waypoint> waypoints_list; // Menggunakan std::vector untuk jumlah waypoint dinamis
int current_target_waypoint_index = -1;
bool executing_waypoints = false;

String tcp_buffer = "";
const int MAX_TCP_BUFFER_LEN = 64;

// --- Target Dinamis ---
float CURRENT_TARGET_X_GRID = 0.0;
float CURRENT_TARGET_Y_GRID = 0.0;
float CURRENT_TARGET_X_CM = 0.0;
float CURRENT_TARGET_Y_CM = 0.0;

// --- NAVIGASI & KONTROL PARAMETERS ---
const float NAV_TARGET_SPEED_CM_S = 10.0;
const float NAV_TURN_SPEED_CM_S = 7.0;
const double ANGLE_THRESHOLD_RAD = 0.08;
const double DISTANCE_THRESHOLD_CM = 2.0;
const unsigned long NAV_LOOP_DELAY_MS = 20;

// --- IMPROVED PID VARIABLES (from Berhasil.ino) ---
// Separate PID parameters for left and right motors
double Kp_kiri = 1.8;   // Left motor PID gains
double Ki_kiri = 0.055;
double Kd_kiri = 0.066;

double Kp_kanan = 2.0;  // Right motor PID gains  
double Ki_kanan = 0.06;
double Kd_kanan = 0.07;

// PID variables for left motor (Motor A)
double error_kiri = 0;
double lastError_kiri = 0;
double integral_kiri = 0;
double derivative_kiri = 0;

// PID variables for right motor (Motor B)
double error_kanan = 0;
double lastError_kanan = 0;
double integral_kanan = 0;
double derivative_kanan = 0;

// Target speeds and PWM outputs
volatile float targetSpeedA_cm_s = 0.0;
volatile float targetSpeedB_cm_s = 0.0;
int pwmKiri = 0;
int pwmKanan = 0;

// PID timing
unsigned long lastPidControlTime = 0;
const int pidControlInterval = 20; // 20ms for smoother control
float dt = pidControlInterval / 1000.0;

// --- ENCODER VARIABLES ---
volatile long ticksA_pid = 0;
volatile long ticksB_pid = 0;
volatile long odometryTicksA = 0;
volatile long odometryTicksB = 0;
volatile int lastStateA_enc = 0;
volatile int lastStateB_enc = 0;

// For RPM calculation
long lastTicksA = 0;
long lastTicksB = 0;

inline float wrapPi(float a){ while(a>M_PI) a-=2*M_PI; while(a<-M_PI) a+=2*M_PI; return a; }
const float HEADING_OFFSET = 0.0f;

// --- ODOMETRY STATE ---
double x_pos_cm = 0.0;
double y_pos_cm = 0.0;
double theta_rad = 0.0;
long lastOdometryTicksA = 0;
long lastOdometryTicksB = 0;

// --- NAVIGATION STATE ---
enum RobotState { IDLE, TURNING, MOVING, GOAL_REACHED };
RobotState currentState = IDLE;
unsigned long lastNavUpdateTime = 0;
bool pid_idle_reset_done = false;

// ========== FUNGSI ISR ENCODER (Tidak Ada Perubahan) ==========
void IRAM_ATTR handleEncoderA() {
  int MSB = digitalRead(encA1);
  int LSB = digitalRead(encA2);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastStateA_enc << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    ticksA_pid++; odometryTicksA++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    ticksA_pid--; odometryTicksA--;
  }
  lastStateA_enc = encoded;
}

void IRAM_ATTR handleEncoderB() {
  int MSB = digitalRead(encB1);
  int LSB = digitalRead(encB2);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastStateB_enc << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    ticksB_pid++; odometryTicksB++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    ticksB_pid--; odometryTicksB--;
  }
  lastStateB_enc = encoded;
}

// ========== FUNGSI KONTROL MOTOR (Updated) ==========
void controlMotor(int motorId, int pwmValue) {
    int localPwmPin, localDirPin;
    bool commandForward = pwmValue >= 0;
    int pwmMagnitude = constrain(abs(pwmValue), 0, 1023);

    if (motorId == 0) { // Motor A (Kiri)
        localPwmPin = pwmA_pin;
        localDirPin = dirA_pin;
        digitalWrite(localDirPin, commandForward ? HIGH : LOW);
    } else { // Motor B (Kanan)
        localPwmPin = pwmB_pin;
        localDirPin = dirB_pin;
        digitalWrite(localDirPin, commandForward ? HIGH : LOW);
    }
    analogWrite(localPwmPin, pwmMagnitude);
}

// ========== FUNGSI HELPER & LOGIKA INTI ==========
double normalizeAngle(double angle) {
  while (angle > PI) angle -= 2.0 * PI;
  while (angle <= -PI) angle += 2.0 * PI;
  return angle;
}

void updateOdometry() {
  long currentOdoA, currentOdoB;
  noInterrupts();
  currentOdoA = odometryTicksA; currentOdoB = odometryTicksB;
  interrupts();
  long deltaTicksA = currentOdoA - lastOdometryTicksA;
  long deltaTicksB = currentOdoB - lastOdometryTicksB;
  lastOdometryTicksA = currentOdoA; lastOdometryTicksB = currentOdoB;
  deltaTicksA *= -1; // Koreksi arah odometry kiri
  double distA = deltaTicksA * DISTANCE_PER_TICK_CM;
  double distB = deltaTicksB * DISTANCE_PER_TICK_CM;
  double deltaDistance = (distA + distB) / 2.0;
  double deltaTheta = (distB - distA) / WHEEL_BASE_CM;
  x_pos_cm += deltaDistance * cos(theta_rad + deltaTheta / 2.0);
  y_pos_cm += deltaDistance * sin(theta_rad + deltaTheta / 2.0);
  theta_rad += deltaTheta;
  theta_rad = normalizeAngle(theta_rad);
}

void resetPidStates() {
    noInterrupts();
    // Reset left motor PID
    integral_kiri = 0.0;
    lastError_kiri = 0.0;
    error_kiri = 0.0;
    
    // Reset right motor PID
    integral_kanan = 0.0;
    lastError_kanan = 0.0;
    error_kanan = 0.0;
    
    // Reset PWM outputs
    pwmKiri = 0;
    pwmKanan = 0;
    
    // Reset tick counters for PID
    lastTicksA = ticksA_pid;
    lastTicksB = ticksB_pid;
    interrupts();
    
    Serial.println("DEBUG: PID states reset.");
}

void resetSystemForNewMission() {
    resetPidStates();
    targetSpeedA_cm_s = 0.0f;
    targetSpeedB_cm_s = 0.0f;
    currentState = IDLE;
    executing_waypoints = false;
    current_target_waypoint_index = -1;
    waypoints_list.clear();
    String msg = "Waypoints cleared. Ready for new SETPOS and waypoints.";
    if (client && client.connected()) client.println(msg);
    Serial.println(msg);
}

void setInitialPoseDeg(float newX_cm, float newY_cm, float newTheta_deg) {
    x_pos_cm = newX_cm;
    y_pos_cm = newY_cm;
    theta_rad = newTheta_deg * PI / 180.0;
    noInterrupts();
    lastOdometryTicksA = odometryTicksA;
    lastOdometryTicksB = odometryTicksB;
    interrupts();
    resetPidStates();
    targetSpeedA_cm_s = 0.0f;
    targetSpeedB_cm_s = 0.0f;
}
void setInitialPoseRad(float x_cm, float y_cm, float th_rad){
    noInterrupts();
    x_pos_cm  = x_cm;
    y_pos_cm  = y_cm;
    theta_rad = wrapPi(th_rad + HEADING_OFFSET);
    lastOdometryTicksA = odometryTicksA;
    lastOdometryTicksB = odometryTicksB;
    interrupts();
    resetPidStates();
    targetSpeedA_cm_s = 0.0f;
    targetSpeedB_cm_s = 0.0f;
    Serial.printf("POSE SET: x=%.2f y=%.2f th(deg)=%.2f\n",
                    x_pos_cm, y_pos_cm, theta_rad * 180.0 / PI);
}


void setNextWaypointAsTarget() {
    if (current_target_waypoint_index >= 0 && (unsigned int)current_target_waypoint_index < waypoints_list.size()) {
        CURRENT_TARGET_X_GRID = waypoints_list[current_target_waypoint_index].x_grid;
        CURRENT_TARGET_Y_GRID = waypoints_list[current_target_waypoint_index].y_grid;

        // Hitung posisi fisik target di TENGAH sel grid
        CURRENT_TARGET_X_CM = (CURRENT_TARGET_X_GRID * GRID_SCALE_CM) + (GRID_SCALE_CM / 2.0f);
        CURRENT_TARGET_Y_CM = (CURRENT_TARGET_Y_GRID * GRID_SCALE_CM) + (GRID_SCALE_CM / 2.0f);

        resetPidStates();
        currentState = IDLE;
        pid_idle_reset_done = false;
        
        String msg = "Navigating to waypoint #" + String(current_target_waypoint_index) +
                     ": Grid(" + String(CURRENT_TARGET_X_GRID) +
                     "," + String(CURRENT_TARGET_Y_GRID) + ") -> CM(" + String(CURRENT_TARGET_X_CM) + "," + String(CURRENT_TARGET_Y_CM) + ")";
        if (client && client.connected()) client.println(msg);
        Serial.println(msg);
    } else {
        String msg = "All waypoints reached or invalid index.";
        if (client && client.connected()) client.println(msg);
        Serial.println(msg);
        executing_waypoints = false;
        current_target_waypoint_index = -1;
        targetSpeedA_cm_s = 0.0f;
        targetSpeedB_cm_s = 0.0f;
        currentState = IDLE;
    }
}

void parseCommand(String cmd) {
    cmd.trim();
    String feedback = "ESP Received: [" + cmd + "]";
    Serial.println(feedback);
    if (client && client.connected()) client.println(feedback);

    String upperCmd = cmd;
    upperCmd.toUpperCase();

    if (upperCmd == "CLEAR") {
        resetSystemForNewMission();
    } else if (upperCmd.startsWith("SETPOS ")) {
    // format radian: SETPOS x,y,thetaRad
      String p = cmd.substring(7);
      int c1 = p.indexOf(','); int c2 = p.indexOf(',', c1+1);
      if (c1!=-1 && c2!=-1) {
        float nx = p.substring(0,c1).toFloat();
        float ny = p.substring(c1+1,c2).toFloat();
        float nth = p.substring(c2+1).toFloat();   // RAD
        setInitialPoseRad(nx, ny, nth);
        if (client && client.connected()) client.println("OK SETPOS");
      } else {
        if (client && client.connected()) client.println("ERROR SETPOS x,y,thetaRad");
      }
    }
    else if (upperCmd.startsWith("SETPOS_DEG ")) {
    String p = cmd.substring(11);
    int c1 = p.indexOf(','); 
    int c2 = p.indexOf(',', c1 + 1);
    if (c1 != -1 && c2 != -1) {
        float nx = p.substring(0, c1).toFloat();
        float ny = p.substring(c1 + 1, c2).toFloat();
        float nthD = p.substring(c2 + 1).toFloat();
        setInitialPoseRad(nx, ny, nthD * PI / 180.0f);
        if (client && client.connected()) client.println("OK SETPOS_DEG");
        Serial.printf("POSE SET: x=%.2f y=%.2f th(deg)=%.2f\n",
              x_pos_cm, y_pos_cm, theta_rad * 180.0 / PI);

    }
    } else if (upperCmd == "EXECUTE") {
        if (!waypoints_list.empty()) {
            executing_waypoints = true;
            current_target_waypoint_index = 0;
            setNextWaypointAsTarget();
            String msg = "Executing waypoint navigation...";
            if (client && client.connected()) client.println(msg);
            Serial.println(msg);
        } else {
            String msg = "No waypoints to execute.";
            if (client && client.connected()) client.println(msg);
            Serial.println(msg);
        }
    } else {
        int commaIndex = cmd.indexOf(',');
        if (commaIndex != -1) {
            String x_str = cmd.substring(0, commaIndex);
            String y_str = cmd.substring(commaIndex + 1);
            if (x_str.length() > 0 && y_str.length() > 0) {
                Waypoint new_wp;
                new_wp.x_grid = x_str.toFloat();
                new_wp.y_grid = y_str.toFloat();
                waypoints_list.push_back(new_wp);
                String msg = "Stored waypoint #" + String(waypoints_list.size() - 1) +
                             ": Grid(" + String(new_wp.x_grid) + "," + String(new_wp.y_grid) + ")";
                if (client && client.connected()) client.println(msg);
                Serial.println(msg);
            } else {
                 String err_msg = "ERROR: Invalid coordinate format: " + cmd;
                if (client && client.connected()) client.println(err_msg);
                Serial.println(err_msg);
            }
        } else {
            String err_msg = "ERROR: Unknown command: " + cmd;
            if (client && client.connected()) client.println(err_msg);
            Serial.println(err_msg);
        }
    }

}

// ========== FUNGSI SETUP ==========
void setup() {
    Serial.begin(115200);
    delay(10);
    Serial.println("\nConnecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: "); Serial.println(WiFi.localIP());
    
    tcpServer.begin();
    Serial.print("TCP server started on port "); Serial.println(tcp_port);
    Serial.println("Waiting for client...");

    pinMode(pwmA_pin, OUTPUT); pinMode(dirA_pin, OUTPUT);
    pinMode(pwmB_pin, OUTPUT); pinMode(dirB_pin, OUTPUT);
    pinMode(encA1, INPUT_PULLUP); pinMode(encA2, INPUT_PULLUP);
    pinMode(encB1, INPUT_PULLUP); pinMode(encB2, INPUT_PULLUP);

    noInterrupts();
    lastStateA_enc = (digitalRead(encA1) << 1) | digitalRead(encA2);
    lastStateB_enc = (digitalRead(encB1) << 1) | digitalRead(encB2);
    interrupts();
    
    waypoints_list.reserve(15);
    tcp_buffer.reserve(MAX_TCP_BUFFER_LEN);

    attachInterrupt(digitalPinToInterrupt(encA1), handleEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA2), handleEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB1), handleEncoderB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encB2), handleEncoderB, CHANGE);
    
    resetSystemForNewMission();
    //setInitialPose(0.0, 0.0, 0.0);
    
    lastNavUpdateTime = millis();
    lastPidControlTime = millis();
    Serial.println("Setup complete. Robot ready for commands via WiFi.");
}

// ========== FUNGSI LOOP UTAMA ==========
void loop() {
    yield(); // Beri waktu untuk proses background ESP8266 (WiFi, TCP)

    unsigned long currentTime = millis();

    // --- Handle Koneksi Klien TCP ---
    if (!client || !client.connected()) {
        if (client) client.stop();
        client = tcpServer.available();
        if (client) {
            Serial.println("New client connected!");
            client.println("ESP8266 Robot Connected. Send commands.");
            tcp_buffer = "";
        }
    }

    if (client && client.connected()) {
        while (client.available() > 0) {
            char incomingChar = (char)client.read();
            if (incomingChar == '\n') {
                if (tcp_buffer.length() > 0) {
                    parseCommand(tcp_buffer);
                }
                tcp_buffer = "";
            } else if (tcp_buffer.length() < MAX_TCP_BUFFER_LEN) {
                tcp_buffer += incomingChar;
            }
        }
    }

    // --- IMPROVED PID Speed Control Loop (from Berhasil.ino) ---
    if (currentTime - lastPidControlTime >= pidControlInterval) {
        // Get current encoder ticks
        noInterrupts();
        long currentTicksA_for_pid = ticksA_pid;
        long currentTicksB_for_pid = ticksB_pid;
        interrupts();

        // Calculate tick differences
        long deltaTicksA = currentTicksA_for_pid - lastTicksA;
        long deltaTicksB = currentTicksB_for_pid - lastTicksB;
        
        // Correct direction for left motor
        deltaTicksA *= -1;
        
        // Calculate actual speeds in cm/s
        float speedA_actual_cm_s = (deltaTicksA / (float)TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM / dt;
        float speedB_actual_cm_s = (deltaTicksB / (float)TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM / dt;
        
        // --- PID for Left Motor (Motor A) ---
        error_kiri = targetSpeedA_cm_s - speedA_actual_cm_s;
        
        // Reset integral when target is zero and actual speed is low (anti-windup)
        if (targetSpeedA_cm_s == 0.0f && abs(speedA_actual_cm_s) < 0.2f) {
            integral_kiri = 0;
        } else {
            integral_kiri += error_kiri * dt;
            integral_kiri = constrain(integral_kiri, -500.0, 500.0);
        }
        
        derivative_kiri = (error_kiri - lastError_kiri) / dt;
        
        double output_kiri = (Kp_kiri * error_kiri) + (Ki_kiri * integral_kiri) + (Kd_kiri * derivative_kiri);
        output_kiri = constrain(output_kiri, -125, 125);
        
        // Convert to PWM (0-1023 range)
        pwmKiri = constrain((int)((output_kiri / 125.0) * 1023), -1023, 1023);
        if (abs(pwmKiri) < 40 && targetSpeedA_cm_s != 0.0f) {
            pwmKiri = (pwmKiri >= 0) ? 40 : -40; // Minimum PWM to overcome friction
        }
        
        // --- PID for Right Motor (Motor B) ---
        error_kanan = targetSpeedB_cm_s - speedB_actual_cm_s;
        
        // Reset integral when target is zero and actual speed is low (anti-windup)
        if (targetSpeedB_cm_s == 0.0f && abs(speedB_actual_cm_s) < 0.2f) {
            integral_kanan = 0;
        } else {
            integral_kanan += error_kanan * dt;
            integral_kanan = constrain(integral_kanan, -500.0, 500.0);
        }
        
        derivative_kanan = (error_kanan - lastError_kanan) / dt;
        
        double output_kanan = (Kp_kanan * error_kanan) + (Ki_kanan * integral_kanan) + (Kd_kanan * derivative_kanan);
        output_kanan = constrain(output_kanan, -125, 125);
        
        // Convert to PWM (0-1023 range)
        pwmKanan = constrain((int)((output_kanan / 125.0) * 1023), -1023, 1023);
        if (abs(pwmKanan) < 40 && targetSpeedB_cm_s != 0.0f) {
            pwmKanan = (pwmKanan >= 0) ? 40 : -40; // Minimum PWM to overcome friction
        }

        // Apply motor control
        controlMotor(0, pwmKiri);   // Left motor
        controlMotor(1, pwmKanan);  // Right motor

        // Store previous values
        lastTicksA = currentTicksA_for_pid;
        lastTicksB = currentTicksB_for_pid;
        lastError_kiri = error_kiri;
        lastError_kanan = error_kanan;

        lastPidControlTime = currentTime;
        
        // Debug output (uncomment if needed)
        /*
        Serial.print("Target L/R: "); Serial.print(targetSpeedA_cm_s); Serial.print("/"); Serial.print(targetSpeedB_cm_s);
        Serial.print(" | Actual L/R: "); Serial.print(speedA_actual_cm_s); Serial.print("/"); Serial.print(speedB_actual_cm_s);
        Serial.print(" | PWM L/R: "); Serial.print(pwmKiri); Serial.print("/"); Serial.println(pwmKanan);
        */
    }

    // --- Navigation and Odometry Loop ---
    if (executing_waypoints) {
        if (currentTime - lastNavUpdateTime >= NAV_LOOP_DELAY_MS) {
            updateOdometry();

            float deltaX = CURRENT_TARGET_X_CM - (float)x_pos_cm;
            float deltaY = CURRENT_TARGET_Y_CM - (float)y_pos_cm;
            float distanceToTarget = sqrt(deltaX * deltaX + deltaY * deltaY);
            float angleToTarget = atan2(deltaY, deltaX);
            float angleError = normalizeAngle(angleToTarget - (float)theta_rad);

            Serial.println("--- Nav Debug ---");
            Serial.print("Pose: x="); Serial.print(x_pos_cm, 2);
            Serial.print(" y="); Serial.print(y_pos_cm, 2);
            Serial.print(" th(deg)="); Serial.println(theta_rad * 180.0 / PI, 2);
            Serial.print("Target: x="); Serial.print(CURRENT_TARGET_X_CM, 2);
            Serial.print(" y="); Serial.println(CURRENT_TARGET_Y_CM, 2);
            Serial.print("Deltas: dx="); Serial.print(deltaX, 2);
            Serial.print(" dy="); Serial.println(deltaY, 2);
            Serial.print("Distance to Target: "); Serial.println(distanceToTarget, 2);
            Serial.print("Angle to Target (deg): "); Serial.println(angleToTarget * 180.0 / PI, 2);
            Serial.print("Angle Error (deg): "); Serial.println(angleError * 180.0 / PI, 2);
            Serial.println("-----------------");

            switch (currentState) {
                case IDLE:
                {
                    if (targetSpeedA_cm_s != 0.0f || !pid_idle_reset_done) {
                        resetPidStates(); pid_idle_reset_done = true;
                    }
                    targetSpeedA_cm_s = 0.0f; targetSpeedB_cm_s = 0.0f;
                    
                    RobotState determinedNextState = IDLE;
                    if (distanceToTarget <= DISTANCE_THRESHOLD_CM) {
                        determinedNextState = GOAL_REACHED;
                    } else if (abs(angleError) > ANGLE_THRESHOLD_RAD) {
                        determinedNextState = TURNING;
                    } else {
                        determinedNextState = MOVING;
                    }
                    if (determinedNextState != IDLE) {
                        resetPidStates();
                        pid_idle_reset_done = false;
                        currentState = determinedNextState;
                    }
                }
                break;
                case TURNING:
                {
                    pid_idle_reset_done = false;
                    if (abs(angleError) <= ANGLE_THRESHOLD_RAD) {
                        targetSpeedA_cm_s = 0.0f; targetSpeedB_cm_s = 0.0f;
                        resetPidStates();
                        currentState = MOVING;
                    } else {
                        if (angleError > 0) {
                            targetSpeedA_cm_s = -NAV_TURN_SPEED_CM_S; targetSpeedB_cm_s = NAV_TURN_SPEED_CM_S;
                        } else {
                            targetSpeedA_cm_s = NAV_TURN_SPEED_CM_S; targetSpeedB_cm_s = -NAV_TURN_SPEED_CM_S;
                        }
                    }
                }
                break;
                case MOVING:
                {
                    pid_idle_reset_done = false;
                    if (distanceToTarget <= DISTANCE_THRESHOLD_CM) {
                        targetSpeedA_cm_s = 0.0f; targetSpeedB_cm_s = 0.0f;
                        resetPidStates();
                        currentState = GOAL_REACHED;
                    } else if (abs(angleError) > ANGLE_THRESHOLD_RAD * 1.5) {
                        targetSpeedA_cm_s = 0.0f; targetSpeedB_cm_s = 0.0f;
                        resetPidStates();
                        currentState = TURNING;
                    } else {
                        float steeringAdjustment = angleError * (NAV_TARGET_SPEED_CM_S * 0.5f);
                        targetSpeedA_cm_s = NAV_TARGET_SPEED_CM_S - steeringAdjustment;
                        targetSpeedB_cm_s = NAV_TARGET_SPEED_CM_S + steeringAdjustment;
                    }
                }
                break;
                case GOAL_REACHED:
                {
                    pid_idle_reset_done = false;
                    String wpReachedMsg = "Waypoint #" + String(current_target_waypoint_index) + " reached.";
                    if(client && client.connected()) client.println(wpReachedMsg);
                    Serial.println(wpReachedMsg);

                    current_target_waypoint_index++;
                    if (current_target_waypoint_index < (int)waypoints_list.size()) {
                        setNextWaypointAsTarget();
                    } else {
                        pwmKiri = 0;
                        pwmKanan = 0;
                        pid_idle_reset_done = false;
                        String allDoneMsg = "ALL_WAYPOINTS_REACHED";
                        if (client && client.connected()) client.println(allDoneMsg);
                        Serial.println("--- SEMUA WAYPOINT TERCAPAI ---");
                        executing_waypoints = false;
                        current_target_waypoint_index = -1;
                        currentState = IDLE;
                        targetSpeedA_cm_s = 0.0f;
                        targetSpeedB_cm_s = 0.0f;
                    }
                }
                break;
            }
            lastNavUpdateTime = currentTime;
        }
    } else {
        if (targetSpeedA_cm_s != 0.0f || targetSpeedB_cm_s != 0.0f) {
            targetSpeedA_cm_s = 0.0f; targetSpeedB_cm_s = 0.0f;
            resetPidStates();
        }
    }
}