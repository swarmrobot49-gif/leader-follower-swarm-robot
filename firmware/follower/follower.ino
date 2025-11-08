#include <Arduino.h>
#include <ESP8266WiFi.h>

/************ USER CONFIG ************/
const char *WIFI_SSID = "Redmi";
const char *WIFI_PASS = "$#454565";
const uint16_t TCP_PORT = 6060;

// Geometry & encoders
float WHEEL_RADIUS_CM = 2.0f;
float WHEEL_TRACK_CM = 10.00f;
int ENC_TICKS_PER_REV = 2800;

// Motor pins
const int PIN_L_PWM = 5;
const int PIN_L_DIR = 0;
const int PIN_R_PWM = 4;
const int PIN_R_DIR = 2;

// Encoder pins
const int PIN_ENC_L_A = 14;
const int PIN_ENC_L_B = 12;
const int PIN_ENC_R_A = 13;
const int PIN_ENC_R_B = 10;

const int ENC_SIGN_L = +1;
const int ENC_SIGN_R = -1;

// ============================================================
// IMPROVED TUNING PARAMETERS
// ============================================================
float V_MAX_CM_S = 8.0f;
float W_MAX_RAD_S = 1.6f;
float KV = 1.10f;
float KW = 1.8f;
float FOLLOW_DIST_CM = 16.0f;

const float ANG_DB_RAD = 8.0f * M_PI / 180.0f;
const float TURN_ONLY_RAD = 60.0f * M_PI / 180.0f; 
const float TARGET_LP = 0.50f;
const float V_MIN_RUN_CM_S = 4.0f;

float FOLLOW_STOP_DIST_CM = 13.0f;
float FOLLOW_RESUME_DIST_CM = 16.0f;
float ANGLE_TOL_RAD = 0.60f;
uint32_t ARRIVE_DWELL_MS = 500;

const uint32_t LEADER_TIMEOUT_MS = 500;

// ============================================================
// IMPROVED PID GAINS with Better Damping
// ============================================================
float KpL = 1.5f, KiL = 0.0f, KdL = 0.0f;  // Higher Kp & Kd for better tracking
float KpR = 1.5f, KiR = 0.0f, KdR = 0.0f;

float I_MAX_L = 30.0f;
float I_MAX_R = 30.0f;
float WHEEL_W_NORM = 20.0f;

/************ INTERNAL STATE ************/
volatile long encL = 0, encR = 0;
float poseX = 0.0f, poseY = 0.0f, poseTh = 0.0f;
volatile float leaderX = 0.0f, leaderY = 0.0f, leaderTh = 0.0f;
volatile uint32_t lastLeaderUpdateMs = 0;
float lastTargetX = 0.0f, lastTargetY = 0.0f;
float lastDistToTarget = 0.0f, lastAngErr = 0.0f;
volatile bool enabled = false;
bool holdStop = false;
uint32_t withinSinceMs = 0;

// ============================================================
// ENHANCED PID STATE with Anti-Jerk Features
// ============================================================
// Wheel PID state
float intL = 0, intR = 0, prevErrL = 0, prevErrR = 0;

// For derivative on measurement (not on error):
float last_speedL = 0.0f, last_speedR = 0.0f;

// Derivative filtering
float dELf = 0.0f, dERf = 0.0f;
const float ALPHA_D = 0.30f;

// ============================================================
// COMMAND VELOCITY SMOOTHING (Anti-Jerk)
// ============================================================
float cmdV_filtered = 0.0f;
float cmdW_filtered = 0.0f;
const float CMD_LP_ALPHA = 0.30f;

// Wheel command smoothing
float wL_cmd_filtered = 0.0f;
float wR_cmd_filtered = 0.0f;
const float ALPHA_WHEEL_CMD = 0.40f;  // Smooth wheel commands

// Network
WiFiServer server(TCP_PORT);
WiFiClient client;
String rxBuf;

// Timing
const float ODOM_DT = 0.01f;
const float CTRL_DT = 0.02f;
const float PID_DT = 0.02f;
const float TEL_DT = 0.10f;

/************ HELPERS ************/
inline float clamp(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
inline float wrapPi(float a) {
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

/************ ENCODER ISRs ************/
void IRAM_ATTR encL_A() {
  encL += (digitalRead(PIN_ENC_L_A) == digitalRead(PIN_ENC_L_B)) ? +1 : -1;
}
void IRAM_ATTR encL_B() {
  encL += (digitalRead(PIN_ENC_L_A) != digitalRead(PIN_ENC_L_B)) ? +1 : -1;
}
void IRAM_ATTR encR_A() {
  encR += (digitalRead(PIN_ENC_R_A) == digitalRead(PIN_ENC_R_B)) ? +1 : -1;
}
void IRAM_ATTR encR_B() {
  encR += (digitalRead(PIN_ENC_R_A) != digitalRead(PIN_ENC_R_B)) ? +1 : -1;
}

/************ MOTOR OUTPUT ************/
void setWheel(float uL, float uR) {
  bool dirL = (uL >= 0);
  bool dirR = (uR >= 0);
  analogWrite(PIN_L_PWM, (int)(clamp(fabs(uL), 0, 1) * 1023));
  digitalWrite(PIN_L_DIR, dirL ? HIGH : LOW);
  analogWrite(PIN_R_PWM, (int)(clamp(fabs(uR), 0, 1) * 1023));
  digitalWrite(PIN_R_DIR, dirR ? HIGH : LOW);
}

/************ KINEMATICS ************/
float ticksToWheelRad(long ticks) {
  return (2.0f * M_PI) * ((float)ticks / (float)ENC_TICKS_PER_REV);
}

void odomUpdate() {
  static long lastL = 0, lastR = 0;
  static uint32_t lastT = millis();
  uint32_t now = millis();
  if (now - lastT < (uint32_t)(ODOM_DT * 1000)) return;
  float dt = (now - lastT) / 1000.0f;
  lastT = now;

  long tL = encL;
  long tR = encR;
  long dL = tL - lastL;
  long dR = tR - lastR;
  lastL = tL;
  lastR = tR;

  float dphiL = ENC_SIGN_L * ticksToWheelRad(dL);
  float dphiR = ENC_SIGN_R * ticksToWheelRad(dR);
  float vL = dphiL * WHEEL_RADIUS_CM / dt;
  float vR = dphiR * WHEEL_RADIUS_CM / dt;

  float v = 0.5f * (vR + vL);
  float w = (vR - vL) / WHEEL_TRACK_CM;

  poseTh = wrapPi(poseTh + w * dt);
  poseX += v * cosf(poseTh) * dt;
  poseY += v * sinf(poseTh) * dt;
}

/************ FOLLOW CONTROL - IMPROVED ************/
void followControl(float &cmdV, float &cmdW) {
  // 1) Target point
  float tgtX = leaderX - FOLLOW_DIST_CM * cosf(leaderTh);
  float tgtY = leaderY - FOLLOW_DIST_CM * sinf(leaderTh);

  // 2) Low-pass filter target
  static float ftX = 0.0f, ftY = 0.0f;
  static bool init = false;
  if (!init) {
    ftX = tgtX;
    ftY = tgtY;
    init = true;
  }
  ftX += TARGET_LP * (tgtX - ftX);
  ftY += TARGET_LP * (tgtY - ftY);

  // 3) Error posisi & sudut
  float dx = ftX - poseX;
  float dy = ftY - poseY;
  float r = sqrtf(dx * dx + dy * dy);
  float bearing = atan2f(dy, dx);

  float alpha = wrapPi(bearing - poseTh);

  // 4) Filter + deadband sudut
  static float alpha_f = 0.0f;
  alpha_f += 0.30f * (alpha - alpha_f);
  alpha = fabsf(alpha_f) < ANG_DB_RAD ? 0.0f : alpha_f;

  // 5) Adaptive W_CAP
  float dist_factor = fminf(r / 40.0f, 1.0f);
  float W_CAP = 0.5f + 1.0f * dist_factor;

  // 6) Drive-turn logic
  if (fabsf(alpha) > TURN_ONLY_RAD && r > FOLLOW_STOP_DIST_CM + 3.0f) {
    cmdV = 0.0f;
    cmdW = clamp(KW * alpha, -W_CAP, W_CAP);
  } else {
    float v_des = KV * r * cosf(alpha);

    // Angle penalty (slow down when angle is large)
    float angle_penalty = 1.0f - (fabsf(alpha) / M_PI);
    angle_penalty = clamp(angle_penalty, 0.3f, 1.0f);
    v_des *= angle_penalty;

    if (v_des < 0) v_des = 0;
    if (r > FOLLOW_STOP_DIST_CM + 2.0f)
      v_des = fmaxf(v_des, V_MIN_RUN_CM_S);

    cmdV = clamp(v_des, 0.0f, V_MAX_CM_S);
    cmdW = clamp(KW * alpha, -W_CAP, W_CAP);
  }

  // ============================================================
  // COMMAND VELOCITY LOW-PASS FILTER (Anti-Jerk)
  // ============================================================
  cmdV_filtered += CMD_LP_ALPHA * (cmdV - cmdV_filtered);
  cmdW_filtered += CMD_LP_ALPHA * (cmdW - cmdW_filtered);

  cmdV = cmdV_filtered;
  cmdW = cmdW_filtered;

  lastTargetX = ftX;
  lastTargetY = ftY;
  lastDistToTarget = r;
  lastAngErr = alpha;
}

/************ VELOCITY → WHEELS ************/
void velocityToWheel(float v, float w, float &wL_cmd, float &wR_cmd) {
  float vL = v - 0.5f * w * WHEEL_TRACK_CM;
  float vR = v + 0.5f * w * WHEEL_TRACK_CM;
  wL_cmd = vL / WHEEL_RADIUS_CM;
  wR_cmd = vR / WHEEL_RADIUS_CM;
}

/************ IMPROVED WHEEL PID (Anti-Jerk) ************/
void wheelPID(float wL_cmd_raw, float wR_cmd_raw) {
  static uint32_t lastT = millis();
  uint32_t now = millis();
  if (now - lastT < (uint32_t)(PID_DT * 1000)) return;
  float dt = (now - lastT) / 1000.0f;
  lastT = now;

  // ============================================================
  // WHEEL COMMAND SMOOTHING (Anti-Jerk Step 1)
  // ============================================================
  wL_cmd_filtered += ALPHA_WHEEL_CMD * (wL_cmd_raw - wL_cmd_filtered);
  wR_cmd_filtered += ALPHA_WHEEL_CMD * (wR_cmd_raw - wR_cmd_filtered);

  float wL_cmd = wL_cmd_filtered;
  float wR_cmd = wR_cmd_filtered;

  // Get actual speeds
  static long lastL = 0, lastR = 0;
  long tL = encL;
  long tR = encR;
  long dL = tL - lastL;
  long dR = tR - lastR;
  lastL = tL;
  lastR = tR;

  float dphiL = ticksToWheelRad(dL);
  float dphiR = ticksToWheelRad(dR);
  float wL = (ENC_SIGN_L * dphiL) / dt;
  float wR = (ENC_SIGN_R * dphiR) / dt;

  // Calculate errors
  float eL = wL_cmd - wL;
  float eR = wR_cmd - wR;

  // ============================================================
  // AGGRESSIVE INTEGRAL RESET (Anti-Jerk Step 2)
  // Reset integral when target is zero and robot is nearly stopped
  // ============================================================
  if (fabsf(wL_cmd) < 0.1f && fabsf(wL) < 0.5f) {
    intL = 0;
    prevErrL = 0;
    last_speedL = 0;
  }
  if (fabsf(wR_cmd) < 0.1f && fabsf(wR) < 0.5f) {
    intR = 0;
    prevErrR = 0;
    last_speedR = 0;
  }

  // ============================================================
  // IMPROVED INTEGRAL with CONDITIONAL ANTI-WINDUP (Anti-Jerk Step 3)
  // Only integrate if output is not saturated
  // ============================================================
  float uL_pre = KpL * eL + KiL * intL;  // Tentative output tanpa I update
  float uR_pre = KpR * eR + KiR * intR;

  bool saturatedL = (fabsf(uL_pre / WHEEL_W_NORM) >= 0.95f);
  bool saturatedR = (fabsf(uR_pre / WHEEL_W_NORM) >= 0.95f);

  // Only integrate if not saturated, or if error opposes integral sign
  if (!saturatedL || ((eL > 0) != (intL > 0))) {
    intL += eL * dt;
    intL = clamp(intL, -I_MAX_L, I_MAX_L);
  }
  if (!saturatedR || ((eR > 0) != (intR > 0))) {
    intR += eR * dt;
    intR = clamp(intR, -I_MAX_R, I_MAX_R);
  }

  // ============================================================
  // DERIVATIVE ON MEASUREMENT (Anti-Jerk Step 4)
  // This eliminates derivative kick when command changes
  // ============================================================
  float dL_raw = -(wL - last_speedL) / dt;  // Negative because derivative of -measurement
  float dR_raw = -(wR - last_speedR) / dt;
  last_speedL = wL;
  last_speedR = wR;

  // Low-pass filter on derivative
  dELf = dELf + ALPHA_D * (dL_raw - dELf);
  dERf = dERf + ALPHA_D * (dR_raw - dERf);

  // Final PID output
  float uL = KpL * eL + KiL * intL + KdL * dELf;
  float uR = KpR * eR + KiR * intR + KdR * dERf;

  // Normalize to PWM [-1,1]
  float nL = clamp(uL / WHEEL_W_NORM, -1.0f, 1.0f);
  float nR = clamp(uR / WHEEL_W_NORM, -1.0f, 1.0f);
  Serial.print("PID wL* / wR*: ");
  Serial.print(wL_cmd, 2);
  Serial.print(" / ");
  Serial.print(wR_cmd, 2);
  Serial.print(" | wL / wR: ");
  Serial.print(wL, 2);
  Serial.print(" / ");
  Serial.print(wR, 2);
  Serial.print(" | nL / nR: ");
  Serial.print(nL, 2);
  Serial.print(" / ");
  Serial.println(nR, 2);

  // ============================================================
  // PWM DEADBAND (Anti-Jerk Step 5)
  // Force zero output if target is zero and PWM is small
  // ============================================================
  const float PWM_DEADBAND = 0.04f;  // ~40 PWM out of 1023
  if (fabsf(wL_cmd) < 0.1f && fabsf(nL) < PWM_DEADBAND) {
    nL = 0.0f;
  }
  if (fabsf(wR_cmd) < 0.1f && fabsf(nR) < PWM_DEADBAND) {
    nR = 0.0f;
  }

  setWheel(nL, nR);
}

/************ NETWORK & PROTOCOL ************/
void handleLine(const String &line) {
  if (line.length() == 0) return;
  //Serial.print("RX> "); Serial.println(line);

  const int MAXTOK = 10;
  String tok[MAXTOK];
  int nt = 0;
  int i = 0, j = 0;
  String s = line;
  s.trim();

  while (j < s.length() && nt < MAXTOK) {
    while (i < s.length() && isspace(s[i])) i++;
    j = i;
    while (j < s.length() && !isspace(s[j])) j++;
    if (j > i) tok[nt++] = s.substring(i, j);
    i = j + 1;
  }

  if (nt == 0) return;

  auto toF = [&](int idx) {
    return (idx < nt) ? tok[idx].toFloat() : 0.0f;
  };

  if (tok[0] == "START") {
    enabled = true;
  } else if (tok[0] == "STOP") {
    enabled = false;
    setWheel(0, 0);
    // CRITICAL: Reset all PID states
    intL = intR = 0;
    prevErrL = prevErrR = 0;
    last_speedL = last_speedR = 0;
    dELf = dERf = 0;
    cmdV_filtered = 0;
    cmdW_filtered = 0;
    wL_cmd_filtered = 0;
    wR_cmd_filtered = 0;
  } else if (tok[0] == "SETPOS" && nt >= 4) {
    poseX = toF(1);
    poseY = toF(2);
    poseTh = toF(3);
  } else if (tok[0] == "SETLEADER" && nt >= 4) {
    leaderX = toF(1);
    leaderY = toF(2);
    leaderTh = toF(3);
    lastLeaderUpdateMs = millis();
  } else if (tok[0] == "SETFOLLOWDIST" && nt >= 2) {
    FOLLOW_DIST_CM = toF(1);
  } else if (tok[0] == "SETVEL" && nt >= 3) {
    V_MAX_CM_S = toF(1);
    W_MAX_RAD_S = toF(2);
  } else if (tok[0] == "SETGAIN" && nt >= 3) {
    KV = toF(1);
    KW = toF(2);
  } else if (tok[0] == "SETPID" && nt >= 4) {
    float kp = toF(1), ki = toF(2), kd = toF(3);
    KpL = KpR = kp;
    KiL = KiR = ki;
    KdL = KdR = kd;
    intL = intR = 0;
    prevErrL = prevErrR = 0;
  } else if (tok[0] == "SETPIDL" && nt >= 4) {
    KpL = toF(1);
    KiL = toF(2);
    KdL = toF(3);
    intL = 0;
    prevErrL = 0;
  } else if (tok[0] == "SETPIDR" && nt >= 4) {
    KpR = toF(1);
    KiR = toF(2);
    KdR = toF(3);
    intR = 0;
    prevErrR = 0;
  } else if (tok[0] == "SETPID6" && nt >= 7) {
    KpL = toF(1);
    KiL = toF(2);
    KdL = toF(3);
    KpR = toF(4);
    KiR = toF(5);
    KdR = toF(6);
    intL = intR = 0;
    prevErrL = prevErrR = 0;
  } else if (tok[0] == "SETSTOP" && nt >= 5) {
    FOLLOW_STOP_DIST_CM = toF(1);
    FOLLOW_RESUME_DIST_CM = toF(2);
    ANGLE_TOL_RAD = toF(3) * M_PI / 180.0f;
    ARRIVE_DWELL_MS = (uint32_t)toF(4);
  } else if (tok[0] == "SETPWMNORM" && nt >= 2) {
    WHEEL_W_NORM = max(1.0f, toF(1));
  } else if (tok[0] == "PING") {
    if (client && client.connected()) client.println("PONG");
  }
}

void networkPump() {
  if (!client || !client.connected()) {
    client = server.available();
    return;
  }
  while (client.available()) {
    char c = (char)client.read();
    if (c == '\n' || c == '\r') {
      handleLine(rxBuf);
      rxBuf = "";
    } else if (rxBuf.length() < 200) {
      rxBuf += c;
    }
  }
}

/************ TELEMETRY ************/
void sendTelemetry() {
  static uint32_t lastT = 0;
  uint32_t now = millis();
  if (now - lastT < (uint32_t)(TEL_DT * 1000)) return;
  lastT = now;

  if (client && client.connected()) {
    float dxT = lastTargetX - poseX;
    float dyT = lastTargetY - poseY;
    float distT = sqrtf(dxT * dxT + dyT * dyT);
    bool leaderFresh = (millis() - lastLeaderUpdateMs) <= LEADER_TIMEOUT_MS;

    client.print("TEL ");
    client.print(poseX, 3);
    client.print(' ');
    client.print(poseY, 3);
    client.print(' ');
    client.print(poseTh, 3);
    client.print(' ');
    client.print(0.0f, 3);
    client.print(' ');
    client.print(0.0f, 3);
    client.print(' ');
    client.print(distT, 3);
    client.print(' ');
    client.print(holdStop ? 1 : 0);
    client.print(' ');
    client.println(leaderFresh ? 1 : 0);
  }
}

/************ SETUP ************/
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Follower (ANTI-JERK) booting...");

  pinMode(PIN_L_PWM, OUTPUT);
  pinMode(PIN_L_DIR, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT);
  pinMode(PIN_R_DIR, OUTPUT);
  setWheel(0, 0);

  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), encL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), encL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), encR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), encR_B, CHANGE);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.print("TCP server listening on port ");
  Serial.println(TCP_PORT);
}

/************ LOOP ************/
void loop() {
  networkPump();
  odomUpdate();

  static uint32_t lastCtrl = 0;
  uint32_t now = millis();

  if (now - lastCtrl >= (uint32_t)(CTRL_DT * 1000)) {
    lastCtrl = now;

    bool leaderFresh = (millis() - lastLeaderUpdateMs) <= LEADER_TIMEOUT_MS;
    float v = 0, w = 0;

    if (enabled && leaderFresh) {
      followControl(v, w);

      // Arrival / stop logic
      uint32_t nowMs = millis();
      bool withinStop = (lastDistToTarget <= FOLLOW_STOP_DIST_CM) && (fabsf(lastAngErr) <= ANGLE_TOL_RAD);

      if (withinStop) {
        if (withinSinceMs == 0) withinSinceMs = nowMs;
        if (nowMs - withinSinceMs >= ARRIVE_DWELL_MS) holdStop = true;
      } else {
        withinSinceMs = 0;
      }

      if (holdStop && (lastDistToTarget >= FOLLOW_RESUME_DIST_CM)) {
        holdStop = false;
      }

      if (holdStop) {
        v = 0;
        w = 0;
        // CRITICAL: Reset PID when holding stop
        intL = intR = 0;
        prevErrL = prevErrR = 0;
        last_speedL = last_speedR = 0;
        dELf = dERf = 0;
      } else {
        v = clamp(v, -V_MAX_CM_S, V_MAX_CM_S);
        w = clamp(w, -W_MAX_RAD_S, W_MAX_RAD_S);
      }
    } else {
      // disabled or no fresh leader
      v = 0;
      w = 0;
      intL = intR = 0;
      prevErrL = prevErrR = 0;
      last_speedL = last_speedR = 0;
      dELf = dERf = 0;
    }

    float wL_cmd = 0, wR_cmd = 0;
    velocityToWheel(v, w, wL_cmd, wR_cmd);
    wheelPID(wL_cmd, wR_cmd);
  }
    // Tambahkan di follower.txt (dalam loop):
  if ((millis() % 100) == 0) {  // Log setiap 100ms
    Serial.print("DATA,");
    Serial.print(millis()); Serial.print(",");
    Serial.print(poseX,2); Serial.print(",");
    Serial.print(poseY,2); Serialf.print(",");
    Serial.print(poseTh,3); Serial.print(",");
    Serial.print(leaderX,2); Serial.print(",");
    Serial.print(leaderY,2); Serial.print(",");
    Serial.print(lastDistToTarget,2); Serial.print(",");
    Serial.print(lastAngErr,3); Serial.print(",");
    Serial.print(holdStop); Serial.println();
  }

  sendTelemetry();
}
