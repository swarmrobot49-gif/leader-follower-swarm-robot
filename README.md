# Robot Arena Navigation System

![Python](https://img.shields.io/badge/python-3.8%2B-blue)
![.NET](https://img.shields.io/badge/.NET-4.7.2%2B-purple)
![Platform](https://img.shields.io/badge/platform-ESP8266-green)

A multi-robot navigation system implementing **Artificial Potential Field (APF)** algorithms for autonomous leader-follower navigation in a controlled arena. The system combines an overhead computer vision pipeline for real-time pose estimation with reactive potential field controllers running on a C# host application, sending direct velocity commands to ESP8266-based differential drive robots over WiFi.

## 🌟 Features

- **Reactive Potential Field Navigation**: Pure APF with attractive, repulsive, and vortex force components
- **Leader-Follower Formation**: Follower tracks a dynamic anchor point behind the leader using independent reactive PF
- **Computer Vision Tracking**: Overhead camera with ArUco marker detection and homography-based pose estimation (±2 cm accuracy)
- **Blue Goal Auto-Detection**: Vision system detects a blue-colored physical goal object and forwards coordinates to the controller automatically
- **Obstacle Clustering**: Adjacent obstacles merged before force computation to prevent force overlap
- **Scenario Data Logging**: Per-run CSV telemetry capturing robot poses, force vectors, velocities, and obstacle distances
- **Multi-Resolution Grid**: Configurable grid cell sizes (15 cm, 10 cm, 5 cm) shared between vision and GUI via file sync
- **GUI Control Interface**: Windows Forms application with live arena visualization, tuning sliders, and force vector overlay

## 📋 Table of Contents

- [System Architecture](#system-architecture)
- [Potential Field Algorithm](#potential-field-algorithm)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Path Planning Modes](#path-planning-modes)
- [Usage Guide](#usage-guide)
- [API Documentation](#api-documentation)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## 🏗️ System Architecture

The system uses a three-layer architecture: an overhead vision layer for localization, a host PC layer for potential field computation and coordination, and a firmware layer for low-level motor control.

```
┌──────────────────────────────────────────────────────────┐
│                    Vision Layer (Python)                  │
│  Camera → ArUco Detection → Homography → Pose Files      │
│  Yellow HSV → Obstacle Positions                         │
│  Blue HSV  → Goal Position (goal_blue.txt)               │
└───────────────────────┬──────────────────────────────────┘
                        │ File IPC (%LocalAppData%\RobotData\)
┌───────────────────────▼──────────────────────────────────┐
│               Controller Layer (C# .NET)                 │
│                                                          │
│  ┌─────────────────────────┐  ┌────────────────────────┐ │
│  │ ReactivePFController    │  │ FollowerReactivePF     │ │
│  │ (Leader)                │  │ Controller (Follower)  │ │
│  │ - Attractive force      │  │ - Anchor point follow  │ │
│  │ - Repulsive force       │  │ - Leader avoidance     │ │
│  │ - Vortex force          │  │ - Obstacle repulsion   │ │
│  │ - Obstacle clustering   │  │ - State machine        │ │
│  │                         │  │   (Follow/Stop/Catch)  │ │
│  │                         │  └───────────┬────────────┘ │
│  └────────────┬────────────┘              │              │
└───────────────┼───────────────────────────┼──────────────┘
                │ TCP SETVEL (20 Hz)        │ TCP SETVEL (20 Hz)
┌───────────────▼───────────┐  ┌────────────▼──────────────┐
│     Leader Robot          │  │      Follower Robot       │
│     ESP8266 : 5555        │  │      ESP8266 : 6060       │
│  - PID wheel speed ctrl   │  │  - PID wheel speed ctrl   │
│  - Odometry (50 Hz)       │  │  - Odometry (50 Hz)       │
│  - Encoder ISR (2800 t/r) │  │  - Encoder ISR (2800 t/r) │
└───────────────────────────┘  └───────────────────────────┘
```

### Data Flow

1. Vision system writes pose and obstacle files at ~20 Hz
2. GUI `PoseTimer` reads files every 100 ms and calls `UpdateLeaderPose` / `UpdateObstacles` on both controllers
3. Each controller runs an independent 20 Hz async loop computing forces and sending `SETVEL v w` over TCP
4. Robot firmware PID loops run at 50 Hz, tracking the commanded wheel angular velocities

## ⚡ Potential Field Algorithm

### Leader Navigation

The leader controller (`ReactivePotentialFieldController.cs`) computes a net force vector each control cycle:

**Attractive force** — linear, capped at `AttractiveMaxForce`:
```
F_att = k_att * d_goal  ·  (goal - robot) / |goal - robot|
```

**Repulsive force** — from each clustered obstacle within `ObstacleInfluenceRange`:
```
F_rep = k_rep * (1/d - 1/d0) / d²  ·  (robot - obs) / |robot - obs|
```

**Vortex (tangential) force** — resolves local minima by adding a tangential component aligned toward the goal when inside obstacle influence range:
```
F_vortex = k_vortex * |F_rep|_max * s  ·  tangent_toward_goal
```
where `s = 1 - d / d0` scales with proximity.

The net force heading is low-pass filtered, then converted to differential drive commands `(v, ω)`. Linear velocity is reduced near obstacles with a √-curve slowdown, and a turn-in-place threshold prevents forward motion during large heading corrections.

**Local minima escape sequence:**
1. If net force magnitude stays below 5 N for 10 consecutive cycles (0.5 s), a random-walk perturbation is activated — the robot turns toward a goal-biased random direction for 3 seconds, then resumes normal PF.
2. If the distance-to-goal stops decreasing for 30+ cycles, wall-following mode activates — the force vector is rotated 90° toward the goal side.

### Follower Navigation

The follower controller (`FollowerReactivePFController.cs`) sets a dynamic **anchor point** 20 cm behind the leader's heading:
```
anchor = leader_pos - follow_dist * (cos θ_leader, sin θ_leader)
```
The anchor is low-pass filtered to prevent sudden jumps. The follower's attractive force targets the anchor; repulsive forces push away from static obstacles and the leader itself (collision prevention). A four-state machine (Idle → Catching → Following → Stopped) with hysteresis prevents oscillation.

### Default Parameter Values

| Parameter | Leader | Follower |
|---|---|---|
| `k_att` | 5.0 | 4.0 |
| `k_rep` | 1300 | 600 |
| Obstacle influence range | 45 cm | 25 cm |
| Max linear velocity | 10 cm/s | 12 cm/s |
| Max angular velocity | 1.3 rad/s | 1.4 rad/s |
| Follow / stop distance | — | 20 / 22 cm |
| Control loop | 20 Hz | 20 Hz |

## 🔧 Hardware Requirements

### Robots (×2)
- ESP8266 microcontroller (NodeMCU or Wemos D1 Mini)
- 2× DC motors with quadrature encoders — 2800 ticks/revolution
- L298N dual motor driver
- Wheel diameter: 4 cm, wheel track: 10 cm
- Robot footprint: 15 cm × 15 cm

### Vision System
- USB camera — 640×480 minimum (Logitech C270 recommended)
- Overhead mount covering full 200×150 cm arena
- ArUco markers (Dictionary: `DICT_4X4_1000`, key `'3'`)
  - ID 1: Leader robot
  - ID 2: Follower robot
  - IDs 10–13: Arena corner calibration markers

### Arena
- Physical size: 200 cm × 150 cm
- Yellow-colored obstacle objects (~15 cm × 15 cm)
- Blue-colored goal object (detected by vision for auto goal-setting)
- Corner markers placed at known world coordinates for homography calibration

## 💻 Software Requirements

### Vision System
- Python 3.8+
- OpenCV 4.5+ with `cv2.aruco` module
- NumPy, PyYAML

### GUI Application
- .NET Framework 4.7.2+ or .NET 6+
- Windows 10/11
- Newtonsoft.Json

### Firmware
- Arduino IDE 1.8.19+ or PlatformIO
- ESP8266 Arduino core 3.0+
- No additional libraries required

## 📦 Installation

### 1. Clone Repository

```bash
git clone https://github.com/yourusername/robot-arena-navigation.git
cd robot-arena-navigation
```

### 2. Vision System Setup

```bash
cd vision-system
pip install opencv-contrib-python numpy pyyaml
cp hsv_settings.yaml.example hsv_settings.yaml
```

### 3. GUI Application Setup

Open `RobotArenaGUI.sln` in Visual Studio 2019+ and build in Release configuration.

### 4. Firmware Upload

Open `leader_reactive_pf.ino` and `follower_reactive_pf.ino` in Arduino IDE. Update WiFi credentials and upload to each ESP8266:

```cpp
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
```

Leader listens on port **5555**; follower listens on port **6060**.

## 🚀 Quick Start

### Step 1: Start Vision System

```bash
python vision_fixed_BLUE_GOAL.py
```

Wait until all four corner markers (IDs 10–13) are detected and `Homography: READY` appears on-screen.

Keyboard controls:
| Key | Action |
|---|---|
| `m` | Toggle yellow obstacle mask |
| `b` | Toggle blue goal mask |
| `g` | Cycle grid size (15 / 10 / 5 cm) |
| `s` | Save screenshot |
| `r` | Reset leader path trail |
| `c` | Clear blue goal file |
| `q` | Quit and save HSV settings |

### Step 2: Launch GUI and Connect

1. Run `RobotArenaGUI.exe`
2. Power on both robots; they will connect to WiFi and begin listening
3. Click **Update Position** to sync leader odometry from vision

### Step 3: Select Algorithm

Open the algorithm combo box and select **Pure Reactive PF** for potential field navigation. The Reactive PF control panel will appear.

### Step 4: Set Goal and Start

- **Manual**: Click any free arena cell to set the goal
- **Automatic**: Place the blue physical goal object in the arena — vision will auto-set the grid goal within ~0.5 s

Click **▶ START**. Both the leader and follower controllers activate simultaneously.

## ⚙️ Configuration

### Vision System (`vision_fixed_BLUE_GOAL.py`)

```python
ARENA_W_CM = 200.0        # Arena width
ARENA_H_CM = 150.0        # Arena height
GRID_CELL_CM = 15.0       # Default cell size (synced with GUI)

LEADER_ID = 1             # ArUco ID for leader
FOLLOWER_ID = 2           # ArUco ID for follower
ARUCO_SIZE_CM = 9.0       # Physical marker side length

BLUE_GOAL_MIN_STABLE = 5  # Frames before writing goal file
```

HSV ranges for yellow obstacles and blue goal are adjustable at runtime via trackbar windows and are persisted to `hsv_settings.yaml` on quit.

### Leader Reactive PF (`ReactivePotentialFieldController.cs`)

```csharp
public float AttractiveGain          { get; set; } = 5.0f;
public float RepulsiveGain           { get; set; } = 600.0f;
public float ObstacleInfluenceRange  { get; set; } = 35.0f;   // cm
public float VortexGain              { get; set; } = 0.7f;
public float MaxLinearVelocity       { get; set; } = 10.0f;   // cm/s
public float MaxAngularVelocity      { get; set; } = 1.3f;    // rad/s
public float GoalReachedThreshold    { get; set; } = 5.0f;    // cm
public float EmergencyStopDistance   { get; set; } = 10.0f;   // cm
public int   StuckCounterThreshold   { get; set; } = 30;
```

Attractive gain, repulsive gain, and obstacle range can also be adjusted live via the GUI tuning sliders.

### Follower Reactive PF (`FollowerReactivePFController.cs`)

```csharp
public float FollowDistance          { get; set; } = 20.0f;   // cm
public float StopDistance            { get; set; } = 22.0f;   // cm
public float ResumeDistance          { get; set; } = 25.0f;   // cm
public float AttractiveGain          { get; set; } = 4.0f;
public float RepulsiveGain           { get; set; } = 600.0f;
public float LeaderAvoidanceGain     { get; set; } = 800.0f;
public float ObstacleInfluenceRange  { get; set; } = 25.0f;   // cm
public float MaxLinearVelocity       { get; set; } = 12.0f;   // cm/s
```

### Arena & GUI (`MainForm.cs`)

```csharp
private const float ARENA_WIDTH_REAL_CM  = 200.0f;
private const float ARENA_HEIGHT_REAL_CM = 150.0f;
private const float ROBOT_WIDTH_CM       = 15.0f;
private const float ROBOT_HEIGHT_CM      = 15.0f;
private const float VISION_OBSTACLE_SIZE_CM = 15.0f;
```

### Robot Firmware

#### Leader (`leader_reactive_pf.ino`):
```cpp
float Kp_L = 2.5f, Ki_L = 0.05f, Kd_L = 0.06f;
float Kp_R = 2.6f, Ki_R = 0.055f, Kd_R = 0.065f;
const uint32_t CMD_TIMEOUT_MS = 200;   // Safety stop if no SETVEL received
const int PID_INTERVAL_MS = 20;        // 50 Hz PID loop
```

#### Follower (`follower_reactive_pf.ino`):
```cpp
float KpL = 1.5f, KiL = 0.0f, KdL = 0.0f;
float KpR = 1.5f, KiR = 0.0f, KdR = 0.0f;
const uint32_t CMD_TIMEOUT_MS = 200;
const int PID_INTERVAL_MS = 20;
```

## 📖 Usage Guide

### Reactive PF Workflow

1. **Calibrate HSV**: Tune yellow and blue trackbars until obstacles and goal are cleanly masked
2. **Verify homography**: Confirm corner markers 10–13 are all detected
3. **Click "Update Position"**: Sends `SETPOS` to leader firmware to sync odometry with vision
4. **Set goal**: Click arena cell or place blue object
5. **Click ▶ START**: Controller loops begin; monitor force vector overlay and status labels
6. **Click ■ STOP**: Sends `SETVEL 0 0` to both robots

### Runtime Tuning (GUI Sliders)

| Slider | Range | Effect |
|---|---|---|
| Attractive Gain | 0.5 – 5.0 | Higher = stronger pull toward goal |
| Repulsive Gain | 200 – 2000 | Higher = stronger obstacle avoidance |
| Obstacle Range | 15 – 80 cm | Radius within which obstacles exert force |

### Runtime Tuning (TCP — Firmware)

Connect via any terminal to the robot IP and port:

```
SETPID 2.5 0.05 0.06    # Update PID gains (Kp Ki Kd)
SETPOS x y theta        # Correct odometry (cm, cm, rad)
SETFOLLOW 20.0          # Follower: change follow distance (cm)
GETPOSE                 # Query current odometry
PING                    # Check connection (replies PONG)
TEL ON / TEL OFF        # Toggle telemetry stream
HELP                    # List all commands
```

### Scenario Logging

Each time **▶ START** is pressed, a timestamped folder is created at `%LocalAppData%\RobotData\scenario_YYYYMMDD_HHMMSS\` containing:

- `pf_log.csv` — per-cycle: time, leader pose, follower pose, goal, force vector (fx, fy), velocity commands, distance to goal, obstacle count, min obstacle distance, PF parameters
- `params.txt` — snapshot of all PF parameters at start
- `obstacles_start.txt` / `obstacles_stop.txt` — obstacle layout snapshots
- `leader_pose_start.txt` / `follower_pose_start.txt` — initial poses

The CSV is flushed every 10 rows and finalized when **■ STOP** is pressed.

## 🔌 API Documentation

### Vision → GUI File Protocol

Location: `%LocalAppData%\RobotData\`

| File | Format | Description |
|---|---|---|
| `leader_pose.txt` | `x_cm,y_cm,theta_rad` | Leader pose, updated ~20 Hz |
| `follower_pose.txt` | `x_cm,y_cm,theta_rad` | Follower pose, updated ~20 Hz |
| `obstacles.txt` | `x_cm,y_cm` (one per line) | Yellow obstacle centroids |
| `goal_blue.txt` | `x_cm,y_cm` | Blue goal centroid (if detected) |
| `grid_config.txt` | `15.0` | Cell size in cm, shared between vision and GUI |
| `pixels_per_grid.txt` | integer | Pixels per grid cell at current camera resolution |

All files are written atomically (write to `.tmp`, then `os.replace`) to prevent partial reads.

### GUI → Robot TCP Protocol

#### Leader (port 5555)

```
SETVEL v w              # Set linear (cm/s) and angular (rad/s) velocity — main command in reactive mode
START                   # Enable motor control
STOP                    # Disable motor control and zero targets
SETPOS x y theta        # Synchronize odometry (cm, cm, rad)
SETPID Kp Ki Kd         # Update wheel PID gains
GETPOSE                 # Reply: POSE,x,y,theta
PING                    # Reply: PONG
TEL ON / TEL OFF        # Toggle 20 Hz telemetry (TEL,time,x,y,theta,v,w,enabled)
```

#### Follower (port 6060)

```
SETVEL v w              # Direct velocity command (Reactive mode — primary)
SETLEADER x y theta     # Leader pose update (Legacy follow mode)
START / STOP            # Enable / disable control
SETPOS x y theta        # Synchronize odometry
SETPID Kp Ki Kd         # Update PID gains
SETFOLLOW dist          # Set follow distance (cm)
MODE REACTIVE           # Switch to SETVEL-driven mode
MODE FOLLOW             # Switch to onboard follow control
GETPOSE                 # Reply: POSE,x,y,theta
PING                    # Reply: PONG
```

## 🐛 Troubleshooting

### Vision Issues

**Homography not computing / markers not detected**
- Verify all four corner markers (IDs 10–13) are fully visible and unoccluded
- Try switching ArUco dictionary with keys `1`–`5`
- Ensure camera resolution is at least 640×480

**Pose jittering**
- Increase `alpha` in `smooth_angle()` from `0.30` toward `0.50`
- Improve lighting uniformity; avoid direct glare on markers

**Blue goal not detected**
- Toggle blue mask with `b` and tune `Trackbars Blue` HSV values
- Increase `BLUE_GOAL_MIN_STABLE` to require more stable frames before writing

### Reactive PF Issues

**Robot oscillates near obstacles**
- Reduce `RepulsiveGain` or increase `ObstacleInfluenceRange` to smooth the repulsive gradient
- Increase `VelocityLowPassAlpha` for heavier velocity filtering

**Robot stuck in local minimum**
- Increase `VortexGain` (try 0.5–1.0) before random-walk kicks in
- Reduce `StuckCounterThreshold` to trigger wall-following sooner
- Verify obstacles are being clustered correctly — check the GUI overlay

**Robot does not reach goal**
- Lower `GoalReachedThreshold` if robot is stopping short
- Check `EmergencyStopDistance` is not too large relative to the arena
- Enable goal-rush mode by ensuring the `isGoalRushMode` condition (< 10 cm, heading error < 45°) can be satisfied

**Follower oscillates between Following and Stopped**
- Increase hysteresis gap: widen the difference between `StopDistance` and `ResumeDistance`
- Reduce `LeaderAvoidanceGain` if follower is being pushed away too aggressively

### Robot Firmware Issues

**Robot not moving after START**
- Confirm TCP connection: send `PING`, expect `PONG`
- Check `controlEnabled` flag — send `START` command explicitly
- Verify `CMD_TIMEOUT_MS` — robot stops if no `SETVEL` received within 200 ms

**Inconsistent wheel speeds / drift**
- Tune `Kp_L` / `Kp_R` individually if motors are asymmetric
- Verify encoder sign constants (`ENC_SIGN_L`, `ENC_SIGN_R`) are correct for your wiring

**WiFi reconnection loops**
- Ensure SSID/password match exactly (case-sensitive)
- Check that robot is within range of the access point

## 📄 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- OpenCV ArUco module for fiducial marker detection
- ESP8266 Arduino core community
- Potential field navigation theory: Khatib (1986), Ge & Cui (2000)
