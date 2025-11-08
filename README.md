# Robot Arena Navigation System

![Python](https://img.shields.io/badge/python-3.8%2B-blue)
![.NET](https://img.shields.io/badge/.NET-4.7.2%2B-purple)
![Platform](https://img.shields.io/badge/platform-ESP8266-green)

A comprehensive multi-robot navigation system with computer vision tracking, path planning, and leader-follower coordination for autonomous mobile robots in a grid-based arena.

## 🌟 Features

- **Computer Vision Tracking**: Real-time ArUco marker detection and pose estimation
- **Grid-Based Path Planning**: Dijkstra pathfinding with obstacle avoidance and robot footprint inflation
- **Leader-Follower Coordination**: Dynamic following behavior with adjustable parameters
- **GUI Control Interface**: Windows Forms application for mission planning and monitoring
- **Multi-Resolution Grid**: Support for 15cm, 10cm, and 5cm grid cell sizes
- **Real-Time Telemetry**: Live position updates and obstacle detection
- **WiFi Communication**: TCP/IP based command and control system

## 📋 Table of Contents

- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Usage Guide](#usage-guide)
- [API Documentation](#api-documentation)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## 🏗️ System Architecture

The system consists of four main components:

```
┌─────────────────┐      ┌──────────────────┐
│  Vision System  │──────│  GUI Application │
│   (Python)      │      │   (C# .NET)      │
└────────┬────────┘      └────────┬─────────┘
         │                        │
         │        WiFi            │
         └────────┬───────────────┘
                  │
         ┌────────┴────────┐
         │                 │
    ┌────▼─────┐     ┌────▼──────┐
    │  Leader  │     │ Follower  │
    │  Robot   │────▶│  Robot    │
    │(ESP8266) │     │ (ESP8266) │
    └──────────┘     └───────────┘
```

### Component Overview

1. **Vision System** - ArUco marker detection, obstacle detection, pose estimation
2. **GUI Application** - Mission planning, path visualization, robot control
3. **Leader Robot** - Waypoint navigation, odometry, PID control
4. **Follower Robot** - Dynamic following, adjustable parameters

## 🔧 Hardware Requirements

### Robots (2x)
- ESP8266 microcontroller
- 2x DC motors with quadrature encoders (2800 ticks/rev)
- L298N motor driver
- 4cm diameter wheels
- 10cm wheel track width
- Robot dimensions: 15cm x 15cm

### Vision System
- USB camera (640x480 or higher)
- Computer running Windows/Linux
- ArUco markers (Dictionary: 4X4_1000)
  - Marker ID 1: Leader robot
  - Marker ID 2: Follower robot

### Arena
- Physical arena: 200cm x 150cm
- Grid: 13x10 @ 15cm cells (default)
- Green-colored floor
- Consistent lighting

## 💻 Software Requirements

### Vision System
- Python 3.8+
- OpenCV 4.5+ with ArUco support
- NumPy, PyYAML

### GUI Application
- .NET Framework 4.7.2+ or .NET 6+
- Windows 10/11
- Newtonsoft.Json

### Firmware
- Arduino IDE 1.8.19+ or PlatformIO
- ESP8266 Board Package 3.0+

## 📦 Installation

### 1. Clone Repository

```bash
git clone https://github.com/yourusername/robot-arena-navigation.git
cd robot-arena-navigation
```

### 2. Vision System Setup

```bash
cd vision-system
pip install -r requirements.txt
cp hsv_settings.yaml.example hsv_settings.yaml
```

### 3. GUI Application Setup

Open `gui-application/RobotArenaGUI.sln` in Visual Studio and build.

### 4. Firmware Upload

Update WiFi credentials in firmware files and upload to ESP8266.

## 🚀 Quick Start

### Step 1: Start Vision System

```bash
cd vision-system
python finalVision.py
```

Controls: `m` - Toggle mask | `s` - Save screenshot | `r` - Reset | `q` - Quit

### Step 2: Launch GUI Application

Run `RobotArenaGUI.exe` or press F5 in Visual Studio

### Step 3: Power On Robots

Power on both robots and wait for WiFi connection.

### Step 4: Run First Mission

1. Click "Update Position" to sync with camera
2. Toggle "Obstacle Mode" to paint obstacles
3. Click target cell to set goal
4. Click "Send Waypoints" to execute

## ⚙️ Configuration

### Vision System

Edit `finalVision.py`:

```python
GRID_COLS = 13           # Columns
GRID_ROWS = 10           # Rows
GRID_CELL_CM = 15        # Cell size in cm
DETECTION_RADIUS_CM = 40 # Follow radius
```

### GUI Configuration

Edit `MainForm.cs`:

```csharp
private const float ARENA_WIDTH_REAL_CM = 200.0f;
private const float ARENA_HEIGHT_REAL_CM = 150.0f;
private const float ROBOT_WIDTH_CM = 15.0f;
private const float ROBOT_HEIGHT_CM = 15.0f;
```

### Robot Firmware

#### Leader (`leader.ino`):
```cpp
const float NAV_TARGET_SPEED_CM_S = 10.0;
const float NAV_TURN_SPEED_CM_S = 7.0;
const double ANGLE_THRESHOLD_RAD = 0.08;
double Kp_kiri = 2.0, Ki_kiri = 0.055, Kd_kiri = 0.066;
```

#### Follower (`follower.ino`):
```cpp
float FOLLOW_DIST_CM = 20.0f;
float KV = 1.20f;  // Linear gain
float KW = 3.00f;  // Angular gain
```

## 📖 Usage Guide

### Basic Operations

1. **Camera Calibration**: Adjust HSV trackbars for yellow obstacles
2. **Grid Selection**: Click "Grid 15x15" to cycle sizes
3. **Obstacle Placement**: Manual mode or automatic vision detection
4. **Path Planning**: Click cell to set goal, path auto-calculates
5. **Mission Execution**: Click "Send Waypoints" to start

### Advanced Features

#### Runtime Tuning Commands (via TCP)

Follower commands:
```
SETFOLLOWDIST 25.0        # Follow distance
SETVEL 12.0 2.5           # Max velocities
SETGAIN 1.5 3.5           # Control gains
SETPID 2.0 0.06 0.07      # PID gains
```

## 🔌 API Documentation

### Vision System File Protocol

Location: `%LocalAppData%\RobotData\`

**leader_pose.txt / follower_pose.txt**:
```
x_cm,y_cm,theta_rad
```

**obstacles.txt**:
```
x_cm,y_cm
x_cm,y_cm
```

### GUI → Robot TCP Protocol

#### Leader Commands
```
CLEAR                          # Reset mission
SETPOS x_cm,y_cm,theta_rad     # Set odometry (radians)
SETPOS_DEG x_cm,y_cm,theta_deg # Set odometry (degrees)
x_grid,y_grid                  # Add waypoint
EXECUTE                        # Start navigation
```

#### Follower Commands
```
START                           # Enable motion
STOP                            # Disable motion
SETPOS x_cm y_cm theta_rad      # Set odometry
SETLEADER x_cm y_cm theta_rad   # Update leader pose
PING                            # Check connection
```

## 🐛 Troubleshooting

### Vision Issues

**Markers not detected:**
- Check lighting (avoid shadows/glare)
- Verify marker dictionary (4X4_1000, key '3')
- Ensure markers are flat and visible

**Pose jittering:**
- Increase smoothing: `alpha=0.3` → `alpha=0.5`
- Improve lighting uniformity

### GUI Issues

**Position not updating:**
- Verify vision system running
- Check file path: `%LocalAppData%\RobotData\`
- Ensure `poseTimer` started

**Path not found:**
- Reduce robot size in config
- Check start/goal validity
- Try larger grid size

### Robot Issues

**Robot not moving:**
- Check motor connections
- Verify PWM pins
- Increase minimum PWM

**Overshoots waypoints:**
- Reduce `NAV_TARGET_SPEED_CM_S`
- Increase `DISTANCE_THRESHOLD_CM`
- Tune PID gains

## 🤝 Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open Pull Request

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed guidelines.

## 📄 License

This project is licensed under the MIT License - see [LICENSE](LICENSE) file.

## 🙏 Acknowledgments

- OpenCV ArUco module for marker detection
- ESP8266 community for WiFi support
- .NET community for GUI framework

## 📞 Contact

- Issues: [GitHub Issues](https://github.com/yourusername/robot-arena-navigation/issues)
- Email: your.email@example.com

## 🗓️ Roadmap

- [ ] Add A* pathfinding
- [ ] Multi-robot coordination (3+ robots)
- [ ] Web-based GUI
- [ ] ROS integration
- [ ] ML-based obstacle prediction

---

**Star ⭐ this repository if you find it helpful!**
