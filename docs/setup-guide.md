# Complete Setup Guide

This guide will walk you through the complete setup process, from hardware assembly to running your first mission.

## Table of Contents
1. [Hardware Assembly](#hardware-assembly)
2. [Software Installation](#software-installation)
3. [Camera Calibration](#camera-calibration)
4. [Robot Calibration](#robot-calibration)
5. [Network Setup](#network-setup)
6. [First Mission](#first-mission)

---

## 1. Hardware Assembly

### Robot Assembly

#### Required Components (per robot)
- 1x ESP8266 NodeMCU or Wemos D1 Mini
- 2x DC motors with quadrature encoders (2800 ticks/revolution)
- 1x L298N motor driver module
- 1x Battery pack (7.4V LiPo or 6x AA batteries)
- 2x Wheels (4cm diameter)
- 1x Robot chassis (15cm x 15cm)
- 1x ArUco marker (printed on stiff paper/cardboard)
- Jumper wires and mounting hardware

#### Wiring Diagram

```
ESP8266 NodeMCU          L298N Motor Driver
─────────────────        ──────────────────
D1 (GPIO5)   ────────→   ENA (PWM Left)
D3 (GPIO0)   ────────→   IN1 (DIR Left)
D2 (GPIO4)   ────────→   ENB (PWM Right)
D4 (GPIO2)   ────────→   IN2 (DIR Right)

GND          ────────→   GND
                         +12V ← Battery

D5 (GPIO14)  ────────→   Encoder L A
D6 (GPIO12)  ────────→   Encoder L B
D7 (GPIO13)  ────────→   Encoder R A
SD3 (GPIO10) ────────→   Encoder R B

3.3V         ────────→   Encoder VCC (if needed)
GND          ────────→   Encoder GND
```

**Important Notes:**
- Use separate power for motors (via L298N) and ESP8266
- Connect common ground between ESP8266 and motor driver
- Encoders require pullup resistors (internal pullups enabled in code)
- Verify encoder polarity: motor forward should increase count

#### ArUco Marker Placement

1. **Print Markers:**
   - Use ArUco Dictionary 4X4_1000 (default)
   - Leader: Marker ID 1
   - Follower: Marker ID 2
   - Size: 5cm x 5cm minimum (larger is better)

2. **Generate Markers** (if needed):
   ```python
   import cv2
   import cv2.aruco as aruco
   
   # Generate marker
   dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
   marker_id = 1
   marker_size = 200  # pixels
   marker_img = aruco.generateImageMarker(dict_aruco, marker_id, marker_size)
   cv2.imwrite(f'marker_{marker_id}.png', marker_img)
   ```

3. **Mount Markers:**
   - Place on TOP of robot, centered
   - Ensure marker is FLAT and PERPENDICULAR to camera view
   - Add white border around marker (10-20% of marker size)
   - Orientation: Top edge aligned with robot's forward direction

### Arena Setup

#### Dimensions
- Physical size: 200cm (width) × 150cm (height)
- Grid: 13 columns × 10 rows @ 15cm per cell
- Boundary: White/light-colored tape or walls

#### Floor Requirements
- Light-colored surface (white or light gray)
- Matte finish (avoid reflections)
- Uniform lighting (avoid shadows)

#### Camera Mounting
- Position: Overhead, centered above arena
- Height: 150-200cm above arena
- Angle: Perpendicular to floor (90°)
- FOV: Entire arena visible with margins

---

## 2. Software Installation

### Prerequisites

#### Windows PC
- Windows 10/11 (64-bit)
- .NET Framework 4.7.2 or .NET 6+
- Visual Studio 2019+ (Community Edition is free)
- Python 3.8 or later
- Arduino IDE 1.8.19+

### Step-by-Step Installation

#### A. Python Environment

1. **Install Python:**
   - Download from python.org
   - Check "Add Python to PATH" during installation

2. **Create Virtual Environment (recommended):**
   ```bash
   cd vision-system
   python -m venv venv
   venv\Scripts\activate  # Windows
   # source venv/bin/activate  # Linux/Mac
   ```

3. **Install Dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Verify OpenCV ArUco:**
   ```python
   import cv2
   import cv2.aruco as aruco
   print(cv2.__version__)  # Should be 4.5+
   print(hasattr(cv2, 'aruco'))  # Should be True
   ```

#### B. GUI Application

1. **Install Visual Studio:**
   - Download Visual Studio Community (free)
   - Select workload: ".NET desktop development"

2. **Open Solution:**
   ```bash
   cd gui-application
   # Double-click RobotArenaGUI.sln
   ```

3. **Restore NuGet Packages:**
   - Right-click solution → "Restore NuGet Packages"
   - Or: Tools → NuGet Package Manager → Restore

4. **Install Newtonsoft.Json** (if not auto-installed):
   ```
   Tools → NuGet Package Manager → Package Manager Console
   Install-Package Newtonsoft.Json
   ```

5. **Build Solution:**
   - Build → Build Solution (Ctrl+Shift+B)
   - Check Output window for errors

#### C. Arduino IDE Setup

1. **Install Arduino IDE:**
   - Download from arduino.cc
   - Version 1.8.19 or later

2. **Add ESP8266 Board Support:**
   - File → Preferences
   - Additional Board Manager URLs:
     ```
     http://arduino.esp8266.com/stable/package_esp8266com_index.json
     ```
   - Tools → Board → Boards Manager
   - Search "esp8266"
   - Install "ESP8266 by ESP8266 Community" version 3.0+

3. **Configure Board Settings:**
   ```
   Board: "NodeMCU 1.0 (ESP-12E Module)"
   Upload Speed: "115200"
   CPU Frequency: "80 MHz"
   Flash Size: "4MB (FS:2MB OTA:~1019KB)"
   Port: (Select your COM port)
   ```

---

## 3. Camera Calibration

### Camera Selection

**Recommended:**
- USB webcam with 640x480 or 1280x720 resolution
- Fixed focus or manual focus
- Good low-light performance

**Not Recommended:**
- Laptop built-in cameras (often poor quality)
- Cameras with auto-focus (causes tracking jitter)

### Physical Calibration

1. **Mount Camera:**
   - Secure camera overhead at 150-200cm height
   - Ensure stable mounting (no vibration)
   - Point directly down at arena center

2. **Check Field of View:**
   - Run: `python finalVision.py`
   - Verify entire arena visible
   - Adjust camera height/angle if needed

3. **Lighting Setup:**
   - Use diffuse overhead lighting
   - Avoid direct sunlight or shadows
   - Test at different times of day if using natural light

### HSV Color Calibration

This calibrates obstacle detection for your specific lighting.

1. **Place Test Obstacle:**
   - Use a yellow object (sticky note, tape, etc.)
   - Place in arena under typical lighting

2. **Start Vision System:**
   ```bash
   python finalVision.py
   ```

3. **Open Mask View:**
   - Press `m` key
   - Window "Mask Yellow" appears

4. **Adjust Trackbars:**
   - Goal: Yellow obstacles WHITE, everything else BLACK
   - Start with: H=20-30, S=120-255, V=120-255
   - Increase H_min/decrease H_max to narrow hue range
   - Increase S_min to filter out pale yellows
   - Increase V_min to filter out dark areas

5. **Test with Multiple Obstacles:**
   - Add more yellow objects
   - Verify all detected correctly
   - Adjust if false positives occur

6. **Save Settings:**
   - Settings auto-save to `hsv_settings.yaml`
   - Or press `q` to quit (saves on exit)

### Grid Calibration

1. **Verify Grid Alignment:**
   - Grid should align with physical arena boundaries
   - Check that `GRID_COLS * GRID_CELL_CM = ARENA_WIDTH_REAL_CM`
   - Default: 13 × 15cm = 195cm (fits in 200cm arena)

2. **Measure Pixel-per-Grid:**
   - Vision system calculates this automatically
   - Saved to `%LocalAppData%\RobotData\pixels_per_grid.txt`
   - Typical value: 40-50 pixels per grid cell

3. **Verify with Physical Measurement:**
   - Place marker at known grid position (e.g., 0,0)
   - Check pose file: should match expected cm coordinates
   - If off: adjust `GRID_CELL_CM` in finalVision.py

---

## 4. Robot Calibration

### Encoder Calibration

1. **Test Encoder Direction:**
   ```cpp
   // Upload this test sketch
   void loop() {
     Serial.print("EncL: "); Serial.print(ticksA_pid);
     Serial.print("  EncR: "); Serial.println(ticksB_pid);
     delay(100);
   }
   ```

2. **Push Robot Forward Manually:**
   - Both encoder counts should INCREASE
   - If one decreases: swap that encoder's A/B pins
   - Or: negate sign in firmware (`ENC_SIGN_L/R`)

3. **Measure Ticks per Revolution:**
   - Mark wheel position
   - Push robot forward exactly 1 wheel revolution
   - Count encoder ticks
   - Update `TICKS_PER_REVOLUTION` if different from 2800

### Motor Direction Calibration

1. **Test Motor Polarity:**
   ```cpp
   void setup() {
     // ... pin setup ...
     setWheel(0.3, 0.3);  // 30% power both wheels forward
   }
   ```

2. **Observe Robot Movement:**
   - Should move straight forward
   - If turns: swap one motor's polarity
   - If goes backward: swap BOTH motor polarities

3. **Fine-Tune with DIR Pins:**
   - If swapping wires impractical, invert in code:
   ```cpp
   digitalWrite(localDirPin, commandForward ? LOW : HIGH);  // Flip LOW/HIGH
   ```

### Wheel Measurement

1. **Measure Wheel Diameter:**
   - Use calipers for accuracy
   - Update `WHEEL_DIAMETER_CM` in firmware
   - Affects odometry accuracy

2. **Measure Wheel Track:**
   - Distance between wheel contact points
   - Update `WHEEL_BASE_CM` in firmware
   - Affects turning accuracy

3. **Verify Odometry:**
   ```cpp
   // Upload firmware, run this test:
   // 1. Note starting position
   // 2. Command robot to move 50cm forward
   // 3. Measure actual distance
   // 4. If off: adjust WHEEL_DIAMETER_CM proportionally
   ```

### PID Tuning (Optional)

The default PID values work for most setups, but fine-tuning improves performance.

**Tuning Process:**

1. **Start with P-only:**
   ```cpp
   Kp_kiri = 2.0; Ki_kiri = 0; Kd_kiri = 0;
   Kp_kanan = 2.0; Ki_kanan = 0; Kd_kanan = 0;
   ```

2. **Test Response:**
   - Command constant speed
   - If oscillates: decrease Kp
   - If sluggish: increase Kp

3. **Add Integral (I):**
   - Start with Ki = Kp / 100
   - Eliminates steady-state error
   - If overshoots: decrease Ki

4. **Add Derivative (D):**
   - Start with Kd = Kp / 10
   - Reduces overshoot
   - If noisy: decrease Kd or increase low-pass filter

**Runtime Tuning via TCP:**
```
SETPID 2.0 0.06 0.07        # Same PID for both
SETPID6 2.0 0.055 0.066 2.0 0.06 0.07  # Independent L/R
```

---

## 5. Network Setup

### WiFi Configuration

1. **Setup WiFi Router:**
   - Use 2.4 GHz network (ESP8266 doesn't support 5 GHz)
   - Disable AP isolation / client isolation
   - Note SSID and password

2. **Configure Leader Firmware:**
   ```cpp
   // In leader.ino
   const char* ssid = "YourActualWiFiName";
   const char* password = "YourActualPassword";
   const int tcp_port = 5555;
   ```

3. **Configure Follower Firmware:**
   ```cpp
   // In follower.ino
   const char* WIFI_SSID = "YourActualWiFiName";
   const char* WIFI_PASS = "YourActualPassword";
   const uint16_t TCP_PORT = 6060;  // Different from leader!
   ```

### Finding Robot IP Addresses

**Method 1: Serial Monitor**
1. Upload firmware and open Serial Monitor (115200 baud)
2. Wait for "WiFi connected!"
3. Note the IP address printed (e.g., "IP: 10.159.150.9")

**Method 2: Router Admin Page**
1. Login to router admin interface
2. Check connected devices / DHCP leases
3. Find devices named "ESP_XXXXXX"

**Method 3: Network Scanner**
```bash
# Windows
arp -a

# Linux/Mac
nmap -sn 192.168.1.0/24
```

### Configure GUI Application

Edit `MainForm.cs`:
```csharp
private const string ROBOT_IP_ADDRESS = "10.159.150.9";  // Leader IP
private const string FOLLOWER_IP = "10.159.150.23";      // Follower IP
```

Rebuild application after changing IPs.

### Firewall Configuration

**Windows Firewall:**
1. Control Panel → Windows Defender Firewall
2. Advanced Settings → Inbound Rules
3. New Rule → Port → TCP → Ports 5555, 6060, 6666
4. Allow the connection → Apply to Domain, Private, Public

**Test Connectivity:**
```bash
# From PC, test TCP connection
telnet 10.159.150.9 5555
# Should connect without timeout
```

---

## 6. First Mission

### System Startup Sequence

**Step 1: Start Vision System**
```bash
cd vision-system
python finalVision.py
```
- Verify markers detected (blue circles around robots)
- Check grid alignment
- Press `m` to verify obstacle mask

**Step 2: Launch GUI Application**
```bash
cd gui-application
RobotArenaGUI.exe
```
Or run from Visual Studio (F5)

**Step 3: Power On Robots**
1. Power on Leader robot
   - Wait for "WiFi connected!" in serial monitor
   - Note IP address

2. Power on Follower robot
   - Wait for "WiFi connected!" in serial monitor
   - Note IP address

### Running First Mission

**1. Sync Initial Position**
- Place Leader robot in arena (within camera view)
- In GUI: Click "Update Position" button
- Verify green "Start" cell appears at robot location

**2. Place Test Obstacle**
- Put yellow sticky note in arena
- Verify it appears as gray cell in GUI

**3. Set Goal**
- Click any empty (white) cell in GUI
- Blue path should appear connecting start to goal

**4. Execute Mission**
- Click "Send Waypoints" button
- Leader robot should start moving along path
- Monitor serial output for debugging

**5. Test Follower (Optional)**
- Follower will automatically follow Leader
- Maintains 20cm distance behind
- Stops when too close, resumes when distance increases

### Troubleshooting First Mission

**Robot doesn't move:**
- Check serial monitor for error messages
- Verify TCP connection established
- Test motors manually: `setWheel(0.5, 0.5)` in setup()

**Path not found:**
- Check if goal is reachable (not surrounded by obstacles)
- Try larger grid size (15cm instead of 5cm)
- Reduce robot footprint if too conservative

**Follower doesn't follow:**
- Verify SETLEADER messages sent (check GUI status)
- Check follower serial monitor for "SETLEADER" reception
- Ensure follower received START command

**Vision tracking lost:**
- Improve lighting (reduce shadows)
- Clean markers (ensure contrast)
- Check camera focus (should be sharp)

---

## Next Steps

After successful first mission:

1. **Experiment with Parameters:**
   - Try different grid sizes (5cm, 10cm, 15cm)
   - Adjust robot speeds in firmware
   - Tune PID gains for smoother motion

2. **Add More Obstacles:**
   - Test pathfinding with complex environments
   - Verify obstacle inflation works correctly

3. **Multi-Waypoint Missions:**
   - Send multiple waypoints in sequence
   - Create patrol routes

4. **Advanced Features:**
   - Try runtime PID tuning via TCP
   - Implement custom mission sequences
   - Add more robots to the system

---

## Additional Resources

- [Hardware Requirements](hardware-requirements.md)
- [API Documentation](../README.md#api-documentation)
- [Troubleshooting Guide](../README.md#troubleshooting)
- [Contributing Guidelines](../README.md#contributing)

---

**Need help?** Open an issue on GitHub with:
- Detailed description of problem
- Serial monitor output
- Vision system screenshot
- Hardware photos (if relevant)
