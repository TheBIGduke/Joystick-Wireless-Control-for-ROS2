# ESP32 Joystick Controller for ROS2

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-orange.svg)](https://platformio.org/)
[![ESP32](https://img.shields.io/badge/ESP32-Compatible-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%2B-blue.svg)](https://docs.ros.org/en/humble/)
[![Socket.IO](https://img.shields.io/badge/Socket.IO-4.0%2B-green.svg)](https://socket.io/)

Control your ROS2 robot wirelessly using an ESP32 and analog joystick. Features real-time Socket.IO communication, variable speed control, and RGB LED status indicators.

## Features

- **Real-time wireless control** with analog joystick
- **Variable speed control** (half/full speed based on joystick position)
- **Seamless ROS2 integration** via delivery_bridge
- **RGB LED status indicators** for system feedback
- **Automatic reconnection** with robust error handling
- **Low-latency Socket.IO** communication

## Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [LED Status Reference](#led-status-reference)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)
- [API Reference](#api-reference)
- [Contributing](#contributing)

## Hardware Requirements

### Components
- ESP32 Development Board (DevKitC, NodeMCU-32S, or compatible)
- Analog Joystick Module (2-axis with VRX/VRY outputs)
- Common Anode RGB LED
- Pushbutton (Dead Man Switch)
- 3× 220Ω resistors (for RGB LED)
- Jumper wires
- USB cable (micro-USB or USB-C)

### Wiring Diagram

<img src="images/Joystick_bb.png" alt="ESP32 Wiring Diagram" width="480" style="max-width:100%;height:auto;">

#### Joystick Connections
```
Joystick → ESP32
VCC      → 3.3V
GND      → GND
VRX      → GPIO39 (ADC3)
VRY      → GPIO36 (ADC0)
```

#### Pushbutton (Dead Man Switch)
```
Pushbutton → ESP32
One side   → GPIO4
Other side → GND
```

#### RGB LED (Common Anode)
```
RGB LED        → ESP32
Red Cathode    → GPIO25 (+ 220Ω resistor)
Green Cathode  → GPIO26 (+ 220Ω resistor)
Blue Cathode   → GPIO27 (+ 220Ω resistor)
Common Anode   → 3.3V
```

## Software Requirements

- **Visual Studio Code** with PlatformIO IDE extension
- **ROS2** (Humble or newer)
- **Python 3.8+**
- **Git**

## Installation

### 1. Install Development Tools

#### Visual Studio Code + PlatformIO
1. Download and install [Visual Studio Code](https://code.visualstudio.com/)
2. Install PlatformIO IDE extension:
   - Open VS Code
   - Go to Extensions (`Ctrl+Shift+X`)
   - Search for "PlatformIO IDE"
   - Click "Install" and restart VS Code

### 2. Clone Repository

```bash
cd ~/colcon_ws/src
git clone https://github.com/TheBIGduke/Joystick-Wireless-Control-for-ROS2.git
cd Joystick-Wireless-Control-for-ROS2
```

### 3. Setup ROS2 Delivery Bridge

**Prerequisites:** ROS2 Humble or newer installed ([installation guide](https://docs.ros.org/en/humble/Installation.html))

```bash
# Install Python dependencies
cd delivery_bridge
pip install -r requirements.txt

# Build the ROS2 package
cd ~/colcon_ws
colcon build --packages-select delivery_bridge --symlink-install
source install/setup.bash

# Test the delivery bridge
ros2 run delivery_bridge server_node
```

**Expected output:**
```
[INFO] [server_node]: Starting node BaseNode ...
[INFO] [server_node]: Database created successfully.
INFO: Uvicorn running on http://0.0.0.0:9009
INFO: connection open
```

Verify at `http://localhost:9009/docs`

### 4. Setup ESP32 Project

```bash
cd ~/colcon_ws/src/Joystick-Wireless-Control-for-ROS2/Joystick_SocketIO_ESP32
code .
```

PlatformIO will automatically detect and initialize the project.

## Configuration

### Step 1: Find Server IP Address

Find the IP address of the computer running the delivery bridge:

**Linux:**
```bash
hostname -I
# or
ip addr show
```

### Step 2: Configure ESP32 Settings

Edit `Joystick_SocketIO_ESP32/src/main.cpp`:

```cpp
// ============ REQUIRED CONFIGURATION ============

// WiFi credentials
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";

// Socket.IO server (delivery bridge)
const char* socketio_host = "192.168.0.103";  // Replace with your server IP
const int socketio_port = 9009;

// ============ OPTIONAL TUNING ============

// Joystick calibration (adjust based on your joystick)
#define CENTER_X 1945
#define CENTER_Y 2005
#define DEADZONE 4

// Speed settings
const float LINEAR_SPEED_FULL = 0.4;   // m/s
const float LINEAR_SPEED_HALF = 0.2;   // m/s
const float ANGULAR_SPEED_FULL = 0.6;  // rad/s
const float ANGULAR_SPEED_HALF = 0.3;  // rad/s
const int SPEED_THRESHOLD = 250;       // Threshold for half vs full speed
```

### Step 3: Calibrate Joystick (Optional)

To get accurate center values:

1. Upload the default code to ESP32
2. Open Serial Monitor (9600 baud)
3. Observe joystick readings at rest position
4. Note the X and Y values (e.g., `x = 1945, y = 2005`)
5. Update `CENTER_X` and `CENTER_Y` in the code
6. Re-upload to ESP32

## Usage

### LED indicators (startup sequence)
Follow the LED colors during startup — do not move the joystick while calibration is in progress.

- 🟠 Orange (blinking) — Calibrating joystick. Do NOT move the joystick until the LED turns solid orange.
- 🟠 Orange (solid)  — Calibration complete; startup continues.
- 🔵 Blue (blinking) — Connecting to WiFi.
- 🔵 Blue (solid)    — WiFi connected.
- 🟡 Yellow (blinking) — Connecting to Socket.IO (delivery_bridge).
- 🟡 Yellow (solid)    — Socket.IO connected.
- 🟢 Green (solid)   — System ready. You may control the robot.

To move the robot: press and hold the deadman switch while operating the joystick.
To stop the robot: release the joystick to center OR (in emergencies) release the deadman switch immediately.

### 1. Start the Delivery Bridge

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run delivery_bridge server_node
```

### 2. Upload Code to ESP32

**Using VS Code:**
1. Connect ESP32 via USB
2. Click PlatformIO icon in sidebar
3. Project Tasks → esp32doit-devkit-v1 → General → Upload

**Using CLI:**
```bash
cd Joystick_SocketIO_ESP32
pio run -t upload
```

### 3. Monitor Serial Output

```bash
pio device monitor -b 9600
```

**Expected output:**
```
Connecting to WiFi...
Connected to WiFi
IP address: 192.168.0.101
[IOc] Connected to url: /socket.io/?EIO=4
=== Joystick Wireless Control for ROS2 Started ===
Joystick - x = 0, y = 0
Movement: STOPPED | linear_x=0.00, angular_z=0.00
```

### 4. Verify ROS2 Communication

```bash
# Check cmd_vel topic
ros2 topic echo /cmd_vel
```

**Expected output when moving joystick:**
```
linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.3
```

## LED Status Reference

| Color | Pattern | Status |
|-------|---------|--------|
| 🔵 Blue | Fast blink | Connecting to WiFi |
| 🔵 Blue | Solid | WiFi connected |
| 🟡 Yellow | Fast blink | Connecting to Socket.IO |
| 🟡 Yellow | Solid | Socket.IO connected |
| 🟢 Green | Solid | System ready |
| 🟣 Magenta | Slow pulse | Dead man switch (no input) |
| 🟣 Magenta | Rapid pulse | Dead man switch (with input) |
| 🟠 Orange | Fast blink | Calibrating |
| 🔴 Red | Triple blink | Error |

## Troubleshooting

### WiFi Connection Issues

**Problem:** ESP32 won't connect to WiFi

**Solutions:**
- Verify WiFi credentials in `main.cpp`
- Ensure network is 2.4GHz (ESP32 doesn't support 5GHz)
- Check ESP32 is within router range
- Try a different WiFi network

### Socket.IO Connection Failed

**Problem:** WiFi connected but Socket.IO fails

**Solutions:**
- Verify delivery bridge is running: `ros2 run delivery_bridge server_node`
- Check server IP address in ESP32 code matches your computer
- Ensure both devices are on the same network
- Test server: `telnet YOUR_IP 9009`

### Joystick Not Responding

**Problem:** Robot doesn't move when joystick is moved

**Solutions:**
- **Check wiring:** VRX→GPIO39, VRY→GPIO36, VCC→3.3V, GND→GND
- **Verify power:** Joystick receives 3.3V
- **Calibrate:** Update `CENTER_X` and `CENTER_Y` values
- **Check serial:** Monitor joystick readings
- **Adjust deadzone:** If too sensitive, increase `DEADZONE` value

### Wrong Movement Direction

**Problem:** Robot moves in unexpected directions

**Solutions:**
- Swap VRX/VRY pin connections
- Or modify code to invert axes:
```cpp
// In readJoystick() function:
valueX = -analogRead(VRY_PIN);  // Add minus sign
```

### Robot Doesn't Move

**Problem:** ESP32 connected but robot unresponsive

**Solutions:**
- Check ROS2 topics: `ros2 topic list`
- Monitor cmd_vel: `ros2 topic echo /cmd_vel`
- Verify delivery bridge logs show received events
- Check robot is in correct operational mode

### Serial Monitor Shows Nothing

**Problem:** No output in serial monitor

**Solutions:**
- Verify baud rate is 9600
- Check USB cable connection
- Try different USB port
- Press ESP32 reset button
- Confirm correct COM port

## Project Structure

```
Joystick-Wireless-Control-for-ROS2/
├── Joystick_SocketIO_ESP32/
│   ├── src/
│   │   └── main.cpp              # ESP32 joystick controller
│   ├── platformio.ini            # PlatformIO configuration
│   └── .vscode/
├── delivery_bridge/
│   ├── delivery_bridge/
│   │   ├── base_node.py          # ROS2 base node
│   │   ├── server_node.py        # Socket.IO server
│   │   └── modules/              # ROS2 functionality
│   ├── package.xml
│   ├── setup.py
│   └── requirements.txt
└── README.md
```

## API Reference

### cmd_vel Message Format

ESP32 sends velocity commands to ROS2:

```json
[
  "cmd_vel",
  {
    "linear_x": 0.2,    // Forward/backward (m/s)
    "angular_z": 0.1    // Rotation (rad/s)
  }
]
```

**Speed Control:**
- **Half Speed:** `|joystick_value| ≤ SPEED_THRESHOLD`
- **Full Speed:** `|joystick_value| > SPEED_THRESHOLD`
- **Stopped:** Joystick in deadzone

### Delivery Bridge Endpoints

**HTTP REST API:**
```
POST /ros/functionality_mode/     # Set robot mode
GET  /waypoints/waypoint          # Get waypoints
POST /navigation/set_goal/{id}    # Navigate to waypoint
POST /navigation/set_wp/pause     # Pause navigation
POST /navigation/set_wp/resume    # Resume navigation
POST /navigation/set_wp/stop      # Stop navigation
```

**Socket.IO Events (Server → ESP32):**
```json
["battery", {"voltage": 12.5, "percentage": 85.2}]
["robot_pose", {"position_x": 1.2, "position_y": 3.4}]
["on_status_change", {"general": {"ready": true}}]
```

Full API documentation: `http://YOUR_SERVER_IP:9009/docs`

## Contributing

Contributions welcome! 

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## Acknowledgments

### Original Delivery Bridge
- **Author:** C. Mauricio Arteaga-Escamilla ([@cmauricioae8](https://github.com/cmauricioae8))
- **Repository:** [delivery_bridge](https://github.com/cmauricioae8/delivery_bridge)

### ESP32 Joystick Controller
- **Author:** Kaléin Tamaríz - TheBIGduke
- **Version:** 2.0.0

---

**Made with care for the ESP32 and ROS2 community**