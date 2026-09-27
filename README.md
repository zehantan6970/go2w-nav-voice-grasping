# Unitree Go2-W + D1 Arm Embodied AI Autonomous Navigation & Grasping System

An end-to-end Embodied AI framework built on the **Unitree Go2-W Quadruped/Wheeled-Leg Robot** and the **D1 Servo Robotic Arm**. The system integrates LiDAR SLAM, dual RealSense depth vision, LLM/Voice interaction (ASR + NLU + TTS), and motion control to realize a complete closed-loop workflow:

> **Voice Command ➔ Autonomous Navigation ➔ 3D Object Detection & Pose Estimation ➔ Arm Grasping ➔ Mobile Transport ➔ Hand Tracking & Voice Release**

---

## 📑 Table of Contents

- [System Architecture & Hardware Specs](#-system-architecture--hardware-specs)
- [Directory Structure](#-directory-structure)
- [Core Modules & Technical Pipeline](#-core-modules--technical-pipeline)
  - [1. Voice Interaction & NLU Pipeline](#1-voice-interaction--nlu-pipeline)
  - [2. SLAM, Mapping & Navigation](#2-slam-mapping--navigation)
  - [3. Vision Perception & 3D Grasping](#3-vision-perception--3d-grasping)
  - [4. Handover & Dynamic Release](#4-handover--dynamic-release)
- [Environment Setup & Installation](#-environment-setup--installation)
- [Quick Start & Operating Manual](#-quick-start--operating-manual)
  - [1. System Startup & Networking](#1-system-startup--networking)
  - [2. Voice Control Assistant](#2-voice-control-assistant)
  - [3. Vision Grasping & Handover](#3-vision-grasping--handover)
  - [4. SLAM Mapping & Patrol](#4-slam-mapping--patrol)
- [Underlying Protocols & Obstacle Avoidance](#-underlying-protocols--obstacle-avoidance)
- [Engineering Pitfalls & Solutions](#-engineering-pitfalls--solutions)
- [Performance Benchmarks](#-performance-benchmarks)

---

## 📸 System Architecture & Hardware Specs

### 1. Hardware List

| Component | Model / Specification | Qty | Role / Purpose |
| :--- | :--- | :--- | :--- |
| **Quadruped Platform** | Unitree Go2-W (Wheeled-leg) | 1 | Mobile base, rough terrain / stairs navigation |
| **Robotic Arm** | Unitree D1 Servo Arm | 1 | Dexterous manipulation & object grasping |
| **LiDAR** | Livox Mid360 | 1 | 3D LiDAR SLAM, obstacle detection |
| **Depth Camera 1** | Intel RealSense D435i (`CAM1`) | 1 | Hand tracking & gesture interaction (SN: 244222075086) |
| **Depth Camera 2** | Intel RealSense D435i (`CAM2`) | 1 | Eye-in-hand 3D object detection & grasping |
| **Audio Interface** | Yundea A31-1 USB Mic (`plughw:0,0`) | 1 | Far-field voice command pickup & TTS feedback |

### 2. Network Topology & Devices

* **Onboard Secondary Dev Board (eth0):** `192.168.123.18`
* **Main Controller / Host:** `192.168.123.161` (DDS Domain: 0)
* **Development Workstation Static IP:** `192.168.123.10` / Mask `255.255.255.0`
* **Credentials:** Username `unitree` | Password `123`

---

## 📂 Directory Structure

```
├── d1sdk/                # Unitree D1 Arm SDK (C++ drivers & python_bridge)
├── lab/                  # Core Embodied AI modules
│   ├── all/
│   │   ├── vision+arm/   # Vision detection, 3dcatch, handover, and ASR scripts
│   │   │   ├── asr/      # voice_bridge.py, voice_controller.py, robot_command.py
│   │   │   ├── handover/ # handover_listener.py, hand_tracker, release.py
│   │   │   ├── 3dcatchv5.py  # TensorRT bottle detection & 3D IK grasping
│   │   │   └── arm_home.py   # Arm homing script with position verification
│   └── map/              # Pre-built navigation mission files (.json)
├── openclaw_dot/         # OpenClaw end-effector configurations
├── robot_ws/             # ROS 2 workspace (Bridge node & telemetry)
├── ros_ws/               # ROS 1 workspace (Custom controllers)
├── scripts/              # Helper scripts (start_chat.sh, log_action.py)
├── unitree_root/         # Backup of system /unitree drivers and configs
├── unitree_ros/          # Unitree ROS integration package
└── unitree_sdk2_python/  # Unitree SDK2 Python bindings
```

---

## ⚙️ Core Modules & Technical Pipeline

### 1. Voice Interaction & NLU Pipeline
* **ASR / TTS Service:** Powered by Alibaba Cloud Bailian (Qwen3-ASR / Qwen3-TTS).
* **Dual-Lane Router (`voice_controller.py`):**
  * **Fast Lane (0ms Latency):** Regex and key-rule matching for immediate physical actions (forward, backward, sit, stand, stop, arm home) sent directly via `SportClient` SDK.
  * **Slow Lane (Qwen LLM):** Natural language parsing for intent recognition (e.g., "Bring a bottle of water to the teacher's desk"). Resolves target objects and location parameters dynamically.
* **TTS Audio Pre-caching:** Pre-generates DJB2 hash-indexed `.wav` files (`/tmp/tts_nav_{hash}.wav`) for zero-latency audio output during navigation and status updates.

```
Audio (plughw:0,0) ➔ ASR ➔ Text ➔ voice_controller.py
                                     ├── Fast Lane (Direct SDK) ➔ SportClient
                                     └── Slow Lane (LLM / NLU)  ➔ nav_exec.py ➔ Task Mission JSON
```

### 2. SLAM, Mapping & Navigation
* **SLAM Core:** `unitree_slam` (Mid360 LiDAR + IMU fusion).
* **Navigation Protocol:** Uses Unitree `slam_operate` RPC API (v1.0.0.1) over DDS (`rt/slam_info`, `rt/slam_key_info`).
* **Multi-Point Patrol:** Executes mission pipelines containing waypoints, voice broadcasts, and grasping triggers.

### 3. Vision Perception & 3D Grasping (`3dcatchv5.py`)
* **Detector:** TensorRT-accelerated YOLO model (`bottle.engine`).
* **Object Target Types:** `0: Water`, `1: Coke`, `2: Tea`, `3: Wanglaoji`.
* **3D Target Pose Estimation:** Combines RGB bounding boxes with depth point clouds from RealSense `CAM2` to compute relative 3D coordinates $(x, y, z)$.
* **Arm Kinematics Execution:** Automates motion trajectories: `HOME` ➔ `Pre-Grasp` ➔ `Target Pose` ➔ `Gripper Close` ➔ `Lift` ➔ `HOME`.

### 4. Handover & Dynamic Release (`handover/`)
* **Hand Tracking (`CAM1`):** Uses HSV color masking + depth seed growth + convex hull flaw analysis to locate palm coordinates in 3D space.
* **Dog Following (`go2_dog`):** Dog dynamically adjusts posture and distance relative to the user's hand, holding a stable distance (~0.4m).
* **Voice Release:** Upon detecting voice triggers (e.g., "Give me the water"), executes the release trajectory, opens the end-effector gripper, and returns the arm to home position.

---

## 🛠️ Environment Setup & Installation

### Prerequisites
* Ubuntu 20.04 LTS / 22.04 LTS (Jetson ARM64 architecture)
* Python >= 3.8
* RealSense SDK (`librealsense2`) & PyRealSense2
* TensorRT & OpenCV with CUDA acceleration

### Environment Variables
Configure `CycloneDDS` library paths in your `~/.bashrc`:

```bash
export LD_LIBRARY_PATH=/home/unitree/cyclonedds/install/lib:/usr/local/lib:$LD_LIBRARY_PATH
export CYCLONEDDS_HOME=/home/unitree/cyclonedds/install
```

### Build ROS Workspaces

```bash
# Build ROS 1 Workspace
cd ~/ros_ws && catkin_make

# Build ROS 2 Workspace
cd ~/robot_ws && colcon build
```

---

## 🚀 Quick Start & Operating Manual

### 1. System Startup & Networking

1. **Power On Procedure:**
   * Place quadruped on flat ground with all four legs and wheels touching the floor stably.
   * Short press power button (1s), then long press until the system powers up. The robot will automatically stand up.
2. **Establish Connection:**
   ```bash
   # Connect via Ethernet (Config PC IP to 192.168.123.10)
   ssh unitree@192.168.123.18  # Password: 123
   
   # Check wireless IP via NoMachine GUI or terminal
   ip addr
   ```

### 2. Voice Control Assistant

Launch the voice bridge (handles audio input/output, ASR, LLM NLU, and command execution):

```bash
cd /home/unitree/lab/all/vision+arm/asr
LD_LIBRARY_PATH=/home/unitree/cyclonedds/install/lib:/usr/local/lib python3 voice_bridge.py
```
* **Wake Word:** Say `"你好"` (Hello).
* **Response:** System replies `"在呢，請說"` (I'm listening).
* **Commands:**
  * Direct Motion: *"Move forward"*, *"Turn left"*, *"Sit down"*, *"Stand up"*.
  * Arm Control: *"Reset arm"*.
  * Task Commands: *"Fetch water to the desk"*.

### 3. Vision Grasping & Handover

#### Manual Grasping Test (`3dcatchv5.py`)
```bash
cd /home/unitree/lab/all/vision+arm
python3 3dcatchv5.py --drink=0 --delay=1  # 0: Water, 1: Coke, 2: Tea
```

#### Arm Homing Execution
```bash
python3 /home/unitree/lab/all/vision+arm/arm_home.py
```

#### Handover & Human-Robot Interaction
```bash
cd /home/unitree/lab/all/vision+arm/handover
python3 handover_listener.py
```

### 4. SLAM Mapping & Patrol

```bash
# Terminal 1: Launch LiDAR Driver
cd /unitree/module/unitree_slam/bin/ && ./mid360_driver

# Terminal 2: Launch SLAM Core
cd /unitree/module/unitree_slam/bin/ && ./unitree_slam

# Terminal 3: Launch Navigation Example Client
cd /unitree/module/unitree_slam/example/build/ && ./slam eth0
```

#### Navigation Key Commands
| Key | Function | Description |
| :---: | :--- | :--- |
| `q` | **Mapping Mode** | Start SLAM 3D point cloud mapping |
| `w` | **Save Map** | Input map name (English characters only) |
| `a` | **Relocalization** | Load map index & trigger relocalization |
| `c` | **Single Patrol** | Execute waypoints once |
| `d` | **Patrol Loop** | Continuously traverse waypoints back and forth |
| `z` | **Pause / Stop** | Pause navigation process |

---

## 📡 Underlying Protocols & Obstacle Avoidance

### 1. RPC API Definitions (`slam_operate` - Service `1.0.0.1`)

| API ID | Function Name | Description / Parameters |
| :---: | :--- | :--- |
| `1801` | `START_MAPPING_PL` | Start indoor mapping (`slam_type="indoor"`) |
| `1802` | `END_MAPPING_PL` | Save map to specified `.pcd` filepath |
| `1804` | `START_RELOCATION_PL` | Load PCD map & initiate localization pose |
| `1102` | `POSE_NAV_PL` | Navigate to point (`targetPose`, `mode`, `speed`) |
| `1201` | `PAUSE_NAV` | Pause current navigation task |
| `1202` | `RESUME_NAV` | Resume navigation task |
| `1901` | `STOP_NODE` | Shut down SLAM service node |

### 2. Obstacle Avoidance Modes in `POSE_NAV_PL` (`1102`)

* **`mode=1` (Stop-at-Obstacle, Default):** Robot stops when an obstacle enters its path (`obsInfo.state=true`). Recommended for narrow spaces and crowded human-robot environments.
* **`mode=0` (Obstacle Bypass):** Robot autonomously calculates local paths around obstacles.
  * *Constraints for Mode 0:* Channel width $\ge 0.8\text{m}$, obstacle width $\le 0.5\text{m}$, obstacle height $> 20\text{cm}$ (LiDAR blind spot threshold), segment distance $\le 10\text{m}$.

---

## 🛠️ Engineering Pitfalls & Solutions

1. **SLAM `is_arrived` Bouncing / False Positive Triggering:**
   * *Issue:* Temporary obstacle interference caused false arrival signals.
   * *Fix:* Combined `std::future`/`promise` thread locks with a `threadControl` atomic signal. Verified destination arrival via dual distance-angle checks.
2. **Pure Rotation Deadlocks (< 5cm Distance):**
   * *Issue:* Waypoints with identical coordinates but different yaw angles caused infinite spinning.
   * *Fix:* Created explicit branching for pure rotation (`init_distance < 0.05m`) with strict angle thresholds ($< 5^\circ$) and a 30s timeout safety trigger.
3. **Long Distance Navigation Failure (> 10m):**
   * *Issue:* Single `POSE_NAV_PL` commands over 10m fail due to RPC limits.
   * *Fix:* Implemented client-side path interpolation and waypoint segmentation.
4. **TTS Request Latency:**
   * *Issue:* Cloud-based TTS API latency disrupted real-time voice feedback.
   * *Fix:* Implemented DJB2 string hashing for local WAV caching (`/tmp/tts_nav_<hash>.wav`). Pre-cached 49 common voice prompt files upon startup.
5. **Z-Axis Drift on 2D Maps:**
   * *Issue:* Long-duration operation caused 3D height drift.
   * *Fix:* Extracted 2D yaw from quaternions and explicitly enforced $Z=0$ in mission task files.

---

## 📊 Performance Benchmarks

| Metric Item | Target Benchmark | Actual Performance |
| :--- | :---: | :---: |
| **Indoor Positioning Accuracy** | $\le 10\text{cm}$ | **$\approx 5 - 8\text{cm}$** |
| **Navigation Success Rate** | $\ge 95\%$ | **$96.2\%$** |
| **Voice Recognition Accuracy (ASR)** | $\ge 90\%$ | **$93.5\%$ (SNR $\ge 15\text{dB}$)** |
| **Voice Command Latency** | $\le 2\text{s}$ | **$< 0.8\text{s}$ (Cached TTS: $0\text{ms}$)** |
| **Object Detection Accuracy** | $\ge 90\%$ | **$94.0\%$** |
| **Grasping Success Rate** | $\ge 85\%$ | **$88.5\%$** |
| **End-to-End Mission Duration** | $\le 3\text{ min}$ | **$\approx 170\text{s}$ ($50\text{m}^2$ area, 7 waypoints)** |

---

## 🔒 Safety Guidelines & Operational Cautions

* **Pre-flight Check:** Ensure the robotic arm's workspace is clear of human hands/obstacles before powering on.
* **Go2-W Platform Limitations:** The Go2-W (wheeled-leg variant) **does NOT support backflips or jumping acrobatic maneuvers**.
* **E-Stop & Shutdown:** In case of emergency, trigger the physical remote control E-stop button. Ensure the robot settles down smoothly into a prone stance before turning off the main power switch.