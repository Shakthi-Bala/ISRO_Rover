# Manipulator Control using Inverse Kinematics (ROS 2) 🤖🦾

This project implements a **ROS 2–based manipulation framework** for a robotic arm using **Inverse Kinematics (IK)**.  
It integrates **custom ROS 2 messages and services**, a **manipulator control package**, and **Arduino-based motor actuation**, enabling end-to-end control from high-level IK planning to low-level motor execution.

---

## 📁 Repository Structure

```bash
.
├── isro_msgs/                     # Custom ROS 2 messages and services
│   ├── msg/                       # Message definitions
│   ├── srv/                       # Service definitions
│   ├── CMakeLists.txt
│   └── package.xml
│
├── manipulator_integration/       # Core manipulator integration package
│   ├── manipulator_integration/   # Python modules (ROS 2 nodes)
│   ├── resource/                  # Resource files
│   ├── test/                      # Tests
│   ├── setup.py
│   ├── setup.cfg
│   └── package.xml
│
├── just_motor.ino                 # Arduino motor control sketch
└── README.md
```
### 📌 Project Overview
The system consists of three main layers:
1.  High-level Planning
- Inverse kinematics computation for target end-effector poses
- Generation of joint-level commands

2. ROS Integration

- Custom ROS messages (isro_msgs)
- ROS services for command execution
- Python-based manipulation logic

3. Low-level Actuation

- Arduino-based motor control
- Direct motor command execution via serial / embedded interface

## 🧠 Key Features
- Inverse kinematics–based arm control
- Custom ROS message and service definitions
- Modular manipulation integration package
- Arduino motor control for hardware execution
- Suitable for simulation and hardware deployment

## 🧰 Software Requirements
### Operating System
- Ubuntu 22.04 / 24.04
### ROS 2
- ROS 2 Foxy / Humble / Iron
### Python
- Python 3.8+
### Arduino
- Arduino IDE

## 🛠️ Build Instructions (ROS 2)
Clone the repository into a ROS 2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <YOUR_REPOSITORY_URL>
```
Build using colcon:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
## 🚀 Running the System
### 1️⃣ Source the Workspace
```bash
source ~/ros2_ws/install/setup.bash
```
### 2️⃣ Run Manipulator Integration Node
```bash
ros2 run manipulator_integration <node_name>
```
Replace <node_name> with the ROS 2 Python node that implements IK and control logic.
List available executables:
```bash
ros2 pkg executables manipulator_integration
```
### 3️⃣ Arduino Motor Control
1. Open just_motor.ino in Arduino IDE
2. Select the correct board and serial port
3. Upload the sketch to the microcontroller
4. Ensure baud rate and serial settings match the ROS 2 node

## 📡 ROS 2 Communication
### Custom Messages & Services
Defined in isro_msgs:

- Joint command messages
- IK execution service calls
- Arm state feedback messages
Inspect interfaces:
```bash
ros2 interface list
ros2 interface show isro_msgs/msg/<MessageName>
ros2 interface show isro_msgs/srv/<ServiceName>
```
## 📐 Inverse Kinematics
---

### ✅ What changed for ROS 2 (important)
- `catkin` → **colcon**
- `rosrun / roscore` → **ros2 run**
- `rostopic / rosservice` → **ros2 topic / ros2 service**
- Python nodes follow **ROS 2 entry points**

If you want next, I can:
- Add **ROS 2 node graph (rqt_graph)**
- Add **IK math derivation section**
- Convert Arduino ↔ ROS 2 comms to **micro-ROS**
- Make all your repos follow a **single professional README template**

Just say 👍

- Computes joint angles from desired end-effector pose

## 📜 License
This project is intended for academic and educational use.
You are free to modify and extend the code for learning and research purposes.

## 👤 Author
Shakthi Bala

