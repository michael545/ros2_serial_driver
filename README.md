# ROS2 Serial Driver with Virtual Port Simulation

A ROS2 C++ driver that demonstrates hardware-in-the-loop development using **rclcpp** (ROS2 C++ API). This simple **publisher node** reads serial data and publishes it to ROS2 topics, utilizing the **wjwwood/serial library** for cross-platform serial communication.

## Key Features

- **ROS2 Integration**: Built with **rclcpp** for native ROS2 C++ API support
- **Simple Publisher Pattern**: Reads serial data and Opublishes to `/arduino_data` topic
- **Cross-Platform Serial Communication**: Uses the wjwwood/serial - *"a cross-platform library for interfacing with rs-232 serial like ports written in C++. It provides a modern C++ interface with a workflow designed to look and feel like PySerial, but with the speed and control provided by C++."*
- **Virtual Hardware Simulation**: Complete testing environment using socat virtual serial ports
- **Parameter-Driven Configuration**: Runtime configurable serial port and baud rate
- **Professional ROS2 Patterns**: Demonstrates timer-based reading, error handling, and node lifecycle

## Architecture

```
Arduino/Simulator → Serial Port → ROS2 Driver (rclcpp) → /arduino_data Topic
     (Data Source)    (Hardware)    (C++ Publisher)      (ROS2 Subscribers)
```

## Quick Start (TL;DR)

**Prerequisites**: Ubuntu 20.04 + ROS2 Foxy + socat + python3-pip

```bash
# 1. Clone and build
git clone @this_url@ serial_driver_cpp
cd serial_driver_cpp
git clone https://github.com/wjwwood/serial.git src/serial
sudo apt install socat python3-pip -y && pip3 install pyserial
colcon build && source install/setup.bash

# 2. run in 4 different terminals:
`   
# terminal #1: virtual ports    
socat -d -d pty,raw,echo=0,link=/tmp/ttyS10 pty,raw,echo=0,link=/tmp/ttyS11

# terminal #2: arduino simulator  
python3 arduino_simulator.py

# terminal #3: ROS2 driver
ros2 run serial_driver_cpp arduino_driver_node --ros-args -p port:=/tmp/ttyS11

# terminal #4: the data
ros2 topic echo /arduino_data
```

**Expected Result**: Should see `data: 'msg #0'`, `data: 'msg #1'`, etc. flowing through the ROS2 topic.

---

## Contents
- [Prerequisites](#prerequisites)
- [Install](#installation)
- [Building](#building-the-project)
- [Running the Virtual Serial Port](#running-the-virtual-serial-port-simulation)
- [Testing](#testing-the-complete-system)
- [Project Structure](#project-structure)
- [Problems](#troubleshooting)

## Prerequisites

- **OS**: Ubuntu 20.04 LTS (WSL2)
- **ROS2 Distor**: Foxy Fitzroy
- **Other Tools**: Python 3, socat, pyserial

### ROS2 Foxy Installation (if not installed yet)

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install locale pls don't ""it works on my machine but not yours""" bugs.
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repo
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg2 lsb-release -y

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repo make sure os is right, packages are good, architecture matches
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install ros-foxy-desktop python3-argcomplete ros-dev-tools -y

# Source ROS2 (add to .bashrc)
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Installation

### 1. Install Required Tools

```bash
# Install socat for virtual ports
sudo apt install socat -y

# Install the serial library
sudo apt install python3-pip -y
pip3 install pyserial
```

### 2. Create ROS2 Workspace and Clone Project

```bash

# Clone this project
git clone https://github.com/michael545/ros2_serial_driver.git serial_driver_cpp
cd serial_driver_cpp

# Clone the serial library
git clone https://github.com/wjwwood/serial.git src/serial
```

### 3. Configure Serial Library

The serial library needs a simplified `CMakeLists.txt` for ROS2 compatibility:

```bash
# Backup original CMakeLists.txt
cd src/serial
mv CMakeLists.txt CMakeLists_original.txt

# The simplified CMakeLists.txt is already provided in this repository
# It handles platform-specific libraries and proper include directories
```

## Building the Project

### 1. Build the Workspace

```bash
cd ~/ros2_ws

# Build all packages
colcon build --packages-select serial_driver_cpp

# can try building with verbose output
# colcon build --packages-select serial_driver_cpp --event-handlers console_direct+
```

### 2. Source the Workspace

```bash
# Source the built workspace
source install/setup.bash

# Verify the package is found
ros2 pkg list | grep serial_driver_cpp
```

## Running the Virtual Serial Port Simulation

This project uses `socat` to create virtual serial ports that simulate hardware communication.

### 1. Create Virtual Serial Ports

**Terminal 1** - Create virtual serial port pair:
```bash
socat -d -d pty,raw,echo=0,link=/tmp/ttyS10 pty,raw,echo=0,link=/tmp/ttyS11
```

Keep this terminal open. The command creates two linked virtual ports:
- `/tmp/ttyS10` - Arduino simulator will write to this
- `/tmp/ttyS11` - ROS2 driver will read from this

### 2. Run Arduino Simulator

**Terminal 2** - Start the Python Arduino simulator:
```bash
cd ~/ros2_ws/src/serial_driver_cpp
python3 arduino_simulator.py
```

This script simulates an Arduino sending "msg #N" at 5Hz to `/tmp/ttyS10`.

### 3. Run ROS2 Driver

**Terminal 3** - Start the ROS2 serial driver:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run serial_driver_cpp arduino_driver_node --ros-args -p port:=/tmp/ttyS11
```

### 4. Monitor ROS2 Topic

**Terminal 4** -  See published data:
```bash
source /opt/ros/foxy/setup.bash
ros2 topic echo /arduino_data
```

Output shoud be this:
```
data: 'msg #0'
---
data: 'msg #1'
---
data: 'msg #2'
---
```

## Testing the Complete System

### Verification Steps

1. **Check if socat is running:**
   ```bash
   ps aux | grep socat
   ```

2. **Verify virtual ports exist:**
   ```bash
   ls -la /tmp/ttyS1*
   ```

3. **List ROS2 topics:**
   ```bash
   ros2 topic list
   # Should show /arduino_data
   ```

4. **Get topic info:**
   ```bash
   ros2 topic info /arduino_data
   # Should show 1 publisher, 0 subscribers (or more if echo is running)
   ```


## Key Integration Points

### 1. Serial Library Integration

The project integrates the `wjwwood/serial` library as a subdirectory not as external package:

```cmake
# In CMakeLists.txt
add_subdirectory(src/serial)
target_link_libraries(arduino_driver_node serial)
```

### 2. ROS2 Parameter Configuration

The driver has runtime parameters for flexibility:

```bash
# Default params
ros2 run serial_driver_cpp arduino_driver_node

# Custom port & baud rate
ros2 run serial_driver_cpp arduino_driver_node --ros-args -p port:=/dev/ttyUSB0 -p baud_rate:=9600
```

### 3. Data Flow

```
Serial Port/Arduino Simulator → /tmp/ttyS10 → socat → /tmp/ttyS11 → ROS2 Driver → /arduino_data topic
```

### Build Issues

1. **CMake can't find serial package:**
   ```bash
   # Ensure you're using add_subdirectory, not find_package
   # Check that src/serial/CMakeLists.txt exists and is simplified
   ```


### Runtime Issues

1. **Serial port permission denied:**
   ```bash
   sudo usermod -a -G dialout $USER
   # Restart terminal or WSL
   ```

2. **Virtual ports don't exist:**
   ```bash
   # socat is running in a new terminal
   # /tmp/ttyS10 and /tmp/ttyS11 exist
   ls -la /tmp/ttyS1*
   ```

- socat must remain running for the virtual connection to persist

## Future Enhancements

---

 Hardware-in-the-loop simulation system using ROS2, providing a robust foundation for serial communication
