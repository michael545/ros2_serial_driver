#!/bin/bash
# this does not work yet, but is a good start``
echo "=== ROS2 Arduino Driver Virtual Port Test ==="
echo

# chekc socat all good?
if ! pgrep -f "socat.*ttyS10.*ttyS11" > /dev/null; then
    echo "!!!!!??Virtual serial ports not detected. Starting socat..."
    echo "Run this cmd in another terminal:"
    echo "socat -d -d pty,raw,echo=0,link=/tmp/ttyS10 pty,raw,echo=0,link=/tmp/ttyS11"
    echo
    echo "Press Enter when socat is running..."
    read
else
    echo " Virtual serial ports detected"
fi

# Check if virtual ports exist
if [[ ! -e /tmp/ttyS10 ]] || [[ ! -e /tmp/ttyS11 ]]; then
    echo "Virtual serial ports /tmp/ttyS10 or /tmp/ttyS11 not found"
    echo "socat is running?"
    exit 1
fi

echo "✅ Virtual serial ports found: /tmp/ttyS10 and /tmp/ttyS11"
echo

# Source ROS2 workspace
echo "Sourcing ROS2 workspace..."
source install/setup.bash

echo " Testing plan:"
echo "  1. Arduino simulator will send data to /tmp/ttyS10"
echo "  2. ROS2 driver will read data from /tmp/ttyS11" 
echo "  3. Driver will publish data on /arduino_data topic"
echo

echo " Available test commands:"
echo "  - Start Arduino simulator: python3 arduino_simulator.py"
echo "  - Start ROS2 driver: ros2 run serial_driver_cpp arduino_driver_node --ros-args -p port:=/tmp/ttyS11"
echo "  - Monitor topic: ros2 topic echo /arduino_data"
echo "  - List topics: ros2 topic list"
echo

echo "Press Enter to continue or Ctrl+C to exit..."
read

echo "🚀 You can now run the commands in separate terminals!"
