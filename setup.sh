#!/usr/bin/env bash
set -e

echo "Setting up calibration dependencies..."
echo ""

# root check
if [ "$EUID" -eq 0 ]; then
    echo "Error: Please do not run this script as root" >&2
    exit 1
fi

# os detection
if [ -f /etc/os-release ]; then
    . /etc/os-release
    os_name=$NAME
    os_ver=$VERSION_ID
else
    echo "Error: Could not detect OS" >&2
    exit 1
fi

echo "OS: $os_name $os_ver"
echo ""

# ros2 check
if ! command -v ros2 &> /dev/null; then
    echo "Error: ROS2 not installed. Install ROS2 Humble first." >&2
    echo "See: https://docs.ros.org/en/humble/Installation.html" >&2
    exit 1
fi

echo "ROS2: $(ros2 --version | head -1)"
echo ""

# colcon
if ! command -v colcon &> /dev/null; then
    echo "Installing colcon..."
    pip3 install colcon-common-extensions
else
    echo "colcon: $(colcon --version | head -1)"
fi

# rosdep
if ! command -v rosdep &> /dev/null; then
    echo "Installing rosdep..."
    sudo apt update
    sudo apt install -y python3-rosdep
    sudo rosdep init
    rosdep update
else
    echo "rosdep found"
fi

# docker
if ! command -v docker &> /dev/null; then
    echo "Installing Docker..."
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    sudo usermod -aG docker $USER
    rm get-docker.sh
    echo "Docker installed. Log out/in or run: newgrp docker"
else
    echo "Docker: $(docker --version)"
fi

# rosbags
if ! command -v rosbags-convert &> /dev/null; then
    echo "Installing rosbags..."
    pip3 install rosbags
else
    echo "rosbags found"
fi

# python deps
echo "Installing python packages..."
pip3 install matplotlib numpy scipy pyyaml

# final check
echo ""
echo "=== Final Check ==="
missing=false

if ! command -v ros2 &> /dev/null; then
    echo "ROS2 missing"
    missing=true
else
    echo "ROS2"
fi

if ! command -v colcon &> /dev/null; then
    echo "colcon missing"
    missing=true
else
    echo "colcon"
fi

if ! command -v rosdep &> /dev/null; then
    echo "rosdep missing"
    missing=true
else
    echo "rosdep"
fi

if ! command -v docker &> /dev/null; then
    echo "Docker missing"
    missing=true
else
    echo "Docker"
fi

if ! command -v rosbags-convert &> /dev/null; then
    echo "rosbags missing"
    missing=true
else
    echo "rosbags"
fi

echo ""

if [ "$missing" = true ]; then
    echo "Some deps missing. Install manually."
    exit 1
else
    echo "All deps installed!"
    echo ""
    echo "Run calibration:"
    echo "  ./pipeline.sh imu.bag camera.bag vio.bag aprilgrid.yaml"
    echo ""
    echo "You need:"
    echo "  1. IMU bag (~2h stationary)"
    echo "  2. Camera bag (static cam, moving grid)"
    echo "  3. VIO bag (static grid, moving sensor)"
    echo "  4. AprilGrid config"
fi

