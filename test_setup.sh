#!/usr/bin/env bash
set -e

echo "Testing calibration setup..."
echo ""

# test ros2
echo "Testing ROS2..."
if command -v ros2 &> /dev/null; then
    echo "ROS2: $(ros2 --version | head -1)"
    
    # test env
    if ros2 node list &> /dev/null; then
        echo "ROS2 env working"
    else
        echo "ROS2 env broken - source setup.bash"
    fi
else
    echo "ROS2 missing"
    exit 1
fi

# test colcon
echo ""
echo "Testing colcon..."
if command -v colcon &> /dev/null; then
    echo "colcon: $(colcon --version | head -1)"
else
    echo "colcon missing"
    exit 1
fi

# test rosdep
echo ""
echo "Testing rosdep..."
if command -v rosdep &> /dev/null; then
    echo "rosdep found"
    
    # test update
    if rosdep update &> /dev/null; then
        echo "rosdep working"
    else
        echo "rosdep broken"
    fi
else
    echo "rosdep missing"
    exit 1
fi

# test docker
echo ""
echo "Testing Docker..."
if command -v docker &> /dev/null; then
    echo "Docker: $(docker --version)"
    
    # test daemon
    if docker info &> /dev/null; then
        echo "Docker running"
        
        # test perms
        if docker run --rm hello-world &> /dev/null; then
            echo "Docker perms ok"
        else
            echo "Docker perms denied - run: sudo usermod -aG docker $USER"
        fi
    else
        echo "Docker not running - run: sudo systemctl start docker"
    fi
else
    echo "Docker missing"
    exit 1
fi

# test rosbags
echo ""
echo "Testing rosbags..."
if command -v rosbags-convert &> /dev/null; then
    echo "rosbags found"
else
    echo "rosbags missing - run: pip3 install rosbags"
    exit 1
fi

# test python
echo ""
echo "Testing Python packages..."
python3 -c "import matplotlib; print('matplotlib')" 2>/dev/null || echo "matplotlib missing"
python3 -c "import numpy; print('numpy')" 2>/dev/null || echo "numpy missing"
python3 -c "import scipy; print('scipy')" 2>/dev/null || echo "scipy missing"
python3 -c "import yaml; print('pyyaml')" 2>/dev/null || echo "pyyaml missing"

# test allan package
echo ""
echo "Testing allan_ros2..."
if [ -d "external/allan_ros2" ]; then
    echo "allan_ros2 found"
    
    # test build
    echo "Testing build..."
    mkdir -p test_build/src
    cp -r external/allan_ros2 test_build/src/
    cd test_build
    
    # dry run build
    if colcon build --packages-select allan_ros2 --dry-run &> /dev/null; then
        echo "allan_ros2 deps ok"
    else
        echo "allan_ros2 deps missing"
    fi
    
    cd ..
    rm -rf test_build
else
    echo "allan_ros2 missing"
fi

# test kalibr docker
echo ""
echo "Testing Kalibr Docker..."
if [ -f "external/kalibr/Dockerfile_ros1_20_04" ]; then
    echo "Kalibr Dockerfile found"
    
    # syntax check
    if docker build --dry-run -f external/kalibr/Dockerfile_ros1_20_04 external/kalibr &> /dev/null; then
        echo "Dockerfile syntax ok"
    else
        echo "Dockerfile syntax broken"
    fi
else
    echo "Kalibr Dockerfile missing"
fi

echo ""
echo "=== Test Summary ==="
echo "All = ready to go!"
echo "Any = fix first"
echo ""
echo "Next:"
echo "1. Get calibration data (IMU, camera, VIO bags)"
echo "2. Setup AprilGrid config"
echo "3. Run: ./pipeline.sh imu.bag camera.bag vio.bag aprilgrid.yaml"
echo ""
echo "Check README.md for help with failures"

