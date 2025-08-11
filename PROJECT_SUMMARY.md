# Visual-Inertial Calibration Pipeline - Project Summary

## What Has Been Completed

This project provides a complete, automated pipeline for calibrating IMU and camera systems. The pipeline has been fully implemented and tested, with the following components:

### 1. Core Pipeline Script (`pipeline.sh`)
- **Fixed bag conversion**: Corrected ROS2 to ROS1 bag conversion paths
- **Proper dependency management**: Added Python package installation and ROS2 dependency handling
- **Error handling**: Comprehensive error checking and validation
- **Progress tracking**: Clear status messages and progress indicators
- **Automatic cleanup**: Removes temporary files after completion

### 2. Setup and Testing Scripts
- **`setup.sh`**: Automated dependency installation and verification
- **`test_setup.sh`**: Comprehensive testing of all components
- **`example_aprilgrid.yaml`**: Template for AprilGrid configuration

### 3. Enhanced Documentation
- **Comprehensive README.md**: Complete usage instructions and troubleshooting
- **Data collection guidelines**: Best practices for calibration data
- **Troubleshooting section**: Solutions for common issues

### 4. Fixed Dependencies
- **allan_ros2 package**: Fixed configuration parsing issues
- **Python dependencies**: Added matplotlib, numpy, scipy, pyyaml
- **ROS2 integration**: Proper workspace building and dependency resolution

## Pipeline Stages

### Stage 1: IMU Allan Variance Analysis
1. **Setup**: Creates temporary workspace and copies allan_ros2 package
2. **Build**: Compiles allan_ros2 with proper dependencies
3. **Analysis**: Runs Allan variance computation on stationary IMU data
4. **Output**: Generates `imu.yaml` with noise parameters

### Stage 2: Camera Intrinsics Calibration
1. **Bag Conversion**: Converts ROS2 camera bag to ROS1 format
2. **Docker Build**: Creates Kalibr Docker image if needed
3. **Calibration**: Runs camera intrinsic calibration
4. **Output**: Generates `camchain.yaml` with camera parameters

### Stage 3: Visual-Inertial Extrinsics
1. **Bag Conversion**: Converts ROS2 VIO bag to ROS1 format
2. **Combined Calibration**: Uses IMU and camera data together
3. **Extrinsic Estimation**: Computes spatial relationship between sensors
4. **Output**: Generates `imu-*.yaml` with extrinsic parameters

## Usage Instructions

### Quick Start
```bash
# 1. Clone and setup
git clone --recursive https://github.com/snktshrma/viso_inertial_calib.git
cd viso_inertial_calib
./setup.sh

# 2. Test setup
./test_setup.sh

# 3. Run calibration
./pipeline.sh imu.bag camera.bag vio.bag aprilgrid.yaml
```

### Required Data
- **IMU Stationary Bag**: 2+ hours of completely stationary IMU data
- **Camera Intrinsics Bag**: Camera static, AprilGrid moving (5-10 min)
- **VIO Calibration Bag**: AprilGrid static, sensor moving (5-10 min)
- **AprilGrid Configuration**: YAML file defining grid parameters

### Output Files
- `imu.yaml`: IMU noise parameters (white noise, bias instability, random walk)
- `camchain.yaml`: Camera intrinsic matrix and distortion coefficients
- `imu-*.yaml`: IMU-camera extrinsic calibration (transformation matrix)

## System Requirements

### Software Dependencies
- **ROS2 Humble** (Ubuntu 22.04)
- **Docker** for Kalibr execution
- **Python 3.8+** with pip
- **Git** with submodule support

### Hardware Requirements
- **IMU**: Any ROS2-compatible IMU (PX4, MPU9250, etc.)
- **Camera**: Any ROS2-compatible camera (USB, CSI, etc.)
- **AprilGrid**: Printed calibration target (configurable size)

## Key Features

### Automation
- **End-to-end pipeline**: Single command execution
- **Automatic dependency management**: Installs missing packages
- **Smart error handling**: Continues or fails gracefully
- **Progress tracking**: Real-time status updates

### Flexibility
- **Multiple IMU types**: Supports various IMU message formats
- **Configurable parameters**: Adjustable sample rates and topics
- **Extensible architecture**: Easy to add new sensor types

### Reliability
- **Comprehensive validation**: Checks all inputs and dependencies
- **Error recovery**: Handles common failure modes
- **Cleanup**: Removes temporary files automatically

## Troubleshooting

### Common Issues
1. **Docker permissions**: Run `sudo usermod -aG docker $USER`
2. **ROS2 environment**: Source `/opt/ros/humble/setup.bash`
3. **Missing dependencies**: Run `./setup.sh` to install
4. **Build failures**: Check ROS2 and colcon installation

### Debug Mode
```bash
# Add debug output to pipeline
set -x  # Add at beginning of pipeline.sh
```

### Manual Execution
If the pipeline fails, each stage can be run manually:
```bash
# Stage 1: Allan variance
cd allan_ws
colcon build --packages-select allan_ros2
source install/setup.bash
ros2 launch allan_ros2 allan_node.py

# Stage 2: Camera calibration
docker run --rm -v $(pwd):/data -it kalibr:ros1 bash -c \
    "kalibr_calibrate_cameras --bag /data/camera.bag --target /data/aprilgrid.yaml --models pinhole-radtan --topics /camera/image_raw"

# Stage 3: IMU-camera calibration
docker run --rm -v $(pwd):/data -it kalibr:ros1 bash -c \
    "kalibr_calibrate_imu_camera --bag /data/vio.bag --target /data/aprilgrid.yaml --cam /data/camchain.yaml --imu /data/imu.yaml"
```

## Integration

### SLAM Systems
The generated calibration files are compatible with:
- **VINS-Fusion**: Popular visual-inertial SLAM
- **OKVIS**: Open Keyframe-based Visual-Inertial SLAM
- **ROVIO**: Robust Visual Inertial Odometry
- **Other systems**: Any Kalibr-compatible framework

### ROS2 Integration
- **Message types**: Standard sensor_msgs/Imu and sensor_msgs/Image
- **Topics**: Configurable topic names
- **Parameters**: ROS2 parameter system integration

## Future Enhancements

### Potential Improvements
1. **Multi-camera support**: Calibrate multiple cameras simultaneously
2. **Online calibration**: Real-time parameter updates
3. **GUI interface**: Web-based calibration interface
4. **Validation tools**: Calibration quality assessment
5. **Export formats**: Support for additional SLAM systems

### Extensibility
- **New sensor types**: Easy to add different IMU or camera models
- **Custom algorithms**: Replace Allan variance with other noise models
- **Alternative backends**: Support for other calibration tools

## Conclusion

This project provides a production-ready, automated pipeline for visual-inertial calibration. It handles the complete workflow from raw sensor data to calibrated parameters, with robust error handling and comprehensive documentation. The system is designed to be reliable, user-friendly, and easily extensible for future requirements.

The pipeline successfully addresses the common challenges in visual-inertial calibration:
- **Complexity**: Automates the entire process
- **Reliability**: Comprehensive error checking and recovery
- **Usability**: Clear documentation and setup scripts
- **Integration**: Compatible with existing SLAM systems

Users can now calibrate their visual-inertial systems with a single command, significantly reducing the time and expertise required for this critical task.

