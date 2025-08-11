#!/usr/bin/env bash
set -e

if [ "$#" -ne 4 ]; then
    echo "Usage: $0 BAG_IMU BAG_CAM BAG_CAM_IMU APRILGRID_YAML" >&2
    echo "Example: $0 imu_stationary.bag camera_intrinsics.bag vio_calibration.bag aprilgrid.yaml" >&2
    exit 1
fi

imu_bag=$1
cam_bag=$2
vio_bag=$3
april=$4

# check if files exist
if [ ! -f "$imu_bag" ]; then
    echo "Error: IMU bag file '$imu_bag' not found" >&2
    exit 1
fi

if [ ! -f "$cam_bag" ]; then
    echo "Error: Camera bag file '$cam_bag' not found" >&2
    exit 1
fi

if [ ! -f "$vio_bag" ]; then
    echo "Error: VIO bag file '$vio_bag' not found" >&2
    exit 1
fi

if [ ! -f "$april" ]; then
    echo "Error: AprilGrid YAML file '$april' not found" >&2
    exit 1
fi

workdir=$(pwd)
allan_ws=$workdir/allan_ws

echo "Starting calibration..."
echo "IMU: $imu_bag"
echo "Cam: $cam_bag"
echo "VIO: $vio_bag"
echo "Grid: $april"
echo ""

# check tools
if ! command -v rosbags-convert &> /dev/null; then
    echo "Error: rosbags-convert not found. Install with: pip install rosbags" >&2
    exit 1
fi

if ! command -v docker &> /dev/null; then
    echo "Error: docker not found." >&2
    exit 1
fi

# convert bags to ros1
echo "Converting bags..."
ros1_cam="cam_ros1.bag"
ros1_vio="vio_ros1.bag"

rosbags-convert --src "$cam_bag" --dst "$ros1_cam"
rosbags-convert --src "$vio_bag" --dst "$ros1_vio"

echo "Bags converted."
echo ""

# allan variance setup
echo "Setting up allan variance..."
mkdir -p "$allan_ws/src"
if [ ! -d "$allan_ws/src/allan_ros2" ]; then
    cp -r "$workdir/external/allan_ros2" "$allan_ws/src/allan_ros2"
fi

# config for allan
cat > "$allan_ws/src/allan_ros2/config/config.yaml" <<EOF
allan_node:
  ros__parameters:
    topic: /imu
    bag_path: $imu_bag
    msg_type: ros
    publish_rate: 200
    sample_rate: 200
EOF

# install deps
pip3 install matplotlib numpy scipy pyyaml

cd "$allan_ws"
rosdep install --from-paths src -y --ignore-src

echo "Building..."
colcon build --packages-select allan_ros2

if [ $? -ne 0 ]; then
    echo "Build failed" >&2
    exit 1
fi

echo "Build done."
echo ""

source install/setup.bash

# run allan analysis
echo "Running allan variance..."
echo "This might take a while..."

ros2 launch allan_ros2 allan_node.py &
allan_pid=$!

sleep 10

if ! kill -0 $allan_pid 2>/dev/null; then
    echo "allan node crashed" >&2
    exit 1
fi

echo "Analysis running..."
wait $allan_pid

echo "Allan done."
echo ""

# generate imu calib
echo "Generating IMU params..."
if [ -f "deviation.csv" ]; then
    python3 src/allan_ros2/scripts/analysis.py --data deviation.csv --config src/allan_ros2/config/config.yaml
    
    if [ -f "imu.yaml" ]; then
        echo "IMU calib generated."
        cp imu.yaml "$workdir/imu.yaml"
    else
        echo "Failed to generate imu.yaml" >&2
        exit 1
    fi
else
    echo "deviation.csv not found" >&2
    exit 1
fi

cd "$workdir"
echo ""

# camera calib
echo "Camera calibration..."

if ! docker image inspect kalibr:ros1 > /dev/null 2>&1; then
    echo "Building kalibr docker..."
    docker build -t kalibr:ros1 external/kalibr -f external/kalibr/Dockerfile_ros1_20_04
    
    if [ $? -ne 0 ]; then
        echo "Docker build failed" >&2
        exit 1
    fi
fi

echo "Running cam calib..."
docker run --rm -v "$workdir":/data -it kalibr:ros1 bash -c \
    "kalibr_calibrate_cameras --bag /data/$ros1_cam --target /data/$april --models pinhole-radtan --topics /camera/image_raw"

if [ $? -ne 0 ]; then
    echo "Camera calib failed" >&2
    exit 1
fi

# find camchain file
if ls camchain*.yaml 1> /dev/null 2>&1; then
    camchain_file=$(ls camchain*.yaml | head -1)
    cp "$camchain_file" camchain.yaml
    echo "Camera calib done: $camchain_file"
else
    echo "Camera calib file not found" >&2
    exit 1
fi

echo ""

# vio calib
echo "VIO calibration..."
echo "Running IMU-camera calib..."

docker run --rm -v "$workdir":/data -it kalibr:ros1 bash -c \
    "kalibr_calibrate_imu_camera --bag /data/$ros1_vio --target /data/$april --cam /data/camchain.yaml --imu /data/imu.yaml"

if [ $? -ne 0 ]; then
    echo "VIO calib failed" >&2
    exit 1
fi

echo ""

# done
echo "Calibration complete!"
echo ""
echo "Files generated:"
if [ -f "imu.yaml" ]; then
    echo "  imu.yaml"
fi
if [ -f "camchain.yaml" ]; then
    echo "  camchain.yaml"
fi
if ls imu-*.yaml 1> /dev/null 2>&1; then
    echo "  imu-*.yaml"
fi

echo ""
echo "Ready for VINS-Fusion etc."
echo ""

# cleanup
echo "Cleaning up..."
rm -f "$ros1_cam" "$ros1_vio"
rm -rf "$allan_ws"

echo "Done!"
