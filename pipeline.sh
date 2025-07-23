#!/usr/bin/env bash
set -e

# Pipeline for IMU and camera calibration
# Requires ROS 2 Humble with colcon and docker installed.

if [ "$#" -ne 4 ]; then
    echo "Usage: $0 BAG_IMU BAG_CAM BAG_CAM_IMU APRILGRID_YAML" >&2
    exit 1
fi

IMU_BAG=$1
CAM_BAG=$2
VIO_BAG=$3
APRIL=$4

WORKDIR=$(pwd)
ALLAN_WS=$WORKDIR/allan_ws

# Convert camera and VIO bags from ros2 to rosbag for Kalibr
ROS1_CAM_BAG="${CAM_BAG%.*}_ros1"
ROS1_VIO_BAG="${VIO_BAG%.*}_ros1"
rosbags-convert --src "$CAM_BAG" --dst "${ROS1_CAM_BAG%.bag}" 
rosbags-convert --src "$VIO_BAG" --dst "${ROS1_VIO_BAG%.bag}"

# Allan params est,
mkdir -p "$ALLAN_WS/src"
if [ ! -d "$ALLAN_WS/src/allan_ros2" ]; then
    cp -r "$WORKDIR/external/allan_ros2" "$ALLAN_WS/src/allan_ros2"
fi

cat > "$ALLAN_WS/src/allan_ros2/config/config.yaml" <<EOF2
allan_node:
  ros__parameters:
    topic: /imu
    bag_path: $IMU_BAG
    msg_type: ros
    publish_rate: 200
    sample_rate: 200
EOF2

cd "$ALLAN_WS"
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select allan_ros2
source install/setup.bash

# run allan_ros2
ros2 launch allan_ros2 allan_node.py

# imu.yaml
python3 src/allan_ros2/scripts/analysis.py --data deviation.csv --config src/allan_ros2/config/config.yaml
cp imu.yaml "$WORKDIR/imu.yaml"
cd "$WORKDIR"

# Camera intrinsics
# ussing kalibr Docker image
if ! docker image inspect kalibr:ros1 > /dev/null 2>&1; then
    docker build -t kalibr:ros1 external/kalibr -f external/kalibr/Dockerfile_ros1_20_04
fi

docker run --rm -v "$WORKDIR":/data -it kalibr:ros1 bash -c \
    "kalibr_calibrate_cameras --bag /data/$ROS1_CAM_BAG --target /data/$APRIL --models pinhole-radtan --topics /camera/image_raw"

cp camchain*.yaml camchain.yaml

# Viso-inertial extrinsics
docker run --rm -v "$WORKDIR":/data -it kalibr:ros1 bash -c \
    "kalibr_calibrate_imu_camera --bag /data/$ROS1_VIO_BAG --target /data/$APRIL --cam /data/camchain.yaml --imu /data/imu.yaml"
