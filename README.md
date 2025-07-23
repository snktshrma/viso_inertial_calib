# viso_inertial_calib

This repository provides a simple pipeline for calibrating an IMU and camera using
[allan_ros2](https://github.com/CruxDevStuff/allan_ros2) and
[Kalibr](https://github.com/ethz-asl/kalibr).

The calibration requires three ROS 2 bags:

1. **IMU stationary bag** : IMU left still for ~2 h. Used to estimate Allan noise parameters.
2. **Camera intrinsics bag** : camera static while an AprilGrid is moved in view. Used for camera intrinsics.
3. **Visual-inertial bag** : camera sees a static AprilGrid while the sensor stack is moved. Used for extrinsic calibration.

Bags 2 and 3 are automatically converted to the ROS 1 format before running Kalibr.

** The output calibration parameters are directly compatible with many existing visual inertial softwares, like VINS-Fusion, etc.

## Quick usage

Clone submodules and run the pipeline script:

```bash
git clone --recursive https://github.com/snktshrma/viso_inertial_calib.git
cd viso_inertial_calib
./pipeline.sh imu.bag camera.bag vio.bag aprilgrid.yaml
```

`pipeline.sh` builds `allan_ros2`, computes `imu.yaml`, converts the camera and VIO bags to ROS 1 using `ros2 bag convert`, and then runs the two Kalibr
calibrations in a Docker container. Results (`imu.yaml` and `camchain*.yaml`)
will be generated in the repository root.

Make sure ROS 2 Humble, `colcon` and Docker are installed on the host.

## References

**[allan_ros2](https://github.com/CruxDevStuff/allan_ros2)** :
A ROS 2 package for analyzing IMU noise parameters using Allan deviation to generate noise specs for use with Kalibr.

**[Kalibr](https://github.com/ethz-asl/kalibr)** :
A visual-inertial calibration toolbox from ETH‑ASL that estimates camera-IMU and multi-sensor calibration parameters, including IMU intrinsic noise models.