# Task 6: Back to the Planar

## Introduction

In this task, we will revisit the first planar robot task that we carried out with EKFs in `robot_localization`, but we will now use the `fixed_lag_smoother_node` in fuse.

We will again be building our state estimate by adding one sensor input at a time to our configuration. We will review the results and satisfy ourselves that the filter is behaving as we want before proceeding to the next sensor.

## Bag Data

We will once again be working with the bag file `$bags/planar/planar.db3`.

## Task 6a: Odometry Only

To start, we will fuse only wheel encoder odometry into our state estimate. This isn't really that useful in practice, but it's a neccesary starting point.

### Steps

1. Edit the file `$task6/config/odometry.yaml`
1. We want to make our first odometry (as in `nav_msgs/Odometry`) input our wheel encoder odometry. For that, we'll need to add a sensor of type `fuse_models::Odometry2D` to the `fuse` configuration.
1. For this exercise, we will fuse the `x` velocity, the `y` velocity, and the `yaw` velocity from the odometry sensor.

1. Run the filter and `rviz2` with:

    Terminal 1: `ros2 launch task6 fls.launch`  
    Terminal 2: `ros2 bag play $bags/planar/planar.db3 --clock`

1. For comparison, we show the raw wheel encoder data and EKF output alongside the `fixed_lag_smoother` output.
1. The bag starts and ends at the same location. As in Task 1, use the rviz2 measurement tool to measure the distance from the robot’s first pose to its last. Make a mental note of the value!

## Task 6b: Odometry + IMU

We will now be adding IMU sensor data to our smoother.

### Steps

1. Edit `$task6/config/odometry_imu.yaml`
1. The wheel encoder odometry configuration has been provided for you
1. You need to now fill out the configuration for the IMU topic. We want to fuse `yaw` velocity and `x` acceleration from the sensor.
1. R2’s holographic projector is bulky and made mounting the IMU difficult, so his designers mounted the IMU such that `+X` points to the ground, `+Y` points to R2’s right, and `+Z` points towards his back. **This will have ramifications for the sensor configuration!**

1. Run the filter and `rviz2` with:

    Terminal 1: `ros2 launch task6 ekf.launch include_imu:=True`  
    Terminal 2: `ros2 bag play $bags/planar/planar.db3 --clock`

1. The launch file runs two `fixed_lag_smoother` instances:
    1. One has our previous odometry-only config
    1. One has odometry + IMU
    1. (Raw wheel encoder data is also displayed)
    
1. In addition, we run an EKF with the same odometry + IMU config so that we can compare the two.

    > **Question**: What do you note about the difference between the `fixed_lag_smoother` and EKF outputs, particularly at the end of the path?

## Task 6c: Odometry + IMU + VO

We will now add visual odometry data as an input to the smoother. As with wheel encoder odometry, we want to fuse x, y, and yaw velocities into the filter.

### Steps

1. Edit the file `$task6/config/odometry_imu_vo.yaml`
1. The wheel encoder odometry and IMU configurations have been provided for you
1. As with wheel encoder odometry, we want to fuse `x`, `y`, and `yaw` velocities into the filter
1. Run the filter and `rviz2` with:

    Terminal 1: `ros2 launch task6 fls.launch include_imu_vo:=True`  
    Terminal 2: `ros2 bag play $bags/planar/planar.db3 --clock`

1. We now have three `fixed_lag_smoother` instances running:
    1. One with just wheel encoder data
    1. One with wheel encoder + IMU data
    1. One with wheel encoder, IMU, and visual odometry data
    1. (Raw wheel encoder data is also displayed.)

1. In addition, we run an EKF with the same odometry + IMU + VO config so that we can compare the two.

    > **Question**: What do you note about the output?

    > **Question**: Try running at `top | grep -e fixed_lag -e ekf -e CPU` to identify the CPU usage of the smoother and EKF instances. What do you note?
