# Task 1: Basic Planar Robot

## Introduction

In the first task, we will help R2-D2 to estimate his state as he attempts to navigate an Imperial warehouse.

We will be building our state estimate by adding one sensor input at a time to our configuration. We will review the results and satisfy ourselves that the filter is behaving as we want before proceeding to the next sensor.

In many systems, when a robot is equipped with wheel encoders, the node that publishes the wheel encoder odometry often publishes the `odom`->`base_link` transform as well. But if you want to fuse other continuous sources of data with the odometry information before publishing the transform, using a state estimator is the way to go.

## Basic Configuration

Here is a brief overview of the most relevant basic parameters for the EKF node in `robot_localization`:

```
ekf_filter_node:
    ros__parameters:
        frequency: 30.0              # How frequently we publish (even if filter has not updated)
        sensor_timeout: 0.1          # If no sensor data in this time, we do a prediction (without correction)
        two_d_mode: false            # If enabled, 3D dimensions (z, roll, pitch, and their derivatives) are forced to 0
        transform_timeout: 0.0       # How long to wait for required transforms to be available
        print_diagnostics: true      # Whether to publish diagnostics
        publish_tf: true             # Whether to publish the output of the filter to /tf

        map_frame: map               # The name of your REP-105 map frame (not needed if world_frame == odom_frame)
        odom_frame: odom             # The name of your REP-105 odom frame
        base_link_frame: base_link   # The name of your REP-105 base_link frame. Will be the child_frame_id in the output.
        world_frame: odom            # The world frame that will be the frame_id in the output.

        odom0: example/odom                   # Topic type + number (odomN, poseN, twistN, imuN) and name
        odom0_config: [false, false, false,   # x, y, z
                       false, false, false,   # roll, pitch, yaw
                       true,  true,  false,   # x velocity, y velocity, z velocity
                       false, false, true,    # roll velocity, pitch velocity, yaw velocity
                       false, false, false]   # x acceleration, y acceleration, z acceleration
```

## Bag Data

The bag we will be working with can be found in `$bags/planar/planar.db3`. Familiarise yourself with the data:

- Look at the bag information with `ros2 bag info planar.db3`
- Play the bag with `ros2 bag play planar.db3 --clock`, then running `ros2 topic echo <topic name>` for the topics you want to analyse.
- Look at the transforms that are available via the `/tf_static` topic

## Task 1a: Odometry Only

To start, we will fuse only wheel encoder odometry into our state estimate. This isn't really that useful in practice, but it's a neccesary starting point.

### Steps

1. Edit the file `$task1/config/odometry.yaml`
1. We want to make our first odometry (as in `nav_msgs/Odometry`) input our wheel encoder odometry. Set the topic for `odom0` accordingly.
1. For this exercise, we will fuse the `x` velocity, the `y` velocity, and the `yaw` velocity from the wheel encoders

    > **Question**: If R2-D2 is a differential drive robot (let's assume he is), why are we fusing the `y` velocity?

1. Run the filter and `rviz2` with:

    Terminal 1: `ros2 launch task1 ekf.launch.xml`  
    Terminal 2: `ros2 bag play $bags/planar/planar.db3 --clock`

    > **Question**: For comparison, we show the raw wheel encoder data alongside the EKF output. What do you note about the output?

1. The bag starts and ends at the same location. Use the `rviz2` measurement tool to measure the distance from the robot’s first pose to its last. Make a mental note of the value!

## Task 1b: Odometry + IMU

We will now be adding IMU sensor data to our filter.

### Steps

1. Edit the file `$task1/config/odometry_imu.yaml`
1. The wheel encoder odometry configuration has been provided for you
1. You need to now fill out the configuration for the IMU topic. We want to fuse `yaw` velocity and `x` acceleration from the sensor.

    > **Question**: Why not fuse the `y` acceleration from the IMU as we did with the `y` velocity for the wheel odometry?

1. R2’s holographic projector is bulky and made mounting the IMU difficult, so his designers mounted the IMU such that `+X` points to the ground, `+Y` points to R2’s right, and `+Z` points towards his back. **This will have ramifications for the sensor configuration!**

1. Run the filter and `rviz2` with:

    Terminal 1: `ros2 launch task1 ekf.launch.xml include_imu:=True`  
    Terminal 2: `ros2 bag play $bags/planar/planar.db3 --clock`

1. The launch file runs two instances:
    1. One has our previous odometry-only config
    1. One has odometry + IMU
    1. (Raw wheel encoder data is also displayed.)

    > **Question**: What do you note about them? How does the distance from the last pose to the first compare to the odometry-only instance?

## Task 1c: Odometry + IMU + VO

We will now add visual odometry data as an input to the filter

### Steps

1. Edit the file `$task1/config/odometry_imu_vo.yaml`
1. The wheel encoder odometry and IMU configurations have been provided for you
1. As with wheel encoder odometry, we want to fuse `x`, `y`, and `yaw` velocities into the filter
1. Run the filter and `rviz2` with:

    Terminal 1: `ros2 launch task1 ekf.launch.xml include_imu_vo:=True`  
    Terminal 2: `ros2 bag play $bags/planar/planar.db3 --clock`

1. We now have three EKF instances running:
    1. One with just wheel encoder data
    1. One with wheel encoder + IMU data
    1. One with wheel encoder, IMU, and visual odometry data
    1. (Raw wheel encoder data is also displayed.)

    > **Question**: What do you note about the output?
