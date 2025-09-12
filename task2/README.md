# Task 2: Advanced Configuration

## Introduction

In this task, we will try out some of the more advanced configuration options for the EKF nodes.

## Advanced Configuration

Here is a brief overview of the most relevant advanced parameters for the EKF node in `robot_localization`:

```
ekf_filter_node:
    ros__parameters:
        # ...basic parameters here...

	   transform_time_offset: 0.0  # Offset we can use to future-date the transform

        debug: false                              # Produces an absurd amount of debug output
        debug_out_file: /path/to/debug/file.txt   #

        odom0_queue_size: 2        # ROS queue size for the topic odom0
        odom0_differential: false  # Converts consecutive pose measurements to velocity data
        odom0_relative: false      # All pose data is reported relative to the first received message
        odom0_pose_rejection_threshold: 5.0    # Mahalanobis distance thresholds that can be used to reject outliers
        odom0_twist_rejection_threshold: 1.0   #

        imu0_remove_gravitational_acceleration: true  # Removes gravitational acceleration if your IMU doesn’t

        smooth_lagged_data: true        # Whether to allow the filter to handle out-of-sequence measurements
        history_length: 1.0             # The length of the history stored for out-of-sequence measurement handling

        predict_to_current_time: true   # Whether or not we always predict to the current time before publishing

        # The initial covariance (P) for the filter state
        initial_estimate_covariance: [1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9]

        # The Q matrix
        process_noise_covariance: [0.05, 0.05, 0.06, 0.03, 0.03, 0.06, 0.025, 0.025, 0.04, 0.01, 0.01, 0.02, 0.01, 0.01, 0.015]

        dynamic_process_noise_covariance: true  # Whether we scale Q based on the robot's velocity
```

## Bag Data

As with Task 1, we will be using the planar bag dataset.

## Task 2a: Process Noise

Tuning the process noise covariance matrix can produce very different results.

> **Question**: Recall that in Task 1a, we fused just wheel encoder odometry, but our output state estimate did not very closely match the input wheel encoder data.
Why?

### Steps

1. Edit the file `$task2/config/odometry_modified_pnc.yaml`
1. Edit the `process_noise_covariance` for the wheel encoder odometry by increasing the values for `x` velocity, `y` velocity (not really necessary), and `yaw` velocity.
1. Now run

    Terminal 1: `ros2 launch task2 ekf.launch.xml modified_pnc:=True`  
    Terminal 2: `ros2 bag play $bags/planar/planar.db3 --clock`

1. The output shows the original configuration alongside your updated configuration

    > **Question**: What do you note about the output from the updated configuration?

## Task 2b: Differential Mode

Sometimes, a topic contains only pose data, but you may not want to fuse that into your state estimate (e.g., if you have two pose sources, or the pose data is too infrequent).

Even though our VO data produces pose and velocity data, we’re going to pretend it only contains pose data, and that we don’t want to use it.

### Steps

1. Edit the file `$task2/config/odometry_vo_diff.yaml`
1. The `odom1` sensor should have a topic of `odometry_visual`, and we should be fusing `x`, `y`, and `yaw` (**not** velocity!)
1. Enable `differential` mode for `odom1`
1. After editing the config, run the following:

    Terminal 1: `ros2 launch task2 ekf.launch.xml differential:=True`  
    Terminal 2: `ros2 bag play $bags/planar/planar.db3 --clock`

1. The `rviz2` output also shows odometry + VO data that is fusing only velocity (note: none of our estimates are using the IMU).

    > **Question**: What do you note?
