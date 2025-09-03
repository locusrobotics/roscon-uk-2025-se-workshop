# Task 2: Advanced Configuration

## Introduction

In this task, we will try out some of the more advanced configuration options for the EKF nodes.

## Bag Data

As with Task 1, we will be using the planar bag dataset.

## Task 2a: Process Noise

Tuning the process noise covariance matrix can produce very different results.

> **Question**: Recall that in Task 1a, we fused just wheel encoder odometry, but our output state estimate did not very closely match the input wheel encoder data.
Why?

1. Edit the file `$(task2)/config/odometry_pnc.yaml`
1. Edit the `process_noise_covariance` for the wheel encoder odometry by increasing the values for `x` velocity, `y` velocity (not really necessary), and `yaw` velocity.
1. Now run

        $ ros2 launch task2 ekf.launch modified_pnc:=True

1. The output shows the original configuration alongside your updated configuration

    > **Question**: What do you note about the output from the updated configuration?

## Task 2b: Differential Mode

Sometimes, a topic contains only pose data, but you may not want to fuse that into your state estimate (e.g., if you have two pose sources, or the pose data is too infrequent).

Even though our VO data produces pose and velocity data, we’re going to pretend it only contains pose data, and that we don’t want to use it.

1. Edit the file `$(task2)/config/odometry_vo_diff.yaml`
1. The `odom1` sensor should have a topic of `odometry_visual`, and we should be fusing `x`, `y`, and `yaw` (**not** velocity!)
1. Enable `differential` mode for `odom1`
1. After editing the config, run the following:

        $ ros2 launch task2 ekf.launch differential:=True

1. The `rviz2` output also shows odometry + VO data that is fusing only velocity (note: none of our estimates are using the IMU).

    > **Question**: What do you note?