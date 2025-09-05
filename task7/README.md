# Task 7: Déjà Two (-tier Setup)

## Introduction

We'll now assemble a two-tier setup with the `fixed_lag_smoother` in `fuse`.

## Basic Configuration for the `fixed_lag_smoother_node`

```
optimization_frequency: 20
transaction_timeout: 0.01
lag_duration: 0.5

motion_models:
  unicycle_motion_model:
    type: fuse_models::Unicycle2D

unicycle_motion_model:
  process_noise_diagonal: [0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.1, 0.1]  # x, y, yaw, vx, vy, vyaw, ax, ay

sensor_models:
  initial_localization_sensor:
    type: fuse_models::Unicycle2DIgnition
    motion_models: [unicycle_motion_model]
    ignition: true
  odometry_sensor:
    type: fuse_models::Odometry2D
    motion_models: [unicycle_motion_model]
  imu_sensor:
    type: fuse_models::Imu2D
    motion_models: [unicycle_motion_model]

initial_localization_sensor:
  publish_on_startup: true
  #                x      y      yaw    vx     vy     vyaw    ax     ay
  initial_state: [0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
  initial_sigma: [0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.100, 0.100]

odometry_sensor:
  topic: 'odom'
  twist_target_frame: 'base_footprint'
  linear_velocity_dimensions: ['x', 'y']
  angular_velocity_dimensions: ['yaw']

imu_sensor:
  topic: 'imu'
  twist_target_frame: 'base_footprint'
  angular_velocity_dimensions: ['yaw']

publishers:
  filtered_publisher:
    type: fuse_models::Odometry2DPublisher

filtered_publisher:
  topic: 'odom_filtered'
  base_link_frame_id: 'base_footprint'
  odom_frame_id: 'odom'
  map_frame_id: 'map'
  world_frame_id: 'odom'
  publish_tf: true
  publish_frequency: 10
```

## Bag Data

As with Task 6, we will be using the planar bag dataset.

## Steps

1. Edit the file `$task7/config/two_tier.yaml`
1. The config for the `odom`->`base_link` instance has been provided for you.
1. Add parameters for a second node to the same config file. The node’s name should be `fls_node_tier2`.
1. The `fls_node_tier2` should have a `world_frame` of `map`.
1. It should have the exact same inputs as the `fls_node_tier1`
1. It should also have a new pose input for a topic called `pose_global`. That topic contains poses in the `map` frame that provide an absolute reference for the smoother. We want to fuse `x`, `y`, and `yaw` from this source.
1. We want the filter to trust the pose data, but not absolutely. Tune your `process_noise_diagonal` accordingly.
1. After editing the config, run the following:

    Terminal 1: `ros2 launch task7 ekf.launch.xml`  
    Terminal 2: `ros2 bag play $bags/planar/planar.db3 --clock`

    > **Question**: What do you note about the difference with the EKF output?
