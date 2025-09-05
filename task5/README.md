# Task 5: Operating in 3D

## Introduction

So far, we’ve operated with `two_d_mode` set to `true`. But many robots operate in 3D. In this task, R2-D2 is going for a swim!

## Bag Data

The bag we will be working with can be found in `$bags/subsea_3d/subsea_3d.db3`. Familiarise yourself with the data:

- Look at the bag information with `ros2 bag info $bags/subsea_3d/subsea_3d.db3`
- Play the bag with `ros2 bag play $bags/subsea_3d/subsea_3d.db3 --clock`, then running `ros2 topic echo <topic name>` for the topics you want to analyse.
- Look at the transforms that are available via the `/tf_static` topic

## Steps

1. Edit the file `$task5/config/subsea_3d.yaml`
1. We have three sensors/input topics: `velocity`, `depth`, and `imu`.
1. For the `velocity` sensor, we want to fuse only the linear velocity dimensions
1. For the depth sensor, we want to fuse only `z` position
1. For the IMU, we want to fuse orientation _and_ angular velocity
1. Fill out the missing values
1. After editing the config, run the following:

    Terminal 1: `ros2 launch task5 ekf.launch.xml`  
    Terminal 2: `ros2 bag play $bags/subsea_3d/subsea_3d.db3 --clock`
