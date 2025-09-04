# Task 3: Two-tier Setup

## Introduction

It turns out that R2-D2 has a map of the warehouse! He’s going to use it to localise himself.

## Bag Data

As with Task 1 and Task 2, we will be using the planar bag dataset.

## Steps

1. Edit the file `$(task3)/config/two_tier.yaml`
1. The config for the `odom`->`base_link` instance has been provided for you.
1. Add parameters for a second node to the same config file. The node’s name should be `ekf_node_tier2`.
1. The `ekf_node_tier2` should have a `world_frame` of `map`.
1. It should have the exact same inputs as the `ekf_node_tier1`
1. It should also have a new pose input for a topic called `pose_global`. That topic contains poses in the `map` frame that provide an absolute reference for the filter. We want to fuse `x`, `y`, and `yaw` from this source.
1. We want the filter to trust the pose data, but not absolutely. Tune your `process_noise_covariance` accordingly.
1. After editing the config, run the following:

        $ ros2 launch task3 ekf.launch
