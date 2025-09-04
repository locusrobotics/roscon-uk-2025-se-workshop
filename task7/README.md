# Task 7: Déjà Two (-tier Setup)

## Introduction

We'll now assemble a two-tier setup with the `fixed_lag_smoother` in `fuse`.

## Bag Data

As with Task 6, we will be using the planar bag dataset.

## Steps

1. Edit the file `$(task7)/config/two_tier.yaml`
1. The config for the `odom`->`base_link` instance has been provided for you.
1. Add parameters for a second node to the same config file. The node’s name should be `fls_node_tier2`.
1. The `fls_node_tier2` should have a `world_frame` of `map`.
1. It should have the exact same inputs as the `fls_node_tier1`
1. It should also have a new pose input for a topic called `pose_global`. That topic contains poses in the `map` frame that provide an absolute reference for the smoother. We want to fuse `x`, `y`, and `yaw` from this source.
1. We want the filter to trust the pose data, but not absolutely. Tune your `process_noise_diagonal` accordingly.
1. After editing the config, run the following:

        $ ros2 launch task7 ekf.launch

    > **Question**: What do you note about the difference with the EKF output?
