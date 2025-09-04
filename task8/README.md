# Task 8: Your First Sensor Model

## Introduction

`fuse` supports plugins for all of its key object types. We’ll focus on sensor models.

Our sensor model will be that of a beacon range sensor. The sensor wirelessly receives messages from beacons placed at regular intervals throughout the warehouse environment that we’ve already seen.

In this case, we need to develop three classes:

1. `SensorModel`: we need a class that is derived from the `SensorModel` base class in `fuse_core`. The job of this class is to receive sensor data and create an instance of a…
1. `Constraint`: we will derive a class whose instances will be added to the actual factor graph. The constraint math itself will be wrapped in a…
1. `CostFunctor`: This is not actually a base class, but a class that must wrap a `()` operator. Ceres uses this method to compute both the residual and, if using auto differentiation, Jacobian matrices.

## Bag Data

As with Task 7, we will be using the planar bag dataset.

## Steps

1. The code files that we need to modify have already been generated. Take some time to review them and read through the comments:

        $(task8)/include/beacon_sensor_model.hpp
        $(task8)/src/beacon_sensor_model.cpp
        $(task8)/include/range_constraint.hpp
        $(task8)/src/range_constraint.cpp
        $(task8)/include/range_cost_functor.hpp

1. Analyse the sensor message type we will be using:

        $ ros2 interface show workshop_msgs/msg/BeaconRangeArray.msg

1. Edit `$(task8)/include/beacon_sensor_model.hpp`
1. A subscriber and callback method have been created, but need the message type to be added.
1. Edit `$(task8)/src/beacon_sensor_model.cpp`
1. Edit the creation of the subscriber on line 45 by adding the correct type
1. Add the correct variable to the call on line 87
1. Add the correct container for iteration on line 97
    > Note the call on line 99!
1. Edit `$(task8)/include/range_constraint.hpp`
1. Add the correct variable type on line 78
1. Edit `$(task8)/src/range_constraint.cpp`
1. Add the correct variable type on line 29
1. Add the correct template parameter values on line 62 (see the comments above it)
1. Edit `$(task8)/include/range_cost_functor.hpp`
1. On line 98 and 99, add the correct values so that we are computing the difference between the robot’s position variable components and the beacon’s reported position
1. Run `cb` to build the workspace (recall that this will handle directory changes before and after building)
1. Run `s` to source the workspace
1. Edit the file `$(task8)/config/two_tier.yaml`
1. Add the beacon sensor to your configuration

        $ ros2 launch task8 fls.launch
