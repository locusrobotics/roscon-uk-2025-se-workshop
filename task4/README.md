# Task 4: GPS Data

## Introduction

In this task, we will use `navsat_transform_node` to provide `map`-frame poses to our Tier 2 EKF instance.

This bag contains a (probably) unintentional, but useful, outage in sensor data.

## Bag Data

The bag we will be working with can be found in bags/gps/gps.db3. Familiarise yourself with the data:

- Look at the bag information with `ros2 bag info gps.db3`
- Play the bag with `ros2 bag play gps.db3 --clock`, then running `ros2 topic echo <topic name>` for the topics you want to analyse.
- Look at the transforms that are available via the `/tf_static` topic

## Steps

1. In your bag replay terminal, navigate to the `$(bags)/gps/` directory
1. Run the following:

        $ ros2 bag play gps.bag --clock

1. In another terminal, run:

        $ ros2 topic echo /r2d2/gps --once

1. Note the reported GPS position
1. Stop the bag replay
1. Go to https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
1. Obtain the magnetic declination. Remember that you must convert it to radians, and that _counter-clockwise_ is positive.
1. Edit the file `$(task4)/config/gps.yaml`
1. Add the GPS sensor. Remember that we are only fusing x and y position.
1. After editing the config, run the following:

        $ ros2 launch task4 ekf.launch

1. In your bag replay terminal, run

        $ ros2 bag play gps.db3 --clock