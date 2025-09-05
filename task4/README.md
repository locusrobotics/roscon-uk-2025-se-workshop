# Task 4: GPS Data

## Introduction

In this task, we will use `navsat_transform_node` to provide `map`-frame poses to our Tier 2 EKF instance.

This bag contains a (probably) unintentional, but useful, outage in sensor data.

## The `sensor_msgs/NavSatFix` Message

```
$ ros2 interface show sensor_msgs/msg/NavSatFix

std_msgs/Header header

NavSatStatus status

float64 latitude
float64 longitude

float64 altitude

float64[9] position_covariance

uint8 position_covariance_type
```

## Configuration

### `navsat_transform_node` Configuration Example

```
navsat_transform_node:
    ros__parameters:
        frequency: 30.0                     # Frequency of main run loop
        delay: 3.0                          # How long we wait until we compute the utm->world (usually map) frame transform

        magnetic_declination_radians: 0.0   # Obtain value from http://www.ngdc.noaa.gov/geomag-web/, convert to radians
        yaw_offset: 0.0                     # If the IMU doesn't report 0 facing east after correcting for magnetic declination, 
                                            # enter the value needed to get the IMU to report 0 when facing east here

        zero_altitude: false                # Zeros out the altitude that gets reported in the output
        broadcast_utm_transform: false      # Whether to publish the utm->world transform
        publish_filtered_gps: false         # Publishes our EKF output as GPS coordinates

        use_odometry_yaw: false             # If your EKF node already has an earth-referenced orientation, you can use it

        wait_for_datum: false               # Tells the node to wait until we manually specify a datum (utm-frame origin)
        datum: [55.944904, -3.186693, 0.0]  # If wait_for_datum is true, we will use this value. If wait_for_datum is true and
                                            # this parameter is not specified, we will wait for a service call.
```

### EKF Configuration Example

```
ekf_filter_node_tier1:
    ros__parameters:
        # ...familiar configuration...


ekf_filter_node_tier2:
    ros__parameters:
        # ...familiar configuration...

	   odom1: odometry/gps
        odom1_config: [true,  true,  false,
                       false, false, false,   # If operating in 3D, we would fuse Z (altitude) here
                       false, false, false,
                       false, false, false,
                       false, false, false]
```

## Bag Data

The bag we will be working with can be found in `$bags/gps/gps.db3`. Familiarise yourself with the data:

- Look at the bag information with `ros2 bag info gps.db3`
- Play the bag with `ros2 bag play gps.db3 --clock`, then running `ros2 topic echo <topic name>` for the topics you want to analyse.
- Look at the transforms that are available via the `/tf_static` topic

## Steps

1. In your bag replay terminal, navigate to the `$bags/gps/` directory
1. Run the following:

        $ ros2 bag play gps.bag --clock

1. In another terminal, run:

        $ ros2 topic echo /r2d2/gps --once

1. Note the reported GPS position
1. Stop the bag replay
1. Go to https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
1. Obtain the magnetic declination. Remember that you must convert it to radians, and that _counter-clockwise_ is positive.
1. Edit the file `$task4/config/gps.yaml`
1. Add the GPS sensor. Remember that we are only fusing x and y position.
1. After editing the config, run the following:

    Terminal 1: `ros2 launch task4 ekf.launch.xml`  
    Terminal 2: `ros2 bag play $bags/gps/gps.db3 --clock`
