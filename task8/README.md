# Task 8: Your First Sensor Model

## Introduction

`fuse` supports plugins for all of its key object types. We’ll focus on sensor models.

Our sensor model will be that of a beacon range sensor. The sensor wirelessly receives messages from beacons placed at regular intervals throughout the warehouse environment that we’ve already seen.

In this case, we need to develop three classes:

1. `SensorModel`: we need a class that is derived from the `SensorModel` base class in `fuse_core`. The job of this class is to receive sensor data and create an instance of a…
1. `Constraint`: we will derive a class whose instances will be added to the actual factor graph. The constraint math itself will be wrapped in a…
1. `CostFunctor`: This is not actually a base class, but a class that must wrap a `()` operator. Ceres uses this method to compute both the residual and, if using auto differentiation, Jacobian matrices.

### Sensor Model Classes

#### Example Sensor Model

There is some threading complexity involved in simply inheriting from `fuse_core::SensorModel`. `fuse` conveniently provides an `AsyncSensorModel` base class that inherits from `SensorModel` and handles that complexity for you. Users are then able to work with a simplified API:

```
class MySensorModel : public fuse_core::AyncSensorModel
{
...

protected:

// Must override. This is where the sensor reads parameters, subscribes to non-sensor topics, etc.
void onInit() override;

// You will likely be receiving sensor data. Within this method, you will likely create transactions
// with constraints on variables and relevant time stamps, and then call sendTransaction().
void dataCallback(const whatever_msgs::msg::SensorMessage & message);

// Can optionally implement this method if your sensor model requires variable values from the graph.
void onGraphUpdate(Graph::ConstSharedPtr graph) override;

// This is where we typically subscribe to sensor data
void onStart() override;

// This is where we typically unsubscribe from sensor data
void onStop() override;
```

[fuse_core::AsyncSensorModel](https://github.com/locusrobotics/fuse/blob/2b6e248a08671e5500e3eedaa0f45a8d44e94ba2/fuse_core/include/fuse_core/async_sensor_model.hpp)

#### Example Constraint

`Constraint` classes are effectively wrappers around Ceres cost functions.

```
class MyConstraint : public fuse_core::Constraint
{
public:
  // Your constraint will involve one or more variables, along with measurements (or priors) of their values
  MyConstraint(const std::string & source,
               const fuse_variables::Pose2DStamped & robot_pose,
               const Eigen::Vector3d& measurement_mean,
               const Eigen::Matrix3d& measurement_covariance);

  // Useful for debugging
  void print(std::ostream & stream = std::cout) const override;

  // This method must be overridden, and must return a pointer to a Ceres cost function. The returned object is what
  // computes the actual residual.
  ceres::CostFunction * costFunction() const override;

private:
  fuse_core::Vector3d measurement_mean_;  // The measured/prior mean vector for this variable
  fuse_core::Matrix3d measurement_sqrt_information_;  // The square root information matrix
```

[fuse_core::Constraint](https://github.com/locusrobotics/fuse/blob/2b6e248a08671e5500e3eedaa0f45a8d44e94ba2/fuse_core/include/fuse_core/constraint.hpp)

#### Example Ceres Cost Function

The actual cost function is implemented here. Ceres will pass the variable data to the functor. The user needs to use the variable value, the measurement and covariance provided in the constructor, and any other data it needs to compute the residual (error). The residual error gets weighted by the square root information matrix. Note that for a single-dimensional measurement and residual, this weighting factor would just be `1 / sigma`.

```
class MyCostFunctor
{
public:
  // Your cost functor receives any values it will need to compute the residual
  MyCostFunctor(const Eigen::Vector3d& measurement_mean, cost Eigen::Matrix3d& measurement_sqrt_information_)
  : measurement_mean_(measurement_mean), measurement_sqrt_information_(measurement_sqrt_information_)


  // Useful for debugging
  void print(std::ostream & stream = std::cout) const override;

  template<typename T>
  bool operator()(const T * const robot_pose, T * residuals) const  // Sizes of these arrays are defined in the constraint
  {
    residuals[0] = measurement_mean_[0] - robot_pose[0];
    ...

    // Map it to Eigen, and weight it
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residual_map(residual);
    residual_map.applyOnTheLeft(measurement_sqrt_information_.template cast<T>());
  }
```

[Example](https://github.com/locusrobotics/fuse/blob/2b6e248a08671e5500e3eedaa0f45a8d44e94ba2/fuse_tutorials/include/fuse_tutorials/range_cost_functor.hpp)

## Bag Data

As with Task 7, we will be using the planar bag dataset.

## Steps

1. The code files that we need to modify have already been generated. Take some time to review them and read through the comments:

        $task8/include/beacon_sensor_model.hpp
        $task8/src/beacon_sensor_model.cpp
        $task8/include/range_constraint.hpp
        $task8/src/range_constraint.cpp
        $task8/include/range_cost_functor.hpp

1. Analyse the sensor message type we will be using:

        $ ros2 interface show workshop_msgs/msg/BeaconRangeArray.msg

1. Edit `$task8/include/beacon_sensor_model.hpp`
1. A subscriber and callback method have been created, but need the message type to be added.
1. Edit `$task8/src/beacon_sensor_model.cpp`
1. Edit the creation of the subscriber on line 45 by adding the correct type
1. Add the correct variable to the call on line 87
1. Add the correct container for iteration on line 97
    > Note the call on line 99!
1. Edit `$task8/include/range_constraint.hpp`
1. Add the correct variable type on line 78
1. Edit `$task8/src/range_constraint.cpp`
1. Add the correct variable type on line 29
1. Add the correct template parameter values on line 62 (see the comments above it)
1. Edit `$task8/include/range_cost_functor.hpp`
1. On line 98 and 99, add the correct values so that we are computing the difference between the robot’s position variable components and the beacon’s reported position
1. Run `cb` to build the workspace (recall that this will handle directory changes before and after building)
1. Run `s` to source the workspace
1. Edit the file `$task8/config/two_tier.yaml`
1. Add the beacon sensor to your configuration

    Terminal 1: `ros2 launch task8 fls.launch.xml`  
    Terminal 2: `ros2 bag play $bags/planar/planar.db3 --clock`
