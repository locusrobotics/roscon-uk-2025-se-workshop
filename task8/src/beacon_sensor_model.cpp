// Copyright 2025 Locus Robotics

#include "task8/beacon_sensor_model.hpp"

#include <fuse_constraints/absolute_constraint.hpp>
#include <fuse_core/async_sensor_model.hpp>
#include <fuse_core/sensor_model.hpp>
#include <fuse_core/transaction.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <workshop_msgs/msg/beacon_range_array.hpp>

#include "task8/range_constraint.hpp"


// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(task8::BeaconSensorModel, fuse_core::SensorModel);

namespace task8
{

void BeaconSensorModel::initialize(
  fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
  const std::string & name,
  fuse_core::TransactionCallback transaction_callback)
{
  interfaces_ = interfaces;
  fuse_core::AsyncSensorModel::initialize(interfaces, name, transaction_callback);
}

void BeaconSensorModel::onInit()
{
  logger_ = interfaces_.get_node_logging_interface()->get_logger();
}

void BeaconSensorModel::onStart()
{
  // Subscribe to the ranges topic. Any received messages will be processed within the message
  // callback function, and the created constraints will be sent to the optimizer. By subscribing to
  // the topic in onStart() and unsubscribing in onStop(), we will only send transactions to the
  // optimizer while it is running.
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_;

  sub_ = rclcpp::create_subscription<?????>(
    interfaces_,
    "beacon_ranges",
    10,
    std::bind(
      &BeaconSensorModel::rangesCallback, this, std::placeholders::_1
    ),
    sub_options
  );
}

void BeaconSensorModel::onStop()
{
  // Unsubscribe from the ranges topic. Since the sensor constraints are created and sent from the
  // subscriber callback, shutting down the subscriber effectively stops the creation of new
  // constraints from this sensor model. This ensures we only send transactions to the optimizer
  // while it is running.
  sub_.reset();
}

void BeaconSensorModel::rangesCallback(const ????? & msg)
{
  // We received a new message for our sensor. This is where most of the processing happens for our
  // sensor model. We take the published ROS message and transform it into one or more Constraints,
  // and send them to the optimizer.

  // Create a transaction object. This is used to package all of the generated constraints from a
  // given timestamp into one update operation for the optimizer.
  auto transaction = fuse_core::Transaction::make_shared();

  // Each transaction has a timestamp. This is used by the optimizer to determine what order the
  // sensor transactions should be added to the graph. Unless you have a very specific reason not
  // to, the transaction timestamp should be the same as the sensor data timestamp. Or
  // rclcpp::Clock(RCL_SYSTEM_TIME).now() if the sensor data is not stamped.
  transaction->stamp(msg.header.stamp);

  // All of the measured range constraints will involve the robot position at the beacon sensor
  // message timestamp. Construct a robot position variable at that timestamp now.
  auto robot_position = fuse_variables::Position2DStamped::make_shared(msg.header.stamp);

  // The transaction needs to know about all of the involved variables as well as the constraints,
  // so insert the robot position variable now.
  transaction->addVariable(?????);

  // Additionally the transaction needs to know about all of the individual timestamps involved in
  // this transaction. Since not every variable is associated with a timestamp, I could not work out
  // a way to populate this list automatically. The robot pose is the only stamped variable
  // involved, so add that timestamp now as well.
  transaction->addInvolvedStamp(msg.header.stamp);

  // Loop over the pointcloud, extracting the beacon ID, range, and measurement uncertainty for each
  // detected beacon
  for (const auto & beacon_range : ?????) {
    // Create a constraint for this beacon range sensor measurement
    auto constraint = task8::RangeConstraint::make_shared(
      this->name(),
      *robot_position,
      beacon_range.position,
      beacon_range.range,
      beacon_range.sigma);
    transaction->addConstraint(constraint);
  }

  // Send the transaction object to the optimizer
  sendTransaction(transaction);
}

}  // namespace task8
