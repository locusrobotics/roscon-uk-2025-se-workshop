// Copyright 2025 Locus Robotics

#ifndef TASK8__BEACON_SENSOR_MODEL_HPP_
#define TASK8__BEACON_SENSOR_MODEL_HPP_

#include <string>

#include <fuse_core/async_sensor_model.hpp>
#include <fuse_core/uuid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <workshop_msgs/msg/beacon_range_array.hpp>


namespace task8
{
/**
 * @brief Implements a range-only sensor model that generates constraints between a given robot
 * variable and any visible beacons (which have fixed, known positions).
 *
 * The main purpose for this sensor model is to demonstrate how to write your own sensor model
 * classes.
 *
 * For the purposes of this tutorial, let's imagine that you have developed a new robotic sensor
 * that receives messages from beacons that are placed around a warehouse at regular intervals.
 * The beacons are aware of their global positions in the map frame, and send their own global
 * position to the robot's sensor. The robot's sensor is sensitive to range, so readings that
 * come from farther away have noisier range readings. The sensor driver is aware of this and
 * publishes certainty (in the form of variance) in the message. The message is of type
 * workshop_msgs/msg/BeaconRangeArray. It contains a vector of workshop_msgs/msg/BeaconRange
 * messages. Each message has the following fields:
 *  - "position", geometry_msgs/Point - The beacon's fixed, known map-frame pose
 *  - "range", float64 - The sensor's noisy measured range to the beacon
 *  - "sigma", float64 - The standard deviation of the measurement, in metres
 *
 * The "sensor model" class provides an interface to ROS, allowing sensor messages to be received.
 * The sensor model class also acts as a "factory" (in a programming sense) that creates new sensor
 * constraints for each received sensor measurement, and forwards those constraints to the fuse
 * optimizer. The optimizer is where the constraints from all configured sensors are combined, and
 * the best possible value for each state variable is determined.
 *
 * Each fuse SensorModel is implemented as a plugin, which is loaded by the optimizer at runtime.
 * This allows new sensor models to be implemented outside of the main fuse package, such as in
 * this task8 package. The fuse SensorModel base class defines a few basic methods for
 * communicating between the derived SensorModel and the optimizer.
 *  - initialize()
 *    This is called by the optimizer after construction. This is a common pattern used by plugins.
 *    This is often when the Parameter Server is queried for configuration data.
 *  - start()
 *    Tells the sensor model to start producing constraints. This is commonly where fuse sensor
 *    models first subscribe to their sensor data topics.
 *  - stop()
 *    Tells the sensor model to stop producing constraints. fuse sensor models typically unsubscribe
 *    from topics here.
 *  - graphCallback()
 *    The optimizer provides the sensor model with the latest set of optimized states. For simple
 *    sensor models, this is likely not needed. But something maintaining a database of visual
 *    landmarks or similar may need access to the current set of landmarks.
 *  - TransactionCallback
 *    This is a little different than the other interfaces. This is a callback provided *to* the
 *    SensorModel plugin. That callback is executed by the plugin whenever new constraints are ready
 *    to be sent to the optimizer.
 *
 * An issue with the SensorModel base class is a rather complex threading model, where
 * TransactionCallbacks can be executed at any time by any of the sensor model plugins. To make
 * derived sensor models easier to implement, the AsyncSensorModel base class is provided, which
 * hides most of the thread synchronization details. The AsyncSensorModel class provides its own
 * callback queue and spinner, making it act much like a typical, single-threaded ROS node. All of
 * the original base SensorModel methods are wrapped. Instead, slightly modified versions are
 * provided, which are executed within the AsyncSensorModel's spinner thread.
 *  - onInit() can be overridden instead of initialize()
 *  - onStart() can be overridden instead of start()
 *  - onStop() can be overridden instead of stop()
 *  - onGraphUpdate() can be overridden instead of graphCallback()
 *  - sendTransaction() can be called instead of TransactionCallback()
 *
 * Additionally, the AsyncSensorModel base class provides a public and private node handle that are
 * pre-configured for use with the spinner's thread and callback queue. These should be used instead
 * of creating your own, in much the same way that nodelets provide methods for accessing properly
 * configured node handles.
 *
 * All of the sensor models provided by the fuse_models package use the AsyncSensorModel base class,
 * and it is the recommended way to start developing new sensor models.
 */
class BeaconSensorModel : public fuse_core::AsyncSensorModel
{
public:
  // It is convenient to have some typedefs for various smart pointer types (shared, unique, etc.).
  // A macro is provided to make it easy to define these typedefs and ensures that the naming is
  // consistent throughout all fuse packages.
  FUSE_SMART_PTR_DEFINITIONS(BeaconSensorModel)

  /**
   * @brief Default constructor
   *
   * A default constructor is required by pluginlib. The real initialization of the sensor model
   * will occur in the onInit() method. This will be called immediately after construction by the
   * optimizer node. We do, however, specify the number of threads to use to spin the callback
   * queue. Generally this will be 1, unless you have a good reason to use a multi-threaded spinner.
   */
  BeaconSensorModel()
  : fuse_core::AsyncSensorModel(1), logger_(rclcpp::get_logger("uninitialized")) {}

  /**
   * @brief Shadowing extension to the AsyncSensorModel::initialize call
   */
  void initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string & name,
    fuse_core::TransactionCallback transaction_callback) override;

  /**
   * @brief Callback for range measurement messages
   *
   * In a real scenario, we would likely model the beacons themselves as variables in our graph.
   * For the sake of simplicity, we assume that the beacon positions are known with extreme
   * precision a priori, such that we can treat their positions as ground truth.
   *
   * We will process all of the detected beacons in the input message, generate one or more
   * RangeConstraint objects, and send all of the constraints to the optimizer at once packaged in a
   * Transaction object.
   *
   * @param[in] msg - The range message to process
   */
  void rangesCallback(const ????? & msg);

protected:
  /**
   * @brief Perform any required initialization for the sensor model
   *
   * This could include things like reading from the parameter server or subscribing to topics. The
   * class's node handles will be properly initialized before AsyncSensorModel::onInit() is called.
   * Spinning of the callback queue will not begin until after the call to
   * AsyncSensorModel::onInit() completes.
   */
  void onInit() override;

  /**
   * @brief Subscribe to the input topic to start sending transactions to the optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe from the input topic to stop sending transactions to the optimizer
   */
  void onStop() override;

  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Waitables
  > interfaces_;  //!< Shadows AsyncSensorModel interfaces_

  rclcpp::Logger logger_;  //!< The sensor model's logger

  //!< ROS subscription for the beacon range sensor measurements
  rclcpp::Subscription<?????>::SharedPtr sub_;
};

}  // namespace task8

#endif  // TASK8__BEACON_SENSOR_MODEL_HPP_
