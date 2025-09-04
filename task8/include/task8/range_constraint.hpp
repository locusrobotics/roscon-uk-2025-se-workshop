// Copyright 2025 Locus Robotics

#ifndef TASK8__RANGE_CONSTRAINT_HPP_
#define TASK8__RANGE_CONSTRAINT_HPP_

#include <ceres/cost_function.h>

#include <ostream>
#include <string>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <fuse_core/constraint.hpp>
#include <fuse_core/fuse_macros.hpp>
#include <fuse_variables/position_2d_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>


namespace task8
{
/**
 * @brief Implements a range-only measurement constraint between the robot and a fixed, known
 * beacon position
 *
 * The main purpose for this constraint is to demonstrate how to write your own cost constraint
 * classes.
 *
 * For more details on how this class is used, see the BeaconSensorModel class.
 *
 * This range-only constraint will use the mathematical model defined in the RangeCostFunctor, and
 * Ceres Solver's automatic differentiation system to create a Ceres Solver cost function.
 */
class RangeConstraint : public fuse_core::Constraint
{
public:
  // There are some boilerplate types and functions that must be implemented for each constraint,
  // such as shared pointer typedefs and clone() methods. These are formulaic, but the derived type
  // is needed for their proper implementation. A few different macro options are provided to make
  // implementing this boilerplate code easy.
  FUSE_CONSTRAINT_DEFINITIONS(RangeConstraint)

  /**
   * @brief Default constructor
   *
   * A default constructor is required to support the serialize/deserialize functionality.
   */
  RangeConstraint() = default;

  /**
   * @brief Create a range-only constraint between the provided robot position and the fixed
   * beacon position
   *
   * This is the constructor that will be used from within the BeaconSensorModel. It accepts
   * references to the variable involved with this specific measurement -- the robot position at
   * the time the measurement was sampled. We also receive the fixed beacon position, which is
   * NOT added to the graph as a variable
   *
   * This constraint has only a one dimension in the measurement, which is why z and sigma are just
   * doubles. In the general case, you'd want to use Eigen types for both.
   *
   * @param[in] source - The name of the sensor that generated this constraint. This is largely
   *                     information to aid in debugging or visualizing the system. If multiple
   *                     sensors of the same type exist, being able to disambiguate the constraints
   *                     from sensor1 versus sensor2 is useful.
   * @param[in] robot_position - The 2D position of the robot at the time the measurement was
   *                             sampled (a variable in the graph)
   * @param[in] beacon_position - The known, fixed 2D position of the sampled beacon
   * @param[in] z - The distance measured between the robot and beacon by our new sensor
   * @param[in] sigma - The uncertainty of measured distance
   */
  RangeConstraint(
    const std::string & source,
    const ????? & robot_position,
    const geometry_msgs::msg::Point & beacon_position,
    const double z,
    const double sigma);

  /**
   * @brief Print a human-readable description of the constraint to the provided stream.
   *
   * This is required by the fuse_core::Constraint base class, but is largely just for debugging and
   * visualization.
   *
   * @param[out] stream - The stream to write to. Defaults to stdout.
   */
  void print(std::ostream & stream = std::cout) const override;

  /**
   * @brief Construct an instance of this constraint's cost function
   *
   * This is the most important operation of a Constraint -- to create a Ceres Solver CostFunction
   * object that is then optimized by the Ceres Solver least-squares solver. This implementation
   * uses the RangeCostFunctor and the Ceres Solver AutoDiffCostFunction class to automatically
   * compute the cost function Jacobians. This is also where the sizes of the input variables and
   * output cost array is defined.
   *
   * @return A base pointer to an instance of a derived Ceres Solver CostFunction. Ownership of the
   *         CostFunction object is transferred Ceres Solver; Ceres Solver will delete the
   *         CostFunction object when it is done. Also note that the fuse project guarantees the
   *         derived Constraint object will outlive the Ceres Solver CostFunction object.
   */
  ceres::CostFunction * costFunction() const override;

private:
  // Allow Boost Serialization access to private methods and members
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the
   *        archive
   *
   * Implementing the serialize() function allows the derived Constraint object to be written to
   * disk, and retrieved again at a later date. In particular, the Graph classes make use of this
   * feature, allowing a full graph to be saved to disk and recalled later. This is extraordinarily
   * useful in debugging and replaying. It is highly recommended that the serialize() method be
   * implemented properly. And for most things, it is trivial to implement. See the Boost
   * Serialization documentation for more details:
   * https://www.boost.org/doc/libs/1_77_0/libs/serialization/doc/index.html
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template<class Archive>
  void serialize(Archive & archive, const unsigned int /* version */)
  {
    archive & boost::serialization::base_object<fuse_core::Constraint>(*this);
    archive & beacon_position_.x;
    archive & beacon_position_.y;
    archive & sigma_;
    archive & z_;
  }

  geometry_msgs::msg::Point beacon_position_;  //!< The known, fixed position of the beacon
  double sigma_ {0.0};    //!< The standard deviation of the range measurement
  double z_ {0.0};    //!< The measured range to the beacon
};

}  // namespace task8

// This is part of the serialization requirement. Boost needs to be told this class is serializable.
BOOST_CLASS_EXPORT_KEY(task8::RangeConstraint);

#endif  // TASK8__RANGE_CONSTRAINT_HPP_
