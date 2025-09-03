// Copyright 2025 Locus Robotics

#include "task8/range_constraint.hpp"

#include <ceres/autodiff_cost_function.h>

#include <ostream>
#include <string>

#include <boost/serialization/export.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "task8/range_cost_functor.hpp"

namespace task8
{

// The Constraint base class holds the list of involved Variable UUIDs. When constructing a new
// RangeConstraint object, the base class constructor must be provided with the list of variable
// UUIDs. Note that the order the variables are added to the list is important. Later, when Ceres
// Solver uses the CostFunction function to minimize the total error, it will provide access to the
// variables *in the same order we provide them to the base class constructor*. This means that the
// variable order defined in the RangeCostFunctor must match the variable order provided to the base
// class Constraint constructor. In this case, robot position, then the beacon position
// fuse_core::Constraint(source, { robot_position.uuid(), beacon_position.uuid() })
RangeConstraint::RangeConstraint(
  const std::string & source,
  const ????? & robot_position,
  const geometry_msgs::msg::Point & beacon_position,
  const double z,
  const double sigma)
: fuse_core::Constraint(source, {robot_position.uuid()}),  // NOLINT
  beacon_position_(beacon_position),
  sigma_(sigma),
  z_(z)
{
}

void RangeConstraint::print(std::ostream & stream) const
{
  stream << type() << "\n"
         << "  source: " << source() << "\n"
         << "  uuid: " << uuid() << "\n"
         << "  robot position variable: " << variables().at(0) << "\n"
         << "  beacon position: " << beacon_position_.x << ", " << beacon_position_.y << "\n"
         << "  range measurement: " << z_ << "\n"
         << "  range sigma: " << sigma_ << "\n";
}

ceres::CostFunction * RangeConstraint::costFunction() const
{
  // Here we use the Ceres Solver AutoDiffCostFunction class to generate a derived CostFunction
  // object from our RangeCostFunctor. The AutoDiffCostFunction requires the cost functor as the
  // first template parameter. The following template parameters provide size information:
  //   2nd: The size of the output residuals array of the cost functor. Our functor only computes a
  //        single distance error, so the size is 1.
  //   3rd: The size of the first (and only) involved variable. This the robot position (x, y), so
  //        the size is 2.
  // If there were additional involved variables, the size of each variable would appear here in
  // order.
  return new ceres::AutoDiffCostFunction<RangeCostFunctor, ?????, ?????>(
    new RangeCostFunctor(beacon_position_.x, beacon_position_.y, z_, sigma_));
}

}  // namespace task8

// This is part of the serialization requirement. Boost needs to be told this class is serializable.
BOOST_CLASS_EXPORT_IMPLEMENT(task8::RangeConstraint);
// Additionally we tell pluginlib about this class. This makes it loadable at runtime, if needed.
PLUGINLIB_EXPORT_CLASS(task8::RangeConstraint, fuse_core::Constraint);
