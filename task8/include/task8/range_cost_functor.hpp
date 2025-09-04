// Copyright 2025 Locus Robotics

#ifndef TASK8__RANGE_COST_FUNCTOR_HPP_
#define TASK8__RANGE_COST_FUNCTOR_HPP_

#include <ceres/jet.h>


namespace task8
{

/**
 * @brief Implements a range-only measurement cost functor between the robot and a beacon.
 *
 * The main purpose for this cost functor is to demonstrate how to write your own cost functor
 * classes. In addition to this tutorial, any tutorials or other resources relating to Google Ceres
 * Solver cost functions are also applicable, as fuse uses Ceres Solver internally as the
 * optimization engine. The Ceres Solver documentation is available here: http://ceres-solver.org/
 *
 * This file is part of the constraint implementation. It provides a functor that implements the
 * desired sensor mathematical model. This functor will be used with one of my favorite features of
 * Ceres Solver, automatic differentiation. Instead of deriving the Jacobians matrices ourselves,
 * we will allow Ceres Solver to discover them for us. Note that there is a computational cost to
 * this, and implementing the Jacobians yourself will likely be the most computationally efficient
 * method. See more information about derivatives in Ceres Solver here:
 * http://ceres-solver.org/derivatives.html
 *
 * For more details on how this class is used, see the BeaconSensorModel and RangeConstraint
 * classes.
 *
 * Our sensor measurement model involves one variables: the 2D position of the robot. The predicted
 * measurement, generally denoted as z_hat, is simply the Euclidean distance between the robot and
 * the known, fixed beacon position:
 *   z_hat = sqrt( (x_robot - x_beacon)^2 + (y_robot - y_beacon)^2 )
 *
 * The error for our cost function is the difference between real measurement, z, and the predicted
 * measurement, z_hat, normalized by the standard deviation of our sensors measurement error.
 *   error = (z - z_hat) / sigma
 * This is a fairly typical error model formulation.
 *
 * Internally, Ceres Solver will minimize the squared error of all cost functions. Thus the final
 * cost will be of the form:
 *   (Z - Z_hat)^T * S^{-1} * (Z - Z_hat)
 * where S is the measurement covariance matrix.
 */
class RangeCostFunctor
{
public:
  /**
   * @brief Constructor
   *
   * The cost functor needs to know the details of the sensor measurement in order to compute costs
   * for the optimizer. In the case of our range-only sensor, this includes the measured range to
   * the beacon and the measurement uncertainty.
   *
   * @param[in] beacon_x The x position of the beacon
   * @param[in] beacon_y The y position of the beacon
   * @param[in] z The measured range to the beacon
   * @param[in] sigma The standard deviation of the range measurement
   */
  RangeCostFunctor(const double beacon_x, const double beacon_y, const double z, const double sigma)
  : beacon_x_(beacon_x), beacon_y_(beacon_y), sigma_(sigma), z_(z) {}

  /**
   * @brief Compute the costs using the provided stored measurement details and the provided
   *        variable values.
   *
   * Ceres Solver will provide this functor with the current values of the robot position. The
   * functor must then populate the "residuals" vector with the computed costs. You can simply
   * assume the size of all arrays are correct. See the costFunction() implementation of the
   * RangeConstraint class for details on how the sizes and order of the variable are defined.
   *
   * Note the use of the template type T to describe the input and output arrays. This is a
   * requirement when using automatic differentiation. Ceres will call the functor with "jet"
   * datatypes sometimes, and with standard doubles at other time. This means that (a) we must
   * implement our functor using the templated type, T. (b) Any constants that we use when
   * computing the residuals/costs must be wrapped in a T object. And (c) we should use the Ceres
   * Solver version of common math functions, such sin, cos, and sqrt, which have been optimized
   * for use with Ceres Solver's jet datatypes. If you implement the Jacobian matrices manually,
   * none of that is required.
   *
   * @param[in] robot_position - An array of component values (x, y) from the Position2DStamped
   *                             variable representing the position of the robot in the global
   *                             frame.
   * @param[out] residuals - An array of computed cost values. In this case, our residual array is
   *                         only a single value.
   * @return True if the cost was computed successfully, false otherwise
   */
  template<typename T>
  bool operator()(
    const T * const robot_position, T * residuals) const
  {
    // Implement our mathematic measurement model:
    //   z_hat = sqrt( (x_robot - x_beacon)^2 + (y_robot - y_beacon)^2 )
    //   error = (z - z_hat) / sigma
    // Unfortunately there is a problem when computing the derivatives. The derivative is undefined
    // when the radicand is zero. To have a well-defined cost function, we implement an alternative
    // cost equation for that case. Thankfully this does not cause any issues with Ceres Solver's
    // auto-diff system. See http://ceres-solver.org/automatic_derivatives.html#pitfalls
    auto dx = ????? - beacon_x_;
    auto dy = ????? - beacon_y_;
    auto norm_sq = dx * dx + dy * dy;
    if (norm_sq > 0.0) {
      auto z_hat = ceres::sqrt(norm_sq);
      residuals[0] = (T(z_) - z_hat) / T(sigma_);
    } else {
      residuals[0] = T(z_) / T(sigma_);
    }
    return true;
  }

private:
  double beacon_x_;  //!< The X position of the beacon
  double beacon_y_;  //!< The Y position of the beacon
  double sigma_;  //!< The standard deviation of the range measurement
  double z_;  //!< The measured range to the beacon
};

}  // namespace task8

#endif  // TASK8__RANGE_COST_FUNCTOR_HPP_
