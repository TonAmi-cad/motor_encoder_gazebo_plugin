#ifndef GAZEBO_ROS_CONTROL__ACCELERATION_PROFILES_HPP_
#define GAZEBO_ROS_CONTROL__ACCELERATION_PROFILES_HPP_

namespace gazebo_ros_control
{

/**
 * @brief Generates a quintic polynomial trajectory
 * @param t Current time
 * @param start Start position
 * @param end End position
 * @param duration Total duration of trajectory
 * @return Position at time t
 */
double QuinticPolynomial(double t, double start, double end, double duration);

/**
 * @brief Generates a cubic polynomial trajectory
 * @param t Current time
 * @param start Start position
 * @param end End position
 * @param duration Total duration of trajectory
 * @return Position at time t
 */
double CubicPolynomial(double t, double start, double end, double duration);

/**
 * @brief Generates an S-curve trajectory
 * @param t Current time
 * @param start Start position
 * @param end End position
 * @param duration Total duration of trajectory
 * @param steepness S-curve steepness parameter
 * @return Position at time t
 */
double SCurve(double t, double start, double end, double duration, double steepness);

/**
 * @brief Generates a trapezoidal velocity profile trajectory
 * @param t Current time
 * @param start Start position
 * @param end End position
 * @param duration Total duration of trajectory
 * @param max_acceleration Maximum allowed acceleration
 * @return Position at time t
 */
double TrapezoidalProfile(double t, double start, double end, double duration, double max_acceleration);

}  // namespace gazebo_ros_control

#endif  // GAZEBO_ROS_CONTROL__ACCELERATION_PROFILES_HPP_ 