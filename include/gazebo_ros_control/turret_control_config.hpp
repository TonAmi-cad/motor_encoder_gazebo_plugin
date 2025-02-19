#ifndef GAZEBO_ROS_CONTROL__TURRET_CONTROL_CONFIG_HPP_
#define GAZEBO_ROS_CONTROL__TURRET_CONTROL_CONFIG_HPP_

#include <string>

namespace gazebo_ros_control
{

// Добавим перечисление для направления вращения
enum class RotationDirection 
{
    CLOCKWISE,      // Не инвертирует команды
    COUNTERCLOCKWISE // Инвертирует команды
};

enum class AccelerationProfile 
{
  QUINTIC_POLYNOMIAL,
  CUBIC_POLYNOMIAL,
  S_CURVE,
  TRAPEZOIDAL
};

enum class TurretState {
  IDLE,
  MOVING_TO_TARGET,
  HOLDING_POSITION,
  RETURNING_HOME
};

namespace config
{
// Default topic names
static const std::string DEFAULT_COMMAND_TOPIC = "turret_angle";
static const std::string DEFAULT_STATE_TOPIC = "turret_state";

// Motion control parameters
static constexpr double DEFAULT_TRAJECTORY_DURATION = 2.0;  // seconds
static constexpr double DEFAULT_MAX_ACCELERATION = 2.0;     // rad/s^2
static constexpr double DEFAULT_MAX_VELOCITY = 1.0;         // rad/s
static constexpr double DEFAULT_S_CURVE_STEEPNESS = 12.0;
static constexpr double DEFAULT_INITIAL_POSITION = 0.0;     // radians

// Encoder parameters
static constexpr double DEFAULT_ENCODER_RESOLUTION = 0.1;   // degrees

// Control behavior
static constexpr double DEFAULT_HOLD_TIME = 5.0;           // seconds

// Default acceleration profile
static const AccelerationProfile DEFAULT_PROFILE = AccelerationProfile::QUINTIC_POLYNOMIAL;

// Default rotation direction
static const RotationDirection DEFAULT_ROTATION_DIRECTION = RotationDirection::COUNTERCLOCKWISE;

} // namespace config
} // namespace gazebo_ros_control

#endif  // GAZEBO_ROS_CONTROL__TURRET_CONTROL_CONFIG_HPP_ 