#ifndef GAZEBO_ROS_CONTROL__ANGLE_UTILS_HPP_
#define GAZEBO_ROS_CONTROL__ANGLE_UTILS_HPP_

#include "gazebo_ros_control/turret_control_config.hpp"

namespace gazebo_ros_control
{

class AngleUtils
{
public:
    static double normalizeAngle(double angle);
    static double quantizeAngle(double angle, double resolution);
    static double applyRotationDirection(double angle, RotationDirection direction);
    static double calculateShortestPath(double current, double target);
    static double radiansToDegrees(double radians);
    static double degreesToRadians(double degrees);
};

}  // namespace gazebo_ros_control

#endif  // GAZEBO_ROS_CONTROL__ANGLE_UTILS_HPP_ 