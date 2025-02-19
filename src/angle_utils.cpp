#include "gazebo_ros_control/angle_utils.hpp"
#include <cmath>

namespace gazebo_ros_control
{

double AngleUtils::normalizeAngle(double angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle;
}

double AngleUtils::quantizeAngle(double angle, double resolution)
{
    double degrees = radiansToDegrees(angle);
    double quantized_degrees = std::round(degrees / resolution) * resolution;
    return degreesToRadians(quantized_degrees);
}

double AngleUtils::applyRotationDirection(double angle, RotationDirection direction)
{
    return (direction == RotationDirection::COUNTERCLOCKWISE) ? -angle : angle;
}

double AngleUtils::calculateShortestPath(double current, double target)
{
    double diff = target - current;
    if (diff > M_PI) {
        diff -= 2 * M_PI;
    } else if (diff < -M_PI) {
        diff += 2 * M_PI;
    }
    return diff;
}

double AngleUtils::radiansToDegrees(double radians)
{
    return radians * 180.0 / M_PI;
}

double AngleUtils::degreesToRadians(double degrees)
{
    return degrees * M_PI / 180.0;
}

}  // namespace gazebo_ros_control 