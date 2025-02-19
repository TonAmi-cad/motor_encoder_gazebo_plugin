#ifndef GAZEBO_ROS_CONTROL__TRAJECTORY_CALCULATOR_HPP_
#define GAZEBO_ROS_CONTROL__TRAJECTORY_CALCULATOR_HPP_

#include "gazebo_ros_control/turret_control_config.hpp"

namespace gazebo_ros_control
{

class TrajectoryCalculator
{
public:
    TrajectoryCalculator() = default;
    
    static double calculateTrajectory(
        double t,
        double start,
        double end,
        double duration,
        AccelerationProfile profile,
        double s_curve_steepness = 12.0,
        double max_acceleration = 2.0);

private:
    static double quinticPolynomial(double t, double start, double end, double duration);
    static double cubicPolynomial(double t, double start, double end, double duration);
    static double sCurve(double t, double start, double end, double duration, double steepness);
    static double trapezoidalProfile(double t, double start, double end, double duration, double max_acceleration);
};

}  // namespace gazebo_ros_control

#endif  // GAZEBO_ROS_CONTROL__TRAJECTORY_CALCULATOR_HPP_ 