#ifndef GAZEBO_ROS_CONTROL__ACTION_SERVER_HPP_
#define GAZEBO_ROS_CONTROL__ACTION_SERVER_HPP_

#include <rclcpp_action/rclcpp_action.hpp>
#include "gazebo_ros_control/action/turret_control.hpp"
#include <memory>
#include <functional>

namespace gazebo_ros_control
{

class TurretActionServer
{
public:
    using TurretAction = gazebo_ros_control::action::TurretControl;
    using GoalHandleTurret = rclcpp_action::ServerGoalHandle<TurretAction>;

    TurretActionServer(
        rclcpp::Node::SharedPtr node,
        const std::string& action_name,
        std::function<double()> get_current_angle_cb,
        std::function<void(double)> set_target_position_cb);
    
    ~TurretActionServer() = default;

    void initialize();
    bool hasActiveGoal() const { return has_active_goal_; }
    void setHasActiveGoal(bool value) { has_active_goal_ = value; }

private:
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const TurretAction::Goal> goal);

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleTurret> goal_handle);

    void handleAccepted(const std::shared_ptr<GoalHandleTurret> goal_handle);
    void executeGoal(const std::shared_ptr<GoalHandleTurret> goal_handle);

    rclcpp::Node::SharedPtr node_;
    std::string action_name_;
    rclcpp_action::Server<TurretAction>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleTurret> current_goal_handle_;
    bool has_active_goal_;

    std::function<double()> get_current_angle_;
    std::function<void(double)> set_target_position_;
};

}  // namespace gazebo_ros_control

#endif  // GAZEBO_ROS_CONTROL__ACTION_SERVER_HPP_ 