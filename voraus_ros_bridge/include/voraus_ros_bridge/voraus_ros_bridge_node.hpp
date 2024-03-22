#ifndef VORAUS_ROS_BRIDGE_NODE_H
#define VORAUS_ROS_BRIDGE_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "voraus_ros_bridge/voraus_robot_control_interface.hpp"

class VorausRosBridgeNode : public rclcpp::Node
{
   public:
    VorausRosBridgeNode();

   private:
    VorausRobotControlInterface robot_control;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher{};
    rclcpp::TimerBase::SharedPtr joint_state_publisher_timer;

    void on_publish_joint_states();
};

#endif  // VORAUS_ROS_BRIDGE_NODE_H
