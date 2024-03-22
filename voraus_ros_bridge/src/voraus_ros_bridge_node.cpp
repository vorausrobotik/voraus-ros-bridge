#include "voraus_ros_bridge/voraus_ros_bridge_node.hpp"
#include "voraus_ros_bridge/voraus_robot_control_interface.hpp"

VorausRosBridgeNode::VorausRosBridgeNode() : Node("voraus_ros_bridge")
{
    joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 5);

    joint_state_publisher_timer = this->create_wall_timer(std::chrono::milliseconds(10),
                                                          [this] { on_publish_joint_states(); });
}

void VorausRosBridgeNode::on_publish_joint_states()
{
    const RobotState robot_state = robot_control.get_robot_state();

    sensor_msgs::msg::JointState msg{};

    msg.header.stamp = this->get_clock()->now();
    msg.name = std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    msg.position = robot_state.joint_positions;
    msg.velocity = robot_state.joint_velocities;
    msg.effort = robot_state.joint_efforts;

    joint_state_publisher->publish(msg);
}