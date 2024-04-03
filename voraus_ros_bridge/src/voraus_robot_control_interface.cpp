#include "voraus_ros_bridge/voraus_robot_control_interface.hpp"

const opcua::NodeId JOINT_POSITIONS_NODE_ID = opcua::NodeId(1, 100111);
const opcua::NodeId JOINT_VELOCITIES_NODE_ID = opcua::NodeId(1, 100115);
const opcua::NodeId JOINT_EFFORTS_NODE_ID = opcua::NodeId(1, 100113);

const auto RC_CLIENT_URL = "opc.tcp://192.168.142.140:17002";

VorausRobotControlInterface::VorausRobotControlInterface()
{
    rc_client.connect(RC_CLIENT_URL);
}

RobotState VorausRobotControlInterface::get_robot_state()
{
    opcua::Node joint_positions_node = rc_client.getNode(JOINT_POSITIONS_NODE_ID);
    opcua::Node joint_velocities_node = rc_client.getNode(JOINT_VELOCITIES_NODE_ID);
    opcua::Node joint_efforts_node = rc_client.getNode(JOINT_EFFORTS_NODE_ID);

    auto joint_positions = joint_positions_node.readValueArray<double>();
    auto joint_velocities = joint_positions_node.readValueArray<double>();
    auto joint_efforts = joint_positions_node.readValueArray<double>();

    return RobotState{
        joint_positions,
        joint_velocities,
        joint_efforts,
    };
}