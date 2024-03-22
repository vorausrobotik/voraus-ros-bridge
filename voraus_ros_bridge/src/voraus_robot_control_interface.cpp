#include "voraus_ros_bridge/voraus_robot_control_interface.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>

VorausRobotControlInterface::VorausRobotControlInterface()
{
    UA_ClientConfig_setDefault(UA_Client_getConfig(ua_client));
    UA_StatusCode status_code = UA_Client_connect(ua_client, "opc.tcp://192.168.142.140:18002");

    if (status_code != UA_STATUSCODE_GOOD)
    {
        throw std::runtime_error("Could not connect to OPC-UA server");
    }
}

VorausRobotControlInterface::~VorausRobotControlInterface()
{
    UA_Client_disconnect(ua_client);
    UA_Client_delete(ua_client);
}

const uint JOINT_POSITIONS_NODE_ID = 100111;
const uint JOINT_VELOCITIES_NODE_ID = 100115;
const uint JOINT_EFFORTS_NODE_ID = 100113;

RobotState VorausRobotControlInterface::get_robot_state()
{
    std::array<UA_ReadValueId, 3> nodes_to_read{};
    for (auto& node : nodes_to_read)
    {
        UA_ReadValueId_init(&node);
        node.attributeId = UA_ATTRIBUTEID_VALUE;
    }
    nodes_to_read.at(0).nodeId = UA_NODEID_NUMERIC(1, JOINT_POSITIONS_NODE_ID);
    nodes_to_read.at(1).nodeId = UA_NODEID_NUMERIC(1, JOINT_VELOCITIES_NODE_ID);
    nodes_to_read.at(2).nodeId = UA_NODEID_NUMERIC(1, JOINT_EFFORTS_NODE_ID);

    UA_ReadRequest request;
    UA_ReadRequest_init(&request);

    request.nodesToRead = nodes_to_read.data();
    request.nodesToReadSize = nodes_to_read.size();

    UA_ReadResponse response = UA_Client_Service_read(ua_client, request);

    if (response.responseHeader.serviceResult != UA_STATUSCODE_GOOD)
    {
        std::ostringstream error_message;
        error_message << "Could not read robot data with error "
                      << UA_StatusCode_name(response.responseHeader.serviceResult);
        throw std::runtime_error(error_message.str());
    }

    auto to_vector = [](UA_Variant& variant) -> std::vector<double> {
        auto* data = static_cast<double*>(variant.data);
        return std::vector<double>(data, data + variant.arrayLength);  // NOLINT
    };

    auto joint_positions = to_vector(response.results[0].value);   // NOLINT
    auto joint_velocities = to_vector(response.results[1].value);  // NOLINT
    auto joint_efforts = to_vector(response.results[2].value);     // NOLINT

    UA_ReadResponse_clear(&response);

    return RobotState{
        joint_positions,
        joint_velocities,
        joint_efforts,
    };
}