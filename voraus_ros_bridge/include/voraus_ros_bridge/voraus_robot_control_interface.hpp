#ifndef VORAUS_ROBOT_CONTROL_INTERFACE_H
#define VORAUS_ROBOT_CONTROL_INTERFACE_H

#include <open62541/client.h>
#include <open62541/client_config_default.h>
#include <array>
#include <vector>

struct RobotState
{
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;
};

class VorausRobotControlInterface
{
   public:
    VorausRobotControlInterface();
    ~VorausRobotControlInterface();

    VorausRobotControlInterface(const VorausRobotControlInterface&) = delete;
    VorausRobotControlInterface& operator=(VorausRobotControlInterface const&) = delete;
    VorausRobotControlInterface(VorausRobotControlInterface&&) = delete;
    VorausRobotControlInterface& operator=(VorausRobotControlInterface&&) = delete;

    RobotState get_robot_state();

   private:
    UA_Client* ua_client = UA_Client_new();
};

#endif  // VORAUS_ROBOT_CONTROL_INTERFACE_h
