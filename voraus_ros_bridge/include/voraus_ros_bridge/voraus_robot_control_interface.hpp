#ifndef VORAUS_ROBOT_CONTROL_INTERFACE_H
#define VORAUS_ROBOT_CONTROL_INTERFACE_H

#include <open62541pp/open62541pp.h>
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

    void connect();

    void disconnect();

    RobotState get_robot_state();

   private:
    opcua::Client rc_client;
};

#endif  // VORAUS_ROBOT_CONTROL_INTERFACE_h
