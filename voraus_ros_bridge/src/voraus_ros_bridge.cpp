#include "voraus_ros_bridge/voraus_ros_bridge_node.hpp"

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VorausRosBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
