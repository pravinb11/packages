//
// Created by hjl on 2021/12/9.
//

#include <explorer/explorer.h>
#include <unordered_map>
#include <control_planner_interface/pci_vehicle.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("explorer_node");
    explorer::Explorer explorer(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
