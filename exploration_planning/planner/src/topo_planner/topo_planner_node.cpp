//
// Created by hjl on 2021/11/16.
//

#include "rclcpp/rclcpp.hpp"

#include "topo_planner/topo_planner.h"


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> nh = std::make_shared<rclcpp::Node>("topo_planner_node");
    std::shared_ptr<rclcpp::Node> nh_private;

    topo_planner::TopoPlanner planner(nh, nh_private);
    rclcpp::spin(nh);

    return 0;
}