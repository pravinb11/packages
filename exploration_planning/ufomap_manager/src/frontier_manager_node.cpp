//
// Created by hjl on 2020/6/19.
//
#include "rclcpp/rclcpp.hpp"
#include <ufomap_manager/frontier_manager.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("frontier_manager");
    // rclcpp::Node nh_private("~");
    // ufomap_manager::FrontierManager frontier(nh,nh_private);
    ufomap_manager::FrontierManager frontier(nh);

    // rclcpp::spin(node);
    rclcpp::spin(nh);
    return 0;
}
