//
// Created by hjl on 2020/6/15.
//
#include "rclcpp/rclcpp.hpp"
#include <ufomap_manager/ufomap_manager.h>


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("ufomap_manager");
    // rclcpp::Node nh_private("~");
    // ufomap_manager::UFOMapManager manager(nh,nh_private);
    ufomap_manager::UFOMapManager manager(nh);

    // rclcpp::spin(node);
    rclcpp::spin(nh);
    return 0;
}