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
    // auto nh_private = rclcpp::Node::make_shared("ufomap_manager_2");
    // ufomap_manager::UFOMapManager manager(nh,nh_private);
    ufomap_manager::UFOMapManager manager(nh);
    // auto manager = std::make_shared<ufomap_manager::UFOMapManager>(nh);

    rclcpp::spin(node);
    return 0;
}