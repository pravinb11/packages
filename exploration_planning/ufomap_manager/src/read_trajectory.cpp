//
// Created by hjl on 2021/9/1.
//


#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("read_trajectory");
    rclcpp::Node nh_private("~");

    // auto pub = nh_private.advertise<nav_msgs::msg::Odometry>("trajectory_odom", 1);
    auto pub = nh_private.create_publisher<nav_msgs::msg::Odometry>("trajectory_odom", 1);
    // auto pub_point = nh_private.advertise<sensor_msgs::msg::PointCloud2>("trajectory_points",1);
    auto pub_point = nh_private.create_publisher<sensor_msgs::msg::PointCloud2>("trajectory_points",1);

    // std::string ns = ros::this_node::getName();
    std::string ns = nh->get_name();
    std::string txt_name = "tracjectory.txt";
    // if (!ros::param::get(ns + "/txt_name", txt_name)) {
    if (!nh->get_parameter(ns + "/txt_name", txt_name)) {
        RCLCPP_WARN(rclcpp::get_logger("UfomapManager"), 
                "No txt_name specified. Looking for %s. Default is 'tracjectory.txt'.",
                (ns + "/txt_name").c_str());
    }

    nav_msgs::msg::Odometry robot_odom;
    sensor_msgs::msg::PointCloud2  point_cloud;
    robot_odom.header.frame_id = "world";
    point_cloud.header.frame_id = "world";

    string line;
    ifstream fin;
    fin.open(txt_name,ios::in);
    if(!fin.is_open()){
        std::cout<<"file open failed"<<std::endl;
    }
    getline(fin,line);

    RCLCPP_INFO(rclcpp::get_logger("UfomapManager"), "start pub trajectory..");
    int seq = 1;
    while(getline(fin,line)){
        int i=0;
        string tmp;
        stringstream ss(line);
        while (getline(ss, tmp, '\t')) {
            if (i == 1) robot_odom.pose.pose.position.x = stod(tmp);
            if (i == 2) robot_odom.pose.pose.position.y = stod(tmp);
            if (i == 3) robot_odom.pose.pose.position.z = stod(tmp);
            i++;
        }

        // robot_odom.header.seq = seq;
        // pub.publish(robot_odom);
        pub->publish(robot_odom);
        // rclcpp::spin_some(node);
        rclcpp::spin_some(nh);
        // rclcpp::Duration(0.005).sleep();
        rclcpp::sleep_for(std::chrono::milliseconds(5));
        seq++;
    }


    RCLCPP_INFO(rclcpp::get_logger("UfomapManager"), "pub finish");

    return 0;
}