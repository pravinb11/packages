//
// Created by hjl on 2022/1/10.
//

#ifndef ROBO_PLANNER_WS_PREPROCESS_H
#define ROBO_PLANNER_WS_PREPROCESS_H

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "perception/map_3d.h"
#include "perception/ufomap.h"
#include "perception/map_2d_manager.h"
#include "preprocess/viewpoint_manager.h"
#include "preprocess/topo_graph.h"

namespace preprocess
{
    class Preprocess
    {
    public:
        typedef std::shared_ptr<Preprocess> Ptr;

        // rclcpp::Node nh_;
        std::shared_ptr<rclcpp::Node> nh_;
        // rclcpp::Node nh_private_;
        std::shared_ptr<rclcpp::Node> nh_private_;

        //params
        std::string frame_id_;
        std::string robot_base_frame_id_;

        //odometry
        // ros::Subscriber odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        utils::Point3D current_position_;
        geometry_msgs::msg::Pose current_pose_;

        utils::Point3D last_directory_position_;
        utils::Point3D forward_directory_;

        // ros::Subscriber explorer_init_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr explorer_init_sub_;
        bool is_explorer_initilized_;
        std::string odometry_topic_name_;

        //dynamic updating
        perception::Ufomap::Ptr frontier_map_;
        perception::Map2DManager::Ptr map_2d_manager_;
        preprocess::ViewpointManager::Ptr viewpoint_manager_;
        preprocess::TopoGraph::Ptr road_map_;

        //mutex
        std::mutex elements_update_mutex_;

        Preprocess(const std::shared_ptr<rclcpp::Node> nh, 
                    const std::shared_ptr<rclcpp::Node> nh_private):
            nh_(nh), 
            nh_private_(nh_private), 
            is_explorer_initilized_(false)
        {
            getParamsFromRos();
            
            odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_name_, 1, std::bind(&Preprocess::odomCallback, this, std::placeholders::_1));
            // explorer_init_sub_ = nh_->create_subscription<std_msgs::msg::Float64>("explorer_inited", 1, std::bind(&Preprocess::explorerInitCallback, this, std::placeholders::_1));
            auto latched_qos = rclcpp::QoS(1).transient_local();
            explorer_init_sub_ = nh_->create_subscription<std_msgs::msg::Float64>(
                "explorer_inited",
                latched_qos,
                std::bind(&Preprocess::explorerInitCallback, this, std::placeholders::_1)
            );
            init(nh, nh_private);
            // std::cout << "Preprocess()" << std::endl;
        }

        void getParamsFromRos()
        {
            nh_->declare_parameter("frame_id", "");
            nh_->get_parameter("frame_id", frame_id_);
            
            nh_->declare_parameter("robot_base_frame_id", "");
            nh_->get_parameter("robot_base_frame_id", robot_base_frame_id_);

            nh_->declare_parameter("odometry_topic_name", "");
            nh_->get_parameter("odometry_topic_name", odometry_topic_name_);

            // std::cout << "============================== Preprocess Parameters : START" << std::endl;
            // std::cout << "frame_id_ = " << frame_id_ << std::endl;
            // std::cout << "robot_base_frame_id_ = " << robot_base_frame_id_ << std::endl;
            // std::cout << "odometry_topic_name_ = " << odometry_topic_name_ << std::endl;
            // std::cout << "============================== Preprocess Parameters : END" << std::endl;
        }

        void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
        {
            current_pose_ = odom->pose.pose;
            current_position_.x() = current_pose_.position.x;
            current_position_.y() = current_pose_.position.y;
            current_position_.z() = current_pose_.position.z;

            viewpoint_manager_->setCurrentPosition(current_position_);
            road_map_->setCurrentPosition(current_position_);

            if(current_position_.distanceXY(last_directory_position_) > 0.5)
            {
                forward_directory_ = current_position_ - last_directory_position_;
                last_directory_position_ = current_position_;
            }
        }

        void explorerInitCallback(const std_msgs::msg::Float64::ConstSharedPtr &msg)
        {
            is_explorer_initilized_ = true;
        }

        void init(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> nh_private)
        {
            frontier_map_ = std::make_shared<perception::Ufomap>(nh, nh_private);
            map_2d_manager_ = std::make_shared<perception::Map2DManager>(nh, nh_private);
            viewpoint_manager_ = std::make_shared<preprocess::ViewpointManager>(nh, nh_private, frontier_map_, map_2d_manager_);
            road_map_ = std::make_shared<preprocess::TopoGraph>(nh, nh_private);
        }

        void updateElements()
        {

            if(is_explorer_initilized_ )
            {
                viewpoint_manager_->updateViewpoints();
                road_map_->updateTopoGraphByMap2DAndViewpoints(map_2d_manager_,viewpoint_manager_,frontier_map_);
            }
            
        }
    };
}

#endif //ROBO_PLANNER_WS_PREPROCESS_H