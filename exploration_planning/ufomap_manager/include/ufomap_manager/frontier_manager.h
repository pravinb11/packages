//
// Created by hjl on 2020/6/17.
//

#ifndef UFOMAP_WITH_FRONTIER_MANAGER_H
#define UFOMAP_WITH_FRONTIER_MANAGER_H

#include <mutex>
#include "rclcpp/rclcpp.hpp"
// #include <ros/package.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <ufo/map/occupancy_map.h>
// #include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/msg/ufo_map_stamped.hpp>
#include <ufo/map/key.h>
#include <ufo/map/code.h>
// #include <ufomap_manager/UfomapWithFrontiers.h>
#include <ufomap_manager/msg/ufomap_with_frontiers.hpp>
// #include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/msg/ufo_map_stamped.hpp>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace ufomap_manager {

    typedef std::unordered_set<ufo::map::Code, ufo::map::Code::Hash> CodeUnorderSet;

    class FrontierManager {
    public:

        // FrontierManager(const rclcpp::Node &nh, const rclcpp::Node &nh_private);
        // FrontierManager(const std::shared_ptr<rclcpp::Node> nh, const std::shared_ptr<rclcpp::Node> nh_private);
        FrontierManager(const std::shared_ptr<rclcpp::Node> nh);

        // map update

        void
        pointCloudOdomCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan, const nav_msgs::msg::Odometry::ConstSharedPtr &odom);

        // void ufomapPublishTimer(const rclcpp::TimerEvent &event);
        void ufomapPublishTimer();

        // void writeUfomapCallback(const rclcpp::TimerEvent &event);
        void writeUfomapCallback();

        void setParametersFromROS();

        ufo::map::OccupancyMap const &getUFOMap() const { return map_; };

        // Frontiers manager

        // void frontierCallback(const rclcpp::TimerEvent &event);
        void frontierCallback();

        void frontierSearch();

        void findLocalFrontier();

        void updateGlobalFrontier();

        void findPlaneLocalFrontier();

        void updatePlaneGlobalFrontier();

        CodeUnorderSet get_3D_NeighborCell(const ufo::map::Code &cell_code, unsigned int depth);

        CodeUnorderSet get_XY_NeighborCell(const ufo::map::Code &cell_code, unsigned int depth) const;

        CodeUnorderSet get_Z_NeighborCell(const ufo::map::Code &cell_code, unsigned int depth) const;

        bool isFrontier(const ufo::map::Code &frontier) const;

        // statistic

        void knownCellOutput();

        std::size_t getKnownNodeNum(const CodeUnorderSet &knownCellCodes);

        std::size_t getKnownPlaneNodeNum(const CodeUnorderSet &knownCellCodes);

        // pub msgs
        void generateMarkerArray(const std::string &tf_frame, visualization_msgs::msg::MarkerArray *frontier_cells,
                                 CodeUnorderSet &frontier_cell_codes, std_msgs::msg::ColorRGBA &rgba);

    private:
        // rclcpp::Node nh_;
        std::shared_ptr<rclcpp::Node> nh_;
        // rclcpp::Node nh_private_;
        std::shared_ptr<rclcpp::Node> nh_private_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> SyncPolicyLocalCloudOdom;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> point_cloud_sub_;
        std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
        typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyLocalCloudOdom>> SynchronizerLocalCloudOdom;
        SynchronizerLocalCloudOdom sync_point_cloud_odom_;

        // ros::Publisher map_pub_;
        rclcpp::Publisher<ufomap_msgs::msg::UFOMapStamped>::SharedPtr map_pub_;
        // ros::Publisher cloud_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
        // ros::Publisher map_and_frontiers_pub_;
        rclcpp::Publisher<ufomap_manager::msg::UfomapWithFrontiers>::SharedPtr map_and_frontiers_pub_;
        // ros::Publisher local_frontiers_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_frontiers_pub_;
        // ros::Publisher global_frontiers_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_frontiers_pub_;

        // rclcpp::Timer pub_timer_;
        rclcpp::TimerBase::SharedPtr pub_timer_;
        // rclcpp::Timer frontier_timer_;
        rclcpp::TimerBase::SharedPtr frontier_timer_;
        // rclcpp::Timer write_ufomap_timer_;
        rclcpp::TimerBase::SharedPtr write_ufomap_timer_;

        // TF2
        // tf2_ros::Buffer tf_buffer_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        // tf2_ros::TransformListener tf_listener_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Duration transform_timeout_;   

        // map
        ufo::map::OccupancyMap map_;
        std::string frame_id_;

        // position
        ufo::math::Pose6 current_robot_pose_;  
        ufo::math::Pose6 current_sensor_pose_;  

        // param
        std::string sensor_frame_id_;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;  
        float scan_voxel_size_;  

        // Integration
        double max_range_;  
        bool insert_discrete_;  
        int insert_depth_;
        bool simple_ray_casting_;   
        int early_stopping_;

        // Clear robot
        bool clear_robot_enabled_;    
        std::string robot_base_frame_id_;
        double robot_height_;     
        double robot_bottom_;      
        double robot_radius_;    
        int clearing_depth_;

        double pub_rate_;      

        // frontiers
        int frontier_depth_;
        CodeUnorderSet global_frontier_cells_;
        CodeUnorderSet local_frontier_cells_;

        //use code to index frontier point
        CodeUnorderSet changed_cell_codes_;  
        CodeUnorderSet known_cell_codes_;  
        std::size_t known_cell_num_;
        std::size_t known_plane_cell_num_;

        std::mutex map_mutex_;
        std::mutex odom_mutex_;

    protected:
        //statistic
        std::string map_txt_name;

        std::ofstream fout;
        std::string txt_known_name;
        // ros::WallTime start_time;
        rclcpp::Time start_time;

        std::string txt_frontier_name;
        double sum_frontier_time;
        int frontier_iteration;

        std::string txt_insert_cloud_name;
        double sum_insert_time;
        int insert_num;
    };

}


#endif //UFOMAP_WITH_FRONTIER_MANAGER_H
