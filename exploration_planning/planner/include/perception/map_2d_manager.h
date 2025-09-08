//
// Created by hjl on 2022/1/11.
//

#ifndef ROBO_PLANNER_WS_MAP_2D_MANAGER_H
#define ROBO_PLANNER_WS_MAP_2D_MANAGER_H

#include "rclcpp/rclcpp.hpp"
// #include <tf/tf.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common.h>

#include "grid_map_2d.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <traversability_analysis/TerrainMap.h>

#include "perception/ufomap.h"

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

using namespace grid_map;

namespace perception{

    struct TerrainMap{
        std::string frame_id;
        double min_x;
        double min_y;
        double max_x;
        double max_y;
        double z_value;
        double grid_size;
        int grid_width_num;
        std::vector<unsigned int> status;
        pcl::PointCloud<pcl::PointXYZI> bottom_points;  

        void clear(){
            status.clear();
        };

        inline bool isInTerrainMap(const Point2D &point) const {
            if (point.x() < min_x + 1e-4 || point.x() > max_x - 1e-4 ||
                point.y() < min_y + 1e-4 || point.y() > max_y - 1e-4) {
                return false;
            } else {
                return true;
            }
        };

        inline int getGridID(const Point2D &point) const {
            return floor((point.x() - min_x) / grid_size) * grid_width_num + floor((point.y() - min_y) / grid_size);
        };
    };

    class Map2DManager {
    public:
        typedef std::shared_ptr<Map2DManager> Ptr;

        // rclcpp::Node nh_;
        std::shared_ptr<rclcpp::Node> nh_;
        // rclcpp::Node nh_private_;
        std::shared_ptr<rclcpp::Node> nh_private_;

        // ros::Subscriber odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        // ros::Subscriber terrain_map_sub_;
        // rclcpp::Subscription<traversability_analysis::msg::TerrainMap>::SharedPtr terrain_map_sub_;

        // ros::Publisher grid_map_2d_pub_;
        // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_map_2d_pub_;

        // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> SyncPolicyLocalCloud;
        // std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> terrain_cloud_sub_;
        // std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> local_cloud_sub_;
        // std::shared_ptr<message_filters::Synchronizer<SyncPolicyLocalCloud>> sync_terrain_local_cloud_;
        // pcl::PointCloud<pcl::PointXYZI> terrain_cloud_;
        // pcl::PointCloud<pcl::PointXYZI> local_cloud_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        // tf2_ros::TransformListener tf_listener_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


        // GridMap2D inflate_map_;
        // GridMap2D map_;  
        geometry_msgs::msg::Pose current_pose_;
        bool is_map_updated_;

        std::string frame_id_;
        double grid_size_;
        // double inflate_radius_;
        // double inflate_empty_radius_;
        // double lower_z_;  
        // double connectivity_thre_;

        std::mutex map_2d_update_mutex_;

        // rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr emf_sub;
        // GridMap gm;
        // void emf_cb(const grid_map_msgs::msg::GridMap::ConstSharedPtr &msg);

        // typedef message_filters::sync_policies::ApproximateTime<grid_map_msgs::msg::GridMap, grid_map_msgs::msg::GridMap> SyncPolicyGridMap;
        // std::shared_ptr<message_filters::Subscriber<grid_map_msgs::msg::GridMap>> fgmg_sub;
        // std::shared_ptr<message_filters::Subscriber<grid_map_msgs::msg::GridMap>> nlgm_sub;
        // std::shared_ptr<message_filters::Synchronizer<SyncPolicyGridMap>> fgmg_nlgm_syn;

        // void terrainLocalCloudCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr &terrain_cloud, const grid_map_msgs::msg::GridMap::ConstSharedPtr &local_cloud);

        rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr fgmg_sub;
        rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr nlgm_sub;
        void fgmg_cb(const grid_map_msgs::msg::GridMap::ConstSharedPtr &msg);
        void nlgm_cb(const grid_map_msgs::msg::GridMap::ConstSharedPtr &msg);
        GridMap fgmg_gm;
        GridMap nlgm_gm;
        double obs_th_;
        double robot_footprint_;
        std::string odometry_topic_name_;
        std::string traversability_topic_name_;
        std::string elevation_topic_name_;
        std::string traversability_layer_name_;
        std::string elevation_layer_name_;
        
        Map2DManager(const std::shared_ptr<rclcpp::Node> nh, const std::shared_ptr<rclcpp::Node> nh_private);

        void getParamsFromRos();

        void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom);

        void terrainLocalCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &terrain_cloud,
                                       const sensor_msgs::msg::PointCloud2::ConstSharedPtr &local_cloud);

        // void terrainMapCallback(const traversability_analysis::TerrainMapConstPtr &terrain_map);

        void updateGridMap2D(const pcl::PointCloud<pcl::PointXYZI> &terrain_cloud,
                             const pcl::PointCloud<pcl::PointXYZI> &local_cloud);

        void updateGridMap2D(const TerrainMap &terrain_map);

        bool isPointTraversable(Point2D p2d);
        bool isGridTraversable(Index i);
        bool isLineTraversable(Point2D s, Point2D d);
        double getPointElevation(Point2D p2d);
        bool isSubmapTraversable(Point2D p2d, double d);
        bool isNeighborhoodTraversable(Point2D p2d);
        std::vector<Point2D> a_star(Point2D goal, Point2D start);
        std::vector<Point2D> smoothen_path(std::vector<Point2D> path);
        // ViewpointQueue getGridCenters();
    };

}



#endif //ROBO_PLANNER_WS_MAP_2D_MANAGER_H
