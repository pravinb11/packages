//
// Created by hjl on 2022/1/10.
//

#ifndef ROBO_PLANNER_WS_TOPO_GRAPH_H
#define ROBO_PLANNER_WS_TOPO_GRAPH_H

// #include <ros/package.h>
#include "graph/plan_graph.h"
#include "preprocess/viewpoint_manager.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace preprocess {

    class TopoGraph {
    public:
        typedef std::shared_ptr<TopoGraph> Ptr;
        // rclcpp::Node nh_;
        std::shared_ptr<rclcpp::Node> nh_;
        // rclcpp::Node nh_private_;
        std::shared_ptr<rclcpp::Node> nh_private_;

        // ros::Publisher graph_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;

        utils::Point3D current_position_;
        graph::PlanGraph graph_;
        bool is_graph_initialized_;

        std::string frame_id_;
        float sample_dist_;
        float connectable_range_;//the maximum connectable distance of newly added points during the graph exhibition
        float connectable_min_range_;
        int connectable_num_;//maximum number of points that every new viewpoint can connect

        std::string each_graph_update_txt_name_;
        double sum_graph_update_time_;
        int graph_update_num_;
        double sensor_height_;

        // explicit TopoGraph(const rclcpp::Node &nh, const rclcpp::Node &nh_private);
        explicit TopoGraph(const std::shared_ptr<rclcpp::Node> nh, const std::shared_ptr<rclcpp::Node> nh_private);

        void getParamsFromRos();// new graph class and set params.

        void setCurrentPosition(const utils::Point3D &current_position);

        void addVertex(const Point3D &point);

        void updateTopoGraphByMap2DAndViewpoints(const Map2DManager::Ptr &map_2d_manager,const ViewpointManager::Ptr &viewpoint_manager, const Ufomap::Ptr &frontier_map);

        // void initTopoGraphByCurrentPositionAndReprePoints(const GridMap2D &grid_map_2d, const Point3D &current_position, const Point3DSet &repre_points);
        void initTopoGraphByCurrentPositionAndReprePoints(const Map2DManager::Ptr &map_2d_manager, const Point3D &current_position, const Point3DSet &repre_points);

        // void growingByMap2dAndSamplePoints(const GridMap2D &grid_map_2d, const Point3DSet &sample_points);
        void growingByMap2dAndSamplePoints(const Map2DManager::Ptr &map_2d_manager, const Point3DSet &sample_points);

        void pubGraphMarkers() const;

        visualization_msgs::msg::MarkerArray generateTopoGraphMarkers() const;


    };

}

#endif //ROBO_PLANNER_WS_TOPO_GRAPH_H
