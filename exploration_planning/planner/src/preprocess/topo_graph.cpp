//
// Created by hjl on 2022/1/10.
//

#include "preprocess/topo_graph.h"

namespace preprocess
{
    TopoGraph::TopoGraph(const std::shared_ptr<rclcpp::Node> nh, 
                            const std::shared_ptr<rclcpp::Node> nh_private):
        nh_(nh), 
        nh_private_(nh_private), 
        is_graph_initialized_(false)
    {
        getParamsFromRos();

        graph_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("topo_graph_markers", 1);
        // std::cout << "TopoGraph()" << std::endl;
    }

    void TopoGraph::getParamsFromRos()
    {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("planner");
        std::string txt_path = pkg_path + "/../../files/exploration_data/";

        each_graph_update_txt_name_ = txt_path + "each_roadmap_update_time.txt";
        sum_graph_update_time_ = 0;
        graph_update_num_ = 0;

        std::ofstream fout;
        fout.open(each_graph_update_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "each roadmap update elisped time \n"
             << " start time \t" << "  end time \t" << "  elisped time \t"
             << "update num \t" << "average time \t"
             << std::endl;
        fout.close();
		// ====================================================================================================

        // nh_->declare_parameter("frame_id", "");
        nh_->get_parameter("frame_id", frame_id_);

        nh_->declare_parameter("connectable_range", 0.0);
        nh_->get_parameter("connectable_range", connectable_range_);

		nh_->declare_parameter("connectable_num", 0);
        nh_->get_parameter("connectable_num", connectable_num_);

		// nh_->declare_parameter("sample_dist", 0.0);
        nh_->get_parameter("sample_dist", sample_dist_);

		// nh_->declare_parameter("sensor_height", 0.0);
        nh_->get_parameter("sensor_height", sensor_height_);

        // std::cout << "============================== TopoGraph Parameters : START" << std::endl;
        // std::cout << "frame_id_ = " << frame_id_ << std::endl;
        // std::cout << "connectable_range_ = " << connectable_range_ << std::endl;
        // std::cout << "connectable_num_ = " << connectable_num_ << std::endl;
        // std::cout << "sample_dist_ = " << sample_dist_ << std::endl;
        // std::cout << "sensor_height_ = " << sensor_height_ << std::endl;
        // std::cout << "============================== TopoGraph Parameters : END" << std::endl;
    }

    void TopoGraph::setCurrentPosition(const Point3D &current_position)
    {
        current_position_ = current_position;
    }

    void TopoGraph::addVertex(const Point3D &point)
    {
        if(!graph_.isPoint3DExisted(point))
            graph_.addVertex(point);
    }

    void TopoGraph::updateTopoGraphByMap2DAndViewpoints(const Map2DManager::Ptr &map_2d_manager,
                                                        const ViewpointManager::Ptr &viewpoint_manager,
                                                        const Ufomap::Ptr &frontier_map)
    {
        if(map_2d_manager->is_map_updated_)
        {
            auto start_time = nh_->get_clock()->now();
            Point3DSet sample_points;

            // for(double x = -map_2d_manager->gm.getLength().x() / 2; x <= map_2d_manager->gm.getLength().x() / 2; x += sample_dist_)
            //     for(double y = -map_2d_manager->gm.getLength().y() / 2; y <= map_2d_manager->gm.getLength().y() / 2; y += sample_dist_)
            double nlgm_len_x = map_2d_manager->nlgm_gm.getLength().x();
            double nlgm_len_y = map_2d_manager->nlgm_gm.getLength().y();

            for(double x = - nlgm_len_x / 2; x <= nlgm_len_x / 2; x += sample_dist_)
                for(double y = - nlgm_len_y / 2; y <= nlgm_len_y / 2; y += sample_dist_)
                {
                    Point2D sample_2d(current_position_.x() + x, current_position_.y() + y);
                    // std::cout << "SP (" << sample_2d.x() << ", " << sample_2d.y() << ")";
                    bool is_traversable = map_2d_manager->isPointTraversable(sample_2d);
                    bool is_nbr_traversable = map_2d_manager->isNeighborhoodTraversable(sample_2d);

                    if(is_traversable 
                        && is_nbr_traversable)
                    {
                        double sample_2d_elev = map_2d_manager->getPointElevation(sample_2d);
                        double sample_point_z = sample_2d_elev + sensor_height_;
                        Point3D sample_point(sample_2d.x(), sample_2d.y(), sample_point_z);
                        // bool is_in = frontier_map->isInExplorationArea(sample_point.x(),sample_point.y());
                        bool is_in = frontier_map->isInGeofencedVolume(sample_point.x(), sample_point.y(), sample_point_z);
                        // bool is_unk = frontier_map->map_.isUnknown(sample_point.x(), sample_point.y(), sample_point.z(), 0);
                        bool is_free = frontier_map->map_.isFree(sample_point.x(), sample_point.y(), sample_point.z(), 0);
                        // bool is_occ = frontier_map->map_.isOccupied(sample_point.x(), sample_point.y(), sample_point.z(), 0);
                        // std::cout << " T Elevation = " << sample_2d_elev << " In = " << is_in << " UFO = " << is_unk << is_free << is_occ << std::endl;
                        
                        if(is_in 
                            && is_free)
                            sample_points.insert(sample_point);
                    }
                    // else
                    //     std::cout << " NT" << std::endl;
                }
            
            // std::cout << "TG::updateTopoGraphByMap2DAndViewpoints: |sample_points| = " << sample_points.size() << std::endl;
		    // ====================================================================================================

            if(is_graph_initialized_)
                growingByMap2dAndSamplePoints(map_2d_manager, sample_points);
            else
                initTopoGraphByCurrentPositionAndReprePoints(map_2d_manager, current_position_, sample_points);
            
            auto end_time = nh_->get_clock()->now();
            sum_graph_update_time_ += (end_time - start_time).seconds();
            graph_update_num_++;

            std::ofstream fout;
            fout.open(each_graph_update_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
            // fout << start_time << "\t" << end_time << "\t" << (end_time - start_time).toSec()
            fout << start_time.seconds() << "\t" << end_time.seconds() << "\t" << (end_time - start_time).seconds()
                 << "\t" << graph_update_num_ << "\t" << sum_graph_update_time_ / graph_update_num_ << "s \t"
                 << std::endl;
            fout.close();
        }
    }

    void TopoGraph::initTopoGraphByCurrentPositionAndReprePoints(const Map2DManager::Ptr &map_2d_manager,
                                                                 const Point3D &current_position,
                                                                 const Point3DSet &repre_points)
    {
        graph_.clearGraph();

        if(!repre_points.empty())
        {
            addVertex(current_position);
            growingByMap2dAndSamplePoints(map_2d_manager, repre_points);

            is_graph_initialized_ = true;
        }
    }

    void TopoGraph::growingByMap2dAndSamplePoints(const Map2DManager::Ptr &map_2d_manager, const Point3DSet &sample_points)
    {
        Point3DQueue local_points;

        for(const auto &vertex: graph_.getAllVertices())
            if(current_position_.distance(vertex) < 12 + 1)     // 11.3 = root(2) * (GridMapLength / 2)       // (map_2d_manager->gm.getLength().x() * 3 / 2 + connectable_range_)
                local_points.push_back(vertex);

        // std::cout << "TG::growingByMap2dAndSamplePoints: |local_points| = " << local_points.size() << std::endl;
        // ====================================================================================================
        Point3DQueue selected_points = local_points;
        // double min_intervel = sample_dist_;     // 1.0
        uint64_t sampled_vertices_added_count = 0;

        for(const auto &point: sample_points)
        {
            bool suitable = true;

            for(const auto &vertex: selected_points)
                // if(point.distance(vertex) < min_intervel)
                if(point.distance(vertex) < sample_dist_)
                {
                    suitable = false;
                    break;
                }
            
            if(suitable)
            {
                selected_points.push_back(point);
                addVertex(point);
                sampled_vertices_added_count++;
            }
        }

        // std::cout << "TG::growingByMap2dAndSamplePoints: |selected_points| = " << selected_points.size() << std::endl;
        std::cout << "#Vertices: Old = " << local_points.size() << ", New = " << sampled_vertices_added_count << std::endl;
        // ====================================================================================================

        pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
        pcl::PointCloud<pcl::PointXYZ> point_cloud;
        
        for(const auto &point: selected_points)
            point_cloud.emplace_back(point.x(), point.y(), point.z());
        
        kd_tree.setInputCloud(point_cloud.makeShared());

        for(const auto &point: selected_points)
        {
            int point_id = -1;

            if(graph_.getPointId(point, point_id))
            {
                pcl::PointXYZ center(point.x(), point.y(), point.z());
                std::vector<int> neighbors_ids;
                std::vector<float> neighbors_distance;
                kd_tree.radiusSearch(center, connectable_range_, neighbors_ids, neighbors_distance);
                int connected_num = 0;

                for(const auto &id: neighbors_ids)
                {
                    Point3D vertex(point_cloud[id].x, point_cloud[id].y, point_cloud[id].z);
                    
                    if((point_id != id) 
                        && map_2d_manager->isLineTraversable(Point2D(point.x(), point.y()), Point2D(vertex.x(), vertex.y())))
                    {
                        int vertex_id = -1;
                        graph_.getPointId(vertex, vertex_id);
                        graph_.addTwoWayEdge(point_id, vertex_id);
                        connected_num++;

                        if(connected_num > connectable_num_)
                            break;
                    }
                }
            }
        }
    }

    visualization_msgs::msg::MarkerArray TopoGraph::generateTopoGraphMarkers() const
    {
        visualization_msgs::msg::Marker vertices;
        visualization_msgs::msg::Marker edges;

        //init headers
        vertices.header.frame_id = edges.header.frame_id = frame_id_;
        // vertices.header.stamp = edges.header.stamp = rclcpp::Time::now();
        vertices.header.stamp = edges.header.stamp = nh_->get_clock()->now();
        vertices.ns = edges.ns = "graph";
        vertices.action = edges.action = visualization_msgs::msg::Marker::ADD;
        vertices.pose.orientation.w = edges.pose.orientation.w = 1.0;

        //setting id for each marker
        vertices.id = 0;
        edges.id = 1;

        //defining types
        edges.type = visualization_msgs::msg::Marker::LINE_LIST;
        vertices.type = visualization_msgs::msg::Marker::POINTS;

        //setting scale
        edges.scale.x = 0.05;
        edges.scale.y = 0.05;
        edges.scale.z = 0.05;

        vertices.scale.x = 0.1;
        vertices.scale.y = 0.1;
        vertices.scale.z = 0.1;

        //assigning colors
        vertices.color.r = 1.0f;
        vertices.color.g = 0.0f;
        vertices.color.b = 0.0f;

        edges.color.r = 0.1f;
        edges.color.g = 0.5f;
        edges.color.b = 1.0f;

        vertices.color.a = 1.0f;
        edges.color.a = 0.3f;

        //assignment
        int num = 0;
        geometry_msgs::msg::Point point;
        
        for(const auto &node: graph_.getAllVertices())
        {
            point.x = node.x();
            point.y = node.y();
            point.z = node.z();
            vertices.points.push_back(point);
            num++;
        }
        // RCLCPP_INFO(rclcpp::get_logger("Planner"), "the graph vertices num is %i", num);

        int number = 0;
        for (int i = 0; i < graph_.getAllEdges().size(); ++i) {
            for (const auto &end_point: graph_.getAllEdges()[i]) {
                auto s_vp = graph_.getVertex(i);
                auto t_vp = graph_.getVertex(end_point);
                point.x = s_vp.x();
                point.y = s_vp.y();
                point.z = s_vp.z();
                edges.points.push_back(point);
                point.x = t_vp.x();
                point.y = t_vp.y();
                point.z = t_vp.z();
                edges.points.push_back(point);
                number++;
            }

        }
        
        std::cout << "Graph: #Vertices = " << num << ", #Edges = " << number << std::endl;

        visualization_msgs::msg::MarkerArray graph_markers;
        graph_markers.markers.resize(2);
        graph_markers.markers[0] = vertices;
        graph_markers.markers[1] = edges;

        return graph_markers;
    }

    void TopoGraph::pubGraphMarkers() const
    {
        graph_pub_->publish(generateTopoGraphMarkers());
    }
}