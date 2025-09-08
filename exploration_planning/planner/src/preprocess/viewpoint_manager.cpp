//
// Created by hjl on 2022/1/10.
//

#include "preprocess/viewpoint_manager.h"

namespace preprocess
{
    ViewpointManager::ViewpointManager(const std::shared_ptr<rclcpp::Node> nh, 
                                        const std::shared_ptr<rclcpp::Node> nh_private, 
                                        const Ufomap::Ptr &frontier_map, 
                                        const Map2DManager::Ptr &map_2d_manager_):
        nh_(nh), 
        nh_private_(nh_private), 
        frontier_map_(frontier_map), 
        map_2d_manager_(map_2d_manager_)
    {
        getParamsFromRos();

        repre_points_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("repre_points_markers", 1);
        frontiers_viewpoints_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers_viewpoints_markers", 1);
        exploration_data_finish_pub_ = nh_->create_publisher<std_msgs::msg::Bool>("exploration_data_finish", 1);

        // std::cout << "ViewpointManager()" << std::endl;
    }

    void ViewpointManager::getParamsFromRos()
    {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("planner");
        std::string txt_path = pkg_path + "/../../files/exploration_data/";

        frontiers_attach_txt_name_ = txt_path + "frontier_attach_time.txt";
        attach_num_ = 0;
        sum_attach_time_ = 0;

        fout_.open(frontiers_attach_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout_ << "each frontier attach time \n"
              << "start time \t" << " end time \t" << " elisped time \t" << "iteration_num \t" << "average time(s) \t"
              << std::endl;
        fout_.close();
        // ====================================================================================================

		// nh_->declare_parameter("frame_id", "");
        nh_->get_parameter("frame_id", frame_id_);

		nh_->declare_parameter("sample_dist", 0.0);
        nh_->get_parameter("sample_dist", sample_dist_);

		// nh_->declare_parameter("frontier_dist", 0.0);
        // nh_->get_parameter("frontier_dist", frontier_dist_);

		nh_->declare_parameter("viewpoint_gain_thre", 0.0);
        nh_->get_parameter("viewpoint_gain_thre", viewpoint_gain_thre_);        

        // std::cout << "============================== ViewpointManager Parameters : START" << std::endl;
        // std::cout << "frame_id_ = " << frame_id_ << std::endl;
        // std::cout << "sample_dist_ = " << sample_dist_ << std::endl;
        // std::cout << "frontier_dist_ = " << frontier_dist_ << std::endl;
        // std::cout << "viewpoint_gain_thre_ = " << viewpoint_gain_thre_ << std::endl;
        // std::cout << "============================== ViewpointManager Parameters : END" << std::endl;
    }

    void ViewpointManager::setCurrentPosition(const utils::Point3D &current_position)
    {
        current_position_ = current_position;
    }

    void ViewpointManager::updateViewpoints()
    {
        if(map_2d_manager_->is_map_updated_)
        {
            auto start_time = nh_->get_clock()->now();
            frontierAttachInUfomap();
            auto end_time = nh_->get_clock()->now();
            sum_attach_time_ += (end_time - start_time).seconds();
            attach_num_++;

            fout_.open(frontiers_attach_txt_name_, std::ios_base::in | std::ios_base::out | std::ios_base::app);
            fout_ << start_time.seconds() << "\t" << end_time.seconds() << "\t" << (end_time - start_time).seconds()
                  << "\t" << attach_num_ << "\t" << sum_attach_time_ / attach_num_ << "s \t"
                  << std::endl;
            fout_.close();

            std::cout << "#ViewPoints = " << representative_points_.size();
            std::cout << ", F -> VP = " << frontiers_viewpoints_.size();
            std::cout << ", VP -> Fs = " << viewpoints_attached_frontiers_.size() << std::endl;

            if(!viewpoints_attached_frontiers_.size())      // Exploration Complete
            {
                std_msgs::msg::Bool bool_msg;
                bool_msg.data = true;
                exploration_data_finish_pub_->publish(bool_msg);
            }
        }
    }

    


    void ViewpointManager::frontierAttachInUfomap()
    {
        Point3DQueue sample_points = samplePointsInGridMap2D();
        representative_points_.clear();

        for(const auto &point: sample_points)
		{
            bool is_suitable = true;

            for(const auto &viewpoint: candidate_viewpoints_)
                // if(point.distance(viewpoint) < 1.0)
                if(point.distance(viewpoint) < sample_dist_)
				{
                    is_suitable = false;
                    break;
                }
			
            if(is_suitable)
                representative_points_.insert(point);
        }

        representative_points_.insert(candidate_viewpoints_.begin(), candidate_viewpoints_.end());
		// std::cout << "VM::frontierAttachInUfomap: |representative_points_| = " << representative_points_.size() << std::endl;
		// ====================================================================================================

        FrontierMap<Viewpoint> frontiers_viewpoints_term;
        // uint f_g_id = 0;

        for(const auto &frontier: frontier_map_->getFrontiers())
		{
			// double dist_to_rob = frontier.distanceXY(current_position_);
			bool is_frontiers_viewpoints_empty = frontiers_viewpoints_.empty();
			bool frontiers_viewpoints_count = frontiers_viewpoints_.count(frontier);
			// bool is_submap_traversable = map_2d_manager_->isSubmapTraversable(Point2D(frontier.x(), frontier.y()), frontier_dist_);

			// std::cout << "f_g " << f_g_id++ << " (" << frontier.x() << ", " << frontier.y() << ", " << frontier.z() << ") dist_to_rob = " << dist_to_rob << ", is_frontiers_viewpoints_empty = " << is_frontiers_viewpoints_empty << ", frontiers_viewpoints_count = " << frontiers_viewpoints_count << ", is_submap_traversable = " << is_submap_traversable << std::endl;
            
			// if(dist_to_rob > frontier_map_->max_range_ * 1.5 
            //     && !is_frontiers_viewpoints_empty
            if(!is_frontiers_viewpoints_empty
				&& frontiers_viewpoints_count)
                frontiers_viewpoints_term[frontier] = frontiers_viewpoints_[frontier];
			else //if(is_submap_traversable)
			{
                double min_distance = 100000;
                ufo::map::Point3 frontier_coord(frontier.x(), frontier.y(), frontier.z());
                ufo::map::Point3 sensor_point;
                double distance;

                for(const auto &point: representative_points_)
				{
                    sensor_point.x() = point.x();
                    sensor_point.y() = point.y();
                    sensor_point.z() = point.z();
                    distance = frontier_coord.distanceXY(sensor_point);
					bool is_obstructed = frontier_map_->isInCollision(sensor_point, frontier_coord);

					// std::cout << "\tdistance = " << distance << ", min_distance = " << min_distance << ", is_obstructed = " << is_obstructed << std::endl;
					// std::cout << "\tvp (" << point.x() << ", " << point.y() << ", " << point.z() << ") distance = " << distance << ", min_distance = " << min_distance << ", is_obstructed = " << is_obstructed << std::endl;


                    if(distance < frontier_map_->max_range_		// - 0.5 
						&& min_distance > distance 
                        // && fabs(frontier_coord.z() - sensor_point.z()) / distance < tan(M_PI * 15 / 180)
                        && !is_obstructed)
					{
                        min_distance = distance;
                        frontiers_viewpoints_term[frontier] = point;
                    }
                }

				// std::cout << "\t\tf -> vp = (" << frontiers_viewpoints_term[frontier].x() << ", " << frontiers_viewpoints_term[frontier].y() << ", " << frontiers_viewpoints_term[frontier].z() << ")" << std::endl;
            }
        }

        frontiers_viewpoints_.clear();
        frontiers_viewpoints_ = frontiers_viewpoints_term;
		// std::cout << "VM::frontierAttachInUfomap: |frontiers_viewpoints_| = " << frontiers_viewpoints_.size() << std::endl;
		// ====================================================================================================

        viewpoints_attached_frontiers_.clear();

        for(const auto &item: frontiers_viewpoints_)
            viewpoints_attached_frontiers_[item.second].push_back(item.first);

        // std::cout << "VM::frontierAttachInUfomap: |viewpoints_attached_frontiers_| = " << viewpoints_attached_frontiers_.size() << std::endl;

        Point3DQueue need_to_erase;
        for(const auto &term:viewpoints_attached_frontiers_)
		{
			// double dist_to_rob = term.first.distanceXY(current_position_);

            // if(dist_to_rob < sample_dist_ / 2)
			// {
            //     need_to_erase.push_back(term.first);

            //     for(const auto &frontier:term.second)
			// 	{
            //         auto code = frontier_map_->map_.toCode(frontier.x(), frontier.y(), frontier.z(), frontier_map_->frontier_depth_);
            //         frontier_map_->global_frontier_cells_.erase(code);
            //     }
            // }

            if(term.second.size() < viewpoint_gain_thre_)
                need_to_erase.push_back(term.first);
        }

		// std::cout << "VM::frontierAttachInUfomap: |need_to_erase| = " << need_to_erase.size() << std::endl;

        for(const auto &point:need_to_erase)
            viewpoints_attached_frontiers_.erase(point);

		// std::cout << "VM::frontierAttachInUfomap: |viewpoints_attached_frontiers_| = " << viewpoints_attached_frontiers_.size() << std::endl;
		// ====================================================================================================

        Point3DSet old_viewpoints = candidate_viewpoints_;
        candidate_viewpoints_.clear();

        for(const auto &item: viewpoints_attached_frontiers_)
            candidate_viewpoints_.insert(item.first);

		// std::cout << "VM::frontierAttachInUfomap: |candidate_viewpoints_| = " << candidate_viewpoints_.size() << std::endl;
		// ====================================================================================================

        new_viewpoints_.clear();

        for(const auto &point: candidate_viewpoints_)
            if(!old_viewpoints.count(point))
                new_viewpoints_.insert(point);

		// std::cout << "VM::frontierAttachInUfomap: |new_viewpoints_| = " << new_viewpoints_.size() << std::endl;
    }

    ViewpointQueue ViewpointManager::samplePointsInGridMap2D()
    {
        ViewpointQueue sample_points;

        // for(double x = -map_2d_manager_->gm.getLength().x() / 2; x <= map_2d_manager_->gm.getLength().x() / 2; x += sample_dist_)
        //     for(double y = -map_2d_manager_->gm.getLength().y() / 2; y <= map_2d_manager_->gm.getLength().y() / 2; y += sample_dist_)
        double nlgm_gm_len_x = map_2d_manager_->nlgm_gm.getLength().x();
        double nlgm_gm_len_y = map_2d_manager_->nlgm_gm.getLength().y();

        for(double x = -nlgm_gm_len_x / 2; x <= nlgm_gm_len_x / 2; x += sample_dist_)
            for(double y = -nlgm_gm_len_y / 2; y <= nlgm_gm_len_y / 2; y += sample_dist_)
            {
                Point2D sample_2d(current_position_.x() + x, current_position_.y() + y);
                double sample_2d_z = map_2d_manager_->getPointElevation(sample_2d);
                double sample_3d_z = sample_2d_z + frontier_map_->sensor_height_;
                // bool is_inside_boundary = frontier_map_->isInExplorationArea(sample_2d.x(), sample_2d.y());
                bool is_inside_boundary = frontier_map_->isInGeofencedVolume(sample_2d.x(), sample_2d.y(), sample_3d_z);
                bool is_traversable = map_2d_manager_->isPointTraversable(sample_2d);
                bool is_nbr_traversable = map_2d_manager_->isNeighborhoodTraversable(sample_2d);
                bool is_free = frontier_map_->map_.isFree(sample_2d.x(), sample_2d.y(), sample_3d_z, 0);
                // double dist_to_rob = (Point2D(current_position_.x(), current_position_.y()) - sample_2d).norm();
                // std::cout << "S (" << sample_2d.x() << ", " << sample_2d.y() << ", " << sample_2d_z << ") is_inside_boundary = " << is_inside_boundary << ", is_traversable = " << is_traversable << ", dist_to_rob = " << dist_to_rob << std::endl;

                if(is_inside_boundary 
                    && is_traversable 
                    && is_nbr_traversable 
                    // && (dist_to_rob < frontier_map_->max_range_))
                    // && map_2d_manager_->isLineTraversable(Point2D(current_position_.x(), current_position_.y()), sample_2d)
                    && is_free)
                {
                    Point3D sample_point(sample_2d.x(), sample_2d.y(), sample_3d_z);
                    sample_points.push_back(sample_point);
                }
            }
        
		// std::cout << "VM::samplePointsInGridMap2D: |sample_points| = " << sample_points.size() << std::endl;

        return sample_points;
    }

    visualization_msgs::msg::MarkerArray ViewpointManager::generatePointsMarkers(const Point3DSet &sample_points) const
    {
        visualization_msgs::msg::Marker points;
        points.header.frame_id = frame_id_;
        points.header.stamp = nh_->get_clock()->now();
        points.ns = "representative_points";
        points.action = visualization_msgs::msg::Marker::ADD;
        points.pose.orientation.w = 1.0;

        points.id = 0;
        points.type = visualization_msgs::msg::Marker::SPHERE_LIST;

        points.scale.x = 0.25;       // 0.3
        points.scale.y = 0.25;
        points.scale.z = 0.25;

        points.color.r = 0.5f;
        points.color.g = 0.1f;
        points.color.b = 1.0f;
        points.color.a = 0.5;       // 1.0f

        for(auto node: sample_points)
		{
            geometry_msgs::msg::Point point;
            point.x = node.x();
            point.y = node.y();
            point.z = node.z();
            points.points.push_back(point);
        }
		
        visualization_msgs::msg::MarkerArray repre_points_markers;
        repre_points_markers.markers.resize(1);
        repre_points_markers.markers[0] = points;

        return repre_points_markers;
    }

    visualization_msgs::msg::MarkerArray ViewpointManager::generateViewPointsWithFrontiersMarkers(const ViewpointMap<FrontierQueue> &viewpoints_attached_frontiers) const
    {
        visualization_msgs::msg::Marker points;
        visualization_msgs::msg::Marker edges;
        visualization_msgs::msg::Marker squares;

        edges.header.frame_id = points.header.frame_id = squares.header.frame_id = frame_id_;
        edges.header.stamp = points.header.stamp = squares.header.stamp = nh_->get_clock()->now();
        edges.ns = "connections";
        points.ns = "viewpoints";
        squares.ns = "frontiers";
        edges.action = points.action = squares.action = visualization_msgs::msg::Marker::ADD;
        edges.pose.orientation.w = points.pose.orientation.w = squares.pose.orientation.w = 1.0;

        points.id = 0;
        points.type = visualization_msgs::msg::Marker::SPHERE_LIST;

        edges.id = 1;
        edges.type = visualization_msgs::msg::Marker::LINE_LIST;

        squares.id = 2;
        squares.type = visualization_msgs::msg::Marker::CUBE_LIST;

        points.scale.x = 0.25;
        points.scale.y = 0.25;
        points.scale.z = 0.25;

        //assigning colors
        points.color.r = 1.0f;
        points.color.g = 0.5f;
        points.color.b = 1.0f;
        points.color.a = 1.0f;

        //setting scale
        edges.scale.x = 0.05;        // 0.1
        edges.scale.y = 0.05;
        edges.scale.z = 0.05;

        edges.color.r = 0.5f;
        edges.color.g = 0.5f;
        edges.color.b = 0.1f;
        edges.color.a = 0.5f;

        squares.scale.x = frontier_map_->map_.getResolution();
        squares.scale.y = frontier_map_->map_.getResolution();
        squares.scale.z = frontier_map_->map_.getResolution();

        //assigning colors
        squares.color.r = 0.0;     // 0.1f
        squares.color.g = 1.0;      // 0.9f
        squares.color.b = 0.0;      // 0.1f
        squares.color.a = 0.5;

        for (const auto &node: viewpoints_attached_frontiers) {
            geometry_msgs::msg::Point point;
            point.x = node.first.x();
            point.y = node.first.y();
            point.z = node.first.z();
            points.points.push_back(point);
        }
        for (const auto &edge: viewpoints_attached_frontiers) {
            auto first = edge.first;
            for (const auto &second:edge.second) {
                geometry_msgs::msg::Point point;
                point.x = first.x();
                point.y = first.y();
                point.z = first.z();
                edges.points.push_back(point);
                point.x = second.x();
                point.y = second.y();
                point.z = second.z();
                edges.points.push_back(point);
                squares.points.push_back(point);
            }
        }

        for (const auto &node: viewpoints_attached_frontiers) {
            geometry_msgs::msg::Point point;
            point.x = node.first.x();
            point.y = node.first.y();
            point.z = node.first.z();
            points.points.push_back(point);
        }
        for (const auto &edge: viewpoints_attached_frontiers) {
            auto first = edge.first;
            for (const auto &second:edge.second) {
                geometry_msgs::msg::Point point;
                point.x = first.x();
                point.y = first.y();
                point.z = first.z();
                edges.points.push_back(point);
                point.x = second.x();
                point.y = second.y();
                point.z = second.z();
                edges.points.push_back(point);
                squares.points.push_back(point);
            }
        }

        visualization_msgs::msg::MarkerArray frontiers_viewpoints_markers;
        frontiers_viewpoints_markers.markers.resize(3);
        frontiers_viewpoints_markers.markers[0] = points;
        frontiers_viewpoints_markers.markers[1] = edges;
        frontiers_viewpoints_markers.markers[2] = squares;

        return frontiers_viewpoints_markers;
    }

    void ViewpointManager::pubMarkers() const
	{
        repre_points_pub_->publish(generatePointsMarkers(representative_points_));
        frontiers_viewpoints_pub_->publish(generateViewPointsWithFrontiersMarkers(viewpoints_attached_frontiers_));
    }
}