//
// Created by hjl on 2022/1/11.
//

#include "perception/map_2d_manager.h"

namespace perception
{
    Map2DManager::Map2DManager(const std::shared_ptr<rclcpp::Node> nh, 
                                const std::shared_ptr<rclcpp::Node> nh_private): 
        nh_(nh), 
        nh_private_(nh_private), 
        is_map_updated_(false)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        getParamsFromRos();
        
        // grid_map_2d_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("gird_map_2d", 1);

        odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_name_, 1, std::bind(&Map2DManager::odomCallback, this, std::placeholders::_1));
        // emf_sub = nh_->create_subscription<grid_map_msgs::msg::GridMap>("elevation_map_filter", 1, std::bind(&Map2DManager::emf_cb, this, std::placeholders::_1));

        fgmg_sub = nh_->create_subscription<grid_map_msgs::msg::GridMap>(traversability_topic_name_, 1, std::bind(&Map2DManager::fgmg_cb, this, std::placeholders::_1));
        nlgm_sub = nh_->create_subscription<grid_map_msgs::msg::GridMap>(elevation_topic_name_, 1, std::bind(&Map2DManager::nlgm_cb, this, std::placeholders::_1));

        // std::cout << "Map2DManager()" << std::endl;
    }

    void Map2DManager::getParamsFromRos()
    {
		// nh_->declare_parameter("frame_id", "");
        nh_->get_parameter("frame_id", frame_id_);

		nh_->declare_parameter("obs_th", 0.0);
        nh_->get_parameter("obs_th", obs_th_);

		nh_->declare_parameter("robot_footprint", 0.0);
        nh_->get_parameter("robot_footprint", robot_footprint_);

        // nh_->declare_parameter("odometry_topic_name", "");
        nh_->get_parameter("odometry_topic_name", odometry_topic_name_);

        nh_->declare_parameter("traversability_topic_name", "");
        nh_->get_parameter("traversability_topic_name", traversability_topic_name_);

        nh_->declare_parameter("elevation_topic_name", "");
        nh_->get_parameter("elevation_topic_name", elevation_topic_name_);

        nh_->declare_parameter("traversability_layer_name", "");
        nh_->get_parameter("traversability_layer_name", traversability_layer_name_);

        nh_->declare_parameter("elevation_layer_name", "");
        nh_->get_parameter("elevation_layer_name", elevation_layer_name_);

        // std::cout << "============================== Map2DManager Parameters : START" << std::endl;
        // std::cout << "frame_id_ = " << frame_id_ << std::endl;
        // std::cout << "obs_th_ = " << obs_th_ << std::endl;
        // std::cout << "robot_footprint_ = " << robot_footprint_ << std::endl;
        // std::cout << "odometry_topic_name_ = " << odometry_topic_name_ << std::endl;
        // std::cout << "traversability_topic_name_ = " << traversability_topic_name_ << std::endl;
        // std::cout << "elevation_topic_name_ = " << elevation_topic_name_ << std::endl;
        // std::cout << "traversability_layer_name_ = " << traversability_layer_name_ << std::endl;
        // std::cout << "elevation_layer_name_ = " << elevation_layer_name_ << std::endl;
        // std::cout << "============================== Map2DManager Parameters : END" << std::endl;
    }

    void Map2DManager::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
    {
        current_pose_ = odom->pose.pose;
    }

    // void Map2DManager::emf_cb(const grid_map_msgs::msg::GridMap::ConstSharedPtr &msg)
    // {
    //     GridMapRosConverter::fromMessage(*msg, gm);
    //     is_map_updated_ = true;
        // std::cout << "Map2DManager::emf_cb: |gm| = " << gm.getSize()(0) << " x " << gm.getSize()(1) << " (" << gm.getResolution() << ")" << std::endl;

        // ================================================== TEST
        // GridMap gm_tmp;
        // gm_tmp.setGeometry(Length(5, 5), 1, Position(0, 0));

        // for(GridMapIterator it(gm_tmp); !it.isPastEnd(); ++it)
        // {
        //     Position p;
        //     gm_tmp.getPosition(*it, p);

        //     Index i;
        //     gm_tmp.getIndex(p, i);

        //     std::cout << "LI = " << it.getLinearIndex() << " I (" << i.x() << ", " << i.y() << ") P (" << p.x() << ", " << p.y() << ")" << std::endl;
        // }

        // Position p_min(-1, -1);
        // Index i_min;
        // gm_tmp.getIndex(p_min, i_min);
        // std::cout << "i_min = " << i_min.x() << ", " << i_min.y() << std::endl;

        // Position p_max(1, 1);
        // Index i_max;
        // gm_tmp.getIndex(p_max, i_max);
        // std::cout << "i_max = " << i_max.x() << ", " << i_max.y() << std::endl;

        // Index i_diff(i_max.x() - i_min.x() + 1, i_max.y() - i_min.y() + 1);
        // Index i_diff(i_min.x() - i_max.x() + 1, i_min.y() - i_max.y() + 1);
        // std::cout << "i_diff = " << i_diff.x() << ", " << i_diff.y() << std::endl;

        // for(SubmapIterator it(gm_tmp, i_min, i_diff); !it.isPastEnd(); ++it)
        // for(SubmapIterator it(gm_tmp, i_max, i_diff); !it.isPastEnd(); ++it)
        // {
        //     Position p;
        //     gm_tmp.getPosition(*it, p);

        //     Index i;
        //     gm_tmp.getIndex(p, i);

        //     std::cout << i.x() << ", " << i.y() << std::endl;
        // }

        // rclcpp::shutdown();
    // }

    void Map2DManager::fgmg_cb(const grid_map_msgs::msg::GridMap::ConstSharedPtr &gm)
    {
        GridMapRosConverter::fromMessage(*gm, fgmg_gm);
        grid_size_ = fgmg_gm.getResolution();
        is_map_updated_ = true;
    }

    void Map2DManager::nlgm_cb(const grid_map_msgs::msg::GridMap::ConstSharedPtr &gm)
    {
        GridMapRosConverter::fromMessage(*gm, nlgm_gm);
        grid_size_ = fgmg_gm.getResolution();
        is_map_updated_ = true;
    }

    bool Map2DManager::isPointTraversable(Point2D p2d)
    {
        Position p(p2d.x(), p2d.y());
        // std::cout << p.x() << ", " << p.y() << " = ";

        // if(!gm.isInside(p))
        if(!fgmg_gm.isInside(p))
        {
            // std::cout << "Outside";
            return false;
        }

        Index i;
        // gm.getIndex(p, i);
        fgmg_gm.getIndex(p, i);

        // if(!gm.isValid(i, "robot_traversability"))
        if(!fgmg_gm.isValid(i, traversability_layer_name_))
        {
            // std::cout << "Invalid";
            return false;
        }

        double cell_val = fgmg_gm.at(traversability_layer_name_, i);
        // std::cout << p.x() << ", " << p.y() << " = " << cell_val << std::endl;

        // if(gm.at("robot_traversability", i) == 2)   // 2 = Nontraversable
        if(cell_val < obs_th_)
            return true;
        else
            return false;
    }

    bool Map2DManager::isGridTraversable(Index i)
    {
        // if(!((i.x() >= 0) && (i.x() < gm.getSize()(0)) && (i.y() >= 0) && (i.y() < gm.getSize()(1))))
        if(!((i.x() >= 0) && (i.x() < fgmg_gm.getSize()(0)) && (i.y() >= 0) && (i.y() < fgmg_gm.getSize()(1))))
        {
            // std::cout << "Outside";
            return false;
        }

        // if(!gm.isValid(i, "robot_traversability"))
        if(!fgmg_gm.isValid(i, traversability_layer_name_))
        {
            // std::cout << "Invalid";
            return false;
        }

        // if(gm.at("robot_traversability", i) == 2)   // 2 = Nontraversable
        if(fgmg_gm.at(traversability_layer_name_, i) < obs_th_)
            return true;
        else
            return false;
    }
    
    bool Map2DManager::isLineTraversable(Point2D s, Point2D d)
    {
        Position pos_s(s.x(), s.y());

        // if(!gm.isInside(pos_s))
        if(!fgmg_gm.isInside(pos_s))
        {
            // std::cout << "Outside";
            return false;
        }

        Position pos_d(d.x(), d.y());

        // if(!gm.isInside(pos_d))
        if(!fgmg_gm.isInside(pos_d))
        {
            // std::cout << "Outside";
            return false;
        }

        // for(LineIterator it(gm, Position(s.x(), s.y()), Position(d.x(), d.y())); !it.isPastEnd(); ++it)
        //     if(gm.at("robot_traversability", *it) == 2)   // 2 = Nontraversable. ToDo = isValid
        //         return false;

        for(LineIterator it(fgmg_gm, Position(s.x(), s.y()), Position(d.x(), d.y())); !it.isPastEnd(); ++it)
        {
            if(!fgmg_gm.isValid(*it, traversability_layer_name_))
            {
                // std::cout << "Invalid";
                return false;
            }

            if(fgmg_gm.at(traversability_layer_name_, *it) >= obs_th_)
                return false;
        }

        return true;
    }
    
    bool Map2DManager::isNeighborhoodTraversable(Point2D center)
    {
        Position center_pos(center.x(), center.y());

        if(!fgmg_gm.isInside(center_pos))
        {
            // std::cout << "Outside";
            return false;
        }

        for(SpiralIterator s_it(fgmg_gm, center_pos, robot_footprint_); !s_it.isPastEnd(); ++s_it)
        {
            if(!fgmg_gm.isValid(*s_it, traversability_layer_name_))
            {
                // std::cout << "Invalid";
                return false;
            }

            if(fgmg_gm.at(traversability_layer_name_, *s_it) >= obs_th_)
                return false;
        }

        return true;
    }
    
    double Map2DManager::getPointElevation(Point2D p2d)
    {
        Position p(p2d.x(), p2d.y());
        // std::cout << << p.x() << ", " << p.y() << " = ";

        // if(!gm.isInside(p))
        if(!nlgm_gm.isInside(p))
        {
            // std::cout << "Outside";
            return current_pose_.position.z;
        }

        Index i;
        // gm.getIndex(p, i);
        nlgm_gm.getIndex(p, i);

        // if(!gm.isValid(i, "min_filter"))
        if(!nlgm_gm.isValid(i, elevation_layer_name_))
        {
            // std::cout << "Invalid";
            return current_pose_.position.z;
        }

        // return gm.at("min_filter", i);
        return nlgm_gm.at(elevation_layer_name_, i);
    }

    bool Map2DManager::isSubmapTraversable(Point2D p2d, double d)
    {
        double x_min = std::max(fgmg_gm.getPosition().x() - fgmg_gm.getLength().x() / 2, p2d.x() - d);
        double x_max = std::min(fgmg_gm.getPosition().x() + fgmg_gm.getLength().x() / 2, p2d.x() + d);
        double y_min = std::max(fgmg_gm.getPosition().y() - fgmg_gm.getLength().y() / 2, p2d.y() - d);
        double y_max = std::min(fgmg_gm.getPosition().y() + fgmg_gm.getLength().y() / 2, p2d.y() + d);

        Position p_min(x_min + fgmg_gm.getResolution() / 2, y_min + fgmg_gm.getResolution() / 2);

        if(!fgmg_gm.isInside(p_min))
        {
            // std::cout << " Outside";
            return false;
        }

        Position p_max(x_max - fgmg_gm.getResolution() / 2, y_max - fgmg_gm.getResolution() / 2);

        if(!fgmg_gm.isInside(p_max))
        {
            // std::cout << " Outside";
            return false;
        }

        Index i_min, i_max;
        fgmg_gm.getIndex(p_min, i_min);
        fgmg_gm.getIndex(p_max, i_max);

        Index i_deff(i_min.x() - i_max.x() + 1, i_min.y() - i_max.y() + 1);
        
        for(SubmapIterator it(fgmg_gm, i_max, i_deff); !it.isPastEnd(); ++it)
            if(!fgmg_gm.at(traversability_layer_name_, *it) < obs_th_)
                return true;

        return false;
    }

    std::vector<Point2D> Map2DManager::a_star(Point2D goal, Point2D start)
    {
        std::vector<Point2D> path_points;
        path_points.clear();

        // uint grid_size_x = gm.getSize()(0);
        // uint grid_size_y = gm.getSize()(1);
        uint grid_size_x = fgmg_gm.getSize()(0);
        uint grid_size_y = fgmg_gm.getSize()(1);
        Size grid_size(grid_size_x, grid_size_y);
        Position start_position(start.x(), start.y());
        Position goal_position(goal.x(), goal.y());

        // if(gm.isInside(start_position) && gm.isInside(goal_position))
        if(fgmg_gm.isInside(start_position) && fgmg_gm.isInside(goal_position))
        {
            Index start_grid_index_2d;
            // gm.getIndex(start_position, start_grid_index_2d);
            fgmg_gm.getIndex(start_position, start_grid_index_2d);

            Index goal_grid_index_2d;
            // gm.getIndex(goal_position, goal_grid_index_2d);
            fgmg_gm.getIndex(goal_position, goal_grid_index_2d);

            Position goal_grid_center;
            // gm.getPosition(goal_grid_index_2d, goal_grid_center);
            fgmg_gm.getPosition(goal_grid_index_2d, goal_grid_center);
            
            size_t start_grid_index_1d = grid_map::getLinearIndexFromIndex(start_grid_index_2d, grid_size);
            size_t goal_grid_index_1d = grid_map::getLinearIndexFromIndex(goal_grid_index_2d, grid_size);

            if(start_grid_index_1d == goal_grid_index_1d)
            {
                std::cout << "start_grid_index_1d = goal_grid_index_1d!" << std::endl;
                path_points.push_back(start);

                return path_points;
            }
            // ==================================================================================================== A* START

            typedef std::pair<double, int> iPair;
            std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq;
            std::vector<double> dist(grid_size_x * grid_size_y, INFINITY);
            std::vector<double> estimation(grid_size_x * grid_size_y, INFINITY);
            const int INF = 0x3f3f3f3f;
            std::vector<int> backpointers(grid_size_x * grid_size_y, INF);
            std::vector<bool> in_pq(grid_size_x * grid_size_y, false);

            dist[start_grid_index_1d] = 0;
            estimation[start_grid_index_1d] = (start - goal).norm();
            pq.push(std::make_pair(estimation[start_grid_index_1d], start_grid_index_1d));
            in_pq[start_grid_index_1d] = true;

            // std::cout << "A* search.." << std::endl;
            int current_grid_index_1d;

            while(!pq.empty())
            {
                current_grid_index_1d = pq.top().second;
                pq.pop();
                in_pq[current_grid_index_1d] = false;

                if(current_grid_index_1d == goal_grid_index_1d)
                    break;

                Index current_grid_index_2d = grid_map::getIndexFromLinearIndex(current_grid_index_1d, grid_size);

                for(int i = -1; i <= 1; ++i)
                {
                    int nbr_grid_index_2d_x = current_grid_index_2d.x() + i;

                    if((nbr_grid_index_2d_x >= 0) && (nbr_grid_index_2d_x < grid_size_x))
                        for(int j = -1; j <= 1; ++j)
                        {
                            int nbr_grid_index_2d_y = current_grid_index_2d.y() + j;

                            if(!(i == 0 && j == 0) && (nbr_grid_index_2d_y >= 0) && (nbr_grid_index_2d_y < grid_size_y))
                            {
                                Index nbr_grid_index_2d(nbr_grid_index_2d_x, nbr_grid_index_2d_y);

                                Position nbr_grid_center;
                                fgmg_gm.getPosition(nbr_grid_index_2d, nbr_grid_center);
                                Point2D nbr_grid_center_p2d(nbr_grid_center.x(), nbr_grid_center.y());
                                
                                size_t nbr_grid_index_1d = grid_map::getLinearIndexFromIndex(nbr_grid_index_2d, grid_size);

                                // if(gm.isValid(nbr_grid_index_2d, "robot_traversability") && isGridTraversable(nbr_grid_index_2d))
                                if(fgmg_gm.isValid(nbr_grid_index_2d, traversability_layer_name_) 
                                    && isGridTraversable(nbr_grid_index_2d) 
                                    && isNeighborhoodTraversable(nbr_grid_center_p2d))
                                {
                                    double nbr_dist = dist[current_grid_index_1d] + grid_size_ * sqrt(pow(i, 2) + pow(j, 2));

                                    if(dist[nbr_grid_index_1d] > nbr_dist)
                                    {
                                        dist[nbr_grid_index_1d] = nbr_dist;

                                        // Position nbr_position;
                                        // gm.getPosition(nbr_grid_index_2d, nbr_position);
                                        // fgmg_gm.getPosition(nbr_grid_index_2d, nbr_position);
                                        // fgmg_gm.getPosition(nbr_grid_index_2d, nbr_position);

                                        // estimation[nbr_grid_index_1d] = dist[nbr_grid_index_1d] + (Point2D(nbr_position.x(), nbr_position.y()) - Point2D(goal_grid_center.x(), goal_grid_center.y())).norm();
                                        estimation[nbr_grid_index_1d] = dist[nbr_grid_index_1d] + (Point2D(nbr_grid_center.x(), nbr_grid_center.y()) - Point2D(goal_grid_center.x(), goal_grid_center.y())).norm();
                                        backpointers[nbr_grid_index_1d] = current_grid_index_1d;

                                        if(!in_pq[nbr_grid_index_1d])
                                        {
                                            pq.push(std::make_pair(estimation[nbr_grid_index_1d], nbr_grid_index_1d));
                                            in_pq[nbr_grid_index_1d] = true;
                                        }
                                    }
                                }
                            }
                        }
                }
            }
            // ==================================================================================================== A* END
            // ==================================================================================================== Backtracking START
            std::vector<int> path;
            std::vector<int> reverse_path;
            int current = goal_grid_index_1d;

            if(backpointers[current] == INF)
                std::cout << "No path!" << std::endl;
            else
            {
                while(current != INF)
                {
                    reverse_path.push_back(current);
                    current = backpointers[current];
                }

                path.clear();

                for(int i = reverse_path.size() - 1; i >= 0; --i)
                    path.push_back(reverse_path[i]);

                path_points.push_back(start);

                for(int i = 1; i < path.size() - 1; ++i)
                {
                    Index current_grid_index_2d = grid_map::getIndexFromLinearIndex(path[i], grid_size);
                    Position current_grid_center;
                    // gm.getPosition(current_grid_index_2d, current_grid_center);
                    fgmg_gm.getPosition(current_grid_index_2d, current_grid_center);
                    path_points.push_back(current_grid_center);
                }

                path_points.push_back(goal);
            }
            // ==================================================================================================== Backtracking END
        }
        else
            std::cout << "Outside!" << std::endl;
        
        return path_points;
    }

    std::vector<Point2D> Map2DManager::smoothen_path(std::vector<Point2D> path)
    {
        if(path.size() > 2)
        {
            std::vector<Point2D> pruned_path;
            std::vector<int> control_point_ids;
            int inner_idx = 0;
            int control_point_id = inner_idx;
            control_point_ids.push_back(control_point_id);
            
            while(inner_idx < path.size() - 1)
            {
                control_point_id = inner_idx;

                for(int i = inner_idx + 1; i < path.size(); ++i)
                    if(isLineTraversable(path[inner_idx], path[i]))
                        control_point_id = i;

                if(control_point_id == inner_idx)
                    control_point_id = inner_idx + 1;

                control_point_ids.push_back(control_point_id);
                inner_idx = control_point_id;
            }

            for(int i = 0; i < control_point_ids.size(); ++i)
                pruned_path.push_back(path[control_point_ids[i]]);
            
            return pruned_path;
        }
        else
            return path;
    }
}