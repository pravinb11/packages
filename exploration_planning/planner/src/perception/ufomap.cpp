//
// Created by hjl on 2022/1/9.
//

#include "perception/ufomap.h"

namespace perception
{
    Ufomap::Ufomap(const std::shared_ptr<rclcpp::Node> nh, 
                    const std::shared_ptr<rclcpp::Node> nh_private) :
        nh_(nh), 
        nh_private_(nh_private), 
        transform_timeout_(0, 100000000), 
        known_plane_cell_num_(0), 
        lidar_(50),
        map_(0.25, 16, true)     // FAEL = 0.4, Sentry = 0.1 OK, 0.2 OK (Must be equal to the resolution of the GridMap)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        setParametersFromROS();

        // rclcpp::Duration(0.5).sleep();

        start_time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()) / 1000000;

        fout.open(txt_known_name, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "record the known space size, starting at " << start_time << "\n" << "now time" << "\t"
             << " total elapsed time(s)" << "\t" << "known node num \t" << "known area(m2 or m3)" << std::endl;
        fout.close();

        frontier_iteration = 0;
        sum_frontier_time = 0;
        fout.open(txt_frontier_name, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "record the each frontier detected time" << "\n" << "start_time" << "\t" << "end_time" << "\t"
             << "elapsed time(s)" << "\t" << "detected_iteration" << "\t" << "average_time(s)" << std::endl;
        fout.close();

        insert_num = 0;
        sum_insert_time = 0;
        fout.open(txt_insert_cloud_name, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
        fout << "record the each insert cloud time" << "\n" << "start_time" << "\t" << "end_time" << "\t"
             << "elapsed time(s)" << "\t" << "insert iteration" << "\t" << "average_time(s)" << std::endl;
        fout.close();

        map_.enableChangeDetection(true);

        point_cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(nh_, lidar_topic_name_, rmw_qos_profile_sensor_data));
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(nh_, odometry_topic_name_, rmw_qos_profile_sensor_data));
        sync_point_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyLocalCloudOdom>(SyncPolicyLocalCloudOdom(100), *point_cloud_sub_, *odom_sub_));
        sync_point_cloud_odom_->registerCallback(std::bind(&Ufomap::pointCloudOdomCallback, this, std::placeholders::_1, std::placeholders::_2));

        voxel_filter_.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);

        write_ufomap_timer_ = nh_->create_wall_timer(std::chrono::seconds(2), std::bind(&Ufomap::writeUfomapCallback, this));

        if(0 < pub_rate_)
        {
            map_pub_ = nh_->create_publisher<ufomap_msgs::msg::UFOMapStamped>("ufomap", 1);
            cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("ufomap_cloud", 1);
            map_and_frontiers_pub_ = nh_->create_publisher<ufomap_manager::msg::UfomapWithFrontiers>("ufomap_and_frontiers", 1);
            pub_timer_ = nh_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Ufomap::ufomapPublishTimer, this));
        }

        expand_cloud_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("expand_cloud", 1);
        local_frontiers_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("local_frontier_marker", 1);
        global_frontiers_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("global_frontier_marker", 1);
        // ploygon_pub_ = nh_->create_publisher<geometry_msgs::msg::PolygonStamped>("polygon", 1);
        boundary_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("boundary", 1);
        

        // std::cout << "UFOMap()" << std::endl;
    }

    void Ufomap::setParametersFromROS()
    {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("planner");
        std::string txt_path = pkg_path + "/../../files/exploration_data/";

        txt_known_name          = txt_path + "known_node_num.txt";
        txt_frontier_name       = txt_path + "frontier_detected_time.txt";
        map_txt_name            = txt_path + "ufomap.txt";
        txt_insert_cloud_name   = txt_path + "insert_cloud.txt";        

		// nh_->declare_parameter("frame_id", "");
        nh_->get_parameter("frame_id", frame_id_);

		// nh_->declare_parameter("robot_base_frame_id", "");
        nh_->get_parameter("robot_base_frame_id", robot_base_frame_id_);

		nh_->declare_parameter("lidar_frame", "");
        nh_->get_parameter("lidar_frame", sensor_frame_id_);

		nh_->declare_parameter("max_range", 0.0);
        nh_->get_parameter("max_range", max_range_);

		nh_->declare_parameter("insert_discrete", false);
        nh_->get_parameter("insert_discrete", insert_discrete_);

		nh_->declare_parameter("insert_depth", 0);
        nh_->get_parameter("insert_depth", insert_depth_);

		nh_->declare_parameter("simple_ray_casting", false);
        nh_->get_parameter("simple_ray_casting", simple_ray_casting_);

		nh_->declare_parameter("early_stopping", 0);
        nh_->get_parameter("early_stopping", early_stopping_);

		nh_->declare_parameter("clear_robot_enabled", false);
        nh_->get_parameter("clear_robot_enabled", clear_robot_enabled_);

		nh_->declare_parameter("robot_height", 0.0);
        nh_->get_parameter("robot_height", robot_height_);

		nh_->declare_parameter("robot_bottom", 0.0);
        nh_->get_parameter("robot_bottom", robot_bottom_);

		nh_->declare_parameter("sensor_height", 0.0);
        nh_->get_parameter("sensor_height", sensor_height_);

		nh_->declare_parameter("frontier_depth", 0);
        nh_->get_parameter("frontier_depth", frontier_depth_);

		nh_->declare_parameter("pub_rate", 0.0);
        nh_->get_parameter("pub_rate", pub_rate_);

		nh_->declare_parameter("scan_voxel_size", 0.0);
        nh_->get_parameter("scan_voxel_size", scan_voxel_size_);

		nh_->declare_parameter("min_x", 0.0);
        nh_->get_parameter("min_x", min_x_);

		nh_->declare_parameter("min_y", 0.0);
        nh_->get_parameter("min_y", min_y_);

		nh_->declare_parameter("min_z", 0.0);
        nh_->get_parameter("min_z", min_z_);

		nh_->declare_parameter("max_x", 0.0);
        nh_->get_parameter("max_x", max_x_);

		nh_->declare_parameter("max_y", 0.0);
        nh_->get_parameter("max_y", max_y_);

		nh_->declare_parameter("max_z", 0.0);
        nh_->get_parameter("max_z", max_z_);

        // nh_->declare_parameter("odometry_topic_name", "");
        nh_->get_parameter("odometry_topic_name", odometry_topic_name_);

        nh_->declare_parameter("lidar_topic_name", "");
        nh_->get_parameter("lidar_topic_name", lidar_topic_name_);

        // std::cout << "============================== UFOMap Parameters : START" << std::endl;
        // std::cout << "frame_id_ = " << frame_id_ << std::endl;
        // std::cout << "robot_base_frame_id_ = " << robot_base_frame_id_ << std::endl;
        // std::cout << "sensor_frame_id_ = " << sensor_frame_id_ << std::endl;
        // std::cout << "max_range_ = " << max_range_ << std::endl;
        // std::cout << "insert_discrete_ = " << insert_discrete_ << std::endl;
        // std::cout << "insert_depth_ = " << insert_depth_ << std::endl;
        // std::cout << "simple_ray_casting_ = " << simple_ray_casting_ << std::endl;
        // std::cout << "early_stopping_ = " << early_stopping_ << std::endl;
        // std::cout << "clear_robot_enabled_ = " << clear_robot_enabled_ << std::endl;
        // std::cout << "robot_height_ = " << robot_height_ << std::endl;
        // std::cout << "robot_bottom_ = " << robot_bottom_ << std::endl;
        // std::cout << "sensor_height_ = " << sensor_height_ << std::endl;
        // std::cout << "frontier_depth_ = " << frontier_depth_ << std::endl;
        // std::cout << "pub_rate_ = " << pub_rate_ << std::endl;
        // std::cout << "scan_voxel_size_ = " << scan_voxel_size_ << std::endl;
        // std::cout << "min_x_ = " << min_x_ << std::endl;
        // std::cout << "min_y_ = " << min_y_ << std::endl;
        // std::cout << "min_z_ = " << min_z_ << std::endl;
        // std::cout << "max_x_ = " << max_x_ << std::endl;
        // std::cout << "max_y_ = " << max_y_ << std::endl;
        // std::cout << "max_z_ = " << max_z_ << std::endl;
        // std::cout << "odometry_topic_name_ = " << odometry_topic_name_ << std::endl;
        // std::cout << "lidar_topic_name_ = " << lidar_topic_name_ << std::endl;
        // std::cout << "============================== UFOMap Parameters : END" << std::endl;
    }

    void Ufomap::pointCloudOdomCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan,
                                        const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
    {
        geometry_msgs::msg::Pose rob_pose = odom->pose.pose;
        current_robot_pose_.translation().x() = rob_pose.position.x;
        current_robot_pose_.translation().y() = rob_pose.position.y;
        current_robot_pose_.translation().z() = rob_pose.position.z;
        current_robot_pose_.rotation().w() = rob_pose.orientation.w;
        current_robot_pose_.rotation().x() = rob_pose.orientation.x;
        current_robot_pose_.rotation().y() = rob_pose.orientation.y;
        current_robot_pose_.rotation().z() = rob_pose.orientation.z;

        // std::cout << "Ufomap::pointCloudOdomCallback: current_robot_pose_ = (" << rob_pose.position.x << ", " << rob_pose.position.y << ", " << rob_pose.position.z << ") @ " << odom->header.frame_id << std::endl;
        // ==================================================================================================== TF: START
        geometry_msgs::msg::TransformStamped ts;

        try
        {
            // ts = tf_buffer_->lookupTransform(frame_id_, sensor_frame_id_, odom->header.stamp);
            ts = tf_buffer_->lookupTransform(frame_id_, sensor_frame_id_, tf2::TimePointZero);
            // ts = tf_buffer_->lookupTransform(frame_id_, sensor_frame_id_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
        }
        catch(tf2::TransformException &ex)
        {
            RCLCPP_WARN(rclcpp::get_logger("Planner"), "Ufomap::pointCloudOdomCallback: TF = odom -> laser_link! %s ", ex.what());
        }

        geometry_msgs::msg::Transform t = ts.transform;
        current_sensor_pose_.translation().x() = t.translation.x;
        current_sensor_pose_.translation().y() = t.translation.y;
        current_sensor_pose_.translation().z() = t.translation.z;
        current_sensor_pose_.rotation().w() = t.rotation.w;
        current_sensor_pose_.rotation().x() = t.rotation.x;
        current_sensor_pose_.rotation().y() = t.rotation.y;
        current_sensor_pose_.rotation().z() = t.rotation.z;

        // std::cout << "Ufomap::pointCloudOdomCallback: current_sensor_pose_ = (" << t.translation.x << ", " << t.translation.y << ", " << t.translation.z << ") @ " << scan->header.frame_id << std::endl;
        // ==================================================================================================== TF: END
        // ==================================================================================================== UFOMap: START
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*scan, *scan_cloud);

        map_mutex_.lock();
        insert_num++;
        auto insert_start_time = nh_->get_clock()->now();

        voxel_filter_.setInputCloud(scan_cloud);
        pcl::PointCloud<pcl::PointXYZ> downsize_scan_cloud;
        voxel_filter_.filter(downsize_scan_cloud);

        ufo::map::PointCloud downsize_cloud;

        for(auto &point: downsize_scan_cloud)
        {
            // std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;            
            downsize_cloud.push_back(ufo::map::Point3(point.x, point.y, point.z));
        }

        downsize_cloud.transform(current_sensor_pose_);

        ufo::map::KeySet key_set;
        ufo::map::PointCloud cloud;

        for(const auto &point: downsize_cloud)
        {
            // std::cout << "T (" << point.x() << ", " << point.y() << ", " << point.z() << ")" << std::endl;

            if(key_set.insert(map_.toKey(point, 0)).second)
                cloud.push_back(map_.toCoord(map_.toKey(point, 0), 0)); 
        }
        
        if(insert_discrete_)
            map_.insertPointCloudDiscrete(current_sensor_pose_.translation(), cloud, max_range_, insert_depth_, simple_ray_casting_, early_stopping_, false);
        else
            map_.insertPointCloud(current_sensor_pose_.translation(), cloud, max_range_, insert_depth_, simple_ray_casting_, early_stopping_, false);

        auto insert_end_time = nh_->get_clock()->now();
        sum_insert_time += (insert_end_time - insert_start_time).seconds();

        fout.open(txt_insert_cloud_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << insert_start_time.seconds() << "\t" << insert_end_time.seconds() << "\t" << (insert_end_time - insert_start_time).seconds() << "\t" << insert_num << "\t" << sum_insert_time / insert_num << "s \t" << std::endl;
        fout.close();
        // ==================================================================================================== UFOMap: END
        map_mutex_.unlock();
    }

    void Ufomap::ufomapPublishTimer()
    {
        map_mutex_.lock();

        std_msgs::msg::Header header;
        header.stamp = nh_->get_clock()->now();
        header.frame_id = frame_id_;
        // ====================================================================================================

        ufomap_msgs::msg::UFOMapStamped msg;
        ufomap_msgs::ufoToMsg(map_, msg.map, false);
        msg.header = header;
        map_pub_->publish(msg);
        // ====================================================================================================

        if(cloud_pub_->get_subscription_count())
        {
            ufo::map::PointCloud cloud;

            for(auto it = map_.beginLeaves(true, false, false, false, 0), it_end = map_.endLeaves(); it != it_end; ++it)
                cloud.push_back(it.getCenter());
            
            sensor_msgs::msg::PointCloud2 cloud_msg;
            ufomap_ros::ufoToRos(cloud, cloud_msg);
            cloud_msg.header = header;
            cloud_pub_->publish(cloud_msg);
        }
        // ====================================================================================================

        statisticAndPubMarkers();
        // ====================================================================================================

        // geometry_msgs::msg::PolygonStamped polygon;
        // polygon.header.frame_id = frame_id_;
        // polygon.header.stamp = nh_->get_clock()->now();
        // geometry_msgs::msg::Point32 point_1;
        // point_1.x = min_x_;
        // point_1.y = min_y_;
        // point_1.z = current_sensor_pose_.z();
        // polygon.polygon.points.push_back(point_1);

        // geometry_msgs::msg::Point32 point_2;
        // point_2.x = max_x_;
        // point_2.y = min_y_;
        // point_2.z = current_sensor_pose_.z();
        // polygon.polygon.points.push_back(point_2);

        // geometry_msgs::msg::Point32 point_3;
        // point_3.x = max_x_;
        // point_3.y = max_y_;
        // point_3.z = current_sensor_pose_.z();
        // polygon.polygon.points.push_back(point_3);

        // geometry_msgs::msg::Point32 point_4;
        // point_4.x = min_x_;
        // point_4.y = max_y_;
        // point_4.z = current_sensor_pose_.z();
        // polygon.polygon.points.push_back(point_4);

        // ploygon_pub_->publish(polygon);
        boundary_pub();
        // ====================================================================================================

        map_mutex_.unlock();
    }

    void Ufomap::boundary_pub()
    {
        visualization_msgs::msg::Marker boundary_marker;
        boundary_marker.header.frame_id = frame_id_;
        boundary_marker.header.stamp = nh_->get_clock()->now();
        boundary_marker.ns = "boundary_ns";
        boundary_marker.id = 0;
        boundary_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        boundary_marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point boundary_point_1;
        boundary_point_1.x = min_x_;
        boundary_point_1.y = min_y_;
        boundary_point_1.z = current_sensor_pose_.z();
        boundary_marker.points.push_back(boundary_point_1);
        
        geometry_msgs::msg::Point boundary_point_2;
        boundary_point_2.x = max_x_;
        boundary_point_2.y = min_y_;
        boundary_point_2.z = current_sensor_pose_.z();
        boundary_marker.points.push_back(boundary_point_2);
        boundary_marker.points.push_back(boundary_point_2);
        
        geometry_msgs::msg::Point boundary_point_3;
        boundary_point_3.x = max_x_;
        boundary_point_3.y = max_y_;
        boundary_point_3.z = current_sensor_pose_.z();
        boundary_marker.points.push_back(boundary_point_3);
        boundary_marker.points.push_back(boundary_point_3);
        
        geometry_msgs::msg::Point boundary_point_4;
        boundary_point_4.x = min_x_;
        boundary_point_4.y = max_y_;
        boundary_point_4.z = current_sensor_pose_.z();
        boundary_marker.points.push_back(boundary_point_4);
        boundary_marker.points.push_back(boundary_point_4);
        boundary_marker.points.push_back(boundary_point_1);

        boundary_marker.scale.x = 0.125;
        boundary_marker.scale.y = 0.125;
        boundary_marker.scale.z = 0.125;
        boundary_marker.color.r = 1.0;
        boundary_marker.color.g = 0.0;
        boundary_marker.color.b = 0.0;
        boundary_marker.color.a = 1.0;
        boundary_pub_->publish(boundary_marker);
    }

    
    void Ufomap::frontierCallback() {

        frontier_iteration++;
        // auto s_time = ros::WallTime::now();
        auto s_time = nh_->get_clock()->now();

        frontierSearch();

        // auto finish_time = ros::WallTime::now();
        auto finish_time = nh_->get_clock()->now();

        // sum_frontier_time = sum_frontier_time + (finish_time - s_time).toSec();
        sum_frontier_time = sum_frontier_time + (finish_time - s_time).seconds();
        fout.open(txt_frontier_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        // fout << s_time << "\t" << finish_time << "\t" << (finish_time - s_time).toSec()
        fout << s_time.seconds() << "\t" << finish_time.seconds() << "\t" << (finish_time - s_time).seconds()
             << "\t" << frontier_iteration << "\t" << sum_frontier_time / frontier_iteration << "s \t" << std::endl;
        fout.close();

        knownCellOutput();

        std_msgs::msg::ColorRGBA global_rgba;
        global_rgba.a = 0.3;
        global_rgba.r = 0;
        global_rgba.g = 1;
        global_rgba.b = 0;
        visualization_msgs::msg::MarkerArray global_frontier_cells;
        generateMarkerArray(frame_id_, &global_frontier_cells, global_frontier_cells_, global_rgba);
        // global_frontiers_pub_.publish(global_frontier_cells);
        global_frontiers_pub_->publish(global_frontier_cells);

        std_msgs::msg::ColorRGBA local_rgba;
        local_rgba.a = 0.3;
        local_rgba.r = 1;
        local_rgba.g = 0;
        local_rgba.b = 0;
        visualization_msgs::msg::MarkerArray local_frontier_cells;
        generateMarkerArray(frame_id_, &local_frontier_cells, local_frontier_cells_, local_rgba);
        // local_frontiers_pub_.publish(local_frontier_cells);
        local_frontiers_pub_->publish(local_frontier_cells);

        // map and frontiers pub..
        // ufomap_manager::UfomapWithFrontiers ufomap_with_frontiers;
        ufomap_manager::msg::UfomapWithFrontiers ufomap_with_frontiers;
        std_msgs::msg::Header header;
        // header.stamp = rclcpp::Time::now();
        header.stamp = nh_->get_clock()->now();
        header.frame_id = frame_id_;
        ufomap_with_frontiers.header = header;

        ufomap_with_frontiers.known_plane_cell_num = known_plane_cell_num_;
        ufomap_with_frontiers.max_range = max_range_;
        ufomap_with_frontiers.frame_id = frame_id_;
        ufomap_with_frontiers.robot_base_frame_id = robot_base_frame_id_;
        ufomap_with_frontiers.robot_height = robot_height_;
        ufomap_with_frontiers.robot_bottom = robot_bottom_;
        ufomap_with_frontiers.robot_radius = sensor_height_;

        for (auto &item: local_frontier_cells_) {
            // ufomap_manager::CellCode local_frontier;
            ufomap_manager::msg::CellCode local_frontier;
            local_frontier.depth = item.getDepth();
            // local_frontier.Code = item.getCode();
            local_frontier.code = item.getCode();
            ufomap_with_frontiers.local_frontiers.push_back(local_frontier);
        }

        for (auto &item: global_frontier_cells_) {
            // ufomap_manager::CellCode global_frontier;
            ufomap_manager::msg::CellCode global_frontier;
            global_frontier.depth = item.getDepth();
            // global_frontier.Code = item.getCode();
            global_frontier.code = item.getCode();
            ufomap_with_frontiers.global_frontiers.push_back(global_frontier);
        }

        for (auto &item: changed_cell_codes_) {
            // ufomap_manager::CellCode changed_cell_code;
            ufomap_manager::msg::CellCode changed_cell_code;
            changed_cell_code.depth = item.getDepth();
            // changed_cell_code.Code = item.getCode();
            changed_cell_code.code = item.getCode();
            ufomap_with_frontiers.changed_cell_codes.push_back(changed_cell_code);
        }

        for (auto &item: known_cell_codes_) {  
            if (0 == item.getDepth()) {
                // ufomap_manager::CellCode known_cell_code;
                ufomap_manager::msg::CellCode known_cell_code;
                known_cell_code.depth = item.getDepth();
                // known_cell_code.Code = item.getCode();
                known_cell_code.code = item.getCode();
                ufomap_with_frontiers.known_cell_codes.push_back(known_cell_code);
            }
        }

        // ufomap_msgs::UFOMap ufomap;
        ufomap_msgs::msg::UFOMap ufomap;
        ufomap_msgs::ufoToMsg(map_, ufomap, false);
        ufomap_with_frontiers.ufomap = ufomap;

        // map_and_frontiers_pub_.publish(ufomap_with_frontiers);
        map_and_frontiers_pub_->publish(ufomap_with_frontiers);
    }

    void Ufomap::frontierSearch()
    {
        changed_cell_codes_.clear();

        for(auto it = map_.changesBegin(); it != map_.changesEnd(); it++)
            changed_cell_codes_.insert(*it);

        // std::cout << "UFOMap::frontierSearch: |changed_cell_codes_| = " << changed_cell_codes_.size() << std::endl;
        known_cell_codes_.insert(changed_cell_codes_.begin(), changed_cell_codes_.end());
        map_.resetChangeDetection();
        findPlaneLocalFrontier();
        updatePlaneGlobalFrontier();

        std::cout << "#Frontiers: Local = " << local_frontier_cells_.size() << ", Global = " << global_frontier_cells_.size() << std::endl;
    }

    void Ufomap::writeUfomapCallback()
    {
        map_.write(map_txt_name);
    }

    void Ufomap::findLocalFrontier() {
        local_frontier_cells_.clear();

        for (const auto &iter_cell: changed_cell_codes_) {
            if (frontier_depth_ == iter_cell.getDepth()) {
                ufo::map::Point3 iter_coord = map_.toCoord(iter_cell.toKey());
                double sensor_dist_xy = iter_coord.distanceXY(current_sensor_pose_.translation());
                double robot_dist_xy = iter_coord.distanceXY(current_robot_pose_.translation());
                if (robot_dist_xy > 0.6 && fabs(iter_coord.z() - current_sensor_pose_.z()) / sensor_dist_xy <
                                           tan(M_PI * 15 / 180)) {  
                    if (isFrontier(iter_cell)) {
                        local_frontier_cells_.insert(iter_cell);
                    }
                }

            }
        }
        changed_cell_codes_.clear();
    }

    void Ufomap::updateGlobalFrontier() {

        CodeUnorderSet global_frontiers;
        if (!global_frontier_cells_.empty()) {
            for (const auto &cell_iter: global_frontier_cells_) {
                ufo::map::Point3 cell_coord = map_.toCoord(cell_iter.toKey());
                double sensor_dist_xy = cell_coord.distanceXY(current_sensor_pose_.translation());
                if (sensor_dist_xy < max_range_ + 1.0) {  
                    if (fabs(cell_coord.z() - current_sensor_pose_.z()) / sensor_dist_xy < tan(M_PI * 15 / 180)
                        && isFrontier(cell_iter)) {  
                        global_frontiers.insert(cell_iter);
                    }
                } else {
                    global_frontiers.insert(cell_iter);
                }
            }
        }
        global_frontier_cells_ = global_frontiers;

        for (const auto &iter_cell: local_frontier_cells_) {  
            global_frontier_cells_.insert(iter_cell);
        }
    }

    CodeUnorderSet Ufomap::get_3D_NeighborCell(const ufo::map::Code &cell_code, unsigned int depth) {
        CodeUnorderSet neighbors;
        ufo::map::Point3 cell_center = map_.toCoord(cell_code.toKey(), depth);

        std::vector<double> coord_x{cell_center.x() - map_.getResolution(), cell_center.x(),
                                    cell_center.x() + map_.getResolution()};
        std::vector<double> coord_y{cell_center.y() - map_.getResolution(), cell_center.y(),
                                    cell_center.y() + map_.getResolution()};
        std::vector<double> coord_z{cell_center.z() - map_.getResolution(), cell_center.z(),
                                    cell_center.z() + map_.getResolution()};

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    if (i != 1 && j != 1 && k != 1) {  
                        neighbors.insert(
                                ufo::map::Code(map_.toKey(coord_x[i], coord_y[j], coord_z[k], depth)));
                    }
                }
            }
        }
        return neighbors;
    }

    void Ufomap::findPlaneLocalFrontier()
    {
        local_frontier_cells_.clear();

        for(const auto &changedCellCode: changed_cell_codes_)
        {
            int changed_cell_depth = changedCellCode.getDepth();
            int changed_cell_count = history_frontier_cells_.count(changedCellCode);
            // std::cout << "Depth = " << changed_cell_depth << ". Count = " << changed_cell_count;

            if((changed_cell_depth == frontier_depth_) && !changed_cell_count)
            {
                ufo::map::Point3 point = map_.toCoord(changedCellCode.toKey());
                // std::cout << ". Voxel = (" << point.x() << ", " << point.y() << ", " << point.z() << ")";

                // if(isInExplorationArea(point.x(), point.y()) 
                    // && point.z() > current_robot_pose_.z() + robot_bottom_ 
                    // && point.z() < current_robot_pose_.z() + robot_height_)
                    // && point.z() > current_robot_pose_.z() - map_.getResolution() 
                    // && point.z() < current_robot_pose_.z() + map_.getResolution())
                if(isInGeofencedVolume(point.x(), point.y(), point.z()) 
                    && point.z() > current_sensor_pose_.z() - map_.getResolution() 
                    && point.z() < current_sensor_pose_.z() + map_.getResolution())
                {
                    // odom_mutex_.lock();
                    // ufo::math::Vector3 current = current_robot_pose_.translation();
                    // odom_mutex_.unlock();
                    // std::cout << ". Distance = " << point.distanceXY(current);

                    // if(point.distanceXY(current) > 0.6)
                    // {
                        // std::cout << ". Frontier = " << isFrontier(changedCellCode);

                        if(isFrontier(changedCellCode))
                        {
                            local_frontier_cells_.insert(changedCellCode);
                            history_frontier_cells_.insert(changedCellCode);
                        }
                    // }
                }
            }

            // std::cout << std::endl;
        }

        // std::cout << "|local_frontier_cells_| = " << local_frontier_cells_.size() << std::endl;
    }

    void Ufomap::updatePlaneGlobalFrontier()
    {
        CodeUnorderSet global_frontiers;

        if(!global_frontier_cells_.empty())
            for(const auto &cell_iter: global_frontier_cells_)
            {  
                // ufo::map::Point3 point = map_.toCoord(cell_iter.toKey());

                // if(point.distanceXY(current_robot_pose_.translation()) < max_range_ + 1.0)
                    if(isFrontier(cell_iter))
                        global_frontiers.insert(cell_iter);
                // else
                //     global_frontiers.insert(cell_iter);
            }

        global_frontier_cells_ = global_frontiers;
        // std::cout << "|global_frontier_cells_| = " << global_frontier_cells_.size() << std::endl;

        for(const auto &iter_cell: local_frontier_cells_)
            global_frontier_cells_.insert(iter_cell);
        
        // std::cout << "|global_frontier_cells_| Merged = " << global_frontier_cells_.size() << std::endl;
    }

    CodeUnorderSet Ufomap::get_XY_NeighborCell(const ufo::map::Code &cell_code, unsigned int depth) const
    {
        CodeUnorderSet changed_cell_neighbor;
        ufo::map::Point3 cell_center = map_.toCoord(cell_code.toKey(), depth);

        changed_cell_neighbor.insert(ufo::map::Code(map_.toKey(cell_center.x() - map_.getResolution(), cell_center.y(), cell_center.z(), depth)));
        changed_cell_neighbor.insert(ufo::map::Code(map_.toKey(cell_center.x() + map_.getResolution(), cell_center.y(), cell_center.z(), depth)));
        changed_cell_neighbor.insert(ufo::map::Code(map_.toKey(cell_center.x(), cell_center.y() - map_.getResolution(), cell_center.z(), depth)));
        changed_cell_neighbor.insert(ufo::map::Code(map_.toKey(cell_center.x(), cell_center.y() + map_.getResolution(), cell_center.z(), depth)));

        return changed_cell_neighbor;
    }

    CodeUnorderSet
    Ufomap::get_Z_NeighborCell(const ufo::map::Code &cell_code, unsigned int depth) const {
        CodeUnorderSet changed_cell_neighbor;
        ufo::map::Point3 cell_center = map_.toCoord(cell_code.toKey(), depth);

        changed_cell_neighbor.insert(ufo::map::Code(
                map_.toKey(cell_center.x(), cell_center.y(), cell_center.z() - map_.getResolution(), depth)));
        changed_cell_neighbor.insert(ufo::map::Code(
                map_.toKey(cell_center.x(), cell_center.y(), cell_center.z() + map_.getResolution(), depth)));

        return changed_cell_neighbor;
    }

    bool Ufomap::isFrontier(const ufo::map::Code &frontier) const
    {
        if(map_.isFree(frontier))
        {
            CodeUnorderSet xy_neighbor_cells = get_XY_NeighborCell(frontier, frontier.getDepth());  
            bool unknowFlag = false;

            for(const auto &iter: xy_neighbor_cells)
                if(map_.isUnknown(iter))
                    unknowFlag = true;
            
            if(unknowFlag)
                return true;
            else
                return false;
        }
        else
            return false;
    }

    bool Ufomap::isInExplorationArea(const double &point_x, const double & point_y) const {
        if (point_x < min_x_ + 1e-4 || point_x > max_x_ - 1e-4 ||
            point_y < min_y_ + 1e-4 || point_y > max_y_ - 1e-4) {
            return false;
        } else {
            return true;
        }
    }

    bool Ufomap::isInGeofencedVolume(const double &point_x, const double & point_y, const double & point_z) const
    {
        if(point_x < min_x_ || point_x > max_x_ || point_y < min_y_ || point_y > max_y_ || point_z < min_z_ || point_z > max_z_)
            return false;
        else
            return true;
    }

    void Ufomap::knownCellOutput()
    {
        known_cell_num_ = getKnownNodeNum(known_cell_codes_);
        known_plane_cell_num_ = getKnownPlaneNodeNum(known_cell_codes_);

        double current_time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()) / 1000000;
        
        fout.open(txt_known_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << current_time << "\t" << (current_time - start_time) << "\t" << known_plane_cell_num_ << "\t" << known_plane_cell_num_ * map_.getResolution() * map_.getResolution() << std::endl;
        fout.close();
    }

    std::size_t Ufomap::getKnownNodeNum(const CodeUnorderSet &knownCellCodes)
    {
        std::size_t size = 0;
        
        for(const auto &iter: knownCellCodes)
            if(!iter.getDepth())
                size++;
        
        return size;
    }

    std::size_t Ufomap::getKnownPlaneNodeNum(const CodeUnorderSet &knownCellCodes)
    {
        std::size_t size = 0;

        for(const auto &iter: knownCellCodes)
            if(!iter.getDepth() && map_.toCoord(iter.toKey()).z() > 0 && map_.toCoord(iter.toKey()).z() < map_.getResolution())
                size++;
        
        return size;
    }

    void Ufomap::generateMarkerArray(const std::string &tf_frame, visualization_msgs::msg::MarkerArray *frontier_cells, CodeUnorderSet &frontier_cell_codes, std_msgs::msg::ColorRGBA &rgba)
    {
        auto tree_depth = map_.getTreeDepthLevels();
        frontier_cells->markers.resize(tree_depth);

        for(int i = 0; i < tree_depth; ++i)
        {
            double size = map_.getNodeSize(i);
            frontier_cells->markers[i].header.frame_id = tf_frame;
            frontier_cells->markers[i].ns = frame_id_;
            frontier_cells->markers[i].id = i;
            frontier_cells->markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
            frontier_cells->markers[i].scale.x = size;
            frontier_cells->markers[i].scale.y = size;
            frontier_cells->markers[i].scale.z = size;
            frontier_cells->markers[i].pose.orientation.w = 1;
        }

        for(const auto &iter: frontier_cell_codes)
        {
            geometry_msgs::msg::Point cube_center;
            cube_center.x = map_.toCoord(iter.toKey()).x();
            cube_center.y = map_.toCoord(iter.toKey()).y();
            cube_center.z = map_.toCoord(iter.toKey()).z();

            auto depth_level = iter.getDepth();
            frontier_cells->markers[depth_level].points.push_back(cube_center);
            frontier_cells->markers[depth_level].colors.push_back(rgba);
        }

        for(int i = 0; i < tree_depth; ++i)
            if(!frontier_cells->markers[i].points.empty())
                frontier_cells->markers[i].action = visualization_msgs::msg::Marker::ADD;
            else
                frontier_cells->markers[i].action = visualization_msgs::msg::Marker::DELETE;
    }

    void Ufomap::frontierUpdate()
    {
        frontier_iteration++;
        auto s_time = nh_->get_clock()->now();
        frontierSearch();
        auto finish_time = nh_->get_clock()->now();

        sum_frontier_time += (finish_time - s_time).seconds();

        fout.open(txt_frontier_name, std::ios_base::in | std::ios_base::out | std::ios_base::app);
        fout << s_time.seconds() << "\t" << finish_time.seconds() << "\t" << (finish_time - s_time).seconds()
             << "\t" << frontier_iteration << "\t" << sum_frontier_time / frontier_iteration << "s \t" << std::endl;
        fout.close();
    }

    void Ufomap::statisticAndPubMarkers()
    {
        knownCellOutput();

        std_msgs::msg::ColorRGBA global_rgba;
        global_rgba.a = 0.5;        // 0.3
        global_rgba.r = 1;
        global_rgba.g = 0;
        global_rgba.b = 0;
        visualization_msgs::msg::MarkerArray global_frontier_cells;
        generateMarkerArray(frame_id_, &global_frontier_cells, global_frontier_cells_, global_rgba);
        global_frontiers_pub_->publish(global_frontier_cells);
        // ====================================================================================================

        std_msgs::msg::ColorRGBA local_rgba;
        local_rgba.a = 0.5;     // 0.3
        local_rgba.r = 0;
        local_rgba.g = 1;
        local_rgba.b = 0;
        visualization_msgs::msg::MarkerArray local_frontier_cells;
        generateMarkerArray(frame_id_, &local_frontier_cells, local_frontier_cells_, local_rgba);
        local_frontiers_pub_->publish(local_frontier_cells);
        // ====================================================================================================

        ufomap_manager::msg::UfomapWithFrontiers ufomap_with_frontiers;
        std_msgs::msg::Header header;
        header.stamp = nh_->get_clock()->now();
        header.frame_id = frame_id_;
        ufomap_with_frontiers.header = header;

        ufomap_with_frontiers.known_plane_cell_num = known_plane_cell_num_;
          
        ufomap_with_frontiers.max_range = max_range_;
        ufomap_with_frontiers.frame_id = frame_id_;
        ufomap_with_frontiers.robot_base_frame_id = robot_base_frame_id_;
        ufomap_with_frontiers.robot_height = robot_height_;
        ufomap_with_frontiers.robot_bottom = robot_bottom_;
        ufomap_with_frontiers.robot_radius = sensor_height_;

        for(auto &item: local_frontier_cells_)
        {
            ufomap_manager::msg::CellCode local_frontier;
            local_frontier.depth = item.getDepth();
            local_frontier.code = item.getCode();
            ufomap_with_frontiers.local_frontiers.push_back(local_frontier);
        }

        for(auto &item: global_frontier_cells_)
        {
            ufomap_manager::msg::CellCode global_frontier;
            global_frontier.depth = item.getDepth();
            global_frontier.code = item.getCode();
            ufomap_with_frontiers.global_frontiers.push_back(global_frontier);
        }

        for(auto &item: changed_cell_codes_)
        {
            ufomap_manager::msg::CellCode changed_cell_code;
            changed_cell_code.depth = item.getDepth();
            changed_cell_code.code = item.getCode();
            ufomap_with_frontiers.changed_cell_codes.push_back(changed_cell_code);
        }

        for(auto &item: known_cell_codes_)
            if(0 == item.getDepth())
            {
                ufomap_manager::msg::CellCode known_cell_code;
                known_cell_code.depth = item.getDepth();
                known_cell_code.code = item.getCode();
                ufomap_with_frontiers.known_cell_codes.push_back(known_cell_code);
            }

        ufomap_msgs::msg::UFOMap ufomap;
        ufomap_msgs::ufoToMsg(map_, ufomap, false);
        ufomap_with_frontiers.ufomap = ufomap;
        map_and_frontiers_pub_->publish(ufomap_with_frontiers);
    }

    bool Ufomap::isInCollision(ufo::map::Point3 sensor_point, ufo::map::Point3 frontier_coord)
    {
        ufo::geometry::LineSegment ls(sensor_point, frontier_coord);

        for(auto it = map_.beginLeaves(ls, true, false, true, false, 0), it_end = map_.endLeaves(); it != it_end; ++it)      // OFU = TFT
            return true;
        
        return false;
    }

    void Ufomap::setBoundingBox( double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)
    {
        min_x_ = min_x;
        max_x_ = max_x;
        min_y_ = min_y;
        max_y_ = max_y;
        min_z_ = min_z;
        max_z_ = max_z;
    }
}
