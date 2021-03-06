#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#include "grid_map_ugv_planner/grid_map_ugv_planner.h"
#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <visualization_msgs/Marker.h>

GridMapUGVPlanner::GridMapUGVPlanner(tf2_ros::Buffer& tf)
 :tf_(tf), map_({"elevation"}),
  filter_chain_("grid_map::GridMap"),
  publish_progress_(true),
  octomap_received_(false)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    plan_pub_ = nh.advertise<nav_msgs::Path>("ugv_global_plan", 1);
    sample_pub_ = private_nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("sampling_point", 1);
    tree_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
    path_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("path", 1);

    private_nh.param("octomap_service_topic", octomap_service_, std::string("/octomap_binary"));
    private_nh.param("filter_chain_parameter_name", filter_chain_parameter_name_, std::string("grid_map_filters"));
    private_nh.param("probe_tf", probe_tf_, std::string("base_link"));

    private_nh.param("probe_range_limit_x", probe_range_limit_x_, NAN);
    private_nh.param("probe_range_limit_y", probe_range_limit_y_, NAN);
    private_nh.param("probe_range_limit_z_down", probe_range_limit_z_down_, NAN);
    private_nh.param("probe_range_limit_z_up", probe_range_limit_z_up_, NAN);
    private_nh.param("probe_traversability_threshold", probe_traversability_threshold_, 0.0);
    private_nh.param("landing_traversability_threshold", landing_traversability_threshold_, 0.8f);
    private_nh.param("visualize_position", visualize_position_, true);
    private_nh.param("visualize_grid_map", visualize_grid_map_, true);
    private_nh.param("visualize_elevation_map", visualize_elevation_map_, true);
    private_nh.param("publish_progress", publish_progress_, true);
    private_nh.param("initial_neaby_radius", initial_neaby_radius_, 2.0);

    private_nh.param("delta", delta_, 0.5);
    private_nh.param("goal_tolerance", goal_tolerance_, 0.1);
    private_nh.param("select_goal_rate", select_goal_rate_, 0.98);
    private_nh.param("max_iteration", max_itr_, 1000);
    private_nh.param("boundary_offset", boundary_offset_, 1.0);
    private_nh.param("trajectory_res", trajectory_res_, 0.1);

    private_nh.param("use_bagfile", use_bagfile_, true);
    private_nh.param("bag_topic", bagTopic_, std::string("/grid_map"));
    private_nh.param("publish_topic", publishTopic_, bagTopic_);
    private_nh.param("file_path", filePath_, std::string());

    private_nh.param("traversability_cost_weight", traversability_cost_weight_, 0.1);

    probe_service_ = nh.advertiseService("/ugv_planner", &GridMapUGVPlanner::get_pos_callback, this);

    // map_.setBasicLayers({"elevation"});

    if (!filter_chain_.configure(filter_chain_parameter_name_, nh)) {
        ROS_ERROR("Could not configure the filter chain!");
    }
    if (visualize_grid_map_) {
        grid_map_publisher_ = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    }
    if (visualize_position_) {
        landing_marker_publisher_ = nh.advertise<visualization_msgs::Marker>("goal_marker", 1, true);
    }
    if(use_bagfile_){
        grid_map::GridMapRosConverter::loadFromBag(filePath_, bagTopic_, outputmap_);
        grid_map_msgs::GridMap gridMapMessage;
        grid_map::GridMapRosConverter::toMessage(outputmap_, gridMapMessage);
        grid_map_publisher_.publish(gridMapMessage);
    } else {
        // std::string err;
        // while (ros::ok() && !tf_.canTransform("map", "base_link", ros::Time(0), ros::Duration(1.0), &err)){
        //     ROS_INFO_STREAM_NAMED("commander","Waiting for transform to be available. (tf says: " << err << ")");
        // }
        octomap_subscriber_ = nh.subscribe("/octomap_binary", 10, &GridMapUGVPlanner::octomap_callback, this);
    }
}

GridMapUGVPlanner::~GridMapUGVPlanner(){}

void GridMapUGVPlanner::octomap_callback(const octomap_msgs::Octomap::ConstPtr& map)
{
    octomap_ = *map;
    octomap_received_ = true;
}

bool GridMapUGVPlanner::get_pos_callback(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res)
{
    ROS_INFO("Service call invoked.");
    start_ << req.start.pose.position.x, req.start.pose.position.y;
    goal_ << req.goal.pose.position.x, req.goal.pose.position.y;
    getBBX(start_, goal_);

    if(!use_bagfile_){
        if (!octomap_received_) {
            ROS_ERROR("No octomap received prior to this service call. Aborting.");
            return false;
        }
        octomap::OcTree* octomap = nullptr;
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(octomap_);
        if (tree) {
            octomap = dynamic_cast<octomap::OcTree*>(tree);
        } else {
            ROS_ERROR("Failed to call convert Octomap.");
            return false;
        }

        grid_map::Position3 min_bound;
        grid_map::Position3 max_bound;
        octomap->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
        octomap->getMetricMax(max_bound(0), max_bound(1), max_bound(2));

        double probe_min[3] = {probe_range_limit_x_, probe_range_limit_y_, probe_range_limit_z_down_};
        double probe_max[3] = {probe_range_limit_x_, probe_range_limit_y_, probe_range_limit_z_up_};

        for (int i : {0, 1}) {
            if(!std::isnan(bbx_origin_(i))) {
                min_bound(i) = std::max(min_bound(i), bbx_origin_(i));
            }
            if(!std::isnan(bbx_range_(i))) {
                max_bound(i) = std::min(max_bound(i), bbx_origin_(i) + bbx_range_(i));
            }
        }

        if (!grid_map::GridMapOctomapConverter::fromOctomap(*octomap, "elevation", map_, &min_bound, &max_bound)) {
            ROS_ERROR("Failed to call convert Octomap.");
            return false;
        }
        map_.setFrameId(octomap_.header.frame_id);

        if (!filter_chain_.update(map_, outputmap_)) {
            ROS_ERROR("could not update the grid map filter chain!");
            return false;
        }
    }

    if (visualize_grid_map_) {
        grid_map_msgs::GridMap gridMapMessage;
        grid_map::GridMapRosConverter::toMessage(outputmap_, gridMapMessage);
        grid_map_publisher_.publish(gridMapMessage);
    }

    rearched_goal_ = false;
    goal_list_.clear();
    global_plan_.clear();

    Vertex* init_v = new Vertex(start_);
    init_v->parents_ptr.clear();
    init_v->parent_v_ptr = NULL;
    init_v->local_cost = 0.0;

    tree_.clear();
    tree_.push_back(init_v);

    if(!search()){
        ROS_WARN("could not find valid plan. ");
        return false;
    }

    ros::Time now = ros::Time::now();
    res.plan.header.frame_id = "map";
    res.plan.header.stamp = now;
    res.plan.poses.clear();
    for(int i = 0; i < global_plan_.size(); i++){

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now;

        grid_map::Position start;
        start(0) = global_plan_[i].x();
        start(1) = global_plan_[i].y();

        if(i == global_plan_.size() - 1){
            grid_map::Index index;
            outputmap_.getIndex(start, index);

            double e = outputmap_.at("elevation", index);
            if(std::isnan(e))
                continue;

            pose.pose.position.x = start.x();
            pose.pose.position.y = start.y();
            pose.pose.position.z = e;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            res.plan.poses.push_back(pose);

        } else {

            grid_map::Position goal;
            goal(0) = global_plan_[i+1].x();
            goal(1) = global_plan_[i+1].y();

            for (grid_map::LineIterator iterator(outputmap_, start, goal); !iterator.isPastEnd(); ++iterator) {

                grid_map::Position p; outputmap_.getPosition(*iterator, p);
                double e = outputmap_.at("elevation", *iterator);

                if(std::isnan(e))
                    continue;

                pose.pose.position.x = p.x();
                pose.pose.position.y = p.y();
                pose.pose.position.z = e;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;

                res.plan.poses.push_back(pose);
            }
        }
    }

    nav_msgs::Path gui_path = res.plan;
    plan_pub_.publish(gui_path);

    delete init_v;

    if (visualize_grid_map_) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "basic_shapes";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        geometry_msgs::PoseStamped pose = res.plan.poses.back();

        geometry_msgs::Point start_point, end_point;
        start_point.x = end_point.x = goal_.x();
        start_point.y = end_point.y = goal_.y();
        start_point.z = end_point.z = pose.pose.position.z;
        start_point.z += 1.0;

        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        marker.scale.x = 0.05;
        marker.scale.y = 0.15;
        marker.scale.z = 0.3;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        landing_marker_publisher_.publish(marker);
    }

    return !global_plan_.empty();
}

void GridMapUGVPlanner::getBBX(const Eigen::Vector2d& start, const Eigen::Vector2d& goal)
{
    double x_min = std::min(start.x(), goal.x());
    double x_max = std::max(start.x(), goal.x());
    double y_min = std::min(start.y(), goal.y());
    double y_max = std::max(start.y(), goal.y());

    bbx_origin_.x() = x_min - boundary_offset_;
    bbx_origin_.y() = y_min - boundary_offset_;

    bbx_range_.x() = (x_max - x_min) + 2.0 * boundary_offset_;
    bbx_range_.y() = (y_max - y_min) + 2.0 * boundary_offset_;
}

bool GridMapUGVPlanner::search()
{
    double c_best = INFINITY;
    double c_new, c_min, c_near;
    double t_cost = 0.0;

    for(int i = 0; i < max_itr_; i++)
    {
        // double c_temp = INFINITY;
        // for(int j = 0; j < goal_list_.size(); j++){
        //
        //     if(getTotalCost(goal_list_[j]) < c_temp){
        //         c_temp = getTotalCost(goal_list_[j]);
        //     }
        // }

        // if(c_temp < c_best){
        //     if(!parseTree())
        //         ROS_ERROR("could not parse tree.");
        // }

        // c_best = c_temp;

        Eigen::Vector2d c_state = sampling();

        double distance = INFINITY;
        int near_id = 0;

        Eigen::Vector2d nearest_c_state;

        for(int j = 0; j < tree_.size(); j++){

            Eigen::Vector2d ith_c_state = tree_[j]->c_state;
            double d = (c_state - ith_c_state).norm();

            if(d < distance){
                distance = d;
                near_id = j;
                nearest_c_state = ith_c_state;
            }
        }

        Eigen::Vector2d normalized_state = c_state;

        if(delta_ < distance){
            normalized_state = nearest_c_state
                             + (c_state - nearest_c_state).normalized() * delta_;
            distance = delta_;
        }

        ROS_INFO("iter:%d cost:%f tree:%zu x:%f y:%f ", i, c_best, tree_.size(), normalized_state.x(), normalized_state.y());

        Vertex* new_v_ptr = new Vertex(normalized_state);

        if(isOutSide(new_v_ptr) || !isCollisionFree(nearest_c_state, normalized_state, t_cost)){
            ROS_DEBUG_NAMED("base_global_planner","OUTOFBOUNDS[%d]", i);
            continue;
        }

        if(publish_progress_ || sample_pub_.getNumSubscribers() > 0)
        {
            pcl::PointXYZRGB new_point;
            new_point.r = 255;
            new_point.g = 255;
            new_point.b = 255;

            new_point.x = normalized_state.x();
            new_point.y = normalized_state.y();
            new_point.z = 0.0;

            sample_points_.points.push_back(new_point);

            auto msg = sample_points_.makeShared();
            msg->header.frame_id = "map";
            pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);

            sample_pub_.publish(msg);
        }

        double c_trav = t_cost;

        Vertex* min_v_ptr = tree_[near_id];
        c_min = getTotalCost(min_v_ptr) + distance + t_cost;
        // c_min = min_v_ptr->total_cost + distance + t_cost;

        std::vector<Vertex*> near_vs_ptr = near(normalized_state);
        if(!near_vs_ptr.empty())
            ROS_DEBUG_NAMED("base_global_planner","near[%zu]", near_vs_ptr.size());

        for(int j = 0; j < near_vs_ptr.size(); j++){

            c_new = getTotalCost(near_vs_ptr[j])
                  + (new_v_ptr->c_state - near_vs_ptr[j]->c_state).norm();
            // c_new = near_vs_ptr[j]->total_cost
            //       + (new_v_ptr->c_state - near_vs_ptr[j]->c_state).norm();

            if(c_new < c_min){
                if(isCollisionFree(new_v_ptr->c_state, near_vs_ptr[j]->c_state, t_cost)){
                    c_new += t_cost;
                    if(c_new < c_min){
                        // ROS_INFO("change parant.");
                        min_v_ptr = near_vs_ptr[j];
                        c_min = c_new;
                        c_trav = t_cost;
                    }
                }
            }
        }

        // min_v_ptr->children_ptr.push_back(new_v_ptr);

        new_v_ptr->parent_v_ptr = min_v_ptr;
        new_v_ptr->parents_ptr = min_v_ptr->parents_ptr;
        new_v_ptr->parents_ptr.push_back(min_v_ptr);
        // new_v_ptr->total_cost = c_min;
        new_v_ptr->local_cost = (new_v_ptr->c_state - min_v_ptr->c_state).norm() + c_trav;
        ROS_INFO("iter:%d local norm:%f trav:%f", i, (new_v_ptr->c_state - min_v_ptr->c_state).norm(), c_trav);

        // new_v_ptr->local_cost = getTotalCost(new_v_ptr) - getTotalCost(new_v_ptr->parent_v_ptr);
        // new_v_ptr->local_cost = new_v_ptr->total_cost - new_v_ptr->parent_v_ptr->total_cost;

        for(int j = 0; j < near_vs_ptr.size(); j++){

            c_near = getTotalCost(near_vs_ptr[j]);
            // c_near = near_vs_ptr[j]->total_cost;
            // ROS_INFO("parent vertex update. c_near:%f", c_near);
            c_new = getTotalCost(new_v_ptr)
                  + (new_v_ptr->c_state - near_vs_ptr[j]->c_state).norm();
            // c_new = new_v_ptr->total_cost
            //       + (new_v_ptr->c_state - near_vs_ptr[j]->c_state).norm();

            if(c_new < c_near){
                if(isCollisionFree(new_v_ptr->c_state, near_vs_ptr[j]->c_state, t_cost)){
                    c_new += t_cost;
                    if(c_new < c_near){
                        // ROS_INFO("rewireing.");
                        near_vs_ptr[j]->parent_v_ptr = new_v_ptr;
                        near_vs_ptr[j]->parents_ptr = new_v_ptr->parents_ptr;
                        near_vs_ptr[j]->parents_ptr.push_back(new_v_ptr);
                        // near_vs_ptr[j]->total_cost = c_new;
                        near_vs_ptr[j]->local_cost = (near_vs_ptr[j]->c_state - new_v_ptr->c_state).norm() + t_cost;
                        ROS_INFO("iter:%d local norm:%f trav:%f", i, (near_vs_ptr[j]->c_state - new_v_ptr->c_state).norm(), t_cost);

                        // near_vs_ptr[j]->local_cost = getTotalCost(near_vs_ptr[j]) - getTotalCost(near_vs_ptr[j]->parent_v_ptr);
                        // near_vs_ptr[j]->local_cost = near_vs_ptr[j]->total_cost - near_vs_ptr[j]->parent_v_ptr->total_cost;

                        // for(int k = 0; k < near_vs_ptr[k]->children_ptr.size(); k++){
                        //     near_vs_ptr[j]->children_ptr[k]->total_cost += c_new - c_near;
                        // }
                        // updateChildCost(near_vs_ptr[j], c_new - c_near);
                    }
                }
            }
        }

        if(publish_progress_ && tree_pub_.getNumSubscribers() > 0)
        {
            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.resize(tree_.size());

            for(int i = 0; i < tree_.size(); i++){

                marker_array.markers[i].header.frame_id = "map";
                marker_array.markers[i].header.stamp = ros::Time::now();
                marker_array.markers[i].ns = "tree";
                marker_array.markers[i].id = i;
                marker_array.markers[i].lifetime = ros::Duration();

                marker_array.markers[i].type = visualization_msgs::Marker::LINE_LIST;
                marker_array.markers[i].action = visualization_msgs::Marker::ADD;
                marker_array.markers[i].scale.x = marker_array.markers[i].scale.y = marker_array.markers[i].scale.z = 0.01;

                if(tree_[i]->parent_v_ptr != NULL){
                    marker_array.markers[i].points.resize(2);
                    geometry_msgs::Point p;
                    p.x = tree_[i]->c_state.x();
                    p.y = tree_[i]->c_state.y();
                    p.z = 0.0;
                    marker_array.markers[i].points[0] = p;
                    p.x = tree_[i]->parent_v_ptr->c_state.x();
                    p.y = tree_[i]->parent_v_ptr->c_state.y();
                    p.z = 0.0;
                    marker_array.markers[i].points[1] = p;
                }

                marker_array.markers[i].color.r = 1.0f;
                marker_array.markers[i].color.g = 1.0f;
                marker_array.markers[i].color.b = 1.0f;
                marker_array.markers[i].color.a = 1.0f;
            }

            tree_pub_.publish(marker_array);

            ros::Duration(0.1).sleep();
        }

        if(isReachedGoal(new_v_ptr, c_best)){

            ROS_DEBUG_NAMED("base_global_planner","InformedRRTStar global planner finds path. [%d]", i);
            // if(c_best < shortest_path_length_ + 100.0){
            //
            //     ROS_DEBUG_NAMED("base_global_planner","Find shortest path [%f]m. Stop itration.  [%d]", c_best, i);
            //
            //     if(!parseTree())
            //         ROS_ERROR_NAMED("base_global_planner","Could't parse tree.");
            //     return rearched_goal_;
            // }

            if(!parseTree())
                ROS_ERROR_NAMED("base_global_planner","Could't parse tree.");
        }

        tree_.push_back(new_v_ptr);
    }

    ROS_INFO_NAMED("base_global_planner","Full iteration. %zu", tree_.size());

    return rearched_goal_;
}

double GridMapUGVPlanner::getTotalCost(const Vertex* v)
{
    if(!v){
        ROS_WARN("Empty vertex. return nan.");
        return NAN;
    }

    double total_cost = 0.0;
    for(int i = 0; i < v->parents_ptr.size(); i++){

        double local_cost = v->parents_ptr[i]->local_cost;
        if(!std::isnan(local_cost))
            total_cost += local_cost;
    }

    total_cost += v->local_cost;

    // ROS_INFO("Parents size:%zu Total cost: %f", v->parents_ptr.size(), total_cost);
    return total_cost;
}

Eigen::Vector2d GridMapUGVPlanner::sampling()
{
    // pcl::PointXYZRGB new_point;
    // new_point.r = 255;
    // new_point.g = 255;
    // new_point.b = 255;

    Eigen::Vector2d c_state;
    std::uniform_real_distribution<> rand_z(-1.0, 1.0);

    double choice = rand_z(engine_);

    if(1.2 < choice){
        for(int i = 0; i < 2; i++)
        c_state(i) = goal_(i);
    } else {
        for(int i = 0; i < 2; i++)
        c_state(i) = bbx_origin_(i) + bbx_range_(i) * std::fabs(rand_z(engine_));
    }

    // if(publish_progress_ || sample_pub_.getNumSubscribers() > 0)
    // {
    //     new_point.x = c_state.x();
    //     new_point.y = c_state.y();
    //     new_point.z = 0.0;
    //
    //     sample_points_.points.push_back(new_point);
    //
    //     auto msg = sample_points_.makeShared();
    //     msg->header.frame_id = "map";
    //     pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    //
    //     sample_pub_.publish(msg);
    // }


    return c_state;
}

bool GridMapUGVPlanner::parseTree()
{
    global_plan_.clear();

    if(tree_.empty()){
        ROS_ERROR_NAMED("base_global_planner","This planner has not been got tree.");
        return false;
    }

    Vertex* end_v_ptr;

    double c_temp = INFINITY;
    for(int i = 0; i < goal_list_.size(); i++){
        ROS_INFO("%f", getTotalCost(goal_list_[i]));

        if(getTotalCost(goal_list_[i]) < c_temp){
            c_temp = getTotalCost(goal_list_[i]);
            end_v_ptr = goal_list_[i];
        }
    }

    // if(publish_progress_ && tree_pub_.getNumSubscribers() > 0)
    {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        int index = 0;

        for(int i = 0; i < goal_list_.size(); i++){
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "tree";
            marker.id = index;
            marker.lifetime = ros::Duration();

            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.03;

            if(goal_list_[i]->parent_v_ptr != NULL){
                marker.points.resize(2);
                geometry_msgs::Point p;
                p.x = goal_list_[i]->c_state.x();
                p.y = goal_list_[i]->c_state.y();
                p.z = 0.0;
                marker.points[0] = p;
                p.x = goal_list_[i]->parent_v_ptr->c_state.x();
                p.y = goal_list_[i]->parent_v_ptr->c_state.y();
                p.z = 0.0;
                marker.points[1] = p;
            }

            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;

            index++;

            marker_array.markers.push_back(marker);

            for(int j = 0; j < goal_list_[i]->parents_ptr.size(); j++){

                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();
                marker.ns = "tree";
                marker.id = index;
                marker.lifetime = ros::Duration();

                marker.type = visualization_msgs::Marker::LINE_LIST;
                marker.action = visualization_msgs::Marker::ADD;
                marker.scale.x = marker.scale.y = marker.scale.z = 0.03;

                if(goal_list_[i]->parents_ptr[j]->parent_v_ptr != NULL){
                    marker.points.resize(2);
                    geometry_msgs::Point p;
                    p.x = goal_list_[i]->parents_ptr[j]->c_state.x();
                    p.y = goal_list_[i]->parents_ptr[j]->c_state.y();
                    p.z = 0.0;
                    marker.points[0] = p;
                    p.x = goal_list_[i]->parents_ptr[j]->parent_v_ptr->c_state.x();
                    p.y = goal_list_[i]->parents_ptr[j]->parent_v_ptr->c_state.y();
                    p.z = 0.0;
                    marker.points[1] = p;
                }

                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;

                marker_array.markers.push_back(marker);

                index++;
            }
        }

        path_pub_.publish(marker_array);
    }

    ROS_INFO_NAMED("base_global_planner","min[%f] size[%zu] goals[%zu]", c_temp, end_v_ptr->parents_ptr.size(), goal_list_.size());

    for(int i = 0; i < end_v_ptr->parents_ptr.size(); i++){

        Vertex* v_ptr = end_v_ptr->parents_ptr[i];
        global_plan_.push_back(v_ptr->c_state);
    }

    global_plan_.push_back(end_v_ptr->c_state);

    return true;
}

std::vector<Vertex*> GridMapUGVPlanner::near(Eigen::Vector2d& c_state)
{
    std::vector<Vertex*> near;
    near.clear();

    for(int i = 0; i < tree_.size(); i++){
        double d = (tree_[i]->c_state - c_state).norm();
        double r = initial_neaby_radius_ * std::cbrt(std::log(tree_.size()) / tree_.size());
        // double r = 5.0 * std::cbrt(std::log(tree_.size()) / tree_.size());
        if(d < r) near.push_back(tree_[i]);
    }

    return near;
}

bool GridMapUGVPlanner::isReachedGoal(Vertex* v, double& c_best)
{
    double d = (goal_ - v->c_state).norm();
    // if(d < goal_tolerance_ && v->cost < 3.0 * shortest_path_length_){
    if(d < goal_tolerance_){
        rearched_goal_ = true;
        goal_list_.push_back(v);

        return true;
    }

    return false;
}

bool GridMapUGVPlanner::isOutSide(Vertex* v)
{
    for(int i = 0; i < 2; i++){
        if((v->c_state(i) < bbx_origin_(i)) ||
           (bbx_origin_(i) + bbx_range_(i) < v->c_state(i))) return true;
    }

    return false;
}

bool GridMapUGVPlanner::isCollisionFree(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,  double& t_cost)
{
    grid_map::Position start;
    start(0) = p1.x();
    start(1) = p1.y();
    grid_map::Position goal;
    goal(0) = p2.x();
    goal(1) = p2.y();

    int i = 0;
    t_cost = 0.0;

    for (grid_map::LineIterator iterator(outputmap_, start, goal); !iterator.isPastEnd(); ++iterator) {
        grid_map::Position p; outputmap_.getPosition(*iterator, p);
        double t = outputmap_.at("traversability_inflated", *iterator);
        if(std::isnan(t))
            continue;
        t_cost += 1.0 - t;
        if(t < probe_traversability_threshold_){
            ROS_INFO("%f", t);
            return false;
        }
        i++;
    }

    t_cost *= traversability_cost_weight_;
    t_cost /= i;

    return true;
}

bool GridMapUGVPlanner::getGlobalPose(geometry_msgs::PoseStamped& global_pose)
{
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = "base_link";
    robot_pose.header.stamp = ros::Time();

    try
    {
        tf_.transform(robot_pose, global_pose, "map");
    }
    catch (tf2::LookupException& ex)
    {
        ROS_ERROR_THROTTLE_NAMED(1.0,"commander","No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
        ROS_ERROR_THROTTLE_NAMED(1.0,"commander","Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
        ROS_ERROR_THROTTLE_NAMED(1.0,"commander","Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
    }

    return true;
}
