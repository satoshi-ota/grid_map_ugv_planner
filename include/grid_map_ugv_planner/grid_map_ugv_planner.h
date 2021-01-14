#pragma once
#include <string>
#include <math.h>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <grid_map_loader/GridMapLoader.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <filters/filter_chain.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/GetPlan.h>
#include <pcl_ros/point_cloud.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

struct Vertex
{
public:
    Vertex(){}
    Vertex(Eigen::Vector2d& _pos)
        :c_state(_pos),
         parent_v_ptr(NULL),
         // total_cost(0.0),
         local_cost(0.0){}
    ~Vertex(){}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector2d c_state;

    // std::vector<Vertex*> children_ptr;
    std::vector<Vertex*> parents_ptr;
    Vertex* parent_v_ptr;

    // double total_cost;
    double local_cost;
};

class GridMapUGVPlanner
{
public:
    GridMapUGVPlanner(tf2_ros::Buffer& tf);
    ~GridMapUGVPlanner();

    void octomap_callback(const octomap_msgs::Octomap::ConstPtr& msg);
    void convert_and_publish();
    bool get_pos_callback(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res);
    bool getGlobalPose(geometry_msgs::PoseStamped& global_pose);

    void getBBX(const Eigen::Vector2d& start, const Eigen::Vector2d& goal);

    bool search();
    Eigen::Vector2d sampling();
    std::vector<Vertex*> near(Eigen::Vector2d& c_state);
    bool isReachedGoal(Vertex* v, double& c_best);
    bool isOutSide(Vertex* v);
    bool isCollisionFree(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double& t_cost);
    bool parseTree();

    double getTraversabilityCost(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);

    double getTotalCost(const Vertex* v);

private:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ros::Publisher plan_pub_;
    ros::Publisher sample_pub_;
    ros::Publisher tree_pub_;
    ros::Publisher path_pub_;
    pcl::PointCloud<pcl::PointXYZRGB> sample_points_;
    //! Path the ROS bag to be published.
    std::string filePath_;

    //! Topic name of the grid map in the ROS bag.
    std::string bagTopic_;

    //! Topic name of the grid map to be loaded.
    std::string publishTopic_;

    bool use_bagfile_;

    Eigen::Vector2d bbx_origin_;
    Eigen::Vector2d bbx_range_;

    std::vector<Vertex*> tree_;
    std::vector<Vertex*> goal_list_;

    std::vector<Eigen::Vector2d> global_plan_;

    Eigen::Vector2d start_;
    Eigen::Vector2d goal_;
    double shortest_path_length_;
    std::random_device seed_gen_;
    std::mt19937 engine_;

    bool rearched_goal_;
    bool publish_progress_;

    int max_itr_;
    double delta_;
    double goal_tolerance_;
    double path_len_tolerance_;
    double select_goal_rate_;
    double obstacle_margin_;
    double boundary_offset_;
    double trajectory_res_;

    double traversability_cost_weight_;
    double initial_neaby_radius_;

    tf2_ros::Buffer& tf_;

    grid_map::GridMap map_;
    grid_map::GridMap outputmap_;

    std::string octomap_service_;
    std::string probe_tf_;
    std::string filter_chain_parameter_name_;
    filters::FilterChain<grid_map::GridMap> filter_chain_;

    ros::ServiceServer probe_service_;
    ros::Publisher grid_map_publisher_;
    ros::Publisher octomap_publisher_;
    ros::Publisher landing_marker_publisher_;
    ros::Subscriber octomap_subscriber_;

    octomap_msgs::Octomap octomap_;

    float probe_range_limit_x_;
    float probe_range_limit_y_;
    float probe_range_limit_z_down_;
    float probe_range_limit_z_up_;
    double probe_traversability_threshold_;
    float landing_traversability_threshold_;
    bool octomap_received_;
    bool visualize_position_;
    bool visualize_grid_map_;
    bool visualize_elevation_map_;
};
