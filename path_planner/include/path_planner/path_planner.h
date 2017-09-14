#pragma once

#include <ros/ros.h>

#include <path_planner/PathPlannerAction.h>
#include "path_planner/search_algorithm.h"

#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <map_msgs/OccupancyGridUpdate.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace path_planner
{

    class PathPlanner
    {

    private:
        ros::NodeHandle & nh_;
        ros::NodeHandle & nh_priv_;

        SearchAlgorithm * search_algorithm_;

        // Action server
        // NodeHandle instance must be created before this line. Otherwise strange error occurs.
        actionlib::SimpleActionServer<path_planner::PathPlannerAction> as_;
        path_planner::PathPlannerFeedback feedback_;
        path_planner::PathPlannerResult result_;

        // Subscribers
        //ros::Subscriber velocity_sub_;
        ros::Subscriber odom_sub_;
        ros::Subscriber map_sub_;
        ros::Subscriber map_update_sub_;

        // Publishers
        ros::Publisher setpoint_pub_;
        ros::Publisher map_pub_;

        // TF2
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;

        // Frame names
        std::string robot_base_frame_;

        // Error parameters
        double altitude_error_;
        double yaw_error_;
        double position_error_;

        // Velocity parameters
        double max_xy_setpoint_velocity_;
        double max_z_setpoint_velocity_;
        double max_yaw_setpoint_rate_;
        //double altitude_velocity_threshold_;
        //double yaw_velocity_threshold_;
        //double position_velocity_threshold_;

        // Map
        nav_msgs::OccupancyGrid map_;

        // Robot
        double radius_;
        //geometry_msgs::TwistStamped current_velocity_;
        double altitude_;
        double current_xy_velocity_;
        double current_z_velocity_;
        double current_yaw_rate_;

        // General
        double frequency_;


    public:
        PathPlanner(ros::NodeHandle & nh, ros::NodeHandle & nh_priv, const std::string & action_server_name = "path_planner");

        ~PathPlanner(void);

    private:
        bool waitForMap();

        void executeCallback(const path_planner::PathPlannerGoal::ConstPtr & goal);

        double getYaw(const geometry_msgs::Quaternion & orientation);

        bool isMapDiff(const nav_msgs::OccupancyGrid & map_1, const nav_msgs::OccupancyGrid & map_2);

        bool reachedCheckpoint(const geometry_msgs::PoseStamped & current_pose, const geometry_msgs::PoseStamped & checkpoint);

        geometry_msgs::PoseStamped getPose(const std::string & target_frame, const std::string & source_frame, double timeout);

        void odomCallback(const nav_msgs::Odometry::ConstPtr & msg);

        int getIndex(const int x, const int y);

        void mapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr & msg);

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg);

        bool isStill();

        void initParams(const ros::NodeHandle & nh);

    };

}
