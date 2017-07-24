#pragma once

#include <ros/ros.h>

namespace navigation_planner_2d
{
    class NavigationPlanner2D
    {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;

        ros::Subscriber map_sub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber waypoint_sub_;

    public:
        NavigationPlanner2D(ros::NodeHandle nh, ros::NodeHandle nh_priv);

    };
}
