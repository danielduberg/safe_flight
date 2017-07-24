#pragma once

#include <ros/ros.h>

namespace navigation_planner_3d
{
    class NavigationPlanner3D
    {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;

    public:
        NavigationPlanner3D(ros::NodeHandle nh, ros::NodeHandle nh_priv);

    };
}
