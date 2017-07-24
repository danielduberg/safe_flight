#include <ros/ros.h>

#include <navigation_planner/navigation_planner_3d.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_planner_3d");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    navigation_planner_3d::NavigationPlanner3D NP3D(nh, nh_priv);

    ros::spin();

    return 0;
}
