#include <ros/ros.h>

#include <navigation_planner/navigation_planner_2d.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_planner_2d");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    navigation_planner_2d::NavigationPlanner2D NP2D(nh, nh_priv);

    ros::spin();

    return 0;
}
