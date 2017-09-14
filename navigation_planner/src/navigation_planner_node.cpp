#include <ros/ros.h>

#include <navigation_planner/navigation_planner.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_planner");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    navigation_planner::NavigationPlanner NP(nh, nh_priv);

    ros::spin();

    return 0;
}
