#include <ros/ros.h>

#include <path_planner/path_planner.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    path_planner::PathPlanner PP(nh, nh_priv);

    ros::spin();

    return 0;
}

