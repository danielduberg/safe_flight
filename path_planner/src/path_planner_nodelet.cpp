#include <pluginlib/class_list_macros.h>

#include <path_planner/path_planner_nodelet.h>

namespace path_planner
{

    void PPNodelet::onInit()
    {
        ros::NodeHandle & nh = getNodeHandle();
        ros::NodeHandle & nh_priv = getPrivateNodeHandle();

        PP = new PathPlanner(nh, nh_priv);
    }


    PLUGINLIB_DECLARE_CLASS(path_planner, PP, path_planner::PPNodelet, nodelet::Nodelet);
}
