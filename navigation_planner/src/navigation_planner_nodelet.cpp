#include <pluginlib/class_list_macros.h>

#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <navigation_planner/navigation_planner.h>

namespace navigation_planner
{

    class NPNodelet : public nodelet::Nodelet
    {
    private:
        navigation_planner::NavigationPlanner * NP;

    private:
        virtual void onInit();
    };

    void NPNodelet::onInit()
    {
        ros::NodeHandle & nh = getNodeHandle();
        ros::NodeHandle & nh_priv = getPrivateNodeHandle();

        NP = new navigation_planner::NavigationPlanner(nh, nh_priv);
    }


    PLUGINLIB_DECLARE_CLASS(navigation_planner, NP, navigation_planner::NPNodelet, nodelet::Nodelet);
}
