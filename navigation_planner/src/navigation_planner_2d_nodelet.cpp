#include <pluginlib/class_list_macros.h>

#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <navigation_planner/navigation_planner_2d.h>

namespace navigation_planner_2d
{
    
    class NP2DNodelet : public nodelet::Nodelet
    {
    private:
        virtual void onInit();
    };

    void NP2DNodelet::onInit()
    {
        ros::NodeHandle & nh = getNodeHandle();
        ros::NodeHandle & nh_priv = getPrivateNodeHandle();
    }


    PLUGINLIB_DECLARE_CLASS(navigation_planner_2d, NP2D, navigation_planner_2d::NP2DNodelet, nodelet::Nodelet);
}
