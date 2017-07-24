#include <pluginlib/class_list_macros.h>

#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <navigation_planner/navigation_planner_3d.h>

namespace navigation_planner_3d
{
    
    class NP3DNodelet : public nodelet::Nodelet
    {
    private:
        virtual void onInit();
    };

    void NP3DNodelet::onInit()
    {
        ros::NodeHandle & nh = getNodeHandle();
        ros::NodeHandle & nh_priv = getPrivateNodeHandle();
    }


    PLUGINLIB_DECLARE_CLASS(navigation_planner_3d, NP3D, navigation_planner_3d::NP3DNodelet, nodelet::Nodelet);
}
