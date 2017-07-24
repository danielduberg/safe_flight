#include <pluginlib/class_list_macros.h>

#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <controller_setpoint/controller_setpoint.h>

namespace controller_setpoint
{
    
    class CSNodelet : public nodelet::Nodelet
    {
    private:
        virtual void onInit();
    };

    void CSNodelet::onInit()
    {
        ros::NodeHandle & nh = getNodeHandle();
        ros::NodeHandle & nh_priv = getPrivateNodeHandle();
        ros::NodeHandle nh_controller("controller");
    }


    PLUGINLIB_DECLARE_CLASS(controller_setpoint, CS, controller_setpoint::CSNodelet, nodelet::Nodelet);
}
