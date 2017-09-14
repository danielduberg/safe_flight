#include <ros/ros.h>

#include <controller_setpoint/controller_setpoint.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_setpoint");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::NodeHandle nh_controller("controller");

    controller_setpoint::ControllerSetpoint cs(nh, nh_priv, nh_controller);

    ros::spin();

    return 0;
}
