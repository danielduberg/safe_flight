#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf/transform_listener.h>

namespace controller_setpoint
{
    class ControllerSetpoint
    {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;
        ros::NodeHandle nh_controller_;

        ros::Subscriber setpoint_sub_;
        ros::Subscriber pose_sub_;

        ros::Publisher controller_pub_;

        geometry_msgs::PoseStamped current_setpoint_;

        double frequency_;

        ros::Timer publish_timer_;

        // Drone
        geometry_msgs::PoseStamped current_pose_;
        tf::TransformListener tf_listener_;


    public:
        ControllerSetpoint(ros::NodeHandle nh, ros::NodeHandle nh_priv, ros::NodeHandle nh_controller);

    private:
        void initParam(ros::NodeHandle & nh);

        void setpointCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);

        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);

        double getYaw(const geometry_msgs::Quaternion & orientation);

        void publishCallback(const ros::TimerEvent & event);
    };
}
