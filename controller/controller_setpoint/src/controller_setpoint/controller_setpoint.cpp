#include <controller_setpoint/controller_setpoint.h>

#include <controller_msgs/Controller.h>

#include <tf/transform_listener.h>

namespace controller_setpoint
{
    ControllerSetpoint::ControllerSetpoint(ros::NodeHandle nh, ros::NodeHandle nh_priv, ros::NodeHandle nh_controller)
        : nh_(nh)
        , nh_priv_(nh_priv_)
        , nh_controller_(nh_controller)
    {
        initParam(nh_);

        controller_pub_ = nh_controller_.advertise<controller_msgs::Controller>("setpoint", 1);

        setpoint_sub_ = nh_.subscribe("/nav_2D", 1, &ControllerSetpoint::setpointCallback, this);
        pose_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &ControllerSetpoint::poseCallback, this);

        current_setpoint_.pose.orientation.w = 1;

        publish_timer_ = nh_.createTimer(ros::Duration(1.0d / frequency_), &ControllerSetpoint::publishCallback, this);
    }

    void ControllerSetpoint::initParam(ros::NodeHandle & nh)
    {
        nh.param<double>("frequency", frequency_, 60);
    }

    void ControllerSetpoint::setpointCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
    {
        current_setpoint_ = *msg;
        current_setpoint_.pose.position.z = 1;
        current_setpoint_.header.stamp = ros::Time(0);
    }

    void ControllerSetpoint::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
    {
        current_pose_ = *msg;
    }

    double ControllerSetpoint::getYaw(const geometry_msgs::Quaternion & orientation)
    {
        double roll, pitch, yaw;

        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        tf2::Matrix3x3 m(q);
        m.getEulerYPR(yaw, pitch, roll);

        return yaw;
    }

    void ControllerSetpoint::publishCallback(const ros::TimerEvent & event)
    {
        geometry_msgs::PoseStamped setpoint;

        try
        {
            tf_listener_.transformPose("robot_yaw", current_setpoint_, setpoint);
        }
        catch (tf::TransformException & ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }

        ROS_FATAL_STREAM("Position:");
        ROS_FATAL_STREAM("\tx: " << current_setpoint_.pose.position.x << " -> " << setpoint.pose.position.x);
        ROS_FATAL_STREAM("\ty: " << current_setpoint_.pose.position.y << " -> " << setpoint.pose.position.y);
        ROS_FATAL_STREAM("\tz: " << current_setpoint_.pose.position.z << " -> " << setpoint.pose.position.z);

        ROS_FATAL_STREAM("Orientation:");
        ROS_FATAL_STREAM("\tx: " << current_setpoint_.pose.orientation.x << " -> " << setpoint.pose.orientation.x);
        ROS_FATAL_STREAM("\ty: " << current_setpoint_.pose.orientation.y << " -> " << setpoint.pose.orientation.y);
        ROS_FATAL_STREAM("\tz: " << current_setpoint_.pose.orientation.z << " -> " << setpoint.pose.orientation.z);
        ROS_FATAL_STREAM("\tw: " << current_setpoint_.pose.orientation.w << " -> " << setpoint.pose.orientation.w);


        controller_msgs::Controller out;

        out.x = setpoint.pose.position.x;
        out.y = setpoint.pose.position.y;
        out.z = setpoint.pose.position.z;

        out.yaw = getYaw(setpoint.pose.orientation);

        out.arm = true;

        controller_pub_.publish(out);
    }
}
