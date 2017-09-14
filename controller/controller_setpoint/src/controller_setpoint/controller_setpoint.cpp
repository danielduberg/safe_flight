#include <controller_setpoint/controller_setpoint.h>

#include <controller_msgs/Controller.h>

#include <tf2/utils.h>

namespace controller_setpoint
{
    ControllerSetpoint::ControllerSetpoint(ros::NodeHandle nh, ros::NodeHandle nh_priv, ros::NodeHandle nh_controller)
        : nh_(nh)
        , nh_priv_(nh_priv_)
        , nh_controller_(nh_controller)
        , tf_listener_(tf_buffer_)
    {
        initParam(nh_);

        controller_pub_ = nh_controller_.advertise<controller_msgs::Controller>("setpoint", 1);

        setpoint_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/nav_2d", 1, &ControllerSetpoint::setpointCallback, this);

        current_setpoint_.pose.orientation.w = 1;

        publish_timer_ = nh_.createTimer(ros::Duration(1.0 / frequency_), &ControllerSetpoint::publishCallback, this);
    }

    void ControllerSetpoint::initParam(ros::NodeHandle & nh)
    {
        nh.param<double>("frequency", frequency_, 60.0);

        nh.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
    }

    void ControllerSetpoint::setpointCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
    {
        current_setpoint_ = *msg;
    }

    double ControllerSetpoint::getYaw(const geometry_msgs::Quaternion & orientation)
    {
        return tf2::getYaw(orientation);
    }

    geometry_msgs::PoseStamped ControllerSetpoint::transformPose(const geometry_msgs::PoseStamped & pose, const std::string & to_frame)
    {
        geometry_msgs::TransformStamped transform;
        try
        {
            transform = tf_buffer_.lookupTransform(to_frame, pose.header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException & ex)
        {
            ROS_WARN_STREAM_THROTTLE_NAMED(1, "safe_flight/controller_setpoint", ex.what());

            throw ex;
        }

        geometry_msgs::PoseStamped setpoint;

        tf2::doTransform(pose, setpoint, transform);

        return setpoint;
    }

    void ControllerSetpoint::publishCallback(const ros::TimerEvent & event)
    {
        geometry_msgs::PoseStamped setpoint;

        try
        {
            current_setpoint_.header.stamp = ros::Time::now();  // So that we get the most recent transform

            setpoint = transformPose(current_setpoint_, robot_base_frame_);
        }
        catch (tf2::TransformException & ex)
        {
            // Such that we just hold position
            setpoint.pose.position.x = 0.0;
            setpoint.pose.position.y = 0.0;
            setpoint.pose.position.z = 0.0;

            tf2::Quaternion q = tf2::Quaternion::getIdentity();
            setpoint.pose.orientation.x = q.x();
            setpoint.pose.orientation.y = q.y();
            setpoint.pose.orientation.z = q.z();
            setpoint.pose.orientation.w = q.w();
        }

        controller_msgs::Controller out;
        out.header.stamp = ros::Time::now();
        out.header.frame_id = robot_base_frame_;

        out.twist_stamped.header.stamp = ros::Time::now();
        out.twist_stamped.header.frame_id = robot_base_frame_;

        out.twist_stamped.twist.linear.x = setpoint.pose.position.x * 0.5;
        out.twist_stamped.twist.linear.y = setpoint.pose.position.y * 0.5;
        out.twist_stamped.twist.linear.z = 0.0;
        if (std::fabs(setpoint.pose.position.z) > 0.05)
        {
            out.twist_stamped.twist.linear.z = setpoint.pose.position.z;
        }

        out.twist_stamped.twist.angular.z = getYaw(setpoint.pose.orientation);

        out.arm = true;

        controller_pub_.publish(out);
    }
}
