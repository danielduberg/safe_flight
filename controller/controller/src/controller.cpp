#include <ros/ros.h>

#include <controller_msgs/Controller.h>

//#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <mavros_msgs/PositionTarget.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ros::Publisher pub;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

double max_xy_vel;
double max_z_vel;
double max_yaw_rate;

//geometry_msgs::Twist current_velocity;

std::string robot_base_frame;

geometry_msgs::PoseStamped hold_position;

void stateCallback(const mavros_msgs::State::ConstPtr & msg)
{
    current_state = *msg;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    current_pose = *msg;
}

void arm()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO_STREAM_THROTTLE_NAMED(1, "safe_flight/controller", "Vehicle armed");
    }
    else
    {
        ROS_INFO_STREAM_THROTTLE_NAMED(1, "safe_flight/controller", "Could not arm vehicle");
    }
}


void disarm()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;

    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO_STREAM_THROTTLE_NAMED(1, "safe_flight/controller", "Vehicle disarmed");
    }
    else
    {
        ROS_INFO_STREAM_THROTTLE_NAMED(1, "safe_flight/controller", "Could not disarm vehicle");
    }
}

void activateOffboard()
{
    if (current_state.mode != "OFFBOARD")
    {
        // Send a few ... before switching to offboard
        for (int i = 0; ros::ok() && i < 100; ++i)
        {
            mavros_msgs::PositionTarget pos_tar;
            pos_tar.header.stamp = ros::Time::now();
            pos_tar.header.frame_id = robot_base_frame;
            pos_tar.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            pos_tar.type_mask = mavros_msgs::PositionTarget::IGNORE_PX
                    | mavros_msgs::PositionTarget::IGNORE_PY
                    | mavros_msgs::PositionTarget::IGNORE_PZ
                    | mavros_msgs::PositionTarget::IGNORE_AFX
                    | mavros_msgs::PositionTarget::IGNORE_AFY
                    | mavros_msgs::PositionTarget::IGNORE_AFZ
                    | mavros_msgs::PositionTarget::FORCE            // Should this one be here?!
                    | mavros_msgs::PositionTarget::IGNORE_YAW;
            pub.publish(pos_tar);
        }

        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = "OFFBOARD";

        if (set_mode_client.call(set_mode) && set_mode.response.success)
        {
            ROS_INFO_STREAM_THROTTLE_NAMED(1, "safe_flight/controller", "Offboard mode enabled");
        }
        else
        {
            ROS_INFO_STREAM_THROTTLE_NAMED(1, "safe_flight/controller", "Could not enable offboard mode");
        }
    }

    hold_position.pose.position.x = current_pose.pose.position.x;
    hold_position.pose.position.y = current_pose.pose.position.y;
}

void land()
{
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "AUTO.LAND"; // Makes it land

    if (set_mode_client.call(set_mode) && set_mode.response.success)
    {
        ROS_INFO_STREAM_THROTTLE_NAMED(1, "safe_flight/controller", "Switched to landing mode");
    }
    else
    {
        ROS_INFO_STREAM_THROTTLE_NAMED(1, "safe_flight/controller", "Could not switch to landing mode");
    }
}

void lift()
{
    // Todo: what should this do?!
}

double clamp(double value, double min, double max)
{
    return std::min(std::max(value, min), max);
}

void fly(geometry_msgs::TwistStamped twist_stamped)
{
    // TODO: Do transform

    // Move!
    mavros_msgs::PositionTarget pos_tar;
    pos_tar.header.stamp = ros::Time::now();
    pos_tar.header.frame_id = robot_base_frame;
    pos_tar.type_mask = mavros_msgs::PositionTarget::IGNORE_PX
            | mavros_msgs::PositionTarget::IGNORE_PY
            | mavros_msgs::PositionTarget::IGNORE_PZ
            | mavros_msgs::PositionTarget::IGNORE_AFX
            | mavros_msgs::PositionTarget::IGNORE_AFY
            | mavros_msgs::PositionTarget::IGNORE_AFZ
            | mavros_msgs::PositionTarget::FORCE            // Should this one be here?!
            | mavros_msgs::PositionTarget::IGNORE_YAW;

    pos_tar.velocity.z = max_z_vel * clamp(twist_stamped.twist.linear.z, -1.0, 1.0);
    pos_tar.yaw_rate = max_yaw_rate * clamp(twist_stamped.twist.angular.z, -1.0, 1.0);

    // Hold position!
    if (twist_stamped.twist.linear.x == 0 && twist_stamped.twist.linear.y == 0)
    {
        // So this is with respect to the "world"
        pos_tar.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        pos_tar.velocity.x = hold_position.pose.position.x - current_pose.pose.position.x;
        pos_tar.velocity.y = hold_position.pose.position.y - current_pose.pose.position.y;
    }
    else
    {
        // So this is with respect to the FCU
        pos_tar.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

        pos_tar.velocity.y = twist_stamped.twist.linear.x;
        pos_tar.velocity.x = -twist_stamped.twist.linear.y;


        hold_position.pose.position.x = current_pose.pose.position.x;
        hold_position.pose.position.y = current_pose.pose.position.y;
    }

    // Restrict the xy velocity
    double xy_vel = std::sqrt(std::pow(pos_tar.velocity.x, 2) + std::pow(pos_tar.velocity.y, 2));
    double direction = std::atan2(pos_tar.velocity.y, pos_tar.velocity.x);

    xy_vel = clamp(xy_vel, -max_xy_vel, max_xy_vel);

    pos_tar.velocity.x = xy_vel * std::cos(direction);
    pos_tar.velocity.y = xy_vel * std::sin(direction);


    // Compansate for current velocity
    //if (std::fabs(current_velocity.linear.x) > std::fabs(pos_tar.velocity.y))
    {
    //    pos_tar.velocity.x += (pos_tar.velocity.x - current_velocity.linear.y);
    }
    //if (std::fabs(current_velocity.linear.y) > std::fabs(pos_tar.velocity.x))
    {
    //    pos_tar.velocity.y += (pos_tar.velocity.y - current_velocity.linear.x);
    }

    // Publish
    pub.publish(pos_tar);
}

void controlCallback(const controller_msgs::Controller::ConstPtr & msg)
{
    if (msg->disarm)
    {
        disarm();
    }
    else if (msg->arm && !current_state.armed)
    {
        /*
        arm();

        if (current_state.mode != "OFFBOARD")
        {
            activateOffboard();
        }
        */
    }
    else if (msg->land)
    {
        land();
    }
    else if (msg->lift)
    {
        lift();
    }
    else // if (current_state.armed)
    {
        /*
        if (current_state.mode != "OFFBOARD")
        {
            activateOffboard();
        }
        */

        fly(msg->twist_stamped);
    }
}

/*
void odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
    current_velocity = msg->twist.twist;
}
*/

void initParams(ros::NodeHandle & nh)
{
    nh.param<double>("max_xy_vel", max_xy_vel, 1.0);
    nh.param<double>("max_z_vel", max_z_vel, 1.0);
    nh.param<double>("max_yaw_rate", max_yaw_rate, 0.5);

    nh.param<std::string>("robot_base_frame", robot_base_frame, "base_link");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::NodeHandle nh_controller("controller");

    initParams(nh_priv);

    pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, poseCallback);
    ros::Subscriber control_sub = nh.subscribe<controller_msgs::Controller>("collision_free_control", 1, controlCallback);
//    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 1, odomCallback);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::spin();

    return 0;
}
