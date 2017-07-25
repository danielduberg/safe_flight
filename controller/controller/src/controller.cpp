#include <ros/ros.h>

#include <controller_msgs/Controller.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ros::Publisher pub;

mavros_msgs::State::ConstPtr current_state;

geometry_msgs::PoseStamped::ConstPtr current_pose;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

double max_x_vel;
double max_y_vel;
double max_z_vel;
double max_yaw_rate;

geometry_msgs::PoseStamped hold_position;

void stateCallback(const mavros_msgs::State::ConstPtr & msg)
{
    current_state = msg;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    current_pose = msg;
}

void arm_disarm(bool arm)
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;

    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        if (arm)
        {
            ROS_INFO_STREAM_THROTTLE(1, "Vehicle armed");

            if (current_state->mode != "OFFBOARD")
            {
                // Send a few vel before starting
                geometry_msgs::PoseStamped pose;
                for (int i = 0; ros::ok() && i < 100; ++i)
                {
                    pub.publish(pose);
                }

                mavros_msgs::SetMode set_mode;
                set_mode.request.custom_mode = "OFFBOARD";

                if (set_mode_client.call(set_mode) && set_mode.response.success)
                {
                    ROS_INFO_STREAM_THROTTLE(1, "Offboard mode enabled");
                }
                else
                {
                    ROS_ERROR_STREAM_THROTTLE(1, "Could not enable offboard mode");
                }
            }            
        }
        else
        {
            ROS_INFO_STREAM_THROTTLE(1, "Vehicle disarmed");
        }
    }
    else
    {
        if (arm)
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Could not arm vehicle");
        }
        else
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Could not disarm vehicle");
        }
    }
}

void land()
{
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "AUTO.LAND"; // Makes it land

    if (set_mode_client.call(set_mode) && set_mode.response.success)
    {
        ROS_INFO_STREAM_THROTTLE(1, "Switched to landing mode");
    }
    else
    {
        ROS_ERROR_STREAM_THROTTLE(1, "Could not switch to landing mode");
    }
}

void lift()
{
    // Todo: what should this do?!
}

double getCurrentYaw()
{
    double roll, pitch, yaw;

    geometry_msgs::Quaternion cur_or = current_pose->pose.orientation;

    tf2::Quaternion q(cur_or.x, cur_or.y, cur_or.z, cur_or.w);
    tf2::Matrix3x3 m(q);
    m.getEulerYPR(yaw, pitch, roll);

    return yaw;
}

void fly(float x, float y, float z, float yaw)
{


    double current_yaw = getCurrentYaw();

    // Move!
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.header.frame_id = "robot";

    if (x == 0 && y == 0)
    {
        ROS_FATAL_STREAM("x: " << hold_position.pose.position.x << " - " << current_pose->pose.position.x);
        ROS_FATAL_STREAM("y: " << hold_position.pose.position.y << " - " << current_pose->pose.position.y);

        // Linear
        twist.twist.linear.x = hold_position.pose.position.x - current_pose->pose.position.x;
        twist.twist.linear.y = hold_position.pose.position.y - current_pose->pose.position.y;
        twist.twist.linear.z = max_z_vel * z;

        // Angular
        twist.twist.angular.x = 0.0;
        twist.twist.angular.y = 0.0;
        twist.twist.angular.z = max_yaw_rate * yaw;

        pub.publish(twist);
    }
    else
    {
        // Linear
        // Source: https://www.siggraph.org/education/materials/HyperGraph/modeling/mod_tran/2drota.htm
        twist.twist.linear.x = max_x_vel * ((x * std::cos(current_yaw)) - (y * std::sin(current_yaw)));
        twist.twist.linear.y = max_y_vel * ((y * std::cos(current_yaw)) + (x * std::sin(current_yaw)));
        twist.twist.linear.z = max_z_vel * z;

        // Angular
        twist.twist.angular.x = 0;
        twist.twist.angular.y = 0;
        twist.twist.angular.z = max_yaw_rate * yaw;

        twist.twist.linear.x = std::max(-1.0d, std::min(1.0d, twist.twist.linear.x));
        twist.twist.linear.y = std::max(-1.0d, std::min(1.0d, twist.twist.linear.y));
        twist.twist.linear.z = std::max(-1.0d, std::min(1.0d, twist.twist.linear.z));
        twist.twist.angular.z = std::max(-1.0d, std::min(1.0d, twist.twist.angular.z));

        pub.publish(twist);

        hold_position.pose.position.x = current_pose->pose.position.x;
        hold_position.pose.position.y = current_pose->pose.position.y;
    }
}

void inCallback(const controller_msgs::Controller::ConstPtr & msg)
{
    if (msg->disarm)
    {
        //ROS_ERROR_STREAM("1");
        arm_disarm(false);
    }
    else if (msg->arm && !current_state->armed)
    {
        //ROS_ERROR_STREAM("2");
        arm_disarm(true);
    }
    else if (msg->land)
    {
        //ROS_ERROR_STREAM("3");
        land();
    }
    else if (msg->lift)
    {
        //ROS_ERROR_STREAM("4");
        lift();
    }
    else if (current_state->armed && current_state->mode == "OFFBOARD")
    {
        //ROS_ERROR_STREAM("5");
        fly(msg->x, msg->y, msg->z, msg->yaw);
    }
    else
    {
        //ROS_ERROR_STREAM("6");
    }
}

void init_param(ros::NodeHandle & nh)
{
    nh.param("max_x_vel", max_x_vel, 1.0d);
    nh.param("max_y_vel", max_y_vel, 1.0d);
    nh.param("max_z_vel", max_z_vel, 1.0d);
    nh.param("max_yaw_rate", max_yaw_rate, 0.7d);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::NodeHandle nh_controller("controller");

    init_param(nh_priv);

    pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, stateCallback);

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, poseCallback);

    ros::Subscriber control_sub = nh.subscribe("collision_free_control", 1, inCallback);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::spin();

    return 0;
}
