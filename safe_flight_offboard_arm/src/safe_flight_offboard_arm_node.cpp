#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>

void arm(ros::ServiceClient & arming_client)
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

void activateOffboard(ros::ServiceClient & set_mode_client, const ros::Publisher & pub)
{
    //if (current_state.mode != "OFFBOARD")
    //{
        // Send a few ... before switching to offboard
        for (int i = 0; ros::ok() && i < 100; ++i)
        {
            mavros_msgs::PositionTarget pos_tar;
            pos_tar.header.stamp = ros::Time::now();
            pos_tar.header.frame_id = "";
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
    //}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offboard_arm");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    arm(arming_client);
    activateOffboard(set_mode_client, setpoint_pub);

    return 0;
}
