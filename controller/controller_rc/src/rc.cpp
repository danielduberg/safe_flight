#include <ros/ros.h>

#include <controller_msgs/Controller.h>

#include <mavros_msgs/RCIn.h>

#include <assert.h>

ros::Publisher pub;

ros::Time arm_time, disarm_time;

float x_min, x_max, x_steady, x_deadzone;
float y_min, y_max, y_steady, y_deadzone;
float z_min, z_max, z_steady, z_deadzone;
float yaw_min, yaw_max, yaw_steady, yaw_deadzone;

int x_channel, y_channel, z_channel, yaw_channel;

float arm_timeout, disarm_timeout;

bool arm(float x, float y)
{
    if (x == -1.0f && y == 1.0f)
    {
        if ((ros::Time::now() - arm_time).toSec() > arm_timeout)
        {
            return true;
        }
    }
    else
    {
        arm_time = ros::Time::now();
    }

    return false;
}

bool disarm(float x, float y)
{
    if (x == -1.0f && y == -1.0f)
    {
        if ((ros::Time::now() - disarm_time).toSec() > disarm_timeout)
        {
            return true;
        }
    }
    else
    {
        disarm_time = ros::Time::now();
    }

    return false;
}

float clamp(float value, float min, float max)
{
    assert(min <= max);

    value = std::max(value, min);

    value = std::min(value, max);

    return value;
}

float getValue(float value, float min, float max, float steady, float deadzone)
{
    assert(min <= max);
    assert(min <= steady);
    assert(max >= steady);

    clamp(value, min, max);

    if (std::fabs(value - steady) < deadzone)
    {
        return 0.0f;
    }

    float diff = max - min;

    value -= min;

    value /= (diff / 2.0f);

    value -= 1.0f;

    return clamp(value, -1.0f, 1.0f);
}

void rcCallback(const mavros_msgs::RCIn::ConstPtr & msg)
{
    controller_msgs::Controller out;

    out.header.stamp = ros::Time::now();

    out.x = getValue(msg->channels[x_channel], x_min, x_max, x_steady, x_deadzone);

    out.y = getValue(msg->channels[y_channel], y_min, y_max, y_steady, y_deadzone);

    out.z = getValue(msg->channels[z_channel], z_min, z_max, z_steady, z_deadzone);

    out.yaw = getValue(msg->channels[yaw_channel], yaw_min, yaw_max, yaw_steady, yaw_deadzone);

    out.arm = arm(out.z, out.yaw);

    out.disarm = disarm(out.z, out.yaw);

    pub.publish(out);
}

void init_param(const ros::NodeHandle & nh)
{
    nh.param("x_min", x_min, 992.0f);
    nh.param("x_max", x_max, 2015.0f);
    nh.param("x_steady", x_steady, 1503.0f);
    nh.param("x_deadzone", x_deadzone, 70.0f);

    nh.param("y_min", y_min, 992.0f);
    nh.param("y_max", y_max, 2015.0f);
    nh.param("y_steady", y_steady, 1503.0f);
    nh.param("y_deadzone", y_deadzone, 70.0f);

    nh.param("z_min", z_min, 992.0f);
    nh.param("z_max", z_max, 2014.0f);
    nh.param("z_steady", z_steady, 1503.0f);
    nh.param("z_deadzone", z_deadzone, 70.0f);

    nh.param("yaw_min", yaw_min, 992.0f);
    nh.param("yaw_max", yaw_max, 2015.0f);
    nh.param("yaw_steady", yaw_steady, 1503.0f);
    nh.param("yaw_deadzone", yaw_deadzone, 70.0f);

    nh.param("x_channel", x_channel, 2);
    nh.param("y_channel", y_channel, 1);
    nh.param("z_channel", z_channel, 0);
    nh.param("yaw_channel", yaw_channel, 3);

    nh.param("arm_time", arm_timeout, 1.0f);
    nh.param("disarm_time", disarm_timeout, 1.0f);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_rc");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::NodeHandle nh_controller("controller");

    arm_time = ros::Time::now();
    disarm_time = ros::Time::now();

    init_param(nh_priv);

    pub = nh_controller.advertise<controller_msgs::Controller>("joy", 1);

    ros::Subscriber rc_sub = nh.subscribe("mavros/rc/in", 1, rcCallback);

    ros::spin();

    return 0;
}
