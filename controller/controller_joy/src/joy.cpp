#include <ros/ros.h>

#include <sensor_msgs/Joy.h>

#include <controller_msgs/Controller.h>


enum Control
{
    x, x_back, x_forward,
    y, y_left, y_right,
    z, z_down, z_up,
    yaw, yaw_left, yaw_right,
    arm_disarm, arm, disarm,
    land_lift, land, lift
};


ros::Publisher pub;

// Axes
int x_axes, x_b_axes, x_f_axes;
int y_axes, y_l_axes, y_r_axes;
int z_axes, z_d_axes, z_u_axes;
int yaw_axes, yaw_l_axes, yaw_r_axes;
int arm_disarm_axes, arm_axes, disarm_axes;
int land_lift_axes, land_axes, lift_axes;
int x_inverted_axes, y_inverted_axes, z_inverted_axes, yaw_inverted_axes, arm_disarm_inverted_axes, land_lift_inverted_axes;

// Buttons
int x_b_button, x_f_button;
int y_l_button, y_r_button;
int z_d_button, z_u_button;
int yaw_l_button, yaw_r_button;
int arm_button, disarm_button;
int land_button, lift_button;

// Mapping
std::map<int, Control> axes_mapped;
std::map<int, Control> buttons_mapped;

float clamp(double value, double min, double max)
{
    assert(min <= max);

    value = std::max(value, min);

    value = std::min(value, max);

    return value;
}

void controllerCallback(const sensor_msgs::Joy::ConstPtr & msg)
{
    controller_msgs::Controller out;

    out.header.stamp = ros::Time::now();

    out.x = 0;
    out.y = 0;
    out.z = 0;
    out.yaw = 0;
    out.arm = false;
    out.disarm = false;
    out.land = false;
    out.lift = false;

    for (std::map<int, Control>::iterator iter = axes_mapped.begin(); iter != axes_mapped.end(); ++iter)
    {
        if (iter->first >= msg->axes.size())
        {
            ROS_FATAL_STREAM("Higher index than there are axes!");
            exit(2);
        }

        double value = msg->axes[iter->first];

        switch (iter->second)
        {
        case x:
            out.x += x_inverted_axes * value;

            break;
        case x_back:
            out.x -= x_inverted_axes * ((value + 1.0d) / 2.0d);

            break;
        case x_forward:
            out.x += x_inverted_axes * ((value + 1.0d) / 2.0d);

            break;
        case y:
            out.y += y_inverted_axes * value;

            break;
        case y_left:
            out.y -= y_inverted_axes * ((value + 1.0d) / 2.0d);

            break;
        case y_right:
            out.y += y_inverted_axes * ((value + 1.0d) / 2.0d);

            break;
        case z:
            out.z += z_inverted_axes * value;

            break;
        case z_down:
            out.z -= z_inverted_axes * ((value + 1.0d) / 2.0d);

            break;
        case z_up:
            out.z += z_inverted_axes * ((value + 1.0d) / 2.0d);

            break;
        case yaw:
            out.yaw += yaw_inverted_axes * value;

            break;
        case yaw_left:
            out.yaw -= yaw_inverted_axes * ((value + 1.0d) / 2.0d);

            break;
        case yaw_right:
            out.yaw += yaw_inverted_axes * ((value + 1.0d) / 2.0d);

            break;
        case arm_disarm:
            if (arm_disarm_inverted_axes * value == 1)
            {
                out.arm = true;
            }
            else if (arm_disarm_inverted_axes * value == -1)
            {
                out.disarm = true;
            }

            break;
        case arm:
            if (arm_disarm_inverted_axes * value == 1)
            {
                out.arm = true;
            }

            break;
        case disarm:
            if (arm_disarm_inverted_axes * value == 1)
            {
                out.disarm = true;
            }

            break;
        case land_lift:
            if (land_lift_inverted_axes * value == 1)
            {
                out.lift = true;
            }
            else if (land_lift_inverted_axes * value == -1)
            {
                out.land = true;
            }

            break;
        case land:
            if (land_lift_inverted_axes * value == 1)
            {
                out.land = true;
            }

            break;
        case lift:
            if (land_lift_inverted_axes * value == 1)
            {
                out.lift = true;
            }

            break;
        }
    }

    for (std::map<int, Control>::iterator iter = buttons_mapped.begin(); iter != buttons_mapped.end(); ++iter)
    {
        if (iter->first >= msg->buttons.size())
        {
            ROS_FATAL_STREAM("Higher index than there are buttons!");
            exit(3);
        }

        double value = msg->buttons[iter->first];

        switch (iter->second)
        {
        case x_back:
            out.x -= value;

            break;
        case x_forward:
            out.x += value;

            break;
        case y_left:
            out.y -= value;

            break;
        case y_right:
            out.y += value;

            break;
        case z_down:
            out.z -= value;

            break;
        case z_up:
            out.z += value;

            break;
        case yaw_left:
            out.yaw -= value;

            break;
        case yaw_right:
            out.yaw += value;

            break;
        case arm:
            if (value == 1)
            {
                out.arm = true;
            }

            break;
        case disarm:
            if (value == 1)
            {
                out.disarm = true;
            }

            break;
        case land:
            if (value == 1)
            {
                out.land = true;
            }

            break;
        case lift:
            if (value == 1)
            {
                out.lift = true;
            }

            break;
        }
    }

    out.x = clamp(out.x, -1.0d, 1.0d);
    out.y = clamp(out.y, -1.0d, 1.0d);
    out.z = clamp(out.z, -1.0d, 1.0d);
    out.yaw = clamp(out.yaw, -1.0d, 1.0d);

    pub.publish(out);
}

void warn(int both_axes, int first_axes, int second_axes, int first_button, int second_button, std::string first_message, std::string second_message, std::string type = std::string(""))
{
    if (both_axes < 0 && ((first_axes < 0 && first_button < 0) || (second_axes < 0 && second_button < 0)))
    {
        if ((first_axes < 0 && first_button < 0) && (second_axes < 0 && second_button < 0))
        {
            ROS_WARN_STREAM("No axes or buttons are mapped to " << type << " '" << first_message << "' and '" << second_message << "'!");
        }
        else if (first_axes < 0 && first_button < 0)
        {
            ROS_WARN_STREAM("No axes or button are mapped to " << type << " '" << first_message << "'!");
        }
        else
        {
            ROS_WARN_STREAM("No axes or button are mapped to " << type << " '" << second_message << "'!");
        }
    }
}

void map(std::map<int, Control> & map, int key, Control value, std::string type)
{
    if (key >= 0)
    {
        if (map.find(key) == map.end())
        {
            map[key] = value;
        }
        else
        {
            exit(1);
        }
    }
}

void map_axes_buttons()
{
    // Warn if something is not mapped
    warn(x_axes, x_b_axes, x_f_axes, x_b_button, x_f_button, "back", "forward", "move");
    warn(y_axes, y_l_axes, y_r_axes, y_l_button, y_r_button, "left", "right", "move");
    warn(z_axes, z_d_axes, z_u_axes, z_d_button, z_u_button, "down", "up", "move");
    warn(yaw_axes, yaw_l_axes, yaw_r_axes, yaw_l_button, yaw_r_button, "left", "right", "rotate");
    warn(arm_disarm_axes, arm_axes, disarm_axes, arm_button, disarm_button, "arm", "disarm");
    warn(land_lift_axes, land_axes, lift_axes, land_button, lift_button, "land", "lift");

    // Map axes
    map(axes_mapped, x_axes, x, "axes");
    map(axes_mapped, x_b_axes, x_back, "axes");
    map(axes_mapped, x_f_axes, x_forward, "axes");

    map(axes_mapped, y_axes, y, "axes");
    map(axes_mapped, y_l_axes, y_left, "axes");
    map(axes_mapped, y_r_axes, y_right, "axes");

    map(axes_mapped, z_axes, z, "axes");
    map(axes_mapped, z_d_axes, z_down, "axes");
    map(axes_mapped, z_u_axes, z_up, "axes");

    map(axes_mapped, yaw_axes, yaw, "axes");
    map(axes_mapped, yaw_l_axes, yaw_left, "axes");
    map(axes_mapped, yaw_r_axes, yaw_right, "axes");

    map(axes_mapped, arm_disarm_axes, arm_disarm, "axes");
    map(axes_mapped, arm_axes, arm, "axes");
    map(axes_mapped, disarm_axes, disarm, "axes");

    map(axes_mapped, land_lift_axes, land_lift, "axes");
    map(axes_mapped, land_axes, land, "axes");
    map(axes_mapped, lift_axes, lift, "axes");

    // Map buttons
    map(buttons_mapped, x_b_button, x_back, "buttons");
    map(buttons_mapped, x_f_button, x_forward, "buttons");

    map(buttons_mapped, y_l_button, y_left, "buttons");
    map(buttons_mapped, y_r_button, y_right, "buttons");

    map(buttons_mapped, z_d_button, z_down, "buttons");
    map(buttons_mapped, z_u_button, z_up, "buttons");

    map(buttons_mapped, yaw_l_button, yaw_left, "buttons");
    map(buttons_mapped, yaw_r_button, yaw_right, "buttons");

    map(buttons_mapped, arm_button, arm, "buttons");
    map(buttons_mapped, disarm_button, disarm, "buttons");

    map(buttons_mapped, land_button, land, "buttons");
    map(buttons_mapped, lift_button, lift, "buttons");
}

void init_param(const ros::NodeHandle & nh)
{
    // Axes
    bool temp;
    nh.param("x_axes", x_axes, -1);
    nh.param("x_b_axes", x_b_axes, -1);
    nh.param("x_f_axes", x_f_axes, -1);
    nh.param("x_inverted_axes", temp, false);
    x_inverted_axes = temp ? -1 : 1;

    nh.param("y_axes", y_axes, -1);
    nh.param("y_l_axes", y_l_axes, -1);
    nh.param("y_r_axes", y_r_axes, -1);
    nh.param("y_inverted_axes", temp, false);
    y_inverted_axes = temp ? -1 : 1;

    nh.param("z_axes", z_axes, -1);
    nh.param("z_d_axes", z_d_axes, -1);
    nh.param("z_u_axes", z_u_axes, -1);
    nh.param("z_inverted_axes", temp, false);
    z_inverted_axes = temp ? -1 : 1;

    nh.param("yaw_axes", yaw_axes, -1);
    nh.param("yaw_l_axes", yaw_l_axes, -1);
    nh.param("yaw_r_axes", yaw_r_axes, -1);
    nh.param("yaw_inverted_axes", temp, false);
    yaw_inverted_axes = temp ? -1 : 1;

    nh.param("arm_disarm_axes", arm_disarm_axes, -1);
    nh.param("arm_axes", arm_axes, -1);
    nh.param("disarm_axes", disarm_axes, -1);
    nh.param("arm_disarm_inverted_axes", temp, false);
    arm_disarm_inverted_axes = temp ? -1 : 1;

    nh.param("land_lift_axes", land_lift_axes, -1);
    nh.param("land_axes", land_axes, -1);
    nh.param("lift_axes", lift_axes, -1);
    nh.param("land_lift_inverted_axes", temp, false);
    land_lift_inverted_axes = temp ? -1 : 1;


    // Buttons
    nh.param("x_b_button", x_b_button, -1);
    nh.param("x_f_button", x_f_button, -1);

    nh.param("y_l_button", y_l_button, -1);
    nh.param("y_r_button", y_r_button, -1);

    nh.param("z_d_button", z_d_button, -1);
    nh.param("z_u_button", z_u_button, -1);

    nh.param("yaw_l_button", yaw_l_button, -1);
    nh.param("yaw_r_button", yaw_r_button, -1);

    nh.param("arm_button", arm_button, -1);
    nh.param("disarm_button", disarm_button, -1);

    nh.param("land_button", land_button, -1);
    nh.param("lift_button", lift_button, -1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_ds4");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::NodeHandle nh_controller("controller");

    init_param(nh_priv);

    map_axes_buttons();

    pub = nh_controller.advertise<controller_msgs::Controller>("joy", 1);

    ros::Subscriber sub = nh.subscribe("joy", 1, controllerCallback);

    ros::spin();

    return 0;
}
