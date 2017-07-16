#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <string>

#include <algorithm>

#include <collision_avoidance/point.h>

#include <exjobb_msgs/Control.h>
#include <exjobb_msgs/SensorReadings.h>
#include <collision_avoidance/point.h>

#include <collision_avoidance/obstacle_restriction_method.h>
#include <collision_avoidance/basic.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <joy_rumble/Rumble_msg.h>

# define M_PI           3.14159265358979323846  /* pi */

namespace collision_avoidance
{

class CANodelet : public nodelet::Nodelet
{
public:
    CANodelet() {}

    ~CANodelet() {}

private:
    // Obstacles
    std::vector<Point> obstacles_;

    // Current pose
    geometry_msgs::PoseStamped current_pose_;

    // Current velocity
    float current_direction_, current_speed_;

    // Obstacle-restriction Method
    ORM * orm_;

    // Basic collision avoidance
    Basic * basic_;

    float radius_, security_distance_;

    // Subscriptions
    ros::Subscriber sensor_readings_sub_, collision_avoidance_sub_, current_pose_sub_, current_velocity_sub_;

    // Publications
    ros::Publisher collision_free_control_pub_;
    ros::Publisher rumble_pub_;

    virtual void onInit();

    void sensorReadingsCallback(const exjobb_msgs::SensorReadings::ConstPtr & msg);

    bool hapticFeedback(float want_direction, float going_direction);

    void collisionAvoidanceCallback(const exjobb_msgs::Control::ConstPtr & msg);

    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);

    void currentVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr & msg);
};

void CANodelet::onInit()
{
    ros::NodeHandle nh(getNodeHandle());
    ros::NodeHandle priv_nh(getPrivateNodeHandle());

    sensor_readings_sub_ = nh.subscribe("/sensor_readings", 1, &CANodelet::sensorReadingsCallback, this);
    collision_avoidance_sub_ = nh.subscribe("/control", 1, &CANodelet::collisionAvoidanceCallback, this);
    current_pose_sub_ = nh.subscribe("mavros/local_position/pose", 1, &CANodelet::currentPoseCallback, this);
    current_velocity_sub_ = nh.subscribe("/mavros/local_position/velocity", 1, &CANodelet::currentVelocityCallback, this);

    collision_free_control_pub_ = nh.advertise<exjobb_msgs::Control>("collision_free_control", 1);
    rumble_pub_ = nh.advertise<joy_rumble::Rumble_msg>("rumble_message", 1);

    float epsilon, max_distance_speed, min_distance_hold;
    priv_nh.param<float>("radius", radius_, 0.25);
    priv_nh.param<float>("security_distance", security_distance_, 0.1);
    priv_nh.param<float>("epsilon", epsilon, 0.1);
    priv_nh.param<float>("max_distance_speed", max_distance_speed, 1.5);
    priv_nh.param<float>("min_distance_hold", min_distance_hold, 0.3);

    orm_ = new ORM(radius_, security_distance_, epsilon);
    basic_ = new Basic(radius_, max_distance_speed, min_distance_hold);
}

void CANodelet::sensorReadingsCallback(const exjobb_msgs::SensorReadings::ConstPtr & msg)
{
    std::vector<Point> newObstacles;    // TODO: Init with correct size?

    for (size_t i = 0; i < msg->x.size(); i++)
    {
        Point p;
        p.x = msg->x[i];
        p.y = msg->y[i];
        newObstacles.push_back(p);
    }

    obstacles_ = newObstacles;
}

bool CANodelet::hapticFeedback(float want_direction, float going_direction)
{
    float diff = want_direction - going_direction;

    if (diff > 180)
    {
        diff -= 360;
    }
    else if (diff < -180)
    {
        diff += 360;
    }

    if (std::fabs(diff) > 70)
    {
        joy_rumble::Rumble_msg msg;
        msg.length = 500;   // Half a second
        msg.strong = 65535;
        msg.weak = 65535;

        rumble_pub_.publish(msg);

        return true;
    }

    return false;
}

void CANodelet::collisionAvoidanceCallback(const exjobb_msgs::Control::ConstPtr & msg)
{
    //ROS_ERROR_STREAM("Current speed: " << current_speed_);

    exjobb_msgs::Control collisionFreeControl = *msg;

    Point current;

    current.x = current_speed_ * std::cos(current_direction_ * M_PI / 180.0);
    current.y = current_speed_ * std::sin(current_direction_ * M_PI / 180.0);

    float ab = 3.0;
    float T = 0.3;

    float dbreak = (current_speed_ * current_speed_) / (2.0 * ab);

    std::vector<Point> obstacles;

    for (size_t i = 0; i < obstacles_.size(); i++)
    {
        if (obstacles_[i].x == 0 && obstacles_[i].y == 0)
        {
            obstacles.push_back(obstacles_[i]);
            continue;
        }

        // Decrease the distance with 5 cm?!
        float dobs = Point::getDistance(obstacles_[i]);

        dobs -= radius_;

        float deff = ab * (T * T) * (std::sqrt(1.0 + ((2 * dobs) / (ab * (T * T)))) - 1.0);

        deff += radius_;

        deff = std::min(Point::getDistance(obstacles_[i]), deff);
        deff = std::max(deff, radius_ + 0.01f);

        /*
        float u = deff * std::cos(Point::getDirection(obstacles_[i]));
        float v = deff * std::sin(Point::getDirection(obstacles_[i]));
        float u2 = u * u;
        float v2 = v * v;
        float twosqrt2 = 2.0 * std::sqrt(2.0);
        float subtermx = 2.0 + u2 - v2;
        float subtermy = 2.0 - u2 + v2;
        float termx1 = subtermx + u * twosqrt2;
        float termx2 = subtermx - u * twosqrt2;
        float termy1 = subtermy + v * twosqrt2;
        float termy2 = subtermy - v * twosqrt2;

        Point point;
        point.x = 0.5 * std::sqrt(termx1) - 0.5 * std::sqrt(termx2);
        point.y = 0.5 * std::sqrt(termy1) - 0.5 * std::sqrt(termy2);
        */

        Point point;
        point.x = deff * std::cos(Point::getDirection(obstacles_[i]));
        point.y = deff * std::sin(Point::getDirection(obstacles_[i]));

        //float distance = dobs - (2.0 * dbreak);
        // Distance can min be the radius else it is inside the drone!
        //distance = std::max(radius_, distance);

        //point.x = distance * std::cos(Point::getDirection(obstacles_[i]));
        //point.y = distance * std::sin(Point::getDirection(obstacles_[i]));


        // 10 hz
        //point.x = obstacles_[i].x - (temp.x);
        //point.y = obstacles_[i].y - (temp.y);
        obstacles.push_back(point);
    }

    // Call ORM
    orm_->avoidCollision(&collisionFreeControl, obstacles);

    //ROS_ERROR_STREAM("Want: " << msg->go_direction << ", get: " << collisionFreeControl.go_direction);

    // TODO: Add code that checks if we should do haptic feedback
    if (hapticFeedback(msg->go_direction, collisionFreeControl.go_direction))
    {
        // Treat it as no input to move to "safety"

    }

    // Call validation
    basic_->avoidCollision(&collisionFreeControl, obstacles, msg->go_direction, current_direction_, current_speed_);

    // Publish

    //ROS_ERROR_STREAM(msg->go_direction << ", " << msg->go_magnitude << "; " << collisionFreeControl.go_direction << ", " << collisionFreeControl.go_magnitude);

    collision_free_control_pub_.publish(collisionFreeControl);
}

void CANodelet::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    current_pose_ = *msg;
}

void CANodelet::currentVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
    double roll, pitch, yaw;
    tf2::Quaternion q(current_pose_.pose.orientation.x, current_pose_.pose.orientation.y, current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getEulerYPR(yaw, pitch, roll);

    Point point;
    point.x = msg->twist.linear.x;
    point.y = msg->twist.linear.y;

    float direction, magnitude;
    Point::getVectorFromPointDegrees(point, direction, magnitude);

    current_speed_ = magnitude;

    current_direction_ = direction - (yaw * 180.0 / M_PI);

    if (current_direction_ < 0)
    {
        current_direction_ += 360;
    }
    else if (current_direction_ > 360)
    {
        current_direction_ -= 360;
    }

    //ROS_ERROR_STREAM("Direction: " << current_direction_ << ", speed: " << current_speed_);
}

// End namespace
}

// Register this plugin with pluginlib.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(collision_avoidance, CA, collision_avoidance::CANodelet, nodelet::Nodelet);
