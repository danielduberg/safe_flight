#pragma once

#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <collision_avoidance/point.h>
#include <collision_avoidance/obstacle_restriction_method.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <safe_flight_msgs/SensorReadings.h>
#include <controller_msgs/Controller.h>

#include <joy_rumble/Rumble_msg.h>

namespace collision_avoidance
{
    
    class CANodelet : public nodelet::Nodelet
    {

    private:
        // Obstacles
        std::vector<Point> obstacles_;

        // Current pose
        geometry_msgs::PoseStamped current_pose_;

        // Current velocity
        double current_x_vel_, current_y_vel_;

        // Obstacle-Restriction Method
        ORM * orm_;
        double radius_, security_distance_, epsilon_;
        // Tele-op
        double min_change_in_direction_, max_change_in_direction_, min_opposite_direction_, max_opposite_direction_;

        // Ego-Dynamic Space
        double ab_, T_;

        // Subscribers
        ros::Subscriber sensor_readings_sub_, collision_avoidance_sub_, current_pose_sub_, current_velocity_sub_;

        // Publishers
        ros::Publisher collision_free_control_pub_, rumble_pub_;

    private:
        virtual void onInit();

        void init_param(ros::NodeHandle & nh);

        void sensorReadingsCallback(const safe_flight_msgs::SensorReadings::ConstPtr & msg);

        bool hapticFeedback(double want_direction, double going_direction);

        void collisionAvoidanceCallback(const controller_msgs::Controller::ConstPtr & msg);

        std::vector<Point> getEgeDynamicSpace();

        void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);

        void currentVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr & msg);
    };
    
}
