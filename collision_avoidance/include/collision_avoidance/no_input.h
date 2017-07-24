#pragma once

#include <ros/ros.h>

#include <controller_msgs/Controller.h>

#include <collision_avoidance/point.h>

namespace collision_avoidance
{
    
    class NoInput
    {
    private:
        double radius_;
        double security_distance_;
        double min_distance_hold_;

    public:
        NoInput(double radius, double security_distance, double min_distance_hold);

        void avoidCollision(controller_msgs::Controller * control, const std::vector<Point> & obstacles);

    private:

    };
    
}
