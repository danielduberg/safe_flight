#ifndef BASIC_H
#define BASIC_H

#include <exjobb_msgs/Control.h>
#include <collision_avoidance/point.h>

class Basic
{
private:
    float radius_, security_distance_, min_distance_hold_;

    void stayInPlace(exjobb_msgs::Control * control, const std::vector<Point> & obstacles, float current_direction, float current_speed);
    
    float getClosestObstacleDistanceBetweenDirections(const std::vector<Point> & obstacles, float current_direction, float wanted_direction);

    void moving(exjobb_msgs::Control * control, const std::vector<Point> & obstacles, float wanted_direction, float current_direction, float current_speed);

public:
    Basic(float radius, float security_distance, float min_distance_hold);

    void avoidCollision(exjobb_msgs::Control * control, const std::vector<Point> & obstacles, float wanted_direction, float current_direction, float current_speed);
};

#endif // BASIC_H
