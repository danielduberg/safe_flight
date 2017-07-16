#include <ros/ros.h>

#include <collision_avoidance/point.h>

#define PI 3.14159265

float Point::getDistance(Point p) {
    return std::sqrt((p.x * p.x) + (p.y * p.y));
}

float Point::getDistance(Point p1, Point p2) {
    return std::sqrt(((p1.x - p2.x) * (p1.x - p2.x)) + ((p1.y - p2.y) * (p1.y - p2.y)));
}

// Source: http://www.purplemath.com/modules/midpoint.htm
Point Point::getMidpoint(Point p1, Point p2) {
    Point midpoint;

    midpoint.x = (p1.x + p2.x) / 2.0;
    midpoint.y = (p1.y + p2.y) / 2.0;

    return midpoint;
}

// Source: http://stackoverflow.com/questions/2526200/find-coordinate-by-angle
Point Point::getPointFromVector(float direction, float magnitude) {
    Point p;

    p.x = magnitude * std::cos(direction);
    p.y = magnitude * std::sin(direction);

    return p;
}

Point Point::getPointFromVectorDegrees(float direction, float magnitude) {
    Point p;

    p.x = magnitude * std::cos(direction * PI / 180.0);
    p.y = magnitude * std::sin(direction * PI / 180.0);

    return p;
}

// TODO: Check so it is correct
void Point::getVectorFromPoint(Point p, float & direction, float & magnitude) {
    magnitude = getDistance(p);

    direction = getDirection(p);
}

void Point::getVectorFromPointDegrees(Point p, float & direction, float & magnitude) {
    getVectorFromPoint(p, direction, magnitude);

    direction *= (180.0 / PI);
}

/**
 * @brief Point::getDirection
 * @param p
 * @return Radians
 */
float Point::getDirection(Point p) {
    return std::atan2(p.y, p.x);
}

/**
 * @brief Point::GetDirectionDegrees
 * @param p
 * @return Degrees
 */
float Point::GetDirectionDegrees(Point p)
{
    float direction = getDirection(p) * 180.0 / PI;

    if (direction < 0)
    {
        return (direction + 360);
    }

    return direction;
}
