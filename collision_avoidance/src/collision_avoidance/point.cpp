#include <collision_avoidance/point.h>

namespace collision_avoidance
{
    Point::Point()
        : x_(0)
        , y_(0)
    {

    }

    Point::Point(const double x, const double y)
        : x_(x)
        , y_(y)
    {

    }

    double Point::getDistance(const Point & p1, const Point & p2)
    {
        double x = p1.x_ - p2.x_;
        double y = p1.y_ - p2.y_;
        return std::sqrt((x*x) + (y*y)); //std::pow(x, 2) + std::pow(y, 2));
    }

    /**
     * @brief Point::getDirection
     * @param p
     * @return Radians
     */
    double Point::getDirection(const Point & p)
    {
        return std::atan2(p.y_, p.x_);
    }

    /**
     * @brief Point::getDirectionDegrees
     * @param p
     * @return Degrees
     */
    double Point::getDirectionDegrees(const Point & p)
    {
        double direction = getDirection(p) * 180.0 / M_PI;

        if (direction < 0)
        {
            return (direction + 360.0);
        }

        return direction;
    }

    // Source: http://www.purplemath.com/modules/midpoint.htm
    Point Point::getMidpoint(const Point & p1, const Point & p2)
    {
        Point midpoint;

        midpoint.x_ = (p1.x_ + p2.x_) / 2.0;
        midpoint.y_ = (p1.y_ + p2.y_) / 2.0;

        return midpoint;
    }

    // Source: http://stackoverflow.com/questions/2526200/find-coordinate-by-angle
    Point Point::getPointFromVector(const double & direction, const double & magnitude)
    {
        Point p;

        p.x_ = magnitude * std::cos(direction);
        p.y_ = magnitude * std::sin(direction);

        return p;
    }

    Point Point::getPointFromVectorDegrees(const double & direction, const double & magnitude)
    {
        return getPointFromVector(direction * M_PI / 180.0, magnitude);
    }

    // TODO: Check so it is correct
    void Point::getVectorFromPoint(const Point & p, double & direction, double & magnitude)
    {
        magnitude = getDistance(p);

        direction = getDirection(p);
    }

    void Point::getVectorFromPointDegrees(const Point & p, double & direction, double & magnitude)
    {
        getVectorFromPoint(p, direction, magnitude);

        direction *= (180.0 / M_PI);
    }


}
