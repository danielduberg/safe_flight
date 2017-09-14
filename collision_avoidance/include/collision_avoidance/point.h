#pragma once

#include <ros/ros.h>

namespace collision_avoidance
{
    
    struct Point
    {
        double x_;
        double y_;

        /** Default constructor */
        Point();

        Point(const double x, const double y);

        static double getDistance(const Point & p1, const Point & p2 = Point(0, 0));

        static double getDirection(const Point & p);

        static double getDirectionDegrees(const Point & p);

        static Point getMidpoint(const Point & p1, const Point & p2);

        static Point getPointFromVector(const double & direction, const double & magnitude);

        static Point getPointFromVectorDegrees(const double & direction, const double & magnitude);

        static void getVectorFromPoint(const Point & p, double & direction, double & magnitude);

        static void getVectorFromPointDegrees(const Point & p, double & direction, double & magnitude);

    };
    
}
