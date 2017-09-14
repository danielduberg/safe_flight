#pragma once

#include <ros/ros.h>

#include <controller_msgs/Controller.h>

#include <collision_avoidance/point.h>

#include <collision_avoidance/no_input.h>

namespace collision_avoidance
{
    // Obstacle-Restriction Method
    class ORM
    {
    private:
        double radius_;
        double security_distance_;
        double epsilon_;
        double min_change_in_direction_;
        double max_change_in_direction_;
        double min_opposite_direction_;
        double max_opposite_direction_;

        ros::NodeHandle nh_;
        ros::Publisher pub_;

        double min_distance_hold_;

        NoInput no_input_;

    public:
        // Done
        ORM(double radius, double security_distance, double epsilon, double min_distance_hold, double min_change_in_direction, double max_change_in_direction, double min_opposite_direction, double max_opposite_direction);

        // Done
        bool avoidCollision(controller_msgs::Controller * controller, const double magnitude, const std::vector<Point> & obstacles);

    private:
        // Done
        Point initGoal(double x, double y);

        // Done
        Point subgoalSelector(const Point & goal, double magnitude, const std::vector<Point> & L);

        // Done
        // TODO: Take an extra look at this one (45 degrees)
        void findPotentialAB(const std::vector<Point> & L, const Point & goal, std::vector<Point> * A, std::vector<Point> * B);

        // Done
        // TODO: Take an extra look at this one
        void getPointsOfInterest(const Point & goal, const std::vector<Point> & L, std::vector<Point> * left, std::vector<Point> * right);

        // Done
        double getLeftBound(const std::vector<Point> & L, double goal_direction);

        // Done
        double getRightBound(const std::vector<Point> & L, double goal_direction);

        // Done
        Point motionComputation(const Point & goal, const std::vector<Point> & L);

        // Done
        bool isPointInPolygon(const Point & point, const std::vector<Point> & verticies);

        // Done
        bool isPointInsideRectangle(const Point & a, const Point & b, const Point & c, const Point & d, const Point & p);

        // Done
        std::vector<Point> getPointsInPolygon(const std::vector<Point> & L, const std::vector<Point> & verticies);

        // Done
        std::vector<Point> getPointsInRectangle(const std::vector<Point> & L, const Point & a, const Point & b, const Point & c, const Point & d);

        // Done
        void getRectangle(const Point & goal, double radius, Point * a, Point * b, Point * c, Point * d);
        void getRectangle(const Point & goal, double radius, std::vector<Point> * verticies);

        // Done
        bool isClearPath(const Point & goal, const std::vector<Point> & L);

        // Done
        double getMidDirection(double d1, double d2);
    };
    
}
