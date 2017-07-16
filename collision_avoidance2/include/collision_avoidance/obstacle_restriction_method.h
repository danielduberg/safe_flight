#ifndef OBSTACLERESTRICTIONMETHOD_H
#define OBSTACLERESTRICTIONMETHOD_H

#include <ros/ros.h>

#include <collision_avoidance/point.h>

#include <exjobb_msgs/SensorReadings.h>
#include <exjobb_msgs/Control.h>

// Obstacle-Restriction Method
class ORM {
public:
    ORM(float radius, float security_distance, float epsilon);
	
    bool avoidCollision(exjobb_msgs::Control * control, const std::vector<Point> & obstacles);
	
private:
    float radius_, security_distance_, epsilon_, min_change_in_direction_, max_change_in_direction_, min_opposite_direction_, max_opposite_direction_;

    ros::Publisher pub_;
	
	Point initGoal(float direction, float magnitude);
	
    Point subgoalSelector(const float direction, const float magnitude, const std::vector<Point> & L);
	
    void findPotentialAB(const std::vector<Point> & L, const Point & goal, std::vector<Point> * A, std::vector<Point> * B);

    void getPointsOfInterest(const Point & goal, const std::vector<Point> & L, std::vector<Point> * left, std::vector<Point> * right);

    float getLeftBound(const std::vector<Point> & L, float goal_direction);

    float getRightBound(const std::vector<Point> & L, float goal_direction);

    float motionComputation(const Point & goal, const std::vector<Point> & L);

    bool isPointInsideRectangle(const Point & a, const Point & b, const Point & c, const Point & d, const Point & p);

    std::vector<Point> getPointsInRectangle(const std::vector<Point> & L, const Point & a, const Point & b, const Point & c, const Point & d);
	
    void getRectangle(const Point & goal, float radius, Point * a, Point * b, Point * c, Point * d);
	
    bool isClearPath(const Point & goal, const std::vector<Point> & L);
	
    //void getBounds(Point goal, std::vector<Point> & L, float & left_bound, float & right_bound);
	
	float getMidDirection(float d1, float d2);
};



#endif // OBSTACLERESTRICTIONMETHOD_H
