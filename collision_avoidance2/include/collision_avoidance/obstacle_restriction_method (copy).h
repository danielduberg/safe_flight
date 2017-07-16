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
	
    bool avoidCollision(exjobb_msgs::Control * control, std::vector<Point> & obstacles);
	
private:
    float radius_, security_distance_, epsilon_;
	
	Point initGoal(float direction, float magnitude);
	
	Point subgoalSelector(Point prefered_goal, float prefered_direction, std::vector<Point> & L, float max_degree);
	
	float motionComputation(Point goal, std::vector<Point> & L);
	
    bool isCircleLineIntersect(Point & a, Point & b, float radius, Point & c);

    bool isPointInsideRectangle(const Point & a, const Point & b, const Point & c, const Point & d, const Point & p);

    bool isRectangleInsideCircle(const Point & a, const Point & b, const Point & c, const Point & d, float radius, const Point & p);

    std::vector<Point> getPointsInRectangle(std::vector<Point> L, Point & a, Point & b, Point & c, Point & d);
	
    void getRectangle(Point goal, float radius, Point * a, Point * b, Point * c, Point * d);
	
    int getSide(Point p1, Point p2);
	
    bool isClearPath(Point goal, std::vector<Point> & L);
	
	void getS1(std::vector<Point> & L, Point goal, std::vector<std::tuple<float, float> > & S1);
	
	void getS2(std::vector<Point> & L, std::vector<std::tuple<float, float> > & S2);
	
	void getBounds(Point goal, std::vector<Point> & L, float & left_bound, float & right_bound);
	
	float getInbetweenPI(float direction);
	
	float getMidDirection(float d1, float d2);
};



#endif // OBSTACLERESTRICTIONMETHOD_H
