#include <ros/ros.h>

#include "collision_avoidance/obstacle_restriction_method.h"

#include "exjobb_msgs/SensorReadings.h"

#include <tuple>

#define PI 3.14159265

ORM::ORM(float radius, float security_distance, float epsilon)
    : radius_(radius)
    , security_distance_(security_distance)
    , epsilon_(epsilon)
{

}

bool ORM::avoidCollision(exjobb_msgs::Control * control, std::vector<Point> & obstacles)
{
    if (control->goMagnitude == 0) {
        return true;
    }

    float direction = control->goDirection * PI / 180.0;

    Point goal = initGoal(control->goDirection, 1.5);

    // A. The Subgoal Selector
    // TODO: FIX
    goal = subgoalSelector(goal, control->goDirection, obstacles, 180);

    // Check if goal = 0

    //ROS_ERROR_STREAM("Between 3: " << Point::getDirection(goal) * 180.0 / PI);

    // B. Motion Computation
    control->goDirection = fmod((motionComputation(goal, obstacles) * 180.0 / PI) + 360, 360);

    //control->goDirection = Point::GetDirectionDegrees(goal);

    //ROS_ERROR_STREAM("Between 4: " << control->goDirection);

    return true;
}

float ORM::getInbetweenPI(float direction) {
    if (direction < -PI) {
        return (direction + (2 * PI));
    } else if (direction > PI) {
        return (direction - (2 * PI));
    }

    return direction;
}

void ORM::getBounds(Point target, std::vector<Point> & L, float & left_bound, float & right_bound) {

    std::vector<Point> Lp;

    float direction = Point::GetDirectionDegrees(target);

    float degreePerIndex = 360.0 / L.size();

    int wantedIndex = direction / degreePerIndex;

    // We cannot go where we "want" so find a subgoal
    // Look for subgoals close to wanted_direction
    float max_degree = 90;
    for (size_t i = 0; i * degreePerIndex < max_degree; i++) {
        for (int j = -1; j < 2; j += 2) {
            int currentIndex = (wantedIndex + (i * j)) % L.size();

            if (L[currentIndex].x == 0 && L[currentIndex].y == 0) {
                // No reading here
                continue;
            }

            Lp.push_back(L[currentIndex]);
        }
    }


    float targetDirection = Point::getDirection(target);

    bool firstUpdateLeftBound = true;
    bool firstUpdateRightBound = true;

    for (size_t i = 0; i < Lp.size(); i++)
    {
        if (Lp[i].x == 0 && Lp[i].y == 0)
        {
            // Nothing here!
            continue;
        }

        float obstacleDirection = Point::getDirection(Lp[i]);
        float obstacleDistance = Point::getDistance(Lp[i]);

        // Skip the S1 crap

        // TODO: Check correct
        float alpha = std::abs(std::atan2((radius_ + security_distance_), obstacleDistance));

        float beta = 0;

        if (obstacleDistance <= security_distance_ + radius_)
        {
            beta = (PI - alpha) * (1 - ((obstacleDistance - radius_) / security_distance_));
        }

        // Removed MIN/MAX, is this allowed?!
        float gammaL = obstacleDirection - (alpha + beta);
        gammaL = getInbetweenPI(gammaL);
        float gammaR = obstacleDirection + (alpha + beta);
        gammaR = getInbetweenPI(gammaR);

        // Which side is the obstacle on?!
        float directionDiff = obstacleDirection - targetDirection;
        directionDiff = getInbetweenPI(directionDiff);

        if (directionDiff > 0)
        {
            // The obstacle is to the left of the target (seen from origo)
            // Right bound
            if (firstUpdateRightBound)
            {
                firstUpdateRightBound = false;

                right_bound = gammaL;
            }
            else
            {
                // Check which side of the target gammaL is on
                float gammaDiff = gammaL - targetDirection;
                gammaDiff = getInbetweenPI(gammaDiff);

                float rightBoundDiff = right_bound - targetDirection;
                rightBoundDiff = getInbetweenPI(rightBoundDiff);

                if (gammaDiff < rightBoundDiff)
                {
                    right_bound = gammaL;
                }
            }
        }
        else
        {
            // The obstacle is to the right of the target (seen from origo)
            // Left bound
            if (firstUpdateLeftBound)
            {
                firstUpdateLeftBound = false;

                left_bound = gammaR;
            }
            else
            {
                // Check which side of the target gammaL is on
                float gammaDiff = gammaR - targetDirection;
                gammaDiff = getInbetweenPI(gammaDiff);

                float leftBoundDiff = left_bound - targetDirection;
                leftBoundDiff = getInbetweenPI(leftBoundDiff);

                if (gammaDiff > leftBoundDiff)
                {
                    left_bound = gammaR;
                }
            }
        }
    }
}

float ORM::getMidDirection(float d1, float d2) {
    if ((d1 >= 0 && d2 >= 0) || (d1 < 0 && d2 < 0)) {
        return ((d1 + d2) / 2.0);
    }

    float zeroDistance = std::fabs(d1) + std::fabs(d2);
    float PIDistance = (2 * PI) - zeroDistance;

    if (zeroDistance < PIDistance) {
        if (d1 >= 0) {
            return (d1 - (zeroDistance / 2.0));
        } else {
            return (d2 - (zeroDistance / 2.0));
        }
    } else {
        if (d1 >= 0) {
            return getInbetweenPI(d1 + (zeroDistance / 2.0));
        } else {
            return getInbetweenPI(d2 + (zeroDistance / 2.0));
        }
    }
}

float ORM::motionComputation(Point goal, std::vector<Point> & L) {
    float leftBound = -3*PI, rightBound = 3*PI;

    getBounds(goal, L, leftBound, rightBound);

    float targetDirection = Point::getDirection(goal);

    float leftDiff = leftBound - targetDirection;
    float rightDiff = rightBound - targetDirection;

    ROS_ERROR_STREAM(leftBound << ", " << targetDirection << ", " << rightBound);
    if (leftDiff < 0 && rightDiff > 0) {
        ROS_ERROR_STREAM("Target direction!");
        return targetDirection;
    } else if (leftDiff < 0 && leftBound > -3*PI) {
        ROS_ERROR_STREAM("Left direction!");
        return leftBound;
    } else if (rightDiff > 0 && rightBound < 3*PI) {
        ROS_ERROR_STREAM("Right direction!");
        return rightBound;
    }
    else if (leftBound > -3*PI && rightBound >= 3*PI)
    {
        return leftBound;
    }
    else if (rightBound < 3*PI && leftBound <= -3*PI)
    {
        return rightBound;
    }
    else
    {
        ROS_ERROR_STREAM("Mid direction!");
        return getMidDirection(leftBound, rightBound);
    }
}

Point ORM::initGoal(float direction, float magnitude)
{
    // TODO: FIX
    return Point::getPointFromVectorDegrees(direction, magnitude * 3);
}

Point ORM::subgoalSelector(Point prefered_goal, float prefered_direction, std::vector<Point> & L, float max_degree)
{
    if (isClearPath(prefered_goal, L))
    {
        //ROS_ERROR_STREAM("WOOOOOOOOOOOOOOOOOOOOOOOOHOOOOOOOOOOOOOO");
        // We can go where we want
        return prefered_goal;
    }

    float degreePerIndex = 360.0 / L.size();

    int wantedIndex = prefered_direction / degreePerIndex;

    // We cannot go where we "want" so find a subgoal
    // Look for subgoals close to wanted_direction
    for (size_t i = 1; i * degreePerIndex < max_degree; i++) {
        for (int j = -1; j < 2; j += 2) {
            int currentIndex = (wantedIndex + (i * j)) % L.size();

            if (L[currentIndex].x == 0 && L[currentIndex].y == 0) {
                // No reading here
                continue;
            }

            for (int k = -1; k < 2; k += 2) {
                int nextToIndex = (currentIndex + k) % L.size();

                if (L[nextToIndex].x == 0 && L[nextToIndex].y == 0) {
                    // Subgoal at the edge of an obstacle

                    float angle = (degreePerIndex * (currentIndex + nextToIndex)) / 2.0;
                    float distance = Point::getDistance(L[currentIndex]) + (radius_ * 2) + epsilon_;

                    Point subgoal = Point::getPointFromVectorDegrees(angle, distance);

                    // Check if this subgoal is clear
                    if (isClearPath(subgoal, L)) {
                        // Return this subgoal
                        return subgoal;
                    }

                } else if (Point::getDistance(L[currentIndex], L[nextToIndex]) > 2 * radius_) {
                    // Subgoal between two obstacles
                    Point subgoal = Point::getMidpoint(L[currentIndex], L[nextToIndex]);

                    // Check if this subgoal is clear
                    if (isClearPath(subgoal, L)) {
                        // Return this subgoal
                        return subgoal;
                    }
                }
            }
        }
    }

    // TODO
    // Could not find a subgoal, go as close as possible to the goal
}

// Source: http://mathworld.wolfram.com/Circle-LineIntersection.html
// Source: http://csharphelper.com/blog/2014/09/determine-where-a-line-intersects-a-circle-in-c/
bool ORM::isCircleLineIntersect(Point & a, Point & b, float radius, Point & c)
{
    // Translate so circle center is at (0, 0)
    Point ac, bc;
    ac.x = a.x - c.x;
    ac.y = a.y - c.y;
    bc.x = b.x - c.x;
    bc.y = b.y - c.y;

    float dx = bc.x - ac.x;
    float dy = bc.y - ac.y;

    float A = (dx * dx) + (dy * dy);
    float B = 2 * ((dx * ac.x) + (dy * ac.y));
    float C = (ac.x * ac.x) + (ac.y * ac.y) - (radius * radius);

    float det = (B * B) - (4 * A * C);

    if (A <= 0.0000001 || det < 0)
    {
        return false;
    }

    return true;
}

/**
 * @brief ORM::isPointInsideRectangle Check  if p is inside rectangle a, b, c, d
 * @param a
 * @param b
 * @param c
 * @param d
 * @param p
 * @return
 */
bool ORM::isPointInsideRectangle(const Point & a, const Point & b, const Point & c, const Point & d, const Point & p)
{
    // Is point inside rectangle?
    Point al, ab, ad;
    al.x = p.x - a.x;
    al.y = p.y - a.y;

    ab.x = b.x - a.x;
    ab.y = b.y - a.y;

    ad.x = d.x - a.x;
    ad.y = d.y - a.y;

    return (0 <= (al.x * ab.x) + (al.y * ab.y) &&
            (al.x * ab.x) + (al.y * ab.y) <= (ab.x * ab.x) + (ab.y * ab.y) &&
            0 <= (al.x * ad.x) + (al.y * ad.y) &&
            (al.x * ad.x) + (al.y * ad.y) <= (ad.x * ad.x) + (ad.y * ad.y));
}

bool ORM::isRectangleInsideCircle(const Point & a, const Point & b, const Point & c, const Point & d, float radius, const Point & p)
{
    return (Point::getDistance(a, p) <= radius || Point::getDistance(b, p) <= radius || Point::getDistance(c, p) <= radius || Point::getDistance(d, p) <= radius);
}

// http://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection
std::vector<Point> ORM::getPointsInRectangle(std::vector<Point> L, Point & a, Point & b, Point & c, Point & d) {
    std::vector<Point> Lp;

    for (size_t i = 0; i < L.size(); i++) {
        if (L[i].x == 0 && L[i].y == 0) {
            // No reading here
            continue;
        }

        // Is point inside rectangle?
        if (isPointInsideRectangle(a, b, c, d, L[i]))
        {
            // Inside rectangle
            Lp.push_back(L[i]);
            //ROS_ERROR_STREAM("1: (" << L[i].x << ", " << L[i].y << "); (" << a.x << ", " << a.y << "), (" << b.x << ", " << b.y << "), (" << c.x << ", " << c.y << "), (" << d.x << ", " << d.y << ")");
        }
        /*
        else if (isCircleLineIntersect(a, b, radius_, L[i]))
        {
            // Intersect
            Lp.push_back(L[i]);
            ROS_ERROR_STREAM("2: (" << L[i].x << ", " << L[i].y << "); (" << a.x << ", " << a.y << "), (" << b.x << ", " << b.y << "), (" << c.x << ", " << c.y << "), (" << d.x << ", " << d.y << ")");
        }
        else if (isCircleLineIntersect(b, c, radius_, L[i]))
        {
            // Intersect
            Lp.push_back(L[i]);
            ROS_ERROR_STREAM("3: (" << L[i].x << ", " << L[i].y << "); (" << a.x << ", " << a.y << "), (" << b.x << ", " << b.y << "), (" << c.x << ", " << c.y << "), (" << d.x << ", " << d.y << ")");
        }
        else if (isCircleLineIntersect(c, d, radius_, L[i]))
        {
            // Intersect
            Lp.push_back(L[i]);
            ROS_ERROR_STREAM("4: (" << L[i].x << ", " << L[i].y << "); (" << a.x << ", " << a.y << "), (" << b.x << ", " << b.y << "), (" << c.x << ", " << c.y << "), (" << d.x << ", " << d.y << ")");
        }
        else if (isCircleLineIntersect(d, a, radius_, L[i]))
        {
            // Intersect
            Lp.push_back(L[i]);
            ROS_ERROR_STREAM("5: (" << L[i].x << ", " << L[i].y << "); (" << a.x << ", " << a.y << "), (" << b.x << ", " << b.y << "), (" << c.x << ", " << c.y << "), (" << d.x << ", " << d.y << ")");
        }
        else if (isRectangleInsideCircle(a, b, c, d, radius_, L[i]))
        {
            // Rectangle inside circle
            Lp.push_back(L[i]);
            ROS_ERROR_STREAM("6: (" << L[i].x << ", " << L[i].y << "); (" << a.x << ", " << a.y << "), (" << b.x << ", " << b.y << "), (" << c.x << ", " << c.y << "), (" << d.x << ", " << d.y << ")");
        }
        */
    }

    return Lp;
}

// Source: http://stackoverflow.com/questions/2825412/draw-a-parallel-line
void ORM::getRectangle(Point goal, float radius, Point * a, Point * b, Point * c, Point * d)
{
    float distance = Point::getDistance(goal);

    float dx = goal.y / distance;
    float dy = -goal.x / distance;

    a->x = radius * dx;
    a->y = radius * dy;

    b->x = goal.x + a->x;
    b->y = goal.y + a->y;

    c->x = goal.x - a->x;
    c->y = goal.y - a->y;

    d->x = -a->x;
    d->y = -a->y;

    //ROS_ERROR_STREAM("Rectangle: (" << a->x << ", " << a->y << "), (" << b->x << ", " << b->y << "), (" << c->x << ", " << c->y << "), (" << d->x << ", " << d->y << ")");
}

// Source: http://stackoverflow.com/questions/22668659/calculate-on-which-side-of-a-line-a-point-is
int ORM::getSide(Point p1, Point p2) {
    float value = (p1.x * p2.y) - (p2.x * p1.y);

    if (value > 0) {
        return 1;
    } else if (value == 0) {
        return 0;
    } else {
        return -1;
    }
}

bool ORM::isClearPath(Point goal, std::vector<Point> & L)
{
    Point a, b, c, d;

    // 2 * radius because then all obstacles that matters will be inside the rectangle
    getRectangle(goal, 2 * radius_, &a, &b, &c, &d);

    // Fill Lp with potential
    std::vector<Point> Lp;

    float direction = Point::GetDirectionDegrees(goal);

    float degreePerIndex = 360.0 / L.size();

    int wantedIndex = direction / degreePerIndex;

    // We cannot go where we "want" so find a subgoal
    // Look for subgoals close to wanted_direction
    float max_degree = 80;
    for (size_t i = 1; i * degreePerIndex < max_degree; i++) {
        for (int j = -1; j < 2; j += 2) {
            int currentIndex = (wantedIndex + (i * j)) % L.size();

            if (L[currentIndex].x == 0 && L[currentIndex].y == 0) {
                // No reading here
                continue;
            }

            Lp.push_back(L[currentIndex]);
        }
    }


    Lp = getPointsInRectangle(Lp, a, b, c, d);

    for (size_t i = 0; i < Lp.size(); i++) {
        int side = getSide(goal, Lp[i]);

        if (side == 0) {
            // An obstacle in the middle
            return false;
        }

        for (size_t j = 0; j < Lp.size(); j++) {
            if (i == j) {
                continue;
            }

            int side2 = getSide(goal, Lp[j]);

            if (side == side2) {
                continue;
            }

            if (Point::getDistance(Lp[i], Lp[j]) < 2 * radius_) {
                //ROS_ERROR_STREAM("(" << goal.x << ", " << goal.y << "), (" << Lp[i].x << ", " << Lp[i].y << "), (" << Lp[j].x << ", " << Lp[j].y << ")");
                return false;
            }
        }
    }

    return true;
}



/*
void ORM::getS1(std::vector<Point> & L, Point goal, std::vector<std::tuple<float, float> > & S1) {
    float targetDirection = Point::getDirectionFromOrigoToPoint(goal);

    for (size_t i = 0; i < L.size(); i++) {
        if (L[i].x == 0 && L[i].y == 0) {
            S1.push_back(std::make_tuple(-1, -1));
            continue;
        }

        float obstacleDirection = Point::getDirectionFromOrigoToPoint(L[i]);

        if (targetDirection < obstacleDirection) {
            S1.push_back(std::make_tuple(obstacleDirection, 180.0));
        } else {
            S1.push_back(std::make_tuple(180.0, obstacleDirection));
        }
    }
}

// TODO: Check so it is correct!
void ORM::getS2(std::vector<Point> & L, std::vector<std::tuple<float, float> > & S2) {
    for (size_t i = 0; i < L.size(); i++) {
        if (L[i].x == 0 && L[i].y == 0) {
            S2.push_back(std::make_tuple(-1, -1));
            continue;
        }

        float obstacleDirection, obstacleDistance;

        Point::getVectorFromPoint(L[i], obstacleDirection, obstacleDistance);

        float alpha = std::fabs(std::atan2((radius_ + security_distance_), obstacleDistance)) * 180.0 / PI;

        float beta = 0;

        if (obstacleDistance <= security_distance_ + radius_) {
            beta = (180.0 - alpha) * (1 - ((obstacleDistance - radius_) / security_distance_));
        }

        // TODO: ???
        float gammaL = std::max<float>(obstacleDirection - (alpha + beta), 180.0);
        float gammaR = std::min<float>(obstacleDirection + (alpha + beta), 180.0);

        S2.push_back(std::make_tuple(gammaL, gammaR));
    }
}
*/

/*
void ORM::getS1(std::vector<Point> & L, Point target, std::vector<std::tuple<float, float> > & S1) {
    float targetDirection = Point::getDirection(goal);

    for (size_t i = 0; i < L.size(); i++) {
        float obstacleDirection = Point::getDirection(L[i]);
        float obstacleDistance = Point::getDistance(L[i]);

        std::tuple<float, float> S1;

        if (targetDirection < obstacleDirection) {
            S1 = std::make_tuple(obstacleDirection, PI);
        } else {
            S1 = std::make_tuple(-PI, obstacleDirection);
        }

        float alpha = std::abs(std::atan2((radius_ + security_distance_) / obstacleDistance));

        float beta = 0;

        if (obstacleDistance <= security_distance_ + radius_) {
            beta = (PI - alpha) * (1 - ((obstacleDistance - radius_) / security_distance_));
        }

        float gammaL = std::max(obstacleDirection - (alpha + beta), -PI);
        float gammaR = std::min(obstacleDirection + (alpha + beta), PI);

        std::tuple<float, float> S2 = std::make_tuple(gammaL, gammaR);
    }
}
*/
