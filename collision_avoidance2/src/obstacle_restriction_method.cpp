#include <ros/ros.h>

#include <collision_avoidance/obstacle_restriction_method.h>

#include <exjobb_msgs/SensorReadings.h>

#include <tuple>

#include <exjobb_msgs/ORM.h>

#define PI 3.14159265

int mod(int a, int b)
{
    return ((a %= b) < 0) ? a + b : a;
}

ORM::ORM(float radius, float security_distance, float epsilon)
    : radius_(radius)
    , security_distance_(security_distance)
    , epsilon_(epsilon)
    , min_change_in_direction_(-15)
    , max_change_in_direction_(45)
    , min_opposite_direction_(0)
    , max_opposite_direction_(10)
{
    ros::NodeHandle nh;
    pub_ = nh.advertise<exjobb_msgs::ORM>("orm", 10);
}

bool ORM::avoidCollision(exjobb_msgs::Control * control, const std::vector<Point> & obstacles)
{
    if (control->go_magnitude == 0)
    {
        return true;
    }

    // A. The Subgoal Selector
    // TODO: FIX
    Point goal = subgoalSelector(control->go_direction, control->go_magnitude, obstacles);

    if (goal.x == 0 && goal.y == 0)
    {
        return false;
    }

    // Check if goal = 0

    //ROS_ERROR_STREAM("Between 3: " << Point::getDirection(goal) * 180.0 / PI);

    // B. Motion Computation
    control->go_direction = motionComputation(goal, obstacles);

    //control->go_direction = Point::GetDirectionDegrees(goal);

    //ROS_ERROR_STREAM("Between 4: " << control->go_Direction);

    return true;
}

// Merge with findPotentialAB
void ORM::getPointsOfInterest(const Point & goal, const std::vector<Point> & L, std::vector<Point> * left, std::vector<Point> * right)
{
    float goal_distance = Point::getDistance(goal);

    float degree_per_index = 360.0 / L.size();

    int wanted_index = Point::GetDirectionDegrees(goal) / degree_per_index;

    // Only scan X number of degrees in both directions
    float X = 180;
    for (size_t i = 1; i * degree_per_index < X; i++)
    {
        for (int j = -1; j < 2; j += 2)
        {
            int current_index = mod(wanted_index + (i * j), L.size());

            if (L[current_index].x == 0 && L[current_index].y == 0)
            {
                // Nothing here
                continue;
            }

            if (Point::getDistance(L[current_index]) > goal_distance)
            {
                // To far away to matter
                //continue;
            }

            if (j == -1)
            {
                // To the right
                right->push_back(L[current_index]);
            }
            else
            {
                // To the left
                left->push_back(L[current_index]);
            }

            /*
            if (i == 0)
            {
                break;
            }
            */
        }
    }
}

float radiansToDegrees(float r)
{
    r *= 180.0 / PI;

    if (r < 0)
    {
        return (r + 360);
    }
    return r;
}

int getQuadrant(const Point & point)
{
    if (point.x >= 0)
    {
        if (point.y >= 0)
        {
            return 1;
        }

        return 4;
    }

    if (point.y >= 0)
    {
        return 2;
    }

    return 3;
}

float ORM::getLeftBound(const std::vector<Point> & L, float goal_direction)
{
    float bound;
    bool first_bound = true;

    for (size_t i = 0; i < L.size(); i++)
    {
        float obstacle_direction = Point::GetDirectionDegrees(L[i]);
        float obstacle_distance = Point::getDistance(L[i]);

        float alpha = radiansToDegrees(std::abs(std::atan((radius_ + security_distance_) / obstacle_distance)));

        float beta = 0;
        if (obstacle_distance <= radius_ + security_distance_)
        {
            beta = (180 - alpha) * (1 - ((obstacle_distance - radius_) / security_distance_));
        }

        //float alpha = std::acos(((2 * obstacle_distance * obstacle_distance) - (radius_ + security_distance_)) / (2 * obstacle_distance * obstacle_distance));
        /*
        float beta = 0;

        if (obstacle_distance <= radius_ + security_distance_)
        {
            beta = (PI - alpha) * (1 - ((obstacle_distance - radius_) / security_distance_));
        }

        float temp_bound = obstacle_direction + radiansToDegrees(alpha + beta + (1.0 * 0.174532925));
        */
        /*
        float alpha;
        if (obstacle_distance > radius_ + security_distance_)
        {
            alpha = radiansToDegrees(std::asin((radius_ + security_distance_) / obstacle_distance));
            ROS_ERROR_STREAM(radiansToDegrees(std::asin((radius_ + security_distance_) / obstacle_distance)) << ", " << radius_ + security_distance_ << ", " << obstacle_distance);
        }
        else
        {
            alpha = 0; //90;// + (90 * (1 - ((obstacle_distance - radius_) / security_distance_)));
        }
        */
        float temp_bound = obstacle_direction + (alpha + beta);

        /*
        int quadrant = getQuadrant(L[i]);

        Point p;
        switch (quadrant)
        {
        case 1:
            p.x = L[i].x - (radius_ + security_distance_);
            p.y = L[i].y;
            temp_bound = Point::GetDirectionDegrees(p);
            break;
        case 2:
            p.x = L[i].x;
            p.y = L[i].y - (radius_ + security_distance_);
            temp_bound = Point::GetDirectionDegrees(p);
            break;
        case 3:
            p.x = L[i].x + (radius_ + security_distance_);
            p.y = L[i].y;
            temp_bound = Point::GetDirectionDegrees(p);
            break;
        case 4:
            p.x = L[i].x;
            p.y = L[i].y + (radius_ + security_distance_);
            temp_bound = Point::GetDirectionDegrees(p);
            break;
        }

        if (obstacle_distance <= radius_ + security_distance_)
        {
            temp_bound += 45 * (1 - ((obstacle_distance - radius_) / security_distance_));
        }
        */

        if (temp_bound > 360)
        {
            temp_bound -= 360;
        }

        if (first_bound)
        {
            first_bound = false;
            bound = temp_bound;
            continue;
        }

        float temp_diff = temp_bound - goal_direction;
        float bound_diff = bound - goal_direction;

        if (temp_diff > 180)
        {
            temp_diff -= 360;
        }
        if (bound_diff > 180)
        {
            bound_diff -= 360;
        }
        if (temp_diff < -180)
        {
            temp_diff += 360;
        }
        if (bound_diff < -180)
        {
            bound_diff += 360;
        }

        if (temp_diff > bound_diff)
        {
            bound = temp_bound;
        }

        /*
        float diff = temp_bound - bound;

        if (diff < -180 || (diff < 180 && diff > 0))
        {
            bound = temp_bound;
        }
        */
    }

    return bound;
}

float ORM::getRightBound(const std::vector<Point> & L, float goal_direction)
{
    float bound;
    bool first_bound = true;

    for (size_t i = 0; i < L.size(); i++)
    {
        float obstacle_direction = Point::GetDirectionDegrees(L[i]);
        float obstacle_distance = Point::getDistance(L[i]);


        float alpha = radiansToDegrees(std::abs(std::atan((radius_ + security_distance_) / obstacle_distance)));

        float beta = 0;
        if (obstacle_distance <= radius_ + security_distance_)
        {
            beta = (180 - alpha) * (1 - ((obstacle_distance - radius_) / security_distance_));
        }

        //float alpha = std::acos(((2 * obstacle_distance * obstacle_distance) - (radius_ + security_distance_)) / (2 * obstacle_distance * obstacle_distance));
        /*
        float beta = 0;

        if (obstacle_distance <= radius_ + security_distance_)
        {
            beta = (PI - alpha) * (1 - ((obstacle_distance - radius_) / security_distance_));
        }

        float temp_bound = obstacle_direction - radiansToDegrees(alpha + beta + (1.0 * 0.174532925));
        */
        /*
        float alpha;
        if (obstacle_distance > radius_ + security_distance_)
        {
            alpha = radiansToDegrees(std::asin((radius_ + security_distance_) / obstacle_distance));
            ROS_ERROR_STREAM(radiansToDegrees(std::asin((radius_ + security_distance_) / obstacle_distance)) << ", " << radius_ + security_distance_ << ", " << obstacle_distance);
        }
        else
        {
            alpha = 0; //90 + (90 * (1 - ((obstacle_distance - radius_) / security_distance_)));
        }
        */

        float temp_bound = obstacle_direction - (alpha + beta);

        /*
        int quadrant = getQuadrant(L[i]);

        Point p;
        switch (quadrant)
        {
        case 1:
            p.x = L[i].x;
            p.y = L[i].y - (radius_ + security_distance_);
            temp_bound = Point::GetDirectionDegrees(p);
            break;
        case 2:
            p.x = L[i].x + (radius_ + security_distance_);
            p.y = L[i].y;
            temp_bound = Point::GetDirectionDegrees(p);
            break;
        case 3:
            p.x = L[i].x;
            p.y = L[i].y + (radius_ + security_distance_);
            temp_bound = Point::GetDirectionDegrees(p);
            break;
        case 4:
            p.x = L[i].x - (radius_ + security_distance_);
            p.y = L[i].y;
            temp_bound = Point::GetDirectionDegrees(p);
            break;
        }

        if (obstacle_distance <= radius_ + security_distance_)
        {
            temp_bound -= 45 * (1 - ((obstacle_distance - radius_) / security_distance_));
        }
        */


        if (temp_bound < 0)
        {
            temp_bound += 360;
        }

        if (first_bound)
        {
            first_bound = false;
            bound = temp_bound;
            continue;
        }

        float temp_diff = temp_bound - goal_direction;
        float bound_diff = bound - goal_direction;

        if (temp_diff > 180)
        {
            temp_diff -= 360;
        }
        if (bound_diff > 180)
        {
            bound_diff -= 360;
        }
        if (temp_diff < -180)
        {
            temp_diff += 360;
        }
        if (bound_diff < -180)
        {
            bound_diff += 360;
        }

        if (temp_diff < bound_diff)
        {
            bound = temp_bound;
        }

        /*
        if (diff > 180 || (diff < 0 && diff > -180))
        {
            bound = temp_bound;
        }
        */
    }

    return bound;
}

float ORM::motionComputation(const Point & goal, const std::vector<Point> & L)
{
    std::vector<Point> left, right;

    getPointsOfInterest(goal, L, &left, &right);

    float goal_direction = Point::GetDirectionDegrees(goal);

    exjobb_msgs::ORM msg;
    msg.goal_direction = goal_direction;
    msg.left_bound = -1;
    msg.right_bound = -1;
    msg.radius = radius_;
    msg.security_distance = security_distance_;

    for (size_t i = 0; i < right.size(); i++)
    {
        msg.left_bound_points_x.push_back(right[i].x);
        msg.left_bound_points_y.push_back(right[i].y);
    }
    for (size_t i = 0; i < left.size(); i++)
    {
        msg.right_bound_points_x.push_back(left[i].x);
        msg.right_bound_points_y.push_back(left[i].y);
    }

    if (left.size() != 0 && right.size() != 0)
    {
        // Get left and right bounds
        float left_bound = getLeftBound(right, goal_direction);
        float right_bound = getRightBound(left, goal_direction);

        msg.left_bound = left_bound;
        msg.right_bound = right_bound;
        pub_.publish(msg);

        float diff_left = left_bound - goal_direction;
        float diff_right = right_bound - goal_direction;

        if (diff_left < 0)
        {
            diff_left += 360;
        }
        if (diff_right < 0)
        {
            diff_right += 360;
        }

        if (diff_left > 180 && diff_left < 360 && diff_right > 0 && diff_right < 180)
        {
            //ROS_ERROR_STREAM("Goal direction \t-\t1");
            //ROS_ERROR_STREAM("Goal: " << goal_direction << ", left_bound: " << left_bound << ", right_bound: " << right_bound);
            return goal_direction;
        }
        else if (diff_right > 0 && diff_right < 180 && diff_right > diff_left)
        {
            //ROS_ERROR_STREAM("Left bound \t-\t1");
            //ROS_ERROR_STREAM("Goal: " << goal_direction << ", left_bound: " << left_bound << ", right_bound: " << right_bound);
            return left_bound;
        }
        else if (diff_left > 180 && diff_left < 360 && diff_right > diff_left)
        {
            //ROS_ERROR_STREAM("Right bound \t-\t1");
            //ROS_ERROR_STREAM("Goal: " << goal_direction << ", left_bound: " << left_bound << ", right_bound: " << right_bound);
            return right_bound;
        }
        else
        {
            //ROS_ERROR_STREAM("Mid \t-\t1");
            //ROS_ERROR_STREAM("Goal: " << goal_direction << ", left_bound: " << left_bound << ", right_bound: " << right_bound);
            return getMidDirection(left_bound, right_bound);
        }
    }
    else if (right.size() != 0)
    {
        float left_bound = getLeftBound(right, goal_direction);

        msg.left_bound = left_bound;
        pub_.publish(msg);

        float diff_left = left_bound - goal_direction;

        if (diff_left < 0)
        {
            diff_left += 360;
        }

        if (diff_left > 180 && diff_left < 360)
        {
            //ROS_ERROR_STREAM("Goal direction \t-\t2");
            //ROS_ERROR_STREAM("Goal: " << goal_direction << ", left_bound: " << left_bound);
            return goal_direction;
        }
        else
        {
            //ROS_ERROR_STREAM("Left bound \t-\t2");
            //ROS_ERROR_STREAM("Goal: " << goal_direction << ", left_bound: " << left_bound);
            return left_bound;
        }
    }
    else if (left.size() != 0)
    {
        float right_bound = getRightBound(left, goal_direction);

        msg.right_bound = right_bound;
        pub_.publish(msg);

        float diff_right = right_bound - goal_direction;

        if (diff_right < 0)
        {
            diff_right += 360;
        }

        if (diff_right > 0 && diff_right < 180)
        {
            //ROS_ERROR_STREAM("Goal direction \t-\t3");
            //ROS_ERROR_STREAM("Goal: " << goal_direction << ", right_bound: " << right_bound);
            return goal_direction;
        }
        else
        {
            //ROS_ERROR_STREAM("Right bound \t-\t2");
            //ROS_ERROR_STREAM("Goal: " << goal_direction << ", right_bound: " << right_bound);
            return right_bound;
        }
    }
    else
    {
        pub_.publish(msg);

        //ROS_ERROR_STREAM("Goal direction \t-\t4");
        //ROS_ERROR_STREAM("Goal: " << goal_direction);
        return goal_direction;
    }


        /*
        if (diff_right > 180 || (diff_right < 0 && diff_right > -180))
        {
            if (diff_left < -180 || (diff_left < 180 && diff_left > 0))
            {

                Case 1.1: 16.5, 4.23096, 28.3301
                Case 1.1: 16.5, 4.23096, 28.3301
                Case 1.1: 16.5, 4.23096, 28.3301
                Case 1.1: 16.5, 4.23096, 28.3301
                Case 1.1: 16.5, 4.23096, 28.3301
                Case 1.1: 16.5, 4.23096, 28.3301
                Case 1.1: 16.5, 4.23096, 28.3301
                Case 1.1: 16.5, 4.23096, 28.3301


                // Case 1
                ROS_ERROR_STREAM("Case 1.1: " << goal_direction << ", " << left_bound << ", " << right_bound);
                return goal_direction;
            }

            // Case 2
            ROS_ERROR_STREAM("Left bound 1: " << goal_direction << ", " << left_bound << ", " << right_bound);
            return left_bound;
        }

        if (diff_left < -180 || (diff_left < 180 && diff_left > 0))
        {
            // Case 2
            ROS_ERROR_STREAM("Right bound 1: " << goal_direction << ", " << left_bound << ", " << right_bound);
            return right_bound;
        }

        // Case 3
        ROS_ERROR_STREAM("Case 3.1: " << goal_direction << ", " << left_bound << ", " << right_bound);
        return getMidDirection(left_bound, right_bound);
    }

    if (left.size() != 0)
    {
        // Get right bound
        float right_bound = getRightBound(left);

        msg.right_bound = right_bound;
        pub_.publish(msg);

        float diff = goal_direction - right_bound;

        if (diff > 180 || (diff < 0 && diff > -180))
        {
            // Case 1
            ROS_ERROR_STREAM("Case 1.2: " << goal_direction << ", right_bound: " << right_bound);
            return goal_direction;
        }

        // Case 2
        ROS_ERROR_STREAM("Right bound 2: " << goal_direction << ", right_bound: " << right_bound);
        return right_bound;
    }

    // Get left bound
    float left_bound = getLeftBound(right);

    msg.left_bound = left_bound;
    pub_.publish(msg);

    float diff = goal_direction - left_bound;

    if (diff < -180 || (diff < 180 && diff > 0))
    {
        // Case 1
        ROS_ERROR_STREAM("Case 1.3: " << goal_direction << ", left_bound: " << left_bound);
        return goal_direction;
    }

    // Case 2
    ROS_ERROR_STREAM("Left bound 2: " << goal_direction << ", left_bound: " << left_bound);
    return left_bound;
    */
}











Point ORM::initGoal(float direction, float magnitude)
{
    // TODO: FIX
    return Point::getPointFromVectorDegrees(direction, magnitude * 3);
}

void ORM::findPotentialAB(const std::vector<Point> & L, const Point & goal, std::vector<Point> * A, std::vector<Point> * B)
{
    float degree_per_index = 360.0 / L.size();

    int goal_index = Point::GetDirectionDegrees(goal) / degree_per_index;

    for (size_t i = 1; i * degree_per_index < 45; i++)
    {
        for (int j = -1; j < 2; j += 2)
        {
            int current_index = mod(goal_index + (i * j), L.size());

            if (L[current_index].x == 0 && L[current_index].y == 0)
            {
                // No reading here
                continue;
            }

            if (j == -1)
            {
                // To the right
                B->push_back(L[current_index]);
            }
            else
            {
                A->push_back(L[current_index]);
            }
        }
    }
}

// Source: http://stackoverflow.com/questions/2825412/draw-a-parallel-line
void ORM::getRectangle(const Point & goal, float radius, Point * a, Point * b, Point * c, Point * d)
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

// http://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection
std::vector<Point> ORM::getPointsInRectangle(const std::vector<Point> & L, const Point & a, const Point & b, const Point & c, const Point & d)
{
    std::vector<Point> Lp;

    for (size_t i = 0; i < L.size(); i++)
    {
        if (L[i].x == 0 && L[i].y == 0)
        {
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
    }

    return Lp;
}

bool ORM::isClearPath(const Point & goal, const std::vector<Point> & L)
{
    // A contains points on the left side of goal
    // B contains points on the right side of goal
    std::vector<Point> A, B;

    findPotentialAB(L, goal, &A, &B);

    // Corners of the rectangle
    Point a, b, c, d;

    // 2 * radius because then all obstacles that matters will be inside the rectangle
    getRectangle(goal, 2 * radius_, &a, &b, &c, &d);

    A = getPointsInRectangle(A, a, b, c, d);
    B = getPointsInRectangle(B, a, b, c, d);

    // Check if path is clear
    for (size_t i = 0; i < A.size(); i++)
    {
        for (size_t j = 0; j < B.size(); j++)
        {
            if (Point::getDistance(A[i], B[j]) < 2 * radius_)
            {
                return false;
            }
        }
    }

    return true;
}

float ORM::getMidDirection(float d1, float d2)
{
    float mid_direction = (d1 + d2) / 2.0;

    if (std::fabs(d1 - d2) <= 180)
    {
        return mid_direction;
    }

    if (mid_direction >= 180)
    {
        return (mid_direction - 180);
    }

    return (mid_direction + 180);
}

Point ORM::subgoalSelector(const float direction, const float magnitude, const std::vector<Point> & L)
{
    Point prefered_goal = initGoal(direction, 3);

    if (isClearPath(prefered_goal, L))
    {
        //ROS_ERROR_STREAM("Clear path");
        // We can go where we want
        return prefered_goal;
    }

    float degree_per_index = 360.0 / L.size();

    int wanted_index = direction / degree_per_index;

    float change_in_direction = ((max_change_in_direction_ - min_change_in_direction_) * magnitude) + min_change_in_direction_;

    float opposite_direction = ((max_opposite_direction_ - min_opposite_direction_) * (1 - magnitude)) + min_opposite_direction_;

    int right_sub_goal = -1, left_sub_goal = -1;

    Point sub_goal;

    // We cannot go where we "want" so find a subgoal
    // Look for subgoals close to wanted_direction
    for (size_t i = 1; i * degree_per_index < change_in_direction; i++)
    {
        if (right_sub_goal != -1 && (i - right_sub_goal) * degree_per_index > opposite_direction)
        {
            return sub_goal;
        }

        if (left_sub_goal != -1 && (i - left_sub_goal) * degree_per_index > opposite_direction)
        {
            return sub_goal;
        }

        for (int j = -1; j < 2; j += 2)
        {
            if (j == -1 && left_sub_goal != -1)
            {
                continue;
            }
            if (j == 1 && right_sub_goal != -1)
            {
                continue;
            }

            int current_index = mod(wanted_index + (i * j), L.size());

            if (L[current_index].x == 0 && L[current_index].y == 0)
            {
                // No reading here
                continue;
            }

            for (int k = -1; k < 2; k += 2)
            {
                int next_to_index = mod(current_index + k, L.size());

                if (L[next_to_index].x == 0 && L[next_to_index].y == 0)
                {
                    // Subgoal at the edge of an obstacle
                    float direction = getMidDirection(degree_per_index * current_index, degree_per_index * next_to_index);
                    float distance = Point::getDistance(L[current_index]) + (radius_ * 2) + epsilon_;

                    Point temp_sub_goal = Point::getPointFromVectorDegrees(direction, distance);

                    //ROS_ERROR_STREAM(round(Point::GetDirectionDegrees(prefered_goal)) << " -> " << round(Point::GetDirectionDegrees(subgoal)));

                    // Check if this subgoal is clear
                    if (isClearPath(temp_sub_goal, L))
                    {
                        //ROS_ERROR_STREAM("Subgoal at the edge");
                        // Return this subgoal
                        //return subgoal;

                        // Goal on both sides!!!
                        if (j == -1 && right_sub_goal != -1)
                        {
                            return prefered_goal;
                        }
                        if (j == 1 && left_sub_goal != -1)
                        {
                            return prefered_goal;
                        }


                        if (j == -1)
                        {
                            left_sub_goal = i;
                        }
                        if (j == 1)
                        {
                            right_sub_goal = i;
                        }

                        sub_goal = temp_sub_goal;
                    }

                }
                else if (Point::getDistance(L[current_index], L[next_to_index]) > 2 * radius_)
                {
                    // Subgoal between two obstacles
                    Point temp_sub_goal = Point::getMidpoint(L[current_index], L[next_to_index]);

                    // Check if this subgoal is clear
                    if (isClearPath(temp_sub_goal, L))
                    {
                        //ROS_ERROR_STREAM("Subgoal between two obstacles");
                        // Return this subgoal
                        //return subgoal;

                        // Goal on both sides!!!
                        if (j == -1 && right_sub_goal != -1)
                        {
                            return prefered_goal;
                        }
                        if (j == 1 && left_sub_goal != -1)
                        {
                            return prefered_goal;
                        }


                        if (j == -1)
                        {
                            left_sub_goal = i;
                        }
                        if (j == 1)
                        {
                            right_sub_goal = i;
                        }

                        sub_goal = temp_sub_goal;
                    }
                }
            }
        }
    }

    /*
    for (float distance = 2.9; distance > radius_ + security_distance_; distance -= 0.1)
    {
        Point temp_goal = initGoal(direction, distance);

        if (isClearPath(temp_goal, L))
        {
            //ROS_ERROR_STREAM("Clear path");
            // We can go where we want
            return temp_goal;
        }
    }
    */

    //ROS_FATAL_STREAM("NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");

    // Did not find a subgoal
    /*
    Point point;
    point.x = 0;
    point.y = 0;
    return point;
    */

    if (left_sub_goal != -1 || right_sub_goal != -1)
    {
        return sub_goal;
    }


    return prefered_goal;
}
