#include <collision_avoidance/obstacle_restriction_method.h>

namespace collision_avoidance
{
    int mod(int a, int b)
    {
        return ((a %= b) < 0) ? a + b : a;
    }

    double radiansToDegrees(double r)
    {
        r *= 180.0d / M_PI;

        if (r < 0)
        {
            return (r + 360);
        }
        return r;
    }

    ORM::ORM(double radius, double security_distance, double epsilon, double min_change_in_direction, double max_change_in_direction, double min_opposite_direction, double max_opposite_direction)
        : radius_(radius)
        , security_distance_(security_distance)
        , epsilon_(epsilon)
        , min_change_in_direction_(min_change_in_direction)
        , max_change_in_direction_(max_change_in_direction)
        , min_opposite_direction_(min_opposite_direction)
        , max_opposite_direction_(max_opposite_direction)
    {

    }

    bool ORM::avoidCollision(controller_msgs::Controller * controller, const std::vector<Point> & obstacles)
    {
        if (controller->x == 0 && controller->y == 0)
        {
            // Nothing to do!
            // TODO: No input?
            return true;
        }

        Point goal = initGoal(controller->x, controller->y);

        // A. The Subgoal Selector
        goal = subgoalSelector(goal, 1, obstacles);

        if (goal.x_ == 0 && goal.y_ == 0)
        {
            // Could not find any good command :(
            // TODO: No input?
            return false;
        }

        // B. Motion Computation
        goal = motionComputation(goal, obstacles);

        controller->x = goal.x_;
        controller->y = goal.y_;

        return true;
    }

    Point ORM::initGoal(double x, double y)
    {
        return Point(x, y);
    }

    Point ORM::subgoalSelector(const Point & goal, double magnitude, const std::vector<Point> & L)
    {
        if (isClearPath(goal, L))
        {
            // We can go where we want
            return goal;
        }

        double degrees_per_index = 360.0d / L.size();

        int wanted_index = Point::getDirection(goal) / degrees_per_index;

        double change_in_direction = ((max_change_in_direction_ - min_change_in_direction_) * magnitude) + min_change_in_direction_;
        double opposite_direction = ((max_opposite_direction_ - min_opposite_direction_) * (1.0d - magnitude)) + min_opposite_direction_;

        int right_sub_goal = -1, left_sub_goal = -1;

        Point sub_goal;

        // We cannot go directly towards where we "want", so find a subgoal
        // Look for subgoals close to goal
        for (size_t i = 1; i * degrees_per_index < change_in_direction; ++i)
        {
            if (right_sub_goal != -1 && (i - right_sub_goal) * degrees_per_index > opposite_direction)
            {
                // Return the subgoal that is to the right
                return sub_goal;
            }

            if (left_sub_goal != -1 && (i - left_sub_goal) * degrees_per_index > opposite_direction)
            {
                // Return the subgoal that is to the left
                return sub_goal;
            }

            for (int j = -1; j < 2; j += 2)
            {
                if (left_sub_goal != -1 && j == -1)
                {
                    // Have already found a subgoal to the left
                    continue;
                }
                if (right_sub_goal != -1 && j == 1)
                {
                    // Have already found a subgoal to the right
                    continue;
                }

                int current_index = mod(wanted_index + (i * j), L.size());

                if (L[current_index].x_ == -1 && L[current_index].y_ == -1)
                {
                    // No reading here
                    continue;
                }

                for (int k = -1; k < 2; k += 2)
                {
                    int next_to_index = mod(current_index + k, L.size());

                    bool found_sub_goal = false;
                    Point temp_sub_goal;

                    if (L[next_to_index].x_ == -1 && L[next_to_index].y_ == -1)
                    {
                        // Subgoal at the edge of an obstacle
                        double direction = getMidDirection(current_index * degrees_per_index, next_to_index * degrees_per_index);
                        double distance = Point::getDistance(L[current_index]) + (radius_ * 2.0d) + epsilon_;

                        temp_sub_goal = Point::getPointFromVectorDegrees(direction, distance);
                        found_sub_goal = true;
                    }
                    else if (Point::getDistance(L[current_index], L[next_to_index]) > 2.0d * radius_)
                    {
                        temp_sub_goal = Point::getMidpoint(L[current_index], L[next_to_index]);
                        found_sub_goal = true;
                    }

                    if (found_sub_goal && isClearPath(temp_sub_goal, L))
                    {
                        // Goal on both sides!!!
                        // Do not know which one to pick so go with goal instead
                        if (right_sub_goal != -1 && j == -1)
                        {
                            return goal;
                        }
                        else if (left_sub_goal != -1 && j == 1)
                        {
                            return goal;
                        }

                        if (j == -1)
                        {
                            left_sub_goal = i;
                        }
                        else if (j == 1)
                        {
                            right_sub_goal = i;
                        }

                        sub_goal = temp_sub_goal;
                    }
                }
            }
        }

        if (left_sub_goal != -1 || right_sub_goal != -1)
        {
            // Found a subgoal
            return sub_goal;
        }

        // No subgoal found
        return goal;
    }

    bool ORM::isClearPath(const Point & goal, const std::vector<Point> & L)
    {
        // A contains points on the left side of goal
        // B contains points on the right side of goal
        std::vector<Point> A, B;

        findPotentialAB(L, goal, &A, &B);

        // Corners of the rectangle (tunnel)
        Point a, b, c, d;

        // 2 * radius because then all obstacles that matters will be inside the rectangle
        getRectangle(goal, 2.0d * radius_, &a, &b, &c, &d);

        A = getPointsInRectangle(A, a, b, c, d);
        B = getPointsInRectangle(B, a, b, c, d);

        // Check if path is clear
        for (size_t i = 0; i < A.size(); ++i)
        {
            for (size_t j = 0; j < B.size(); ++j)
            {
                if (Point::getDistance(A[i], B[j]) < 2.0d * radius_)
                {
                    return false;
                }
            }
        }

        return true;
    }

    void ORM::findPotentialAB(const std::vector<Point> & L, const Point & goal, std::vector<Point> * A, std::vector<Point> * B)
    {
        double degrees_per_index = 360.0d / L.size();

        int goal_index = Point::getDirectionDegrees(goal) / degrees_per_index;

        // Why 45? :O
        for (size_t i = 1; i * degrees_per_index < 45; ++i)
        {
            for (int j = -1; j < 2; j += 2)
            {
                int current_index = mod(goal_index + (i * j), L.size());

                if (L[current_index].x_ == -1 && L[current_index].y_ == -1)
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
                    // To the left
                    A->push_back(L[current_index]);
                }
            }
        }
    }

    // Source: http://stackoverflow.com/questions/2825412/draw-a-parallel-line
    void ORM::getRectangle(const Point & goal, double radius, Point * a, Point * b, Point * c, Point * d)
    {
        double distance = Point::getDistance(goal);

        double dx = goal.y_ / distance;
        double dy = -goal.x_ / distance;

        a->x_ = radius_ * dx;
        a->y_ = radius_ * dy;

        b->x_ = goal.x_ + a->x_;
        b->y_ = goal.y_ + a->y_;

        c->x_ = goal.x_ - a->x_;
        c->y_ = goal.y_ - a->y_;

        d->x_ = -a->x_;
        d->y_ = -a->y_;
    }

    // http://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection
    std::vector<Point> ORM::getPointsInRectangle(const std::vector<Point> & L, const Point & a, const Point & b, const Point & c, const Point & d)
    {
        std::vector<Point> Lp;

        for (size_t i = 0; i < L.size(); ++i)
        {
            if (L[i].x_ == -1 && L[i].y_ == -1)
            {
                // No reading here
                continue;
            }

            // Is point inside rectangle?
            if (isPointInsideRectangle(a, b, c, d, L[i]))
            {
                // Inside rectangle
                Lp.push_back(L[i]);
            }
        }

        return Lp;
    }

    bool ORM::isPointInsideRectangle(const Point & a, const Point & b, const Point & c, const Point & d, const Point & p)
    {
        Point al, ab, ad;
        al.x_ = p.x_ - a.x_;
        al.y_ = p.y_ - a.y_;

        ab.x_ = b.x_ - a.x_;
        ab.y_ = b.y_ - a.y_;

        ad.x_ = d.x_ - a.x_;
        ad.y_ = d.y_ - a.y_;

        return (0 <= (al.x_ * ab.x_) + (al.y_ * ab.y_) &&
                (al.x_ * ab.x_) + (al.y_ * ab.y_) <= (ab.x_ * ab.x_) + (ab.y_ * ab.y_) &&
                0 <= (al.x_ * al.x_) + (al.y_ * al.y_) &&
                (al.x_ * ad.x_) + (al.y_ * ad.y_) <= (ad.x_ * ad.x_) + (ad.y_ * ad.y_));
    }

    double ORM::getMidDirection(double d1, double d2)
    {
        double mid_direction = (d1 + d2) / 2.0d;

        if (std::fabs(d1 - d2) <=  180)
        {
            return mid_direction;
        }

        if (mid_direction >= 180)
        {
            return (mid_direction - 180);
        }

        return (mid_direction + 180);
    }

    Point ORM::motionComputation(const Point & goal, const std::vector<Point> & L)
    {
        std::vector<Point> left, right;

        getPointsOfInterest(goal, L, &left, &right);

        double goal_direction = Point::getDirectionDegrees(goal);

        if (left.size() != 0 && right.size() != 0)
        {
            // Get left and right bounds
            double left_bound = getLeftBound(right, goal_direction);
            double right_bound = getRightBound(left, goal_direction);

            double diff_left = left_bound - goal_direction;
            double diff_right = right_bound - goal_direction;

            if (diff_left < 0)
            {
                diff_left += 360;
            }
            if (diff_right < 0)
            {
                diff_right += 360;
            }

            // TODO: Have to check under 360?!
            if (diff_left > 180 && diff_left < 360 && diff_right > 0 && diff_right < 180)
            {
                return Point::getPointFromVectorDegrees(goal_direction, Point::getDistance(goal));
            }
            else if (diff_right > 0 && diff_right < 180 && diff_right > diff_left)
            {
                return Point::getPointFromVectorDegrees(left_bound, Point::getDistance(goal));
            }
            else if (diff_left > 180 && diff_left < 360 && diff_right > diff_left)
            {
                return Point::getPointFromVectorDegrees(right_bound, Point::getDistance(goal));
            }
            else
            {
                return Point::getPointFromVectorDegrees(getMidDirection(left_bound, right_bound), Point::getDistance(goal));
            }
        }
        else if (right.size() != 0)
        {
            double left_bound = getLeftBound(right, goal_direction);

            double diff_left = left_bound - goal_direction;

            if (diff_left < 0)
            {
                diff_left += 360;
            }

            if (diff_left > 180 && diff_left < 360)
            {
                return Point::getPointFromVectorDegrees(goal_direction, Point::getDistance(goal));
            }
            else
            {
                return Point::getPointFromVectorDegrees(left_bound, Point::getDistance(goal));
            }
        }
        else if (left.size() != 0)
        {
            double right_bound = getRightBound(left, goal_direction);

            double diff_right = right_bound - goal_direction;

            if (diff_right < 0)
            {
                diff_right += 360;
            }

            if (diff_right > 0 && diff_right < 180)
            {
                return Point::getPointFromVectorDegrees(goal_direction, Point::getDistance(goal));
            }
            else
            {
                return Point::getPointFromVectorDegrees(right_bound, Point::getDistance(goal));
            }
        }
        else
        {
            return Point::getPointFromVectorDegrees(goal_direction, Point::getDistance(goal));
        }
    }

    // Merge with findPotentialAB
    void ORM::getPointsOfInterest(const Point & goal, const std::vector<Point> & L, std::vector<Point> * left, std::vector<Point> * right)
    {
        double goal_distance = Point::getDistance(goal);

        double degrees_per_index = 360.0d / L.size();

        int wanted_index = Point::getDirectionDegrees(goal) / degrees_per_index;

        // Only scan X number of degrees in both directions
        double X = 180;
        for (size_t i = 1; i * degrees_per_index < X; ++i)
        {
            for (int j = -1 ; j < 2; j += 2)
            {
                int current_index = mod(wanted_index + (i * j), L.size());

                if (L[current_index].x_ == -1 && L[current_index].y_ == -1)
                {
                    // No reading here
                    continue;
                }

                if (Point::getDistance(L[current_index]) > goal_distance)
                {
                    // Too far away to matter
                    // continue;
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
            }
        }
    }

    double ORM::getLeftBound(const std::vector<Point> & L, double goal_direction)
    {
        double bound;
        bool first_bound = true;

        for (size_t i = 0; i < L.size(); ++i)
        {
            double obstacle_direction = Point::getDirection(L[i]);
            double obstacle_distance = Point::getDistance(L[i]);

            double alpha = radiansToDegrees(std::abs(std::atan((radius_ + security_distance_) / obstacle_distance)));

            double beta = 0;
            if (obstacle_distance <= radius_ + security_distance_)
            {
                beta = (180.0d - alpha) * (1 - ((obstacle_distance - radius_) / security_distance_));
            }

            double temp_bound = obstacle_direction + (alpha + beta);

            if (temp_bound > 360)
            {
                temp_bound -= 360;
            }

            if (first_bound)
            {
                first_bound = false;
                bound = temp_bound;
            }
            else
            {
                double temp_diff = temp_bound - goal_direction;
                double bound_diff = bound - goal_direction;

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
            }
        }

        return bound;
    }

    double ORM::getRightBound(const std::vector<Point> & L, double goal_direction)
    {
        double bound;
        bool first_bound = true;

        for (size_t i = 0; i < L.size(); ++i)
        {
            double obstacle_direction = Point::getDirection(L[i]);
            double obstacle_distance = Point::getDistance(L[i]);

            double alpha = radiansToDegrees(std::abs(std::atan((radius_ + security_distance_) / obstacle_distance)));

            double beta = 0;
            if (obstacle_distance <= radius_ + security_distance_)
            {
                beta = (180.0d - alpha) * (1 - ((obstacle_distance - radius_) / security_distance_));
            }

            double temp_bound = obstacle_direction - (alpha + beta);

            if (temp_bound < 0)
            {
                temp_bound += 360;
            }

            if (first_bound)
            {
                first_bound = false;
                bound = temp_bound;
            }
            else
            {
                double temp_diff = temp_bound - goal_direction;
                double bound_diff = bound - goal_direction;

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
            }
        }

        return bound;
    }

}
