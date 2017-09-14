#include <collision_avoidance/obstacle_restriction_method.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace collision_avoidance
{
    int mod(int a, int b)
    {
        return ((a %= b) < 0) ? a + b : a;
    }

    double radiansToDegrees(double r)
    {
        r *= 180.0 / M_PI;

        if (r < 0)
        {
            return (r + 360);
        }

        return r;
    }

    ORM::ORM(double radius, double security_distance, double epsilon, double min_distance_hold, double min_change_in_direction, double max_change_in_direction, double min_opposite_direction, double max_opposite_direction)
        : radius_(radius)
        , security_distance_(security_distance)
        , epsilon_(epsilon)
        , min_distance_hold_(min_distance_hold)
        , min_change_in_direction_(min_change_in_direction)
        , max_change_in_direction_(max_change_in_direction)
        , min_opposite_direction_(min_opposite_direction)
        , max_opposite_direction_(max_opposite_direction)
        , no_input_(NoInput(radius, security_distance, min_distance_hold))
    {
        pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI> >("point", 1);
    }

    bool ORM::avoidCollision(controller_msgs::Controller * controller, const double magnitude, const std::vector<Point> & obstacles)
    {
        if (controller->twist_stamped.twist.linear.x == 0 && controller->twist_stamped.twist.linear.y == 0)
        {
            // Nothing to do!
            // TODO: No input?
            no_input_.avoidCollision(controller, obstacles);
            return true;
        }

        Point goal(controller->twist_stamped.twist.linear.x, controller->twist_stamped.twist.linear.y);

        // A. The Subgoal Selector
        goal = subgoalSelector(goal, magnitude, obstacles);

        pcl::PointCloud<pcl::PointXYZI> cloud;
        /*
        {
            pcl::PointXYZI point;
            point.x = goal.x_;
            point.y = goal.y_;
            point.z = 0.0;
            point.intensity = 0.5;
            cloud.points.push_back(point);
        }
        */

        controller->twist_stamped.twist.linear.x = goal.x_;
        controller->twist_stamped.twist.linear.y = goal.y_;

        if (goal.x_ == 0 && goal.y_ == 0)
        {
            // Could not find any good command :(
            // TODO: No input?
            no_input_.avoidCollision(controller, obstacles);
            return false;
        }

        // B. Motion Computation
        goal = motionComputation(goal, obstacles);

        controller->twist_stamped.twist.linear.x = goal.x_;
        controller->twist_stamped.twist.linear.y = goal.y_;

        {
            pcl::PointXYZI point;
            point.x = goal.x_;
            point.y = goal.y_;
            point.z = 0.0;
            point.intensity = 0.8;
            cloud.points.push_back(point);
        }
        /*
        for (size_t i = 0; i < obstacles.size(); ++i)
        {
            pcl::PointXYZI point;
            point.x = obstacles[i].x_;
            point.y = obstacles[i].y_;
            point.z = 0.0;
            point.intensity = 0.2;
            cloud.points.push_back(point);
        }
        */

        cloud.header.frame_id = "base_link";
        pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
        pub_.publish(cloud);

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

        double degrees_per_index = 360.0 / L.size();

        int wanted_index = Point::getDirectionDegrees(goal) / degrees_per_index;

        double change_in_direction = ((max_change_in_direction_ - min_change_in_direction_) * magnitude) + min_change_in_direction_; // 180;
        double opposite_direction = 0; //((max_opposite_direction_ - min_opposite_direction_) * (1.0 - magnitude)) + min_opposite_direction_; // 0;

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

                if (L[current_index].x_ == 0 && L[current_index].y_ == 0)
                {
                    // No reading here
                    continue;
                }

                for (int k = -1; k < 2; k += 2)
                {
                    int next_to_index = mod(current_index + k, L.size());

                    bool found_sub_goal = false;
                    Point temp_sub_goal;

                    if (L[next_to_index].x_ == 0 && L[next_to_index].y_ == 0)
                    {
                        // Subgoal at the edge of an obstacle
                        //ROS_FATAL_STREAM("Edge of obstacle");
                        double direction = getMidDirection(current_index * degrees_per_index, next_to_index * degrees_per_index);
                        double distance = Point::getDistance(L[current_index]) + (radius_ * 2.0) + epsilon_;

                        temp_sub_goal = Point::getPointFromVectorDegrees(direction, distance);
                        found_sub_goal = true;
                    }
                    else if (Point::getDistance(L[current_index], L[next_to_index]) > 2.0 * radius_)
                    {
                        //ROS_FATAL_STREAM("Between obstacles");
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
                        return sub_goal;
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
        std::vector<Point> rectangle;
        //Point a, b, c, d;

        // 2 * radius because then all obstacles that matters will be inside the rectangle
        //getRectangle(goal, 2.0d * radius_, &a, &b, &c, &d);
        getRectangle(goal, 2.0 * radius_, &rectangle);

        A = getPointsInPolygon(A, rectangle); //getPointsInRectangle(A, a, b, c, d);
        B = getPointsInPolygon(B, rectangle); //getPointsInRectangle(B, a, b, c, d);

        /*
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.header.frame_id = "base_link";
        pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
        {
            pcl::PointXYZI point;
            point.x = goal.x_;
            point.y = goal.y_;
            point.z = 0.0;
            point.intensity = 0.5;
            cloud.points.push_back(point);
        }
        for (size_t i = 0; i < A.size(); ++i)
        {
            pcl::PointXYZI point;
            point.x = A[i].x_;
            point.y = A[i].y_;
            point.z = 0.0;
            point.intensity = 0.2;
            cloud.points.push_back(point);
        }
        for (size_t i = 0; i < B.size(); ++i)
        {
            pcl::PointXYZI point;
            point.x = B[i].x_;
            point.y = B[i].y_;
            point.z = 0.0;
            point.intensity = 0.65;
            cloud.points.push_back(point);
        }
        pub_.publish(cloud);
        */


        // Check if path is clear
        for (size_t i = 0; i < A.size(); ++i)
        {
            for (size_t j = 0; j < B.size(); ++j)
            {
                if (Point::getDistance(A[i], B[j]) < 2.0 * radius_)
                {
                    return false;
                }
            }
        }

        return true;
    }

    void ORM::findPotentialAB(const std::vector<Point> & L, const Point & goal, std::vector<Point> * A, std::vector<Point> * B)
    {
        double degrees_per_index = 360.0 / L.size();

        int goal_index = Point::getDirectionDegrees(goal) / degrees_per_index;

        // Why 45? :O
        for (int i = 1; i * degrees_per_index < 90; ++i)
        {
            for (int j = -1; j < 2; j += 2)
            {
                int current_index = mod(goal_index + (i * j), L.size());

                if (L[current_index].x_ == 0 && L[current_index].y_ == 0)
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
    void ORM::getRectangle(const Point & goal, double radius, std::vector<Point> * verticies)
    {
        /*
        double x1 = 0.0d;
        double x2 = goal.x_;
        double y1 = 0.0d;
        double y2 = goal.y_;

        double L = distance...;

        */


        double distance = Point::getDistance(goal);

        double dx = goal.y_ / distance;
        double dy = -goal.x_ / distance;

        Point a(radius * dx, radius * dy);
        Point b(goal.x_ + a.x_, goal.y_ + a.y_);
        Point c(goal.x_ - a.x_, goal.y_ - a.y_);
        Point d(-a.x_, -a.y_);

        verticies->push_back(a);
        verticies->push_back(b);
        verticies->push_back(c);
        verticies->push_back(d);
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

    std::vector<Point> ORM::getPointsInPolygon(const std::vector<Point> & L, const std::vector<Point> & verticies)
    {
        std::vector<Point> Lp;

        for (size_t i = 0; i < L.size(); ++i)
        {
            if (L[i].x_ == 0 && L[i].y_ == 0)
            {
                // No reading here
                continue;
            }

            // Is point inside polygon?
            if (isPointInPolygon(L[i], verticies))
            {
                // Inside polygon
                Lp.push_back(L[i]);
            }
        }

        return Lp;
    }

    // http://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection
    std::vector<Point> ORM::getPointsInRectangle(const std::vector<Point> & L, const Point & a, const Point & b, const Point & c, const Point & d)
    {
        std::vector<Point> Lp;

        for (size_t i = 0; i < L.size(); ++i)
        {
            if (L[i].x_ == 0 && L[i].y_ == 0)
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


    // Source: https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
    bool ORM::isPointInPolygon(const Point & point, const std::vector<Point> & verticies)
    {
        int i;
        int j;
        bool c = false;
        for (i = 0, j = verticies.size() - 1; i < verticies.size(); j = i++)
        {
            if (((verticies[i].y_ > point.y_) != (verticies[j].y_ > point.y_)) &&
                    (point.x_ < (verticies[j].x_ - verticies[i].x_) * (point.y_ - verticies[i].y_) / (verticies[j].y_ - verticies[i].y_) + verticies[i].x_))
            {
                c = !c;
            }
        }

        return c;
    }

    bool ORM::isPointInsideRectangle(const Point & a, const Point & b, const Point & c, const Point & d, const Point & p)
    {
        Point al;
        Point ab;
        Point ad;

        al.x_ = p.x_ - a.x_;
        al.y_ = p.y_ - a.y_;

        ab.x_ = b.x_ - a.x_;
        ab.y_ = b.y_ - a.y_;

        ad.x_ = d.x_ - a.x_;
        ad.y_ = d.y_ - a.y_;

        return (0 <= (al.x_ * ab.x_) + (al.y_ * ab.y_) &&
                (al.x_ * ab.x_) + (al.y_ * ab.y_) <= (ab.x_ * ab.x_) + (ab.y_ * ab.y_) &&
                0 <= (al.x_ * ad.x_) + (al.y_ * ad.y_) &&
                (al.x_ * ad.x_) + (al.y_ * ad.y_) <= (ad.x_ * ad.x_) + (ad.y_ * ad.y_));
    }

    double ORM::getMidDirection(double d1, double d2)
    {
        double mid_direction = (d1 + d2) / 2.0;

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
        std::vector<Point> left;
        std::vector<Point> right;

        getPointsOfInterest(goal, L, &left, &right);

        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.header.frame_id = "base_link";
        pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
        {
            pcl::PointXYZI point;
            point.x = goal.x_;
            point.y = goal.y_;
            point.z = 0.0;
            point.intensity = 0.5;
            cloud.points.push_back(point);
        }
        for (size_t i = 0; i < left.size(); ++i)
        {
            pcl::PointXYZI point;
            point.x = left[i].x_;
            point.y = left[i].y_;
            point.z = 0.0;
            point.intensity = 0.2;
            cloud.points.push_back(point);
        }
        for (size_t i = 0; i < right.size(); ++i)
        {
            pcl::PointXYZI point;
            point.x = right[i].x_;
            point.y = right[i].y_;
            point.z = 0.0;
            point.intensity = 0.65;
            cloud.points.push_back(point);
        }
        pub_.publish(cloud);

        double goal_direction = Point::getDirectionDegrees(goal);

        if (left.size() != 0 && right.size() != 0)
        {
          ROS_FATAL("Hello");
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
              ROS_FATAL("Middle");
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

        double degrees_per_index = 360.0 / L.size();

        int wanted_index = Point::getDirectionDegrees(goal) / degrees_per_index;

        // Only scan X number of degrees in both directions
        double X = 180;
        for (size_t i = 1; i * degrees_per_index < X; ++i)
        {
            for (int j = -1 ; j < 2; j += 2)
            {
                int current_index = mod(wanted_index + (i * j), L.size());

                if (L[current_index].x_ == 0 && L[current_index].y_ == 0)
                {
                    // No reading here
                    continue;
                }

                if (Point::getDistance(L[current_index]) > goal_distance + radius_)
                {
                    // Too far away to matter
                    continue;
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
            double obstacle_direction = Point::getDirectionDegrees(L[i]);
            double obstacle_distance = Point::getDistance(L[i]);

            double alpha = radiansToDegrees(std::abs(std::atan((radius_ + security_distance_) / obstacle_distance)));

            double beta = 0;
            if (obstacle_distance <= radius_ + security_distance_)
            {
                beta = (180.0 - alpha) * (1 - ((obstacle_distance - radius_) / security_distance_));
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
            double obstacle_direction = Point::getDirectionDegrees(L[i]);
            double obstacle_distance = Point::getDistance(L[i]);

            double alpha = radiansToDegrees(std::abs(std::atan((radius_ + security_distance_) / obstacle_distance)));

            double beta = 0;
            if (obstacle_distance <= radius_ + security_distance_)
            {
                beta = (180.0 - alpha) * (1 - ((obstacle_distance - radius_) / security_distance_));
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
