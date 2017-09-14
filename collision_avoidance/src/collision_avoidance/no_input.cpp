#include <collision_avoidance/no_input.h>

namespace collision_avoidance
{
    NoInput::NoInput(double radius, double security_distance, double min_distance_hold)
        : radius_(radius)
        , security_distance_(security_distance)
        , min_distance_hold_(min_distance_hold)
    {

    }

    void NoInput::avoidCollision(controller_msgs::Controller * control, const std::vector<Point> & obstacles)
    {
        double x_min = 10000;
        double y_min = 10000;
        double x_max = -10000;
        double y_max = -10000;

        for (size_t i = 0; i < obstacles.size(); ++i)
        {
            if (obstacles[i].x_ == 0 && obstacles[i].y_ == 0)
            {
                // No reading here
                continue;
            }

            double distance = Point::getDistance(obstacles[i]);

            if (distance <= radius_ + min_distance_hold_)
            {
                double direction = Point::getDirectionDegrees(obstacles[i]) + 180.0;
                if (direction >= 360)
                {
                    direction -= 360;
                }

                double magnitude = (radius_ + min_distance_hold_) - distance;

                Point p = Point::getPointFromVectorDegrees(direction, magnitude);

                x_min = std::min(x_min, p.x_);
                x_max = std::max(x_max, p.x_);
                y_min = std::min(y_min, p.y_);
                y_max = std::max(y_max, p.y_);
            }
        }

        control->twist_stamped.twist.linear.x = (x_min + x_max) / 2.0;
        control->twist_stamped.twist.linear.y = (y_min + y_max) / 2.0;

        /*
        if (x_min == 10000 && y_min == 10000)
        {
            // There are no obstacles close
            // Try to stop completely

            control->x = 0;
            control->y = 0;
        }
        else
        {
            Point p;
            p.x_ = (x_min + x_max) / 2.0d;
            p.y_ = (y_min + y_max) / 2.0d;
        }
        */
    }
}
