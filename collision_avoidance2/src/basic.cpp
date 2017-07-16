#include <ros/ros.h>

#include <collision_avoidance/basic.h>

Basic::Basic(float radius, float security_distance, float min_distance_hold)
    : radius_(radius)
    , security_distance_(security_distance)
    , min_distance_hold_(min_distance_hold)
{

}

void Basic::avoidCollision(exjobb_msgs::Control * control, const std::vector<Point> & obstacles, float wanted_direction, float current_direction, float current_speed)
{
    float diff_direction = control->go_direction - wanted_direction;

    if (diff_direction > 180)
    {
        diff_direction -= 360;
    }
    else if (diff_direction < -180)
    {
        diff_direction += 360;
    }

    if (std::fabs(diff_direction) > 70)
    {
        control->go_magnitude = 0;
    }

    if (control->go_magnitude == 0)
    {
        // Stay in place
        stayInPlace(control, obstacles, current_direction, current_speed);
    }
    else
    {
        // We are moving
        moving(control, obstacles, wanted_direction, current_direction, current_speed);
    }
}


void Basic::stayInPlace(exjobb_msgs::Control * control, const std::vector<Point> & obstacles, float current_direction, float current_speed)
{
    float x_min = 1000, y_min = 1000, x_max = -1000, y_max = -1000;

    for (size_t i = 0; i < obstacles.size(); i++)
    {
        if (obstacles[i].x == 0 && obstacles[i].y == 0)
        {
            // Nothing here
            continue;
        }

        float distance = Point::getDistance(obstacles[i]);

        if (distance <= radius_ + min_distance_hold_)
        {
            float direction = Point::GetDirectionDegrees(obstacles[i]) + 180;
            if (direction >= 360)
            {
                direction -= 360;
            }

            float magnitude = (radius_ + min_distance_hold_) - distance;

            Point point = Point::getPointFromVectorDegrees(direction, magnitude);

            x_min = std::min(x_min, point.x);

            x_max = std::max(x_max, point.x);

            y_min = std::min(y_min, point.y);

            y_max = std::max(y_max, point.y);
        }
    }

    if (x_min == 1000 && y_min == 1000)
    {
        // There are no obstacles close
        // Try to stop completely

        float direction = current_direction + 180; // Opposite direction

        if (direction >= 360)
        {
            direction -= 360;
        }

        control->go_magnitude = current_speed * 2;
        control->go_direction = direction;
    }

    Point go_to_point;
    go_to_point.x = (x_min + x_max) / 2.0;
    go_to_point.y = (y_min + y_max) / 2.0;

    Point current;

    current.x = current_speed * std::cos(current_direction * M_PI / 180.0);
    current.y = current_speed * std::sin(current_direction * M_PI / 180.0);

    go_to_point.x += (go_to_point.x - current.x);
    go_to_point.y += (go_to_point.y - current.y);

    control->go_magnitude = Point::getDistance(go_to_point) * 2;            // TODO: Make it depend on the distance to the obstacles?
    control->go_direction = Point::GetDirectionDegrees(go_to_point);
}

float Basic::getClosestObstacleDistanceBetweenDirections(const std::vector<Point> & obstacles, float current_direction, float wanted_direction)
{
    float left_direction, right_direction;

    float direction_diff = current_direction - wanted_direction;
    if (direction_diff > 180)
    {
        direction_diff -= 360;
    }
    else if (direction_diff < -180)
    {
        direction_diff += 360;
    }

    if (direction_diff < 0)
    {
        left_direction = wanted_direction;
        right_direction = current_direction;
    }
    else
    {
        left_direction = current_direction;
        right_direction = wanted_direction;
    }

    // We want left bound of left_direction and right bound of right_direction


}

// Take the speed control from VFH. V' = v_max * max(1, d_closest / d_empirically_determined) and then V = V' * (1 - (steering_angle / max_steering_angle) + v_min

void Basic::moving(exjobb_msgs::Control * control, const std::vector<Point> & obstacles, float wanted_direction, float current_direction, float current_speed)
{
    float velocity_max_ = 2.0;

    float closest_obstacle_distance = 1000;
    for (size_t i = 0; i < obstacles.size(); i++)
    {
        if (obstacles[i].x == 0 && obstacles[i].y == 0)
        {
            continue;
        }

        float distance = Point::getDistance(obstacles[i]);

        if (distance < closest_obstacle_distance)
        {
            closest_obstacle_distance = distance;
        }
    }

    float direction_diff = std::fabs(current_direction - control->go_direction);

    if (direction_diff > 180)
    {
        direction_diff = 360 - direction_diff;
    }

    control->go_magnitude = velocity_max_ * std::min(closest_obstacle_distance / (radius_ + security_distance_), control->go_magnitude);
    //control->go_magnitude = velocity_max_ * (closest_obstacle_distance / (radius_ + security_distance_));

    Point current;

    current.x = current_speed * std::cos(current_direction * M_PI / 180.0);
    current.y = current_speed * std::sin(current_direction * M_PI / 180.0);

    Point wanted;

    wanted.x = control->go_magnitude * std::cos(control->go_direction * M_PI / 180.0);
    wanted.y = control->go_magnitude * std::sin(control->go_direction * M_PI / 180.0);

    //ROS_ERROR_STREAM("(" << current.x << ", " << current.y << "), (" << wanted.x << ", " << wanted.y << ")");

    wanted.x += (wanted.x - current.x);
    wanted.y += (wanted.y - current.y);

    //ROS_ERROR_STREAM("(" << wanted.x << ", " << wanted.y << ")");

    //ROS_ERROR_STREAM(control->go_direction << ", " << current_direction << " -> " << Point::GetDirectionDegrees(wanted));

    //control->go_magnitude = Point::getDistance(wanted);
    control->go_direction = Point::GetDirectionDegrees(wanted);

    /*
    float direction_epsilon = 40; // Five degrees
    float speed_epsilon = 0.2; // 0.2 meters per second?

    float direction_diff = std::fabs(control->go_direction - current_direction);
    if (direction_diff > 180)
    {
        direction_diff = 360 - direction_diff;
    }


    if (direction_diff > direction_epsilon &&
            current_speed > speed_epsilon)
    {
        // We are going the wrong way! Stop!
        control->go_direction = current_direction + 180;
        if (control->go_direction > 360)
        {
            control->go_direction -= 360;
        }

        control->go_magnitude = current_speed;

        ROS_ERROR_STREAM("STOPPING!!!");
    }


    float shortest_distance = 10000;
    for (size_t i = 0; i < obstacles.size(); i++)
    {
        if (obstacles[i].x == 0 && obstacles[i].y == 0)
        {
            continue;
        }

        float direction = Point::GetDirectionDegrees(obstacles[i]);



        float distance = Point::getDistance(obstacles[i]) * 0.1;

        if (distance < shortest_distance)
        {
            shortest_distance = distance;
        }
    }

    control->go_magnitude = std::min(control->go_magnitude, shortest_distance);

    */
    // TODO: Take into account the current movement

    // Create a rectangle of the path
}
