#include <pluginlib/class_list_macros.h>

#include <collision_avoidance/collision_avoidance_nodelet.h>

#include <limits>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Test
namespace collision_avoidance
{

    void CANodelet::onInit()
    {
        ros::NodeHandle & nh = getNodeHandle();
        ros::NodeHandle & nh_priv = getPrivateNodeHandle();

        init_param(nh_priv);

        sensor_readings_sub_ = nh.subscribe("/sensor_readings", 1, &CANodelet::sensorReadingsCallback, this);
        collision_avoidance_setpoint_sub_ = nh.subscribe("/controller/setpoint", 1, &CANodelet::collisionAvoidanceSetpointCallback, this);
        collision_avoidance_joy_sub_ = nh.subscribe("/controller/joy", 1, &CANodelet::collisionAvoidanceJoyCallback, this);
        //current_pose_sub_ = nh.subscribe("/mavros/local_position/pose", 1, &CANodelet::currentPoseCallback, this);
        //current_velocity_sub_ = nh.subscribe("/mavros/local_position/velocity", 1, &CANodelet::currentVelocityCallback, this);
        odometry_sub_ = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &CANodelet::odometryCallback, this);

        collision_free_control_pub_ = nh.advertise<controller_msgs::Controller>("collision_free_control", 1);
        rumble_pub_ = nh.advertise<joy_rumble::Rumble_msg>("rumble_message", 1);

        orm_ = new ORM(radius_, security_distance_, epsilon_, min_distance_hold_, min_change_in_direction_, max_change_in_direction_, min_opposite_direction_, max_opposite_direction_);
    }

    void CANodelet::init_param(ros::NodeHandle & nh)
    {
        nh.param<double>("radius", radius_, 0);
        nh.param<double>("security_distance", security_distance_, 0);
        nh.param<double>("epsilon", epsilon_, 0.1);

        nh.param<double>("min_change_in_direction", min_change_in_direction_, -180);
        nh.param<double>("max_change_in_direction", max_change_in_direction_, 180);

        nh.param<double>("min_opposite_direction", min_opposite_direction_, -180);
        nh.param<double>("max_opposite_direction", max_opposite_direction_, 180);

        nh.param<double>("ab", ab_, 3.0);
        nh.param<double>("T", T_, 0.3);

        nh.param<double>("min_distance_hold", min_distance_hold_, 0.3);
    }

    void CANodelet::sensorReadingsCallback(const safe_flight_msgs::SensorReadings::ConstPtr & msg)
    {
        std::vector<Point> new_obstacles;
        new_obstacles.reserve(msg->x.size());

        for (size_t i = 0; i < msg->x.size(); ++i)
        {
            new_obstacles.push_back(Point(msg->x[i], msg->y[i]));
        }

        obstacles_ = new_obstacles;
    }

    bool CANodelet::hapticFeedback(double want_direction, double going_direction)
    {
        // TODO
    }

    void CANodelet::collisionAvoidance(const controller_msgs::Controller::ConstPtr & msg, const double magnitude)
    {
        controller_msgs::Controller collision_free_control = *msg;

        std::vector<Point> obstacles; // = obstacles_;

        getEgeDynamicSpace(&obstacles);

        // First pass
        orm_->avoidCollision(&collision_free_control, magnitude, obstacles);

        // Second pass
        controller_msgs::Controller second_pass_collision_free_control = collision_free_control;

        orm_->avoidCollision(&second_pass_collision_free_control, magnitude, obstacles);

        // If there is a big difference between first and second pass then it is not safe to move!
        Point first_pass(collision_free_control.twist_stamped.twist.linear.x, collision_free_control.twist_stamped.twist.linear.y);
        Point second_pass(second_pass_collision_free_control.twist_stamped.twist.linear.x, second_pass_collision_free_control.twist_stamped.twist.linear.y);
        double dir_diff = std::fmod(std::fabs(Point::getDirectionDegrees(first_pass) - Point::getDirectionDegrees(second_pass)), 180.0);
        if (dir_diff > 140)
        {
          collision_free_control.twist_stamped.twist.linear.x = 0.0;
          collision_free_control.twist_stamped.twist.linear.y = 0.0;
          orm_->avoidCollision(&collision_free_control, magnitude, obstacles);
        }

        // Adjust the velocity
        adjustVelocity(&collision_free_control, magnitude);

        // Publish the collision free control
        collision_free_control_pub_.publish(collision_free_control);
    }

    void CANodelet::collisionAvoidanceSetpointCallback(const controller_msgs::Controller::ConstPtr & msg)
    {
        collisionAvoidance(msg, 1.0);
    }

    void CANodelet::collisionAvoidanceJoyCallback(const controller_msgs::Controller::Ptr & msg)
    {
        Point goal(msg->twist_stamped.twist.linear.x, msg->twist_stamped.twist.linear.y);

        // Multiple by a number to make a setpoint... x and y are between [0,1]
        msg->twist_stamped.twist.linear.x *= 10;
        msg->twist_stamped.twist.linear.y *= 10;

        collisionAvoidance(msg, Point::getDistance(goal));
    }

    void CANodelet::getEgeDynamicSpace(std::vector<Point> * obstacles)
    {
        for (size_t i = 0; i < obstacles_.size(); ++i)
        {
            if (obstacles_[i].x_ == 0 && obstacles_[i].y_ == 0)
            {
                // No reading here
                obstacles->push_back(obstacles_[i]);
                continue;
            }

            /*
            // Decrease the distance with 5 cm?!
            double dobs = Point::getDistance(obstacles_[i]);

            dobs -= radius_;

            double deff = ab_ * (T_ * T_) * (std::sqrt(1.0d + ((2.0d * dobs) / (ab_ * (T_ * T_)))) - 1.0d);

            deff += radius_;

            deff = std::min(Point::getDistance(obstacles_[i]), deff);
            deff = std::max(deff, radius_ + 0.01d);

            Point p;
            p.x_ = deff * std::cos(Point::getDirection(obstacles_[i]));
            p.y_ = deff * std::sin(Point::getDirection(obstacles_[i]));

            obstacles->push_back(p);
            */

            Point p;
            p.x_ = obstacles_[i].x_ - current_x_vel_;
            p.y_ = obstacles_[i].y_ - current_y_vel_;

            obstacles->push_back(p);
        }
    }

    void CANodelet::adjustVelocity(controller_msgs::Controller * control, const double magnitude)
    {
        double closest_obstacle_distance = 1000;
        for (size_t i = 0; i < obstacles_.size(); ++i)
        {
            double distance = Point::getDistance(obstacles_[i]);

            if (distance != 0 && distance < closest_obstacle_distance)
            {
                closest_obstacle_distance = distance;
            }
        }

        Point goal(control->twist_stamped.twist.linear.x, control->twist_stamped.twist.linear.y);
        double direction = Point::getDirectionDegrees(goal);

        double updated_magnitude = std::min(closest_obstacle_distance / (radius_ + security_distance_), magnitude);

        Point updated_goal = Point::getPointFromVectorDegrees(direction, updated_magnitude);

        control->twist_stamped.twist.linear.x = updated_goal.x_;
        control->twist_stamped.twist.linear.y = updated_goal.y_;
    }

    /*
    void CANodelet::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
    {
        current_pose_ = *msg;
    }

    double CANodelet::getCurrentYaw()
    {
        double roll, pitch, yaw;

        geometry_msgs::Quaternion cur_or = current_pose_.pose.orientation;

        tf2::Quaternion q(cur_or.x, cur_or.y, cur_or.z, cur_or.w);
        tf2::Matrix3x3 m(q);
        m.getEulerYPR(yaw, pitch, roll);

        return yaw;
    }

    void CANodelet::currentVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr & msg)
    {
        double current_yaw = getCurrentYaw();

        // TODO: Is this correct?!
        // Source: https://www.siggraph.org/education/materials/HyperGraph/modeling/mod_tran/2drota.htm
        current_x_vel_ = (msg->twist.linear.x * std::cos(current_yaw)) + (msg->twist.linear.y * std::sin(current_yaw));
        current_y_vel_ = (msg->twist.linear.y * std::cos(current_yaw)) - (msg->twist.linear.x * std::sin(current_yaw));

        ROS_FATAL_STREAM_THROTTLE(1, current_yaw << ": [" << msg->twist.linear.x << ", " << msg->twist.linear.y << "] -> [" << current_x_vel_ << ", " << current_y_vel_ << "]");
    }
    */

    void CANodelet::odometryCallback(const nav_msgs::Odometry::ConstPtr & msg)
    {
        current_x_vel_ = msg->twist.twist.linear.y; // This should be y?
        current_y_vel_ = msg->twist.twist.linear.x; // This should be x?
    }


    PLUGINLIB_DECLARE_CLASS(collision_avoidance, CA, collision_avoidance::CANodelet, nodelet::Nodelet)
}
