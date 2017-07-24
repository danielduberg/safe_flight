#include <pluginlib/class_list_macros.h>

#include <collision_avoidance/collision_avoidance_nodelet.h>

namespace collision_avoidance
{

    void CANodelet::onInit()
    {
        ros::NodeHandle & nh = getNodeHandle();
        ros::NodeHandle & nh_priv = getPrivateNodeHandle();

        init_param(nh_priv);

        sensor_readings_sub_ = nh.subscribe("/sensor_readings", 1, &CANodelet::sensorReadingsCallback, this);
        collision_avoidance_sub_ = nh.subscribe("/controller", 1, &CANodelet::collisionAvoidanceCallback, this);
        current_pose_sub_ = nh.subscribe("/mavros/local_position/pose", 1, &CANodelet::currentPoseCallback, this);
        current_velocity_sub_ = nh.subscribe("/mavros/local_position/velocity", 1, &CANodelet::currentVelocityCallback, this);

        collision_free_control_pub_ = nh.advertise<controller_msgs::Controller>("collision_free_control", 1);
        rumble_pub_ = nh.advertise<joy_rumble::Rumble_msg>("rumble_message", 1);

        orm_ = new ORM(radius_, security_distance_, epsilon_, min_change_in_direction_, max_change_in_direction_, min_opposite_direction_, max_opposite_direction_);
    }

    void CANodelet::init_param(ros::NodeHandle & nh)
    {
        
    }

    void CANodelet::sensorReadingsCallback(const safe_flight_msgs::SensorReadings::ConstPtr & msg)
    {
        std::vector<Point> newObstacles(msg->x.size());

        for (size_t i = 0; i < msg->x.size(); ++i)
        {
            newObstacles.push_back(Point(msg->x[i], msg->y[i]));
        }

        obstacles_ = newObstacles;
    }

    bool CANodelet::hapticFeedback(double want_direction, double going_direction)
    {
        // TODO
    }

    void CANodelet::collisionAvoidanceCallback(const controller_msgs::Controller::ConstPtr & msg)
    {
        controller_msgs::Controller collision_free_control = *msg;

        std::vector<Point> obstacles = getEgeDynamicSpace();

        orm_->avoidCollision(&collision_free_control, obstacles);

        collision_free_control_pub_.publish(collision_free_control);
    }

    std::vector<Point> CANodelet::getEgeDynamicSpace()
    {
        std::vector<Point> obstacles(obstacles_.size());

        for (size_t i = 0; i < obstacles_.size(); ++i)
        {
            if (obstacles_[i].x_ == -1 && obstacles_[i].y_ == -1)
            {
                // No reading here
                obstacles.push_back(obstacles_[i]);
                continue;
            }

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

            obstacles.push_back(p);
        }

        return obstacles;
    }

    void CANodelet::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
    {
        current_pose_ = *msg;
    }

    void CANodelet::currentVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr & msg)
    {
        // TODO: Is this needed?!
    }


    PLUGINLIB_DECLARE_CLASS(collision_avoidance, CA, collision_avoidance::CANodelet, nodelet::Nodelet);
}
