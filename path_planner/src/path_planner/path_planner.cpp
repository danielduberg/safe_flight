#include <path_planner/path_planner.h>

#include <path_planner/d_star_lite.h>
#include <path_planner/a_star.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace path_planner
{

    PathPlanner::PathPlanner(ros::NodeHandle & nh, ros::NodeHandle & nh_priv, const std::string & action_server_name)
        : nh_(nh)
        , nh_priv_(nh_priv)
        , search_algorithm_(new AStar())
        , as_(nh_, action_server_name, boost::bind(&PathPlanner::executeCallback, this, _1), false)
        , tf_listener(tf_buffer)
    {
        initParams(nh_priv_);

        // Read in temp parameters
        std::string map_sub_topic = nh_.param<std::string>("map_in_topic", "/explore_server/explore_costmap/costmap");
        std::string odom_sub_topic = nh_.param<std::string>("odom_in_topic", "/mavros/local_position/odom");
        std::string map_pub_topic = nh_.param<std::string>("map_out_topic", "expanded_map");
        std::string setpoint_pub_topic = nh_.param<std::string>("setpoint_out_topic", "/nav_2d");

        map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(map_sub_topic, 1, &PathPlanner::mapCallback, this);
        map_update_sub_ = nh_.subscribe<map_msgs::OccupancyGridUpdate>(map_sub_topic + "_updates", 10, &PathPlanner::mapUpdateCallback, this);
        //velocity_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(velocity_sub_topic, 1, &PathPlanner::velocityCallback, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odom_sub_topic, 1, &PathPlanner::odomCallback, this);

        map_pub_ = nh_priv_.advertise<nav_msgs::OccupancyGrid>(map_pub_topic, 1, true);
        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(setpoint_pub_topic, 1);

        as_.start();
    }

    PathPlanner::~PathPlanner()
    {
        delete search_algorithm_;
    }

    bool PathPlanner::waitForMap()
    {
        // Wait for map to be updated
        ros::Rate rate(frequency_);
        while (map_.data.size() == 0)
        {
            ROS_WARN_THROTTLE_NAMED(1.0d, "safe_flight/path_planner", "Map is not available yet, waiting...");

            ros::spinOnce();

            rate.sleep();

            if (as_.isPreemptRequested() || !ros::ok())
            {
                return false;
            }
        }

        return true;
    }

    void PathPlanner::executeCallback(const path_planner::PathPlannerGoal::ConstPtr & goal)
    {
        if (!waitForMap())
        {
            ROS_FATAL_STREAM_NAMED("safe_flight/path_planner", "Preempted");

            result_.success = false;
            as_.setPreempted(result_, "Preempted while waiting for map.");
            return;
        }

        geometry_msgs::PoseStamped current_pose;

        // Get current robot pose
        try
        {
            current_pose = getPose(robot_base_frame_, map_.header.frame_id, 3);
        }
        catch (tf2::TransformException & ex)
        {
            ROS_WARN_NAMED("safe_flight/path_planner", "%s. Canceling goal.", ex.what());

            result_.success = false;
            as_.setAborted(result_, "Could not find current robot pose.");
            return;
        }

        ros::Time map_last_updated = map_.header.stamp;

        // Start executing the action
        ROS_FATAL_STREAM_NAMED("safe_flight/path_planner", "Start computing new path!");
        feedback_.path = search_algorithm_->getPath(current_pose, goal->goal, map_);
        //feedback_.path = search_algorithm_->smoothenPath(search_algorithm_->computePath(current_pose, goal->goal, map_), map_);
        for (size_t i = 0; i < feedback_.path.poses.size(); ++i)
        {
            feedback_.path.poses[i].pose.position.z = altitude_;
        }
        feedback_.path.header.stamp = ros::Time::now();
        feedback_.path.header.frame_id = map_.header.frame_id;
        ROS_FATAL_STREAM_NAMED("safe_flight/path_planner", "Done computing new path!");

        // publish the feedback
        as_.publishFeedback(feedback_);

        ros::Rate rate(frequency_);
        while (feedback_.path.poses.size() > 0)
        {
            // Check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_FATAL_STREAM_NAMED("safe_flight/path_planner", "Preempted");

                as_.setPreempted();
                return;
            }

            // Get current robot pose
            try
            {
                current_pose = getPose(robot_base_frame_, map_.header.frame_id, 1);
            }
            catch (tf2::TransformException & ex)
            {
                ROS_WARN_NAMED("safe_flight/path_planner", "%s. Canceling goal.", ex.what());

                result_.success = false;
                as_.setAborted(result_, "Could not find current robot pose.");
                return;
            }

            // Update path if map has been updated
            nav_msgs::Path temp_path = feedback_.path;
            temp_path.poses.insert(temp_path.poses.begin(), current_pose);
            if (map_last_updated != map_.header.stamp && search_algorithm_->isPathBlocked(temp_path, map_))
            {
                map_last_updated = map_.header.stamp;

                ROS_FATAL_STREAM_NAMED("safe_flight/path_planner", "Start updating path!");
                feedback_.path = search_algorithm_->getUpdatedPath(current_pose, feedback_.path, map_);
                //feedback_.path = search_algorithm_->smoothenPath(search_algorithm_->updatePath(current_pose, feedback_.path, map_), map_);
                for (size_t i = 0; i < feedback_.path.poses.size(); ++i)
                {
                    feedback_.path.poses[i].pose.position.z = altitude_;
                }
                feedback_.path.header.stamp = ros::Time::now();
                feedback_.path.header.frame_id = map_.header.frame_id;
                ROS_FATAL_STREAM_NAMED("safe_flight/path_planner", "Done updating path!");

                if (feedback_.path.poses.size() == 0)
                {
                    result_.success = false;

                    as_.setSucceeded(result_);

                    feedback_.path.poses.clear();
                    as_.publishFeedback(feedback_);

                    return;
                }
            }

            // Move along path
            // Note that we do not need to publish this every time (once should be enough)
            // since controller_setpoint republishes the last setpoint it got
            geometry_msgs::PoseStamped checkpoint = feedback_.path.poses[0];
            if (checkpoint.pose.orientation.w == 0)
            {
                checkpoint.pose.orientation.w = 1;
            }
            setpoint_pub_.publish(checkpoint);

            // Check if we have reached current checkpoint
            if (reachedCheckpoint(current_pose, checkpoint) && isStill())
            {
                feedback_.path.poses.erase(feedback_.path.poses.begin());
            }

            // Publish the feedback
            as_.publishFeedback(feedback_);

            rate.sleep();
        }

        result_.success = true;

        as_.setSucceeded(result_);
    }

    double PathPlanner::getYaw(const geometry_msgs::Quaternion & orientation)
    {
        double roll, pitch, yaw;

        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        tf2::Matrix3x3 m(q);
        m.getEulerYPR(yaw, pitch, roll);

        return yaw;
    }

    bool PathPlanner::isMapDiff(const nav_msgs::OccupancyGrid & map_1, const nav_msgs::OccupancyGrid & map_2)
    {
        if (map_1.data.size() != map_2.data.size())
        {
            // One map is bigger than the other therefore they are different
            return true;
        }

        for (size_t i = 0; i < map_1.data.size(); ++i)
        {
            if (map_1.data[i] != map_2.data[i])
            {
                return true;
            }
        }

        return false;
    }

    bool PathPlanner::reachedCheckpoint(const geometry_msgs::PoseStamped & current_pose, const geometry_msgs::PoseStamped & checkpoint)
    {
        ROS_FATAL("Current: [%f, %f, %f], checkpoint: [%f, %f, %f]", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, checkpoint.pose.position.x, checkpoint.pose.position.y, checkpoint.pose.position.z);


        double current_yaw = getYaw(current_pose.pose.orientation);
        double checkpoint_yaw = getYaw(checkpoint.pose.orientation);


        if (std::fabs(current_pose.pose.position.z - checkpoint.pose.position.z) > altitude_error_)
        {
            // Need better altitude!
            //ROS_FATAL_STREAM("Need better altitude!");
            //return false;
        }

        double yaw_diff = std::fabs(current_yaw - checkpoint_yaw);

        if (yaw_diff > M_PI)
        {
            yaw_diff = std::fabs((2 * M_PI) - yaw_diff);
        }

        if (yaw_diff > yaw_error_)
        {
            // Need better orientation!
            //ROS_FATAL_STREAM("Need better orientation!");
            return false;
        }

        if (std::sqrt(std::pow(current_pose.pose.position.x - checkpoint.pose.position.x, 2) +
                      std::pow(current_pose.pose.position.y - checkpoint.pose.position.y, 2)) > position_error_)
        {
            // Need better position!
            //ROS_FATAL_STREAM("Need better position!");
            return false;
        }

        return true;
    }

    geometry_msgs::PoseStamped PathPlanner::getPose(const std::string & target_frame, const std::string & source_frame, double timeout)
    {
        geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform(source_frame, target_frame, ros::Time::now(), ros::Duration(timeout));

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = transform_stamped.header;

        pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
        pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
        pose_stamped.pose.position.z = transform_stamped.transform.translation.z;

        pose_stamped.pose.orientation = transform_stamped.transform.rotation;

        return pose_stamped;
    }

    void PathPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
    {
        current_xy_velocity_ = std::sqrt(std::pow(msg->twist.twist.linear.y, 2.0d) + std::pow(msg->twist.twist.linear.x, 2.0d));
        current_z_velocity_ = msg->twist.twist.linear.z;

        current_yaw_rate_ = msg->twist.twist.angular.z;

        /*
        current_velocity_.twist.linear.x = msg->twist.twist.linear.y; // Should this be y?
        current_velocity_.twist.linear.y = msg->twist.twist.linear.x; // Should this be x?
        current_velocity_.twist.linear.z = msg->twist.twist.linear.z;

        current_velocity_.twist.angular.z = msg->twist.twist.angular.z;
        */
    }

    int PathPlanner::getIndex(int x, int y)
    {
        return (y * map_.info.width) + x;
    }

    // Source: https://github.com/DLu/navigation/commit/92b5a4f2a2c6788bd1d7616c21694b1e49c548c0
    void PathPlanner::mapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr & msg)
    {
        // Can check stamp to see if the map has been update
        map_.header.stamp = msg->header.stamp;

        size_t index = 0;
        for (size_t y = msg->y; y < msg->y + msg->height; ++y)
        {
            for (size_t x = msg->x; x < msg->x + msg->width; ++x)
            {
                if (msg->data[index++] < 1)
                {
                    map_.data[getIndex(x, y)] = 0;
                }
                else
                {
                    map_.data[getIndex(x, y)] = 100;
                }
            }
        }

        map_pub_.publish(map_);
    }

    void PathPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
    {
        nav_msgs::OccupancyGrid new_map = *msg;

        for (size_t i = 0; i < new_map.data.size(); ++i)
        {
            if (new_map.data[i] < 1)
            {
                new_map.data[i] = 0;
            }
            else
            {
                new_map.data[i] = 100;
            }
        }

        // Check if map has been updated!
        if (!isMapDiff(map_, new_map))
        {
            // Map has not been updated
            return;
        }

        map_ = new_map;

        map_pub_.publish(map_);
    }

    bool PathPlanner::isStill()
    {
        if (std::fabs(current_xy_velocity_) > max_xy_setpoint_velocity_)
        {
            return false;
        }
        if (std::fabs(current_z_velocity_) > max_z_setpoint_velocity_)
        {
            return false;
        }
        if (std::fabs(current_yaw_rate_) > max_yaw_setpoint_rate_)
        {
            return false;
        }

        /*
        if (current_velocity_.twist.linear.z > altitude_velocity_threshold_)
        {
            return false;
        }

        if (current_velocity_.twist.angular.z > yaw_velocity_threshold_)
        {
            return false;
        }

        if (std::sqrt(std::pow(current_velocity_.twist.linear.x, 2) + std::pow(current_velocity_.twist.linear.y, 2)) >
                position_velocity_threshold_)
        {
            return false;
        }
        */

        return true;
    }

    void PathPlanner::initParams(const ros::NodeHandle & nh)
    {
        nh.param<double>("altitude_error", altitude_error_, 0.1d);
        nh.param<double>("yaw_error", yaw_error_, 0.1d);
        nh.param<double>("position_error", position_error_, 0.1d);

        nh.param<double>("frequency", frequency_, 10.0d);

        nh.param<double>("radius", radius_, 0.40d);
        nh.param<double>("altitude", altitude_, 0.40d);

        /*
        nh.param<double>("altitude_velocity_threshold", altitude_velocity_threshold_, 0.05d);
        nh.param<double>("yaw_velocity_threshold", yaw_velocity_threshold_, 0.05d);
        nh.param<double>("position_velocity_threshold", position_velocity_threshold_, 0.05d);
        */
        nh.param<double>("max_xy_setpoint_velocity", max_xy_setpoint_velocity_, 0.05d);
        nh.param<double>("max_z_setpoint_velocity", max_z_setpoint_velocity_, 0.05d);
        nh.param<double>("max_yaw_setpoint_rate", max_yaw_setpoint_rate_, 0.05d);

        nh.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
    }

}
