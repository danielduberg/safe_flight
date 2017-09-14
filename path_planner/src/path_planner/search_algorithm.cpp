#include <path_planner/search_algorithm.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <queue>

namespace path_planner
{
    SearchAlgorithm::SearchAlgorithm()
        : tf2_listener_(tf2_buffer_)
    {

    }

    nav_msgs::Path SearchAlgorithm::getPath(geometry_msgs::PoseStamped start,
                           geometry_msgs::PoseStamped goal,
                           const nav_msgs::OccupancyGrid & map)
    {
        geometry_msgs::PoseStamped free_start = closestFreeArea(start, map);

        nav_msgs::Path path = computePath(free_start, goal, map);

        return smoothenPath(path, map);
    }

    nav_msgs::Path SearchAlgorithm::getUpdatedPath(geometry_msgs::PoseStamped start,
                                  nav_msgs::Path old_path,
                                  const nav_msgs::OccupancyGrid & map)
    {
        geometry_msgs::PoseStamped free_start = closestFreeArea(start, map);

        nav_msgs::Path path = updatePath(free_start, old_path, map);

        return smoothenPath(path, map);
    }

    geometry_msgs::PoseStamped SearchAlgorithm::transformPose(const geometry_msgs::PoseStamped & pose,
                                  const std::string & to_frame_id)
    {
        geometry_msgs::TransformStamped transform;

        geometry_msgs::PoseStamped new_pose = pose;

        try
        {
            transform = tf2_buffer_.lookupTransform(pose.header.frame_id,
                                                    to_frame_id,
                                                    ros::Time(0),
                                                    ros::Duration(1.0));
        }
        catch (tf2::TransformException & ex)
        {
            ROS_WARN_STREAM("Could NOT transform " << pose.header.frame_id
                            << " to " << to_frame_id << ": " << ex.what());
            return new_pose;
        }

        ROS_FATAL("[%f, %f, %f] -> [%f, %f, %f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z);

        tf2::doTransform(pose, new_pose, transform);

        new_pose.header.frame_id = to_frame_id;

        return new_pose;
    }


    int8_t SearchAlgorithm::getData(const int x, const int y, const nav_msgs::OccupancyGrid & map)
    {
        return map.data[getIndex(x, y, map)];
    }

    int8_t SearchAlgorithm::getData(const Position & position, const nav_msgs::OccupancyGrid & map)
    {
        return map.data[getIndex(position.x_, position.y_, map)];
    }

    int8_t SearchAlgorithm::getData(const geometry_msgs::PoseStamped & pose, const nav_msgs::OccupancyGrid & map)
    {
        return map.data[getIndex(pose, map)];
    }


    int SearchAlgorithm::getIndex(const geometry_msgs::PoseStamped & pose, const nav_msgs::OccupancyGrid & map)
    {
        return getIndex(getPosition(pose, map), map);
    }

    int SearchAlgorithm::getIndex(const Position & position, const nav_msgs::OccupancyGrid & map)
    {
        return getIndex(position.x_, position.y_, map);
    }

    int SearchAlgorithm::getIndex(const int x, const int y, const nav_msgs::OccupancyGrid & map)
    {
        if (x < 0 || x >= map.info.width || y < 0 || y >= map.info.height)
        {
            // TODO: ERROR!!!
        }

        // TODO: Check if correct
        return (y * map.info.width) + x;
    }





    SearchAlgorithm::Position SearchAlgorithm::getPosition(const geometry_msgs::PoseStamped & pose, const nav_msgs::OccupancyGrid & map)
    {
        // TODO: Check if it was not possible to transform pose
        geometry_msgs::PoseStamped transformed_pose = transformPose(pose, map.header.frame_id);

        // TODO: Check if correct
        //int x = (map.info.width / 2) + (transformed_pose.pose.position.x / map.info.resolution);
        //int y = (map.info.height / 2) + (transformed_pose.pose.position.y / map.info.resolution);
        // Source: https://answers.ros.org/question/10268/where-am-i-in-the-map/
        int x = (unsigned int)((transformed_pose.pose.position.x - map.info.origin.position.x) / map.info.resolution);
        int y = (unsigned int)((transformed_pose.pose.position.y - map.info.origin.position.y) / map.info.resolution);

        //ROS_FATAL_STREAM("Mine: " << x << " of " << map.info.width << ", " << y << " of " << map.info.height);
        //ROS_FATAL_STREAM("Pose: " << pose.pose.position.x << ", " << pose.pose.position.y);
        //ROS_FATAL_STREAM("Transformed pose: " << transformed_pose.pose.position.x << ", " << transformed_pose.pose.position.y);
        //ROS_FATAL_STREAM("Map: " << map.info.origin.position.x << ", " << map.info.origin.position.y);

        return Position(x, y);
    }

    SearchAlgorithm::Position SearchAlgorithm::getPosition(const int index, const nav_msgs::OccupancyGrid & map)
    {
        if (index < 0 || index >= map.data.size())
        {
            // TODO: ERROR!!!
        }

        return Position(index % map.info.height, index / map.info.height);
    }


    geometry_msgs::PoseStamped SearchAlgorithm::getPose(const int index, const nav_msgs::OccupancyGrid & map)
    {
        getPose(getPosition(index, map), map);
    }

    geometry_msgs::PoseStamped SearchAlgorithm::getPose(const Position & position, const nav_msgs::OccupancyGrid & map)
    {
        geometry_msgs::PoseStamped pose;

        pose.header.frame_id = map.header.frame_id;
        pose.header.stamp = ros::Time::now();

        // Reverse of getPosition
        pose.pose.position.x = (position.x_ * map.info.resolution) + map.info.origin.position.x;
        pose.pose.position.y = (position.y_ * map.info.resolution) + map.info.origin.position.y;

        return pose;
    }


    std::vector<int> SearchAlgorithm::rayTrace(const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & end, const nav_msgs::OccupancyGrid & map)
    {
        return rayTrace(getPosition(start, map), getPosition(end, map), map);
    }

    std::vector<int> SearchAlgorithm::rayTrace(const Position & start, const Position & end, const nav_msgs::OccupancyGrid & map)
    {
        // TODO: Check if start and end are valid?

        std::vector<int> visited_cells;

        // Source: http://playtechs.blogspot.se/2007/03/raytracing-on-grid.html
        // Tool: http://bert.stuy.edu/pbrooks/graphics/demos/BresenhamDemo.htm
        int dx = std::abs(end.x_ - start.x_);
        int dy = std::abs(end.y_ - start.y_);
        int x = start.x_;
        int y = start.y_;
        int n = dx + dy;
        int x_inc = (end.x_ > start.x_) ? 1 : -1;
        int y_inc = (end.y_ > start.y_) ? 1 : -1;
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        for (; n > 0; --n)
        {
            visited_cells.push_back(getIndex(x, y, map));

            if (error > 0)
            {
                x += x_inc;
                error -= dy;
            }
            else
            {
                if (error == 0)
                {
                    visited_cells.push_back(getIndex(x + x_inc, y, map));
                }

                y += y_inc;
                error += dx;
            }
        }

        visited_cells.push_back(getIndex(end.x_, end.y_, map));

        return visited_cells;
    }

    std::vector<int> SearchAlgorithm::rayTrace(const int start, const int end, const nav_msgs::OccupancyGrid & map)
    {
        return rayTrace(getPosition(start, map), getPosition(end, map), map);
    }


    /*
    nav_msgs::OccupancyGrid SearchAlgorithm::expandMap(int new_resolution, const nav_msgs::OccupancyGrid & map)
    {
        nav_msgs::OccupancyGrid expanded_map = map;

        return expanded_map;
    }
    */

    nav_msgs::Path SearchAlgorithm::smoothenPath(const nav_msgs::Path & path, const nav_msgs::OccupancyGrid & map)
    {
        nav_msgs::Path smoothed_path;
        smoothed_path.header = path.header;

        if (path.poses.size() <= 2)
        {
            smoothed_path.poses = path.poses;
            return smoothed_path;
        }

        smoothed_path.poses.push_back(path.poses[0]);
        geometry_msgs::PoseStamped last_reachable_pose = path.poses[1];
        for (size_t i = 2; i < path.poses.size(); ++i)
        {
            if (canReach(smoothed_path.poses[smoothed_path.poses.size() - 1], path.poses[i], map))
            {
                last_reachable_pose = path.poses[i];
            }
            else
            {
                smoothed_path.poses.push_back(last_reachable_pose);
            }
        }

        // Add the last pose if it has not been added
        if (!isSame(smoothed_path.poses[smoothed_path.poses.size() - 1], path.poses[path.poses.size() - 1]))
        {
            smoothed_path.poses.push_back(path.poses[path.poses.size() - 1]);
        }


        orientTowardsNext(&smoothed_path);

        for (size_t i = 0; i < smoothed_path.poses.size(); ++i)
        {
            smoothed_path.poses[i].pose.position.z = 0.35d;
        }

        return smoothed_path;
    }

    void SearchAlgorithm::orientTowardsNext(nav_msgs::Path * path)
    {
        for (size_t i = 1; i < path->poses.size(); i += 2)
        {
            // TODO: Do something fancy with quaternions such that it works in 3D
            double yaw = std::atan2(path->poses[i].pose.position.y - path->poses[i-1].pose.position.y,
                    path->poses[i].pose.position.x - path->poses[i-1].pose.position.x);

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);

            path->poses[i-1].pose.orientation.x = q.getX();
            path->poses[i-1].pose.orientation.y = q.getY();
            path->poses[i-1].pose.orientation.z = q.getZ();
            path->poses[i-1].pose.orientation.w = q.getW();

            // Add a new pose such that it does not start rotating until it is at the new position
            path->poses.insert(path->poses.begin() + i, path->poses[i]);
            path->poses[i].pose.orientation.x = q.getX();
            path->poses[i].pose.orientation.y = q.getY();
            path->poses[i].pose.orientation.z = q.getZ();
            path->poses[i].pose.orientation.w = q.getW();
        }
    }

    bool SearchAlgorithm::isPathBlocked(const nav_msgs::Path & path, const nav_msgs::OccupancyGrid & map)
    {
        for (size_t i = 1; i < path.poses.size(); ++i)
        {
            if (!canReach(path.poses[i-1], path.poses[i], map))
            {
                return true;
            }
        }

        return false;
    }


    bool SearchAlgorithm::canReach(const geometry_msgs::PoseStamped & from, const geometry_msgs::PoseStamped & to, const nav_msgs::OccupancyGrid & map)
    {
        std::vector<int> visited_cells = rayTrace(from, to, map);

        for (size_t i = 0; i < visited_cells.size(); ++i)
        {
            if (map.data[visited_cells[i]] >= 100)
            {
                // Blocked
                return false;
            }
        }

        return true;
    }

    bool SearchAlgorithm::isSame(const geometry_msgs::PoseStamped & pose_1, const geometry_msgs::PoseStamped & pose_2)
    {
        return (pose_1.pose.position.x == pose_2.pose.position.x &&
                pose_1.pose.position.y == pose_2.pose.position.y &&
                pose_1.pose.position.z == pose_2.pose.position.z);
    }

    geometry_msgs::PoseStamped SearchAlgorithm::closestFreeArea(const geometry_msgs::PoseStamped & pose, const nav_msgs::OccupancyGrid & map)
    {
        // Using Breadth-first search
        std::set<std::pair<int, int> > have_been;

        std::queue<Position> next_up;
        next_up.push(getPosition(pose, map));

        while (next_up.size() > 0)
        {

            ROS_FATAL("%d", next_up.size());

            Position current_position = next_up.front();
            next_up.pop();

            if (getData(current_position, map) == 0)
            {
                // Current position is free
                return getPose(current_position, map);
            }

            bool inserted = have_been.insert(std::make_pair<int, int>(current_position.x_, current_position.y_)).second;

            if (inserted)
            {
                if (current_position.x_ - 1 >= 0)
                {
                    next_up.push(Position(current_position.x_ - 1, current_position.y_));
                }
                if (current_position.x_ + 1 < map.info.width)
                {
                    next_up.push(Position(current_position.x_ + 1, current_position.y_));
                }
                if (current_position.y_ - 1 >= 0)
                {
                    next_up.push(Position(current_position.x_, current_position.y_ - 1));
                }
                if (current_position.y_ + 1 < map.info.height)
                {
                    next_up.push(Position(current_position.x_, current_position.y_ + 1));
                }
            }
        }

        return pose;
    }
}
