#include <path_planner/a_star.h>

#include <limits>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace path_planner
{

    AStar::AStar()
        : tf2_listener_(tf2_buffer_)
    {

    }

    void AStar::transformPose(geometry_msgs::PoseStamped * pose,
                                  std::string to_frame_id)
    {
        geometry_msgs::TransformStamped transform;

        try
        {
            transform = tf2_buffer_.lookupTransform(pose->header.frame_id,
                                                    to_frame_id, ros::Time(0),
                                                    ros::Duration(1.0));
        }
        catch (tf2::TransformException & ex)
        {
            ROS_WARN_STREAM("Could NOT transform " << pose->header.frame_id
                            << " to " << to_frame_id << ": " << ex.what());
            return;
        }

        tf2::doTransform(*pose, *pose, transform);
    }


    nav_msgs::Path AStar::computePath(geometry_msgs::PoseStamped start,
                                                  geometry_msgs::PoseStamped goal,
                                                  const nav_msgs::OccupancyGrid & map)
    {
        transformPose(&start, map.header.frame_id);
        transformPose(&goal, map.header.frame_id);

        occupancy_grid_utils::Cell start_cell = occupancy_grid_utils::pointCell(map.info, start.pose.position);
        occupancy_grid_utils::Cell goal_cell = occupancy_grid_utils::pointCell(map.info, goal.pose.position);

        boost::optional<occupancy_grid_utils::AStarResult> result = occupancy_grid_utils::shortestPathAStar(map, start_cell, goal_cell);

        nav_msgs::Path path;

        if (result)
        {
            //geometry_msgs::Point previous_point = start.pose.position;
            //previous_point.z = 1;

            for (size_t i = 0; i < result->first.size()-1; ++i)
            {
                // Orient towards current pose
                geometry_msgs::PoseStamped orient_pose;
                orient_pose.header.stamp = ros::Time::now();
                orient_pose.header.frame_id = map.header.frame_id;


                geometry_msgs::Point next_point = occupancy_grid_utils::cellCenter(map.info, result->first[i]);
                next_point.z = 1;

                /*
                double yaw = std::atan2(next_point.y - previous_point.y, next_point.x - previous_point.x);

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);

                orient_pose.pose.position = previous_point;
                orient_pose.pose.orientation.x = q.getX();
                orient_pose.pose.orientation.y = q.getY();
                orient_pose.pose.orientation.z = q.getZ();
                orient_pose.pose.orientation.w = q.getW();

                path.poses.push_back(orient_pose);
                */


                geometry_msgs::PoseStamped pose = orient_pose;
                pose.pose.position = next_point;

                path.poses.push_back(pose);

                //previous_point = next_point;
            }

            path.poses.push_back(goal);
        }

        return path;
    }

    nav_msgs::Path AStar::updatePath(geometry_msgs::PoseStamped start,
                                     nav_msgs::Path old_path,
                                     const nav_msgs::OccupancyGrid & map)
    {
        return computePath(start, old_path.poses[old_path.poses.size() - 1], map);
    }
}
