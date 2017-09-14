#include <path_planner/d_star_lite.h>

#include <limits>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace path_planner
{

    DStarLite::DStarLite()
        : tf2_listener_(tf2_buffer_)
    {

    }

    void DStarLite::transformPose(geometry_msgs::PoseStamped * pose,
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

    int DStarLite::getRow(const nav_msgs::OccupancyGrid & map,
               const geometry_msgs::PoseStamped & pose)
    {
        geometry_msgs::PoseStamped temp_pose = pose;
        if (map.header.frame_id != temp_pose.header.frame_id)
        {
            transformPose(&temp_pose, map.header.frame_id);
        }


    }

    int DStarLite::getColumn(const nav_msgs::OccupancyGrid & map,
               const geometry_msgs::PoseStamped & pose)
    {

    }

    std::pair<double, double> DStarLite::calculateKey(Vertex & s)
    {
        return std::make_pair<double, double>(
                    std::min(g_[s], rhs_[s] + h(s_start_, s) + km_),
                    std::min(g_[s], rhs_[s])
                    );
    }

    void DStarLite::initialize()
    {
        U_.clear();

        km_ = 0;

        // TODO: For all s

        rhs_[s_goal_] = 0;

        U_.insert(s_goal_, calculateKey(s_goal_));
    }

    void DStarLite::updateVertex(Vertex & u)
    {
        if (u != s_start_)
        {
            // TODO: rhs_[u] = ...
        }

        // TODO: Does find work here?!
        U_.remove(u);

        if (g_[u] != rhs_[u])
        {
            U_.insert(u, calculateKey(u));
        }
    }

    void DStarLite::smoothingPath(const nav_msgs::MapMetaData & map_info, boost::optional<occupancy_grid_utils::AStarResult> * path, geometry_msgs::Point start_point)
    {
        geometry_msgs::Point current_point = start_point;

        for (size_t i = 0; i < (*path)->first.size(); ++i)
        {
            geometry_msgs::Point next_point = occupancy_grid_utils::cellCenter(map_info, (*path)->first[i]);

            occupancy_grid_utils::RayTraceIterRange it_range = occupancy_grid_utils::rayTrace(map_info, current_point, next_point);

            occupancy_grid_utils::RayTraceIterator it = it_range.first;

            //while (it)
        }
    }

    nav_msgs::Path DStarLite::computePath(geometry_msgs::PoseStamped start,
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
            geometry_msgs::Point previous_point = start.pose.position;
            previous_point.z = 1;

            for (size_t i = 0; i < result->first.size()-1; ++i)
            {
                // Orient towards current pose
                geometry_msgs::PoseStamped orient_pose;
                orient_pose.header.stamp = ros::Time::now();
                orient_pose.header.frame_id = map.header.frame_id;


                geometry_msgs::Point next_point = occupancy_grid_utils::cellCenter(map.info, result->first[i]);
                next_point.z = 1;

                double yaw = std::atan2(next_point.y - previous_point.y, next_point.x - previous_point.x);

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);

                orient_pose.pose.position = previous_point;
                orient_pose.pose.orientation.x = q.getX();
                orient_pose.pose.orientation.y = q.getY();
                orient_pose.pose.orientation.z = q.getZ();
                orient_pose.pose.orientation.w = q.getW();

                path.poses.push_back(orient_pose);


                geometry_msgs::PoseStamped pose = orient_pose;
                pose.pose.position = next_point;

                path.poses.push_back(pose);

                previous_point = next_point;
            }

            path.poses.push_back(goal);
        }

        return path;

        /*
        s_start_.point_ = std::make_pair<int, int>(getColumn(map, start), getRow(map, start));

        s_goal_.point_ = std::make_pair<int, int>(getColumn(map, goal), getRow(map, goal));

        s_last_ = s_start_;

        initialize();

        // Compute shortest path
        while (U_.getTopKey() < calculateKey(s_start_) ||
               rhs_[s_start_] != g_[s_start_])
        {
            std::pair<double, double> k_old = U_.getTopKey();

            Vertex u = U_.pop();

            if (k_old < calculateKey(u))
            {
                U_.insert(u, calculateKey(u));
            }
            else if (g_[u] > rhs_[u])
            {
                g_[u] = rhs_[u];

                std::vector<Vertex> temp = pred(u);
                for (size_t i = 0; i < temp.size(); ++i)
                {
                    updateVertex(temp[i]);
                }
            }
            else
            {
                g_[u] = std::numeric_limits<double>::infinity();

                std::vector<Vertex> temp = pred(u);
                for (size_t i = 0; i < temp.size(); ++i)
                {
                    updateVertex(temp[i]);
                }
                updateVertex(u);
            }
        }


        // TODO: Generate path

        nav_msgs::Path path;
        path.poses.push_back(goal);

        return path;
        */
    }

    nav_msgs::Path DStarLite::updatePath(geometry_msgs::PoseStamped start,
                                     nav_msgs::Path old_path,
                                     const nav_msgs::OccupancyGrid & map)
    {
        return computePath(start, old_path.poses[old_path.poses.size() - 1], map);
    }

    double DStarLite::h(Vertex & s, Vertex & s_goal)
    {
        // TODO
        return c(s, s_goal);
    }

    double DStarLite::c(Vertex & u, Vertex & s)
    {
        // TODO
        return std::sqrt(std::pow(u.point_.first - s.point_.first, 2) + std::pow(u.point_.second - s.point_.second, 2));
    }

    std::vector<Vertex> DStarLite::succ(const Vertex & s)
    {
        std::vector<Vertex> succesors;

        return succesors;
    }

    std::vector<Vertex> DStarLite::pred(const Vertex & s)
    {
        return succ(s);
    }

}
