#pragma once

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/PoseStamped.h>

#include <path_planner/search_algorithm.h>

#include <tf2_ros/transform_listener.h>


#include <occupancy_grid_utils/coordinate_conversions.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <occupancy_grid_utils/ray_tracer.h>

namespace path_planner
{

    class AStar : public SearchAlgorithm
    {

    private:
        // tf2
        tf2_ros::Buffer tf2_buffer_;
        tf2_ros::TransformListener tf2_listener_;

    public:
        AStar();

        virtual nav_msgs::Path computePath(geometry_msgs::PoseStamped start,
                                           geometry_msgs::PoseStamped goal,
                                           const nav_msgs::OccupancyGrid & map);

        virtual nav_msgs::Path updatePath(geometry_msgs::PoseStamped start,
                                           nav_msgs::Path old_path,
                                           const nav_msgs::OccupancyGrid & map);

    private:
        void transformPose(geometry_msgs::PoseStamped * pose, std::string to_frame_id);

        int getRow(const nav_msgs::OccupancyGrid & map,
                   const geometry_msgs::PoseStamped & pose);

        int getColumn(const nav_msgs::OccupancyGrid & map,
                   const geometry_msgs::PoseStamped & pose);

        void smoothingPath(const nav_msgs::MapMetaData & map_info,
                           boost::optional<occupancy_grid_utils::AStarResult> * path,
                           geometry_msgs::Point start_point);

    };

}
