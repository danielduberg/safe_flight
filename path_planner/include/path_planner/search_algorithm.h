#pragma once

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_listener.h>

namespace path_planner
{

    class SearchAlgorithm
    {

    private:
        // tf2
        tf2_ros::Buffer tf2_buffer_;
        tf2_ros::TransformListener tf2_listener_;

    protected:
        struct Position
        {

            int x_;
            int y_;

            Position()
            {

            }

            Position(int x, int y)
                : x_(x)
                , y_(y)
            {

            }
        };

    public:
        SearchAlgorithm();

        /**
         * @brief getPath
         * Computes and returns a path from start to goal if one exist.
         * The difference between this and computePath is that this also smoothes the path, inserts orientation setpoints,
         * and places the start at the closest free space if it is blocked.
         * @param start Where the path starts
         * @param goal Where the path ends
         * @param map A map with obstacles that the path avoidance
         * @return The path
         */
        nav_msgs::Path getPath(geometry_msgs::PoseStamped start,
                               geometry_msgs::PoseStamped goal,
                               const nav_msgs::OccupancyGrid & map);

        /**
         * @brief getUpdatedPath
         * Updates and returns an updated path from start to the last pose in old_path if one exist.
         * The difference between this and updatePath is that this also smoothes the path, inserts orientation setpoints,
         * and places the start at the closest free space if it is blocked.
         * @param start Where the path starts
         * @param old_path The old path
         * @param map A map with obstacles that the path avoidance
         * @return The path
         */
        nav_msgs::Path getUpdatedPath(geometry_msgs::PoseStamped start,
                                      nav_msgs::Path old_path,
                                      const nav_msgs::OccupancyGrid & map);

        /**
         * @brief computePath
         * Computes and returns a path from start to goal if one exist.
         * @param start Where the path starts
         * @param goal Where the path ends
         * @param map A map with obstacles that the path avoidance
         * @return The path
         */
        virtual nav_msgs::Path computePath(geometry_msgs::PoseStamped start,
                                           geometry_msgs::PoseStamped goal,
                                           const nav_msgs::OccupancyGrid & map) = 0;

        /**
         * @brief updatePath
         * Updates and returns an updated path from start to the last pose in old_path if one exist.
         * @param start Where the path starts
         * @param old_path The old path
         * @param map A map with obstacles that the path avoidance
         * @return The path
         */
        virtual nav_msgs::Path updatePath(geometry_msgs::PoseStamped start,
                                           nav_msgs::Path old_path,
                                           const nav_msgs::OccupancyGrid & map) = 0;

        bool isPathBlocked(const nav_msgs::Path & path, const nav_msgs::OccupancyGrid & map);

        nav_msgs::Path smoothenPath(const nav_msgs::Path & path, const nav_msgs::OccupancyGrid & map);

    protected:
        geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped & pose,
                                            const std::string & to_frame_id);

        //nav_msgs::OccupancyGrid expandMap(int new_resolution, const nav_msgs::OccupancyGrid & map);

        int8_t getData(const int x, const int y, const nav_msgs::OccupancyGrid & map);
        int8_t getData(const Position & position, const nav_msgs::OccupancyGrid & map);
        int8_t getData(const geometry_msgs::PoseStamped & pose, const nav_msgs::OccupancyGrid & map);

        int getIndex(const geometry_msgs::PoseStamped & pose, const nav_msgs::OccupancyGrid & map);
        int getIndex(const Position & position, const nav_msgs::OccupancyGrid & map);
        int getIndex(const int x, const int y, const nav_msgs::OccupancyGrid & map);

        Position getPosition(const geometry_msgs::PoseStamped & pose, const nav_msgs::OccupancyGrid & map);
        Position getPosition(const int index, const nav_msgs::OccupancyGrid & map);

        geometry_msgs::PoseStamped getPose(const int index, const nav_msgs::OccupancyGrid & map);
        geometry_msgs::PoseStamped getPose(const Position & position, const nav_msgs::OccupancyGrid & map);

        std::vector<int> rayTrace(const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & end, const nav_msgs::OccupancyGrid & map);
        std::vector<int> rayTrace(const Position & start, const Position & end, const nav_msgs::OccupancyGrid & map);
        std::vector<int> rayTrace(const int start, const int end, const nav_msgs::OccupancyGrid & map);


    private:
        void orientTowardsNext(nav_msgs::Path * path);

        bool canReach(const geometry_msgs::PoseStamped & from, const geometry_msgs::PoseStamped & to, const nav_msgs::OccupancyGrid & map);

        bool isSame(const geometry_msgs::PoseStamped & pose_1, const geometry_msgs::PoseStamped & pose_2);

        geometry_msgs::PoseStamped closestFreeArea(const geometry_msgs::PoseStamped & pose, const nav_msgs::OccupancyGrid & map);

    };
}
