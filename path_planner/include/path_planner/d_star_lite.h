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

    struct Vertex
    {
    public:
        std::pair<int, int> point_;

        std::pair<double, double> k_;

        double rhs_;

        double g_;

    public:
        bool operator==(const Vertex & other) const
        {
            return point_ == other.point_;
        }

        bool operator!=(const Vertex & other) const
        {
            return point_ != other.point_;
        }

        bool operator<(const Vertex & other) const
        {
            return k_ < other.k_;
        }

        bool operator>(const Vertex & other) const
        {
            return k_ > other.k_;
        }

        bool operator<=(const Vertex & other) const
        {
            return k_ <= other.k_;
        }

        bool operator>=(const Vertex & other) const
        {
            return k_ >= other.k_;
        }
    };

    class PriorityQueue
    {
    private:
        std::set<Vertex> queue_;

    public:
        std::pair<double, double> getTopKey()
        {
            if (queue_.size())
            {
                return std::make_pair<double, double>(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
            }
            else
            {
                std::set<Vertex>::iterator it = queue_.begin();
                return it->k_;
            }
        }

        Vertex pop()
        {
            std::set<Vertex>::iterator it = queue_.begin();

            Vertex s = *it;

            queue_.erase(it);

            return s;
        }

        void insert(Vertex s, std::pair<double, double> k)
        {
            s.k_ = k;

            queue_.insert(s);
        }

        void update(Vertex s, std::pair<double, double> k)
        {
            std::set<Vertex>::iterator it = queue_.find(s);

            if (it != queue_.end())
            {
                if (it->k_ == k)
                {
                    return;
                }

                queue_.erase(it);
            }

            insert(s, k);
        }

        void remove(Vertex s)
        {
            std::set<Vertex>::iterator it = queue_.find(s);

            if (it != queue_.end())
            {
                queue_.erase(it);
            }
        }

        void clear()
        {
            queue_.clear();
        }
    };



    class DStarLite : public SearchAlgorithm
    {

    private:
        geometry_msgs::PoseStamped goal_;
        nav_msgs::OccupancyGrid map_;

        // D* Lite
        PriorityQueue U_;
        std::map<Vertex, double> rhs_;
        std::map<Vertex, double> g_;
        double km_;
        nav_msgs::OccupancyGrid S_;

        Vertex s_last_;
        Vertex s_start_;
        Vertex s_goal_;

        // tf2
        tf2_ros::Buffer tf2_buffer_;
        tf2_ros::TransformListener tf2_listener_;

    public:
        DStarLite();

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

        std::pair<double, double> calculateKey(Vertex & s);

        void initialize();

        void updateVertex(Vertex & u);

        double c(Vertex & u, Vertex & s);

        double h(Vertex & s, Vertex & s_goal);

        std::vector<Vertex> succ(const Vertex & s);

        std::vector<Vertex> pred(const Vertex & s);

    };

}
