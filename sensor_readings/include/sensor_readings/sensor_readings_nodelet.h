#pragma once

#include <nodelet/nodelet.h>

namespace sensor_readings
{
    
    class SRNodelet : public nodelet::Nodelet
    {
    private:
        // General
        std::string pub_topic_;
        double max_time_, frequency_;
        bool three_dimensions_;

        // Drone
        enum Shape { circle, poly };
        Shape shape_;
        double radius_, height_;
        // Connected order (clockwise)
        std::vector<std::pair<double, double> > poly_points_;

        // Sensors
        std::vector<std::string> sub_topics_;
        std::vector<double> min_ranges_, max_ranges_;
        std::vector<int> num_points_;

    private:
        virtual void onInit();

        void init_param(ros::NodeHandle & nh);
    };
    
}
