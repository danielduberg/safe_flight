#pragma once

#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <sensor_readings/sensor.h>

namespace sensor_readings
{
    
    class SRNodelet : public nodelet::Nodelet
    {
    private:
        // General
        std::string pub_topic_;
        double max_time_, frequency_;
        bool three_dimensions_;
        int horizontal_resolution_;
        int vertical_resolution_;
        double min_distance_;
        double max_distance_;

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

        // Other
        ros::Timer get_sensors_timer_;
        ros::Timer publish_timer_;

        std::vector<Sensor *> sensors_;
        std::vector<ros::Subscriber> sensor_subs_;
        ros::Publisher pub_;


        ros::Publisher cloud_pub_;

    private:
        double getDirection(double x, double y);

        double getDistance(double x, double y);

        void init_vectors(std::vector<double> * x, std::vector<double> * y, std::vector<double> * distance);

        void publish(const ros::TimerEvent & timer);

        virtual void onInit();

        void getSensors(const ros::TimerEvent & timer);

        void init_param(ros::NodeHandle & nh);
    };
    
}
