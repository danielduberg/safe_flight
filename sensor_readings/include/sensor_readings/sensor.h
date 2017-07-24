#pragma once

#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>

namespace sensor_readings
{

    class Sensor
    {
    public:
        std::string topic_;

    protected:
        tf::TransformListener tf_listener_;

        double min_range_, max_range_;

        int num_points_;

    private:
        pcl::PointCloud<pcl::PointXYZRGB> sensor_readings_;

        ros::Time last_update_;


    public:
        Sensor(std::string topic);

        Sensor(std::string topic, double min_range, double max_range, int num_points);

        // Copy constructor
        Sensor(const Sensor & other);

        // Copy constructor
        Sensor(Sensor & other);

        /**
         * @brief timeSinceLastUpdate
         * @return Time in seconds since last update
         */
        double timeSinceLastUpdate();

        // Maybe protected?
        void updateSensorReadings(pcl::PointCloud<pcl::PointXYZRGB> & updated_sensor_readings);

        /**
         * @brief getDistances
         * @return Distances
         */
        pcl::PointCloud<pcl::PointXYZRGB> getSensorReadings();
    };
}
