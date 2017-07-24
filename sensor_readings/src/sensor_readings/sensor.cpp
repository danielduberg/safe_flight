#include <sensor_readings/sensor.h>

#include <pcl/filters/filter.h>

namespace sensor_readings
{
    Sensor::Sensor(std::string topic)
        : topic_(topic)
    {

    }

    Sensor::Sensor(std::string topic, double min_range = 0.0d, double max_range = std::numeric_limits<double>::infinity(), int num_points = 0)
        : topic_(topic)
        , min_range_(min_range)
        , max_range_(max_range)
        , num_points_(num_points)
    {

    }

    Sensor::Sensor(const Sensor & other)
        : topic_(other.topic_)
        , min_range_(other.min_range_)
        , max_range_(other.max_range_)
        , num_points_(other.num_points_)
        , sensor_readings_()
        , last_update_()
    {

    }

    Sensor::Sensor(Sensor & other)
        : topic_(other.topic_)
        , min_range_(other.min_range_)
        , max_range_(other.max_range_)
        , num_points_(other.num_points_)
        , sensor_readings_()
        , last_update_()
    {

    }

    void Sensor::updateSensorReadings(pcl::PointCloud<pcl::PointXYZRGB> & updated_sensor_readings)
    {
        last_update_ = ros::Time::now();

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(updated_sensor_readings, updated_sensor_readings, indices);

        sensor_readings_ = updated_sensor_readings;
    }

    double Sensor::timeSinceLastUpdate()
    {
        // Will last_update_ really be zero the first time?
        if (last_update_.toSec() == 0)
        {
            return -1;
        }

        return (ros::Time::now() - last_update_).toSec();
    }

    pcl::PointCloud<pcl::PointXYZRGB> Sensor::getSensorReadings()
    {
        return sensor_readings_;
    }
}
