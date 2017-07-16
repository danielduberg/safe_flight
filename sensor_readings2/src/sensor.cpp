#include <ros/ros.h>

#include <sensor_readings/sensor.h>

// C++
#include <limits>

Sensor::Sensor(std::string topic)
    : topic_(topic) {

}

Sensor::Sensor(std::string topic, float min_distance = 0.0, float max_distance = std::numeric_limits<float>::infinity(), int num_points = 0)
    : topic_(topic)
    , min_distance_(min_distance)
    , max_distance_(max_distance)
    , num_points_(num_points) {
}

Sensor::Sensor(const Sensor & other)
    : topic_(other.topic_)
    , min_distance_(other.min_distance_)
    , max_distance_(other.max_distance_)
    , num_points_(other.num_points_)
    , sensor_readings_()
    , last_update_() {
    std::cout << "Copy 1: Sensor" << std::endl;
}

Sensor::Sensor(Sensor & other)
    : topic_(other.topic_)
    , min_distance_(other.min_distance_)
    , max_distance_(other.max_distance_)
    , num_points_(other.num_points_)
    , sensor_readings_()
    , last_update_() {
    std::cout << "Copy 2: Sensor" << std::endl;
}

void Sensor::updateSensorReadings(pcl::PointCloud<pcl::PointXYZ> & updated_sensor_readings) {
     last_update_ = ros::Time::now(); // Got new

    sensor_readings_ = updated_sensor_readings;
}

float Sensor::timeSinceLastUpdate() {
    // Will lastUpdate really be zero the first time?
    if (last_update_.toSec() == 0) {
        //std::cout << "Tjo" << std::endl;
        return -1;
    }
    //std::cout << "Boo" << std::endl;
    return (ros::Time::now() - last_update_).toSec();
}

pcl::PointCloud<pcl::PointXYZ> Sensor::getSensorReadings() {
    return sensor_readings_;
}
