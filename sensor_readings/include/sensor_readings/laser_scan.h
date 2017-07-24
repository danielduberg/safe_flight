#ifndef LASERSCAN_H
#define LASERSCAN_H

#include <sensor_readings/sensor.h>

#include <sensor_msgs/LaserScan.h>

namespace sensor_readings
{

    class LaserScan : public Sensor
    {

    public:
        LaserScan(std::string topic, float min_distance, float max_distance, int num_points);

        void callback(const sensor_msgs::LaserScan::ConstPtr & scan_in);
    };

}

#endif // LASERSCAN_H
