#ifndef LASERSCAN_H
#define LASERSCAN_H

#include <sensor_readings/sensor.h>

#include <sensor_msgs/LaserScan.h>

class LaserScan : public Sensor {
public:
	LaserScan(std::string topic);

    LaserScan(std::string topic, float min_distance, float max_distance, int num_points);

	void callback(const sensor_msgs::LaserScan::ConstPtr & msg);
};


#endif // LASERSCAN_H
