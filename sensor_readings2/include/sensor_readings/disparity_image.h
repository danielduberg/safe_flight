#ifndef DISPARITYIMAGE_H
#define DISPARITYIMAGE_H

#include <sensor_readings/sensor.h>

#include <stereo_msgs/DisparityImage.h>

class DisparityImage : public Sensor {
public:
	DisparityImage(std::string topic);

    DisparityImage(std::string topic, float min_distance, float max_distance, int num_points);

	void callback(const stereo_msgs::DisparityImage::ConstPtr & msg);
};

#endif // DISPARITYIMAGE_H
