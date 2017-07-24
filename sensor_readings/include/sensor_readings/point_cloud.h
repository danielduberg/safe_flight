#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <sensor_readings/sensor.h>

#include <pcl_ros/point_cloud.h>

#include <stereo_msgs/DisparityImage.h>

namespace sensor_readings
{

    class PointCloud : public Sensor
    {

    public:
        PointCloud(std::string topic, float min_range, float max_range, int num_points);

        void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & msg);

        void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & msg);
    };

}

#endif // POINTCLOUD_H
