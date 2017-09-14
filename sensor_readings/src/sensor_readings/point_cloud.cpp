#include <ros/ros.h>

#include <sensor_readings/point_cloud.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

// C++
#include <limits>

namespace sensor_readings
{

    PointCloud::PointCloud(std::string topic, float min_range = 0.0, float max_range = std::numeric_limits<float>::infinity(), int num_points = 0)
        : Sensor(topic, min_range, max_range, num_points)
    {

    }


    void PointCloud::callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud_in)
    {
        tf::StampedTransform transform;
        try {
            tf_listener_.lookupTransform("base_link", cloud_in->header.frame_id, ros::Time(0), transform);
        } catch (tf::TransformException & ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB> sensor_readings;

        pcl_ros::transformPointCloud(*cloud_in, sensor_readings, transform);

        sensor_readings.header.frame_id = "base_link";
        sensor_readings.header.stamp = cloud_in->header.stamp;

        updateSensorReadings(sensor_readings);
    }

    void PointCloud::callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_in)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Does this work?
        pcl::copyPointCloud(*cloud_in, *cloud_out);

        callback(cloud_out);
    }

}
