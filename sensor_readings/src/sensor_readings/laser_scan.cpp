#include <ros/ros.h>

#include <sensor_readings/laser_scan.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/filter.h>

// C++
#include <limits>


#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


namespace sensor_readings
{

    LaserScan::LaserScan(std::string topic, float min_distance = 0.0, float max_distance = std::numeric_limits<float>::infinity(), int num_points = 0)
        : Sensor(topic, min_distance, max_distance, num_points)
    {

    }


    void LaserScan::callback(const sensor_msgs::LaserScan::ConstPtr & scan_in)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;

        cloud.is_dense = true;
        // To decrease number of reallocations
        cloud.points.reserve(scan_in->ranges.size());

        float currentAngle = scan_in->angle_min - scan_in->angle_increment;
        for (size_t i = 0; i < scan_in->ranges.size(); ++i) {
            currentAngle += scan_in->angle_increment;
            if (scan_in->ranges[i] < scan_in->range_min || scan_in->ranges[i] > scan_in->range_max) {
                // Invalid range
                continue;
            }

            if (scan_in->ranges[i] < min_range_ || scan_in->ranges[i] > max_range_)
            {
                // Out of wanted range
                continue;
            }

            pcl::PointXYZRGB point;
            point.x = scan_in->ranges[i] * std::cos(currentAngle);
            point.y = scan_in->ranges[i] * std::sin(currentAngle);
            point.z = 0;
            cloud.points.push_back(point);

        }

        // Check number of points

        tf::StampedTransform transform;
        try {
            tf_listener_.lookupTransform("drone", scan_in->header.frame_id, ros::Time(0), transform);
        } catch (tf::TransformException & ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB> sensor_readings;

        pcl_ros::transformPointCloud(cloud, sensor_readings, transform);

        sensor_readings.header.frame_id = "drone";
        pcl_conversions::toPCL(scan_in->header.stamp, sensor_readings.header.stamp);

        updateSensorReadings(sensor_readings);
    }

}
