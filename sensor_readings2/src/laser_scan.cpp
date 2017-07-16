#include <ros/ros.h>

#include <sensor_readings/laser_scan.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

// C++
#include <limits>

LaserScan::LaserScan(std::string topic)
    : Sensor(topic) {

}

LaserScan::LaserScan(std::string topic, float min_distance = 0.0, float max_distance = std::numeric_limits<float>::infinity(), int num_points = 0)
    : Sensor(topic, min_distance, max_distance, num_points) {
}


// TODO
void LaserScan::callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.is_dense = true;

    float currentAngle = msg->angle_min - msg->angle_increment;
    for (size_t i = 0; i < msg->ranges.size(); i++) {
        currentAngle += msg->angle_increment;
        if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max) {
            // Invalid
            continue;
        }

        pcl::PointXYZ point;
        point.x = msg->ranges[i] * std::cos(currentAngle);
        point.y = msg->ranges[i] * std::sin(currentAngle);
        point.z = 0;
        cloud.points.push_back(point);

       }


    tf::StampedTransform transform;
    try {
        tf_listener_.lookupTransform(msg->header.frame_id, "drone", ros::Time(0), transform);
    } catch (tf::TransformException & ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> sensor_readings;

    pcl_ros::transformPointCloud(cloud, sensor_readings, transform.inverse());

    sensor_readings.header.frame_id = "drone";
    pcl_conversions::toPCL(msg->header.stamp, sensor_readings.header.stamp);

    updateSensorReadings(sensor_readings);
}
