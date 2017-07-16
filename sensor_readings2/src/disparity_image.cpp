#include <ros/ros.h>

#include <sensor_readings/disparity_image.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

// C++
#include <limits>

DisparityImage::DisparityImage(std::string topic)
    : Sensor(topic) {

}

DisparityImage::DisparityImage(std::string topic, float min_distance = 0.0, float max_distance = std::numeric_limits<float>::infinity(), int num_points = 0)
    : Sensor(topic, min_distance, max_distance, num_points) {
}


// TODO
void DisparityImage::callback(const stereo_msgs::DisparityImage::ConstPtr & msg) {
    const cv::Mat_<float> dmat(msg->image.height,
                               msg->image.width,
                               (float*)&msg->image.data[0],
                               msg->image.step);

    if (num_points_ == 0) {
        num_points_ = msg->image.width;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;

    //cloud.width = num_points;
    //cloud.height = 1;
    cloud.is_dense = false;
    //cloud.points.resize(cloud.width * cloud.height);

    //std::cout << topic << std::endl;
    for (size_t x = 0; x < msg->image.width; x++) {
        bool first = true;
        for (size_t y = (msg->image.height / 2) - 5; y < (msg->image.height / 2) + 10; y++) {
            if (dmat(y, x) < msg->min_disparity) {
                // Invailid
                continue;
            }

            //int index = (x * num_points) / (float) msg->image.width;

            float depth = (msg->f * msg->T) / dmat(y, x);

            if (first) {
                first = false;
                pcl::PointXYZ point;
                point.x = (x - (msg->image.width / 2.0)) * depth / msg->f;
                point.y = 0;
                point.z = depth;
                cloud.points.push_back(point);
            } else if (depth < cloud.points[cloud.points.size() - 1].z){
                cloud.points[cloud.points.size() - 1].x = (x - (msg->image.width / 2.0)) * depth / msg->f;
                cloud.points[cloud.points.size() - 1].y = 0;
                cloud.points[cloud.points.size() - 1].z = depth;
            }

            /*
            if (depth > min_distance_ && depth < max_distance_ && (first || depth < cloud.points[index].z)) {
                cloud.points[index].x = (x - (msg->image.width / 2.0)) * depth / msg->f;
                cloud.points[index].y = 0;
                cloud.points[index].z = depth;
            }
            */

            //std::cout << dmat(y, x) << ", ";
        }
    }

    //std::cout << std::endl << std::endl;

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

    /*
    for (size_t i = 0; i < sensor_readings.points.size(); i++) {
        std::cout << sensor_readings.points[i].x << ", " << sensor_readings.points[i].y << ", " << std::fmod(((std::atan2(sensor_readings.points[i].y, sensor_readings.points[i].x) * 180 / 3.14159265) + 360), 360) << " | ";
    }

    std::cout << std::endl << std::endl;

    exit(0);
    */
}
