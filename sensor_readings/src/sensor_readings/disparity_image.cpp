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

namespace sensor_readings
{

    DisparityImage::DisparityImage(std::string topic, float min_range = 0.0, float max_range = std::numeric_limits<float>::infinity(), int num_points = 0)
        : Sensor(topic, min_range, max_range, num_points)
    {

    }


    void DisparityImage::callback(const stereo_msgs::DisparityImage::ConstPtr & msg)
    {
        const cv::Mat_<float> dmat(msg->image.height,
                                   msg->image.width,
                                   (float*)&msg->image.data[0],
                                   msg->image.step);

        pcl::PointCloud<pcl::PointXYZRGB> cloud;

        cloud.is_dense = true;
        cloud.points.reserve(msg->image.width * msg->image.height);

        for (size_t x = 0; x < msg->image.width; ++x) {
            for (size_t y = 0; y < msg->image.height; ++y)
            {
                if (dmat(y, x) < msg->min_disparity)
                {
                    //Invalid
                    continue;
                }

                // https://www.ptgrey.com/KB/10102
                pcl::PointXYZRGB point;
                point.z = (msg->f * msg->T) / dmat(y, x);

                double u = x - (msg->image.width / 2.0);
                double v = y - (msg->image.height / 2.0);

                point.x = u * point.z / msg->f;
                point.y = v * point.z / msg->f;

                cloud.points.push_back(point);
            }
        }

        //std::cout << std::endl << std::endl;

        tf::StampedTransform transform;
        try {
            tf_listener_.lookupTransform("base_link", msg->header.frame_id, ros::Time(0), transform);
        } catch (tf::TransformException & ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB> sensor_readings;

        pcl_ros::transformPointCloud(cloud, sensor_readings, transform);

        sensor_readings.header.frame_id = "base_link";
        pcl_conversions::toPCL(msg->header.stamp, sensor_readings.header.stamp);

        updateSensorReadings(sensor_readings);
    }

}
