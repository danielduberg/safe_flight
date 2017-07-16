#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <pcl_ros/transforms.h>

#include <sensor_readings/sensor.h>
#include <sensor_readings/disparity_image.h>
#include <sensor_readings/laser_scan.h>
#include <exjobb_msgs/SensorReadings.h>

#include <math.h> // Round

// To get topics
#include <ros/master.h>

#include <limits>
#define PI 3.14159265

float getDirection(float x, float y) {
    // Changed order of x and y
    return std::fmod(((std::atan2(y, x) * 180 / PI) + 360), 360);
}

float getDistance(float x, float y) {
    return std::sqrt((x*x) + (y*y));
}

void initXY(std::vector<float> & x, std::vector<float> & y, float distance) {
    for (size_t i = 0; i < x.size(); i++) {
        float direction = i * (360.0 / x.size());

        x[i] = distance * std::cos(direction * PI / 180.0);
        y[i] = distance * std::sin(direction * PI / 180.0);
    }
}

void publish(ros::Publisher pub, std::vector<Sensor *> sensors, float min_distance, float max_distance, float max_time) {
    std::vector<float> x(360, 0); //, std::numeric_limits<float>::infinity());
    std::vector<float> y(360, 0); //, std::numeric_limits<float>::infinity());
    //initXY(x, y, min_distance);
    std::vector<float> distance(360, 0); //min_distance);   // Min distance
    std::vector<bool> updated(360, false);

    for (size_t i = 0; i < sensors.size(); i++) {
        if (sensors[i]->timeSinceLastUpdate() > max_time) {
            // It was more than 'max_time' second(s) ago this sensor last updated!
            continue;
        }

        // They should be in order! Do they have to really?
        pcl::PointCloud<pcl::PointXYZ> cloud = sensors[i]->getSensorReadings();

        int lastIndex = -1;

        for (size_t j = 0; j < cloud.points.size(); j++) {
            float direction = getDirection(cloud.points[j].x, cloud.points[j].y);

            int index = round(direction * (x.size() / 360.0));
            index = index % x.size();

            // Check if this is the closest point at this index
            float currentDistance = getDistance(cloud.points[j].x, cloud.points[j].y);
            if (currentDistance < min_distance || currentDistance > max_distance) {
                continue;
            }

            if (!updated[index] || currentDistance < distance[index]) {
                updated[index] = true;
                x[index] = cloud.points[j].x;
                y[index] = cloud.points[j].y;
                distance[index] = currentDistance;
            }

            // Set everything in between to "far away", 0 = too far away to be seen
            if (lastIndex != -1) {
                if (std::max(lastIndex, index) - std::min(lastIndex, index) < 180) {
                    for (size_t k = std::min(lastIndex, index); k < std::max(lastIndex, index); k++) {
                        if (!updated[k]) {
                            // Do not change this to updated!
                            x[k] = 0;
                            y[k] = 0;
                            distance[k] = 0;
                        }
                    }
                } else {
                    for (size_t k = std::max(lastIndex, index); k < 360 + std::min(lastIndex, index); k++) {
                        int k360 = k % 360;

                        if (!updated[k360]) {
                            // Do not change this to updated!
                            x[k360] = 0;
                            y[k360] = 0;
                            distance[k360] = 0;
                        }
                    }
                }
            }

            lastIndex = index;
        }
    }

    exjobb_msgs::SensorReadings output;
    output.x = x;
    output.y = y;
    output.distance = distance;

    pub.publish(output);
}

void getSensors(ros::NodeHandle & nh, std::vector<Sensor *> & sensors, std::vector<ros::Subscriber> & disparity_subs, std::vector<std::string> & sub_topics) {
    if (sub_topics.size() == 0) {
        return;
    }

    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    for (size_t i = 0; i < sub_topics.size(); i++) {
        for (size_t j = 0; j < topic_infos.size(); j++) {
            if (topic_infos[j].name == sub_topics[i]) {
                if (topic_infos[j].datatype == "sensor_msgs/LaserScan") {
                    LaserScan * sensor = new LaserScan(sub_topics[i]); //, min_distance, max_distance, num_points);
                    sensors.push_back(sensor);
                    disparity_subs.push_back(nh.subscribe(sub_topics[i], 1000, &LaserScan::callback, sensor));
                    // Remove this since we are now subscribed!
                    sub_topics.erase(sub_topics.begin() + i);
                    i--;
                    break;

                } else if (topic_infos[j].datatype == "stereo_msgs/DisparityImage"){
                    DisparityImage * sensor = new DisparityImage(sub_topics[i]); //, min_distance, max_distance, num_points);
                    sensors.push_back(sensor);
                    disparity_subs.push_back(nh.subscribe(sub_topics[i], 1000, &DisparityImage::callback, sensor));
                    // Remove this since we are now subscribed!
                    sub_topics.erase(sub_topics.begin() + i);
                    i--;
                    break;

                } else {
                    // Does not exist :(
                    break;
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_readings");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    float min_distance, max_distance;
    nh_priv.param<float>("min_distance", min_distance, 0.1);
    nh_priv.param<float>("max_distance", max_distance, 5.0);


    int num_points;
    nh_priv.param<int>("num_points", num_points, 0);

    std::string pub_topic;
    nh_priv.param<std::string>("pub_topic", pub_topic, "sensor_readings");

    ros::Publisher pub = nh.advertise<exjobb_msgs::SensorReadings>(pub_topic, 1000);

    std::vector<std::string> sub_topics;
    nh_priv.getParam("sub_topics", sub_topics);

    std::vector<Sensor *> sensors;

    std::vector<ros::Subscriber> disparity_subs(sub_topics.size());

    /*
    for (size_t i = 0; i < sub_topics.size(); i++) {
        DisparityImage * sensor = new DisparityImage(sub_topics[i]); //, min_distance, max_distance, num_points);
        sensors.push_back(sensor);
        disparity_subs.push_back(nh.subscribe(sub_topics[i], 1000, &DisparityImage::callback, sensor));
    }
    */

    float max_time;
    nh_priv.param<float>("max_time", max_time, 1.0);

    float frequency;
    nh_priv.param<float>("frequency", frequency, 100.0);

    ros::Rate rate(frequency);
    while (ros::ok())
    {
        getSensors(nh, sensors, disparity_subs, sub_topics);

        ros::spinOnce();

        publish(pub, sensors, min_distance, max_distance, max_time);

        rate.sleep();
    }

    return 0;
}
