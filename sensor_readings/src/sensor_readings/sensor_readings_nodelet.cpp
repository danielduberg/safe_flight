#include <pluginlib/class_list_macros.h>

#include <sensor_readings/sensor_readings_nodelet.h>

#include <ros/assert.h>


#include <sensor_readings/sensor.h>
#include <sensor_readings/laser_scan.h>
#include <sensor_readings/disparity_image.h>
#include <sensor_readings/point_cloud.h>

#include <safe_flight_msgs/SensorReadings.h>

namespace sensor_readings
{
    double SRNodelet::getDirection(double x, double y)
    {
        // Changed order of x and y
        return std::fmod(((std::atan2(y, x) * 180 / M_PI) + 360), 360);
    }

    double SRNodelet::getDistance(double x, double y)
    {
        return std::sqrt((x*x) + (y*y));
    }

    void SRNodelet::init_vectors(std::vector<double> * x, std::vector<double> * y, std::vector<double> * distance)
    {
        double degrees_per_index = (2.0d * M_PI) / x->size();

        for (size_t i = 0; i < x->size(); ++i)
        {
            (*x)[i] = min_distance_ * std::cos(((double) i) * degrees_per_index);
            (*y)[i] = min_distance_ * std::sin(((double) i) * degrees_per_index);
            (*distance)[i] = min_distance_;
        }
    }

    void SRNodelet::publish(const ros::TimerEvent & timer)
    {
        //NODELET_FATAL_STREAM("1");
        std::vector<double> x(horizontal_resolution_, 0);
        std::vector<double> y(horizontal_resolution_, 0);
        std::vector<double> distance(horizontal_resolution_, -1);
        std::vector<bool> updated(horizontal_resolution_, false);

        init_vectors(&x, &y, &distance);

        pcl::PointCloud<pcl::PointXYZ> cloud;

        //NODELET_FATAL_STREAM("2");
        for (size_t i = 0; i < sensors_.size(); ++i)
        {
            if (sensors_[i]->timeSinceLastUpdate() > max_time_)
            {
                // It was more than 'max_time_' second(s) ago this sensor was last updated!
                continue;
            }

            pcl::PointCloud<pcl::PointXYZRGB> obst_points = sensors_[i]->getSensorReadings();

            int last_index = -1;

            for (size_t j = 0; j < obst_points.size(); ++j)
            {
                double direction = getDirection(obst_points[j].x, obst_points[j].y);

                int index = round(direction * (horizontal_resolution_ / 360.0d));
                index = index % horizontal_resolution_;

                // Check if this is the closest point at this index
                double current_distance = getDistance(obst_points[j].x, obst_points[j].y);
                if (current_distance < min_distance_)
                {
                    continue;
                }

                if (current_distance > max_distance_)
                {
                    // We cannot even see anything here :O
                    updated[index] = true;
                    x[index] = 0.0d;
                    y[index] = 0.0d;
                    distance[index] = 0.0d;
                }

                if (!updated[index] || current_distance < distance[index])
                {
                    updated[index] = true;
                    x[index] = obst_points[j].x;
                    y[index] = obst_points[j].y;
                    distance[index] = current_distance;
                }

                // Set everything in between to "far away", 0 = too far away to be seen
                if (last_index != -1)
                {
                    if (std::max(last_index, index) -std::min(last_index, index) < horizontal_resolution_ / 2.0d)
                    {
                        for (size_t k = std::min(last_index, index); k < std::max(last_index, index); ++k)
                        {
                            if (!updated[k])
                            {
                                // Do not change this to updated!
                                x[k] = 0;
                                y[k] = 0;
                                distance[k] = 0;
                            }
                        }
                    }
                    else
                    {
                        for (size_t k = std::max(last_index, index); k < horizontal_resolution_ + std::min(last_index, index); ++k)
                        {
                            size_t k360 = k % 360;

                            if (!updated[k360])
                            {
                                // Do not change this to updated!
                                x[k360] = 0;
                                y[k360] = 0;
                                distance[k360] = 0;
                            }
                        }
                    }
                }

                last_index = index;
            }
        }

        safe_flight_msgs::SensorReadings output;
        output.header.stamp = ros::Time::now();
        output.header.frame_id = "drone";
        output.x = x;
        output.y = y;
        output.distance = distance;
        output.horizontal_resolution = horizontal_resolution_;
        output.vertical_resolution = vertical_resolution_;

        pub_.publish(output);

        for (size_t i = 0; i < x.size(); ++i)
        {
            pcl::PointXYZ point;
            point.x = x[i];
            point.y = y[i];
            point.z = 0;
            cloud.push_back(point);
        }

        pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
        cloud.header.frame_id = "drone";
        cloud_pub_.publish(cloud);
    }

    void SRNodelet::onInit()
    {
        ros::NodeHandle & nh = getNodeHandle();
        ros::NodeHandle & nh_priv = getPrivateNodeHandle();

        init_param(nh_priv);

        pub_ = nh.advertise<safe_flight_msgs::SensorReadings>(pub_topic_, 1);

        cloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("cloud", 1);

        // Look for sensors ones every second
        get_sensors_timer_ = nh_priv.createTimer(ros::Duration(1.0d), &SRNodelet::getSensors, this);

        publish_timer_ = nh_priv.createTimer(ros::Duration(1.0d / frequency_), &SRNodelet::publish, this);
    }

    void SRNodelet::getSensors(const ros::TimerEvent & timer)
    {
        if (sub_topics_.size() == 0)
        {
            get_sensors_timer_.stop();
            return;
        }

        ros::NodeHandle & nh = getNodeHandle();

        ros::master::V_TopicInfo topic_infos;
        ros::master::getTopics(topic_infos);

        for (size_t i = 0; i < sub_topics_.size(); ++i)
        {
            for (size_t j = 0; j < topic_infos.size(); ++j)
            {
                if (sub_topics_[i] == topic_infos[j].name)
                {
                    if (topic_infos[j].datatype == "sensor_msgs/LaserScan")
                    {
                        LaserScan * sensor = new LaserScan(sub_topics_[i], min_ranges_[i], max_ranges_[i], num_points_[i]);
                        sensors_.push_back(sensor);
                        sensor_subs_.push_back(nh.subscribe(sub_topics_[i], 1, &LaserScan::callback, sensor));
                        // Remove this since we are not subscribed!
                        sub_topics_.erase(sub_topics_.begin() + i);
                        min_ranges_.erase(min_ranges_.begin() + i);
                        max_ranges_.erase(max_ranges_.begin() + i);
                        num_points_.erase(num_points_.begin() + i);
                    }
                    else if (topic_infos[j].datatype == "stereo_msgs/DisparityImage")
                    {
                        DisparityImage * sensor = new DisparityImage(sub_topics_[i], min_ranges_[i], max_ranges_[i], num_points_[i]);
                        sensors_.push_back(sensor);
                        sensor_subs_.push_back(nh.subscribe(sub_topics_[i], 1, &DisparityImage::callback, sensor));
                        // Remove this since we are not subscribed!
                        sub_topics_.erase(sub_topics_.begin() + i);
                        min_ranges_.erase(min_ranges_.begin() + i);
                        max_ranges_.erase(max_ranges_.begin() + i);
                        num_points_.erase(num_points_.begin() + i);
                    }
                    else if (topic_infos[i].datatype == "sensor_msgs/PointCloud2")
                    {
                        PointCloud * sensor = new PointCloud(sub_topics_[i], min_ranges_[i], max_ranges_[i], num_points_[i]);
                        sensors_.push_back(sensor);
                        sensor_subs_.push_back(nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >(sub_topics_[i], 1, &PointCloud::callback, sensor));
                        // Remove this since we are not subscribed!
                        sub_topics_.erase(sub_topics_.begin() + i);
                        min_ranges_.erase(min_ranges_.begin() + i);
                        max_ranges_.erase(max_ranges_.begin() + i);
                        num_points_.erase(num_points_.begin() + i);
                    }
                    else
                    {
                        ROS_ERROR_STREAM(topic_infos[j].datatype << " is not supported by sensor_readings");
                        ++i; // Because we take -- next
                    }

                    --i;
                    break;
                }
            }
        }
    }

    void SRNodelet::init_param(ros::NodeHandle & nh)
    {
        // General
        nh.param("general/pub_topic", pub_topic_, std::string("sen"));
        nh.param("general/max_time", max_time_, 1.0);
        nh.param("general/frequency", frequency_, 50.0);
        nh.param("general/three_dimensions", three_dimensions_, false);
        nh.param("general/horizontal_resolution", horizontal_resolution_, 360);
        nh.param("general/vertical_resolution", vertical_resolution_, 360);
        nh.param("general/min_distance", min_distance_, 0.0d);
        nh.param("general/max_distance", max_distance_, 1000.0d);


        // Drone
        std::string shape;
        if (!nh.getParam("drone/shape", shape))
        {
            NODELET_FATAL_STREAM("Have to specify drone shape!");
            exit(1);
        }
        std::transform(shape.begin(), shape.end(), shape.begin(), tolower);
        if (shape == "circle")
        {
            shape_ = circle;
        }
        else if (shape == "poly")
        {
            shape_ = poly;
        }
        else
        {
            NODELET_FATAL_STREAM("Shape has to be either 'circle' or 'poly'");
            exit(2);
        }

        switch (shape_)
        {
        case circle:
            nh.param("drone/radius", radius_, 0.0d);

            break;
        case poly:


            std::vector<double> x, y;
            nh.getParam("drone/poly_points/x", x);
            nh.getParam("drone/poly_points/y", y);

            if (shape_ == poly && x.size() != y.size())
            {
                NODELET_FATAL_STREAM("There has to be an equal number of x- and y-coordinates for describing the polygon shape. Now there are: " << x.size() << " x and " << y.size() << " y.");
                exit(3);
            }

            poly_points_.reserve(x.size());
            for (size_t i = 0; i < x.size(); ++i)
            {
                poly_points_.push_back(std::make_pair<double, double>(x[i], y[i]));
            }

            break;
        }

        nh.param("drone/height", height_, 0.0d);


        // Sensors
        nh.getParam("sensors/sub_topics", sub_topics_);
        nh.getParam("sensors/min_ranges", min_ranges_);
        nh.getParam("sensors/max_ranges", max_ranges_);
        nh.getParam("sensors/num_points", num_points_);
        // Less reallocations
        sensors_.reserve(sub_topics_.size());
    }



    PLUGINLIB_DECLARE_CLASS(sensor_readings, SR, sensor_readings::SRNodelet, nodelet::Nodelet);
}
