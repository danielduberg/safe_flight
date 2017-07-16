#include <pluginlib/class_list_macros.h>

#include <sensor_readings/sensor_readings_nodelet.h>

#include <ros/ros.h>

#include <ros/assert.h>

namespace sensor_readings
{
    void SRNodelet::onInit()
    {
        ros::NodeHandle & nh_priv = getPrivateNodeHandle();

        init_param(nh_priv);

        ros::Timer timer = nh_priv.createTimer(ros::Duration(1.0d / frequency_), publish);
    }

    void SRNodelet::init_param(ros::NodeHandle & nh)
    {
        // General
        nh.param("general/pub_topic", pub_topic_, std::string("sen"));
        nh.param("general/max_time", max_time_, 1.0);
        nh.param("general/frequency", frequency_, 50.0);
        nh.param("general/three_dimensions", three_dimensions_, false);


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
        nh.getParam("sub_topics", sub_topics_);
        nh.getParam("min_ranges", min_ranges_);
        nh.getParam("max_ranges", max_ranges_);
        nh.getParam("num_points", num_points_);
    }



    PLUGINLIB_DECLARE_CLASS(sensor_readings, SR, sensor_readings::SRNodelet, nodelet::Nodelet);
}
