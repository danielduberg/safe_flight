#include <pluginlib/class_list_macros.h>

#include <sensor_readings/sensor_readings_nodelet.h>

#include <ros/ros.h>

namespace sensor_readings
{
    void SRNodelet::onInit()
    {
        NODELET_ERROR("Initializing nodelet...");
    }

    PLUGINLIB_DECLARE_CLASS(sensor_readings, SR, sensor_readings::SRNodelet, nodelet::Nodelet);
}
