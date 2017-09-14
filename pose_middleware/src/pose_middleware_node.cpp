#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <message_filters/subscriber.h>

std::string source_frame;
std::string target_frame;
std::string out_topic;

double frequency;

void initParams(const ros::NodeHandle & nh)
{
    nh.param<std::string>("source_frame", source_frame, "map");
    nh.param<std::string>("target_frame", target_frame, "base_link");
    nh.param<std::string>("out_topic", out_topic, "/mavros/mocap/pose");

    nh.param<double>("frequency", frequency, 30);
}

void poseCallback(const ros::TimerEvent & timer, const std::string & target_frame, const std::string & source_frame, ros::Publisher & pub, const tf2_ros::Buffer * tf_buffer)
{
    try
    {
        geometry_msgs::TransformStamped transform = tf_buffer->lookupTransform(source_frame, target_frame, ros::Time());

        ROS_FATAL_THROTTLE(5, "[%f, %f, %f], [%f, %f, %f, %f]", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = source_frame;
        pose.header.stamp = ros::Time::now();

        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;

        pose.pose.orientation = transform.transform.rotation;

        pub.publish(pose);
    }
    catch (tf2::TransformException & ex)
    {
        //l ROS_WARN_THROTTLE(1, ex.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_middleware");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    initParams(nh_priv);


    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);


    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(out_topic, 1);

    ros::Timer pose_timer = nh_priv.createTimer(ros::Duration(1.0d / frequency), boost::bind(poseCallback, _1, target_frame, source_frame, pose_pub, &tf_buffer));

    ros::spin();

    return 0;
}
