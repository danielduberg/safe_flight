#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

ros::Timer imu_publisher_timer;

void altitudeBroadcast(double altitude, const std::string & from_frame, const std::string & to_frame)
{
    static tf2_ros::TransformBroadcaster altitude_br;

    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = from_frame;
    transform_stamped.child_frame_id = to_frame;

    transform_stamped.transform.translation.x = 0.0d;
    transform_stamped.transform.translation.y = 0.0d;
    transform_stamped.transform.translation.z = altitude;

    transform_stamped.transform.rotation.x = 0.0d;
    transform_stamped.transform.rotation.y = 0.0d;
    transform_stamped.transform.rotation.z = 0.0d;
    transform_stamped.transform.rotation.w = 1.0d;

    altitude_br.sendTransform(transform_stamped);
}

void imuBroadcast(tf2::Quaternion q, const std::string & from_frame, const std::string & to_frame)
{
    static tf2_ros::TransformBroadcaster imu_br;

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    q.setRPY(roll, pitch, 0.0d);

    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = from_frame;
    transform_stamped.child_frame_id = to_frame;

    transform_stamped.transform.translation.x = 0.0d;
    transform_stamped.transform.translation.y = 0.0d;
    transform_stamped.transform.translation.z = 0.0d;

    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    imu_br.sendTransform(transform_stamped);
}

void lpeCallback(geometry_msgs::PoseStamped::ConstPtr const & msg, const std::string & from_frame, const std::string & to_frame)
{
    ROS_INFO_STREAM_THROTTLE_NAMED(1, "tf_broadcaster", "Using local position estimate for altitude");

    altitudeBroadcast(msg->pose.position.z, from_frame, to_frame);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr & msg, const std::string & from_frame, const std::string & to_frame)
{
    ROS_INFO_STREAM_THROTTLE_NAMED(1, "tf_broadcaster", "Using imu for pitch and roll");

    imu_publisher_timer.setPeriod(ros::Duration(1.0d), true);

    tf2::Quaternion q(msg->orientation.x,
                      msg->orientation.y,
                      msg->orientation.z,
                      msg->orientation.w);

    imuBroadcast(q, from_frame, to_frame);
}

void altitudeCallback(const sensor_msgs::Range::ConstPtr & msg, const std::string & from_frame, const std::string & to_frame, ros::Subscriber & lpe_sub)
{
    ROS_INFO_STREAM_THROTTLE_NAMED(1, "tf_broadcaster", "Using teraranger for altitude");

    // We do not need to use the local position estimate since we have the altitude
    lpe_sub.shutdown();

    altitudeBroadcast(msg->range, from_frame, to_frame);
}

void publishImuCallback(const ros::TimerEvent & event, const std::string & from_frame, const std::string & to_frame)
{
    ROS_FATAL_STREAM_NAMED("tf_broadcaster", "Publishing zero for pitch and roll");

    tf2::Quaternion q = tf2::Quaternion::getIdentity();

    imuBroadcast(q, from_frame, to_frame);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_broadcaster");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    std::string imu_topic = nh_priv.param<std::string>("imu_topic", "/mavros/imu/data");
    std::string altitude_topic = nh_priv.param<std::string>("altitude_topic", "/mavros/distance_sensor/teraranger");
    std::string lpe_topic = nh_priv.param<std::string>("lpe_topic", "/mavros/local_position/pose");

    std::string world_frame = nh_priv.param<std::string>("world_frame", "world");
    std::string map_frame = nh_priv.param<std::string>("map_frame", "map");
    std::string base_footprint_frame = nh_priv.param<std::string>("base_footprint_frame", "base_footprint");
    std::string base_stabilized_frame = nh_priv.param<std::string>("base_stabilized_frame", "base_stabilized");
    std::string base_link_frame = nh_priv.param<std::string>("base_link_frame", "base_link");
    std::string laser_link_frame = nh_priv.param<std::string>("laser_link_frame", "laser_link");

    imu_publisher_timer = nh.createTimer(ros::Duration(1.0d), boost::bind(&publishImuCallback, _1, base_stabilized_frame, base_link_frame));

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, boost::bind(&imuCallback, _1, base_stabilized_frame, base_link_frame));
    ros::Subscriber lpe_sub = nh.subscribe<geometry_msgs::PoseStamped>(lpe_topic, 1, boost::bind(&lpeCallback, _1, base_footprint_frame, base_stabilized_frame));
    ros::Subscriber altitude_sub = nh.subscribe<sensor_msgs::Range>(altitude_topic, 1, boost::bind(&altitudeCallback, _1, base_footprint_frame, base_stabilized_frame, lpe_sub));


    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped_1;

    static_transformStamped_1.header.stamp = ros::Time::now();
    static_transformStamped_1.header.frame_id = base_link_frame;
    static_transformStamped_1.child_frame_id = laser_link_frame;
    static_transformStamped_1.transform.translation.x = 0.0;
    static_transformStamped_1.transform.translation.y = 0.0;
    static_transformStamped_1.transform.translation.z = 0.1;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);

    static_transformStamped_1.transform.rotation.x = q.x();
    static_transformStamped_1.transform.rotation.y = q.y();
    static_transformStamped_1.transform.rotation.z = q.z();
    static_transformStamped_1.transform.rotation.w = q.w();
    static_broadcaster.sendTransform(static_transformStamped_1);

    geometry_msgs::TransformStamped static_transformStamped_2;

    static_transformStamped_2.header.stamp = ros::Time::now();
    static_transformStamped_2.header.frame_id = world_frame;
    static_transformStamped_2.child_frame_id = map_frame;
    static_transformStamped_2.transform.translation.x = 0.0;
    static_transformStamped_2.transform.translation.y = 0.0;
    static_transformStamped_2.transform.translation.z = 0.0;

    static_transformStamped_2.transform.rotation.x = q.x();
    static_transformStamped_2.transform.rotation.y = q.y();
    static_transformStamped_2.transform.rotation.z = q.z();
    static_transformStamped_2.transform.rotation.w = q.w();
    static_broadcaster.sendTransform(static_transformStamped_2);

    ros::spin();

    return 0;
}
