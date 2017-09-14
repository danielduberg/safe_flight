#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

double getYaw(const geometry_msgs::Quaternion & orientation)
{
    double roll, pitch, yaw;

    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 m(q);
    m.getEulerYPR(yaw, pitch, roll);

    return yaw;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    static tf2_ros::TransformBroadcaster br;

    geometry_msgs::TransformStamped transform;

    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "world";

    transform.transform.translation.x = msg->pose.position.x;
    transform.transform.translation.y = msg->pose.position.y;
    transform.transform.translation.z = msg->pose.position.z;

    transform.transform.rotation.x = msg->pose.orientation.x;
    transform.transform.rotation.y = msg->pose.orientation.y;
    transform.transform.rotation.z = msg->pose.orientation.z;
    transform.transform.rotation.w = msg->pose.orientation.w;

    transform.child_frame_id = "robot";
    br.sendTransform(transform);



    tf2::Quaternion q;
    q.setRPY(0, 0, getYaw(msg->pose.orientation));

    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    transform.child_frame_id = "robot_yaw";
    br.sendTransform(transform);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_world_robot_broadcaster");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, poseCallback);

    ros::spin();

    return 0;
}
