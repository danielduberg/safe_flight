#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
	static tf::TransformBroadcaster br;

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
	transform.setRotation(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot"));
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