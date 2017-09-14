#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <path_planner/PathPlannerAction.h>

#include <actionlib/server/simple_action_server.h>

#include <move_base_msgs/MoveBaseAction.h>

namespace navigation_planner
{
    class NavigationPlanner
    {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;

        ros::Subscriber map_sub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber setpoint_sub_;

        ros::Publisher path_pub_;

        actionlib::SimpleActionClient<path_planner::PathPlannerAction> ac_;

        actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;


        double frequency_;

    public:
        NavigationPlanner(ros::NodeHandle nh, ros::NodeHandle nh_priv);

    private:
        void setpointCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);

        void executeCallback(const move_base_msgs::MoveBaseGoal::ConstPtr & goal);

        void doneCallback(const actionlib::SimpleClientGoalState & state,
                          const path_planner::PathPlannerResult::ConstPtr & result);

        void activeCallback();

        void feedbackCallback(const path_planner::PathPlannerFeedback::ConstPtr & feedback);

        void initParams(const ros::NodeHandle & nh);

    };
}
