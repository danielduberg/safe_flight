#include <navigation_planner/navigation_planner.h>

#include <nav_msgs/Path.h>

namespace navigation_planner
{
    NavigationPlanner::NavigationPlanner(ros::NodeHandle nh, ros::NodeHandle nh_priv)
        : nh_(nh)
        , nh_priv_(nh_priv)
        , ac_("path_planner", true)
        , as_(nh_, "move_base", boost::bind(&NavigationPlanner::executeCallback, this, _1), false)
        , setpoint_sub_(nh_.subscribe("/move_base_simple/goal", 1, &NavigationPlanner::setpointCallback, this))
        , path_pub_(nh.advertise<nav_msgs::Path>("path", 1))
    {
        initParams(nh_priv_);

        // Wait for the action server to start
        ac_.waitForServer();    // Will wait for infinite time

        //as_.start();
    }

    void NavigationPlanner::setpointCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
    {
        ac_.cancelGoalsAtAndBeforeTime(ros::Time::now());

        // Send a goal to the action
        path_planner::PathPlannerGoal goal;
        goal.goal = *msg;
        ac_.sendGoal(goal,
                     boost::bind(&NavigationPlanner::doneCallback, this, _1, _2),
                     boost::bind(&NavigationPlanner::activeCallback, this),
                     boost::bind(&NavigationPlanner::feedbackCallback, this, _1));
    }

    void NavigationPlanner::doneCallback(const actionlib::SimpleClientGoalState & state,
                      const path_planner::PathPlannerResult::ConstPtr & result)
    {
        ROS_ERROR_STREAM_THROTTLE_NAMED(100, "safe_fligh/navigation_planner", "Done");
    }

    void NavigationPlanner::activeCallback()
    {

    }

    void NavigationPlanner::feedbackCallback(const path_planner::PathPlannerFeedback::ConstPtr & feedback)
    {
        path_pub_.publish(feedback->path);
    }




    void NavigationPlanner::executeCallback(const move_base_msgs::MoveBaseGoal::ConstPtr & goal)
    {
        path_planner::PathPlannerGoal path_goal;
        path_goal.goal = goal->target_pose;

        ac_.sendGoal(path_goal,
                     boost::bind(&NavigationPlanner::doneCallback, this, _1, _2),
                     boost::bind(&NavigationPlanner::activeCallback, this),
                     boost::bind(&NavigationPlanner::feedbackCallback, this, _1));

        while (ros::ok())
        {
            if (as_.isPreemptRequested())
            {
                ROS_INFO_THROTTLE_NAMED(1, "Navigation Planner", "Preempted");

                as_.setPreempted();

                break;
            }

            if (ac_.waitForResult(ros::Duration(1.0d / frequency_)))
            {
                ROS_INFO_THROTTLE_NAMED(1, "Navigation Planner", "Successful");

                as_.setSucceeded();

                break;
            }
        }
    }

    void NavigationPlanner::initParams(const ros::NodeHandle & nh)
    {
        nh.param("frequency", frequency_, 10.0d);
    }

}
