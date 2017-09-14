#pragma once

#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <path_planner/path_planner.h>

namespace path_planner
{
    
    class PPNodelet : public nodelet::Nodelet
    {

    private:
        PathPlanner * PP;

    private:
        virtual void onInit();

    };
}
