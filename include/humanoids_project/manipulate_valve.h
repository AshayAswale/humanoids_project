#ifndef MANIPULATE_VALVE
#define MANIPULATE_VALVE

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

class ManipulateValve
{
private:
    /* data */
public:
    ManipulateValve(/* args */);
    ~ManipulateValve();
    void reachToManipulate(const geometry_msgs::PoseStamped& goal);
    void followCircularTrajectory(const float radius, const geometry_msgs::PoseStamped& center, const float rotations = M_PI);
};

#endif