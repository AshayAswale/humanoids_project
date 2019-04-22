#ifndef WALK_TO_MANIPULATE
#define WALK_TO_MANIPULATE

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

class WalkToManipulate
{
private:
    /* data */
public:
    WalkToManipulate(/* args */);
    ~WalkToManipulate();
    void modifyGoalForWalking(geometry_msgs::PoseStamped& goal_walk);
    bool walkRobot(const geometry_msgs::PoseStamped& goal_walk);
    bool walkRobotForManipulation(const geometry_msgs::PoseStamped& goal_walk);
};

#endif