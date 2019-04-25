#ifndef WALK_TO_MANIPULATE
#define WALK_TO_MANIPULATE

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <tough_footstep/robot_walker.h>

class WalkToManipulate
{
private:
  RobotWalker* walking_controller_;
  RobotDescription* rd_;
  RobotStateInformer* state_informer_;

public:
  WalkToManipulate(ros::NodeHandle nh);
  ~WalkToManipulate();
  void modifyGoalForWalking(geometry_msgs::PoseStamped& goal_walk);
  bool walkRobot(const geometry_msgs::PoseStamped& goal_walk);
  bool walkRobotForManipulation(const geometry_msgs::PoseStamped& goal_walk);
};

#endif