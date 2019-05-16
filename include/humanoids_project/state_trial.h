#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <humanoids_project/manipulate_valve.h>
#include <humanoids_project/walk_to_manipulate.h>
#include <humanoids_project/pick_bag.h>

class StateMachine
{
private:
  ManipulateValvePtr manipulation;
  PickBag* bag_picker;
  WalkToManipulate* robot_walker;
  int retry_counter;

public:
  StateMachine(ros::NodeHandle nh);
  ~StateMachine();
  void runStateMachine();
};
