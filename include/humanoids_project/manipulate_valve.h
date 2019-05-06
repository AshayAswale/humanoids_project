#ifndef MANIPULATE_VALVE
#define MANIPULATE_VALVE

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/gripper_control_interface.h>
#include <tough_controller_interface/wholebody_control_interface.h>
#include <tough_moveit_planners/taskspace_planner.h>

class ManipulateValve
{
private:
  RobotDescription* rd_;
  RobotStateInformer* state_informer_;
  ArmControlInterface *arm_controller_;
  GripperControlInterface* gripper_controller_;
  TaskspacePlanner* taskspace_planner_;
  WholebodyControlInterface* wholebody_controller_;
  ros::Duration time_for_exec;

  // void insertDataToMultiArray();
  void getCircularTrajectoryVector(std::vector<geometry_msgs::Pose> &pose_vec, const geometry_msgs::PoseStamped valve_center, const float radius, RobotSide side = RobotSide::RIGHT, const float rotations = M_PI);

  bool executePointTrajectory(geometry_msgs::PoseStamped& temp_pose, std::string planning_group,
                              moveit_msgs::RobotTrajectory& robot_traj);

public:
  ManipulateValve(ros::NodeHandle nh);
  ~ManipulateValve();
  void operateValve(const geometry_msgs::PoseStamped& valve_center, const float radius, RobotSide side = RobotSide::RIGHT, const float rotations = M_PI);
  bool reachToManipulate(const geometry_msgs::PoseStamped& valve_center, const float radius, RobotSide side = RobotSide::RIGHT);
  bool retractHand(const geometry_msgs::PoseStamped& valve_center, const float radius, RobotSide side = RobotSide::RIGHT, float time = 1.0f, const float rotations = M_PI);
  double rotateValve(const std::vector<geometry_msgs::Pose>& pose_vec, RobotSide side);
};

#endif