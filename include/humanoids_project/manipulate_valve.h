#ifndef MANIPULATE_VALVE
#define MANIPULATE_VALVE

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/gripper_control_interface.h>

class ManipulateValve
{
private:
  RobotDescription* rd_;
  RobotStateInformer* state_informer_;
  ArmControlInterface::ArmTaskSpaceData arm_data_;
  ArmControlInterface *arm_controller_;
  std::vector<ArmControlInterface::ArmTaskSpaceData> arm_data_vec_;
  GripperControlInterface* gripper_controller_;

  std::vector<ArmControlInterface::ArmTaskSpaceData> getCircularTrajectoryVector(const geometry_msgs::PoseStamped valve_center, const float radius, RobotSide side = RobotSide::RIGHT, const float rotations = M_PI);
public:
  ManipulateValve(ros::NodeHandle nh);
  ~ManipulateValve();
  void operateValve(const geometry_msgs::PoseStamped& valve_center, const float radius, RobotSide side = RobotSide::RIGHT, const float rotations = M_PI);
  void reachToManipulate(const geometry_msgs::PoseStamped& valve_center, const float radius, RobotSide side = RobotSide::RIGHT, float time = 1.0f);
  void retractHand(const geometry_msgs::PoseStamped& valve_center, const float radius, RobotSide side = RobotSide::RIGHT, float time = 1.0f, const float rotations = M_PI);
  void rotateValve(const std::vector<ArmControlInterface::ArmTaskSpaceData>& arm_data_vec_);
};

#endif