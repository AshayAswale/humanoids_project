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
  // ros::Publisher *right_hand_msg_pub;
  // std::vector<double> close_config{1.45, 1.45, 1.45, 0, 0, 0, 0, 0, 0};
  // std_msgs::Float64MultiArray hand_msg_close;

  // void insertDataToMultiArray();
  void getCircularTrajectoryVector(std::vector<ArmControlInterface::ArmTaskSpaceData>& arm_data_vec_, const geometry_msgs::PoseStamped valve_center, const float radius, RobotSide side = RobotSide::RIGHT, const float rotations = M_PI);
public:
  ManipulateValve(ros::NodeHandle nh);
  ~ManipulateValve();
  void operateValve(const geometry_msgs::PoseStamped& valve_center, const float radius, RobotSide side = RobotSide::RIGHT, const float rotations = M_PI);
  void reachToManipulate(const geometry_msgs::PoseStamped& valve_center, const float radius, RobotSide side = RobotSide::RIGHT, float time = 1.0f);
  void retractHand(const geometry_msgs::PoseStamped& valve_center, const float radius, RobotSide side = RobotSide::RIGHT, float time = 1.0f, const float rotations = M_PI);
  void rotateValve(const std::vector<ArmControlInterface::ArmTaskSpaceData>& arm_data_vec_);
  void rotateValve(int joint_number,  float target_angle, RobotSide side = RobotSide::RIGHT, float time = 2.0f);
};

#endif