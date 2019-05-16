#include <humanoids_project/pick_bag.h>

PickBag::PickBag(ros::NodeHandle nh)
{
  rd_ = RobotDescription::getRobotDescription(nh);
  state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
  arm_controller_ = new ArmControlInterface(nh);
  gripper_controller_ = new GripperControlInterface(nh);
  taskspace_planner_ = new TaskspacePlanner(nh);
  wholebody_controller_ = new WholebodyControlInterface(nh);
}

PickBag::~PickBag()
{
  delete arm_controller_;
  delete gripper_controller_;
}

bool PickBag::executePointTrajectory(geometry_msgs::PoseStamped& temp_pose, std::string planning_group, moveit_msgs::RobotTrajectory& robot_traj)
{
  bool execution_status;
  execution_status = taskspace_planner_->getTrajectory(temp_pose, planning_group, robot_traj);
  if(execution_status)
    wholebody_controller_->executeTrajectory(robot_traj);

  time_for_exec = robot_traj.joint_trajectory.points.back().time_from_start;
  ros::Duration(time_for_exec).sleep();
  return execution_status;
}

bool PickBag::reachToManipulate(const geometry_msgs::PoseStamped& valve_center_pelvis, const float radius,
                                        RobotSide side)
{
  geometry_msgs::PoseStamped temp_pose;
  temp_pose = valve_center_pelvis;
  temp_pose.pose.position.z += radius;
  temp_pose.pose.position.x -= 0.05;

  moveit_msgs::RobotTrajectory robot_traj;
  std::string planning_group =
      (side == RobotSide::LEFT) ? TOUGH_COMMON_NAMES::LEFT_ARM_7DOF_GROUP : TOUGH_COMMON_NAMES::RIGHT_ARM_7DOF_GROUP;
  bool execution_status = executePointTrajectory(temp_pose, planning_group, robot_traj);

  if(execution_status)
  {
    robot_traj.joint_trajectory.points.resize(0);
    temp_pose.pose.position.x = valve_center_pelvis.pose.position.x;
    execution_status = executePointTrajectory(temp_pose, planning_group, robot_traj);
    if(!execution_status)
      ROS_ERROR("Error in Reaching Near the pose.");
  }
  else
    ROS_ERROR("Error in Reaching to the pose.");

  return execution_status;
}

double PickBag::rotateValve(const std::vector<geometry_msgs::Pose>& pose_vec, RobotSide side)
{
  moveit_msgs::RobotTrajectory robot_traj;
  std::string planning_group =
      (side == RobotSide::LEFT) ? TOUGH_COMMON_NAMES::LEFT_ARM_7DOF_GROUP : TOUGH_COMMON_NAMES::RIGHT_ARM_7DOF_GROUP;
  double frac = taskspace_planner_->getTrajFromCartPoints(pose_vec, planning_group, robot_traj);
  
  // This shold not be here. User should have control 
  //    over whether to execute trajectory or not. 
  // No hardcoded run.
   
  if(frac>0.0)
    wholebody_controller_->executeTrajectory(robot_traj); 
  return frac;
}

void PickBag::getCircularTrajectoryVector(std::vector<geometry_msgs::Pose> &pose_vec, const geometry_msgs::PoseStamped valve_center_pelvis, const float radius,
                                                  RobotSide side, const float rotations)
{
  geometry_msgs::Quaternion quat;
  quat.w = 1;
  geometry_msgs::Pose pose;
  pose.orientation = quat;
  pose.position.x = valve_center_pelvis.pose.position.x;
  float time_from_start = 0.0, time_step = 0.2;
  float theta = 0.0;
  
  for (int i = 10; i >= 1; theta = rotations/i--)
  {
    // pose.pose.position.z = valve_center_pelvis.pose.position.z + radius * cos(theta);
    // pose.pose.position.y = valve_center_pelvis.pose.position.y - radius * sin(theta);

    pose.position.z = valve_center_pelvis.pose.position.z;
    pose.position.y = valve_center_pelvis.pose.position.y;

    tf::Quaternion quat_tf = tf::createQuaternionFromRPY(theta, 0.0, 0.0);
    tf::quaternionTFToMsg(quat_tf, quat);

    pose.orientation = quat;
    time_from_start += time_step;
    pose_vec.push_back(pose);
  }
}

bool PickBag::retractHand(const geometry_msgs::PoseStamped& valve_center_pelvis, const float radius, RobotSide side,
                                  float time, const float rotations)
{
  geometry_msgs::PoseStamped temp_pose;
  temp_pose = valve_center_pelvis;
  // temp_pose.position.z -= radius;
  temp_pose.pose.position.x -= 0.05;

  moveit_msgs::RobotTrajectory robot_traj;
  std::string planning_group =
      (side == RobotSide::LEFT) ? TOUGH_COMMON_NAMES::LEFT_ARM_7DOF_GROUP : TOUGH_COMMON_NAMES::RIGHT_ARM_7DOF_GROUP;
  bool execution_status = executePointTrajectory(temp_pose, planning_group, robot_traj);
  return execution_status;
}

void PickBag::operateValve(const geometry_msgs::PoseStamped& valve_center, const float radius,
                                   RobotSide side, const float rotations)
{
  geometry_msgs::PoseStamped valve_center_pelvis;
  valve_center_pelvis.header.frame_id = rd_->getPelvisFrame();
  state_informer_->transformPose(valve_center.pose, valve_center_pelvis.pose, valve_center.header.frame_id,
                                 rd_->getPelvisFrame());

  float time = 1.5;
  bool to_continue = false;
  ROS_INFO("Opening Gripper.");
  gripper_controller_->openGripper(side);
  ROS_INFO("Reaching to manipulate.");
  to_continue = reachToManipulate(valve_center_pelvis, radius, side);

  ROS_INFO("Closing Gripper.");
  gripper_controller_->closeGripper(side);
  ros::Duration(time).sleep();

  if(to_continue)
  {
  std::vector<geometry_msgs::Pose> pose_vec;
  getCircularTrajectoryVector(pose_vec, valve_center_pelvis, radius);
  ROS_INFO("Rotating the valve");
  double frac = rotateValve(pose_vec, side);
  // Actual sleep time, not hardcoded.
  ros::Duration(time).sleep();

  if(frac>0.0)
  {
  ROS_INFO("Opening Grippers");
  gripper_controller_->openGripper(side);
  ros::Duration(time).sleep();
  ROS_INFO("Retracting Hand.");
  retractHand(valve_center_pelvis, radius, side, time, rotations);
  ros::Duration(time).sleep();
  }
  }
  else
    ROS_ERROR("Failed to reach goal!");

  ROS_INFO("Going Home");
  arm_controller_->moveToDefaultPose(side);
  return;
}