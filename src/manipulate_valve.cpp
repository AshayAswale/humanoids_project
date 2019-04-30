#include <humanoids_project/manipulate_valve.h>

ManipulateValve::ManipulateValve(ros::NodeHandle nh)
{
  rd_ = RobotDescription::getRobotDescription(nh);
  state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
  arm_controller_ = new ArmControlInterface(nh);
  gripper_controller_ = new GripperControlInterface(nh);
}

ManipulateValve::~ManipulateValve()
{
  delete arm_controller_;
  delete gripper_controller_;
}
void modifyPoseForIHMCFrame(geometry_msgs::PoseStamped& valve_center)
{
  valve_center.pose.position.x -= 0.100;
  valve_center.pose.position.y += 0.017;
  valve_center.pose.position.z -= 0.087;
}
void ManipulateValve::reachToManipulate(const geometry_msgs::PoseStamped& valve_center, const float radius,
                                        RobotSide side, float time)
{
  geometry_msgs::Pose temp_pose;
  temp_pose = valve_center.pose;
  // temp_pose.position.z += radius;
  temp_pose.position.x -= 0.05;

  arm_controller_->moveArmInTaskSpace(side, temp_pose, time);
  ros::Duration(time).sleep();
  // track frame to wait till it is moving
  // check the final position and return true or false

  temp_pose.position.x = valve_center.pose.position.x;
  arm_controller_->moveArmInTaskSpace(side, temp_pose, time);
  ros::Duration(time).sleep();
}

void ManipulateValve::rotateValve(const std::vector<ArmControlInterface::ArmTaskSpaceData>& arm_data_vec_)
{
  arm_controller_->moveArmInTaskSpace(arm_data_vec_);
}

void ManipulateValve::rotateValve(int joint_number, float target_angle, RobotSide side, float time)
{
  arm_controller_->moveArmJoint(side, joint_number, target_angle, time);
}

void ManipulateValve::getCircularTrajectoryVector(std::vector<ArmControlInterface::ArmTaskSpaceData>& arm_data_vec_,
                                                  const geometry_msgs::PoseStamped valve_center, const float radius,
                                                  RobotSide side, const float rotations)
{
  geometry_msgs::Quaternion quat;
  quat.w = 1;
  arm_data_.pose.orientation = quat;
  arm_data_.side = side;
  arm_data_.pose.position.x = valve_center.pose.position.x;
  float time_from_start = 0.0, time_step = 0.2;

  // equal spacing
  for (double theta = 0; theta <= rotations; theta += 0.2)
  {
    // arm_data_.pose.position.z = valve_center.pose.position.z + radius * cos(theta);
    // arm_data_.pose.position.y = valve_center.pose.position.y - radius * sin(theta);

    arm_data_.pose.position.z = valve_center.pose.position.z;
    arm_data_.pose.position.y = valve_center.pose.position.y;

    tf::Quaternion quat_tf = tf::createQuaternionFromRPY(theta, 0.0, 0.0);
    tf::quaternionTFToMsg(quat_tf, quat);

    arm_data_.pose.orientation = quat;
    time_from_start += time_step;
    arm_data_.time = time_from_start;
    arm_data_vec_.push_back(arm_data_);
  }

  float theta = rotations;
  // arm_data_.pose.position.z = valve_center.pose.position.z + radius * cos(theta);
  // arm_data_.pose.position.y = valve_center.pose.position.y - radius * sin(theta);

  arm_data_.pose.position.z = valve_center.pose.position.z;
  arm_data_.pose.position.y = valve_center.pose.position.y;

  tf::Quaternion quat_tf = tf::createQuaternionFromRPY(theta, 0, 0);
  tf::quaternionTFToMsg(quat_tf, quat);

  arm_data_.pose.orientation = quat;
  time_from_start += time_step;
  arm_data_.time = time_from_start;
  arm_data_vec_.push_back(arm_data_);
}

void ManipulateValve::retractHand(const geometry_msgs::PoseStamped& valve_center, const float radius, RobotSide side,
                                  float time, const float rotations)
{
  geometry_msgs::Pose temp_pose;
  temp_pose = valve_center.pose;
  // temp_pose.position.z -= radius;
  temp_pose.position.x -= 0.05;

  tf::Quaternion quat_tf = tf::createQuaternionFromRPY(rotations, 0, 0);
  tf::quaternionTFToMsg(quat_tf, temp_pose.orientation);

  arm_controller_->moveArmInTaskSpace(side, temp_pose, time);
}

void ManipulateValve::operateValve(const geometry_msgs::PoseStamped& valve_center_world, const float radius,
                                   RobotSide side, const float rotations)
{
  geometry_msgs::PoseStamped valve_center;
  valve_center.header.frame_id = rd_->getPelvisFrame();
  state_informer_->transformPose(valve_center_world.pose, valve_center.pose, valve_center_world.header.frame_id,
                                 rd_->getPelvisFrame());

  modifyPoseForIHMCFrame(valve_center);

  float time = 1.5;
  ROS_INFO("Opening Grippers.");
  gripper_controller_->openGripper(side);
  ROS_INFO("Reaching to manipulate.");
  reachToManipulate(valve_center, radius, side, time);

  // ROS_INFO_STREAM("Closing Grippers  \n"<<hand_msg_close);
  gripper_controller_->closeFingers(side);
  // right_hand_msg_pub->publish(hand_msg_close);
  ros::Duration(time).sleep();

  std::vector<ArmControlInterface::ArmTaskSpaceData> arm_data_vec_;
  getCircularTrajectoryVector(arm_data_vec_, valve_center, radius);
  ROS_INFO("Rotating the valve");
  rotateValve(arm_data_vec_);
  float time_for_traj = arm_data_vec_.back().time;
  ros::Duration(time_for_traj).sleep();

  // ROS_INFO("Rotating Valve");
  // rotateValve(6, M_PI);
  // ros::Duration(2.0).sleep();

  ROS_INFO("Opening Grippers");
  gripper_controller_->openGripper(side);
  ros::Duration(time).sleep();
  ROS_INFO("Retracting Hand.");
  retractHand(valve_center, radius, side, time, rotations);
  ros::Duration(time).sleep();
  ROS_INFO("Going Home");
  arm_controller_->moveToDefaultPose(side);
}