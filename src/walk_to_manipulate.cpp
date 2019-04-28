#include <humanoids_project/walk_to_manipulate.h>

WalkToManipulate::WalkToManipulate(ros::NodeHandle nh)
{
  walking_controller_ = new RobotWalker(nh, 0.5, 0.5);
  rd_ = RobotDescription::getRobotDescription(nh);
  state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
}

WalkToManipulate::~WalkToManipulate()
{
}

bool WalkToManipulate::walkRobot(const geometry_msgs::PoseStamped& goal_walk)
{
  tf::Quaternion rot_quat;
  tf::quaternionMsgToTF(goal_walk.pose.orientation, rot_quat);

  tf::Matrix3x3 matx(rot_quat);

  double roll, pitch, yaw;
  matx.getRPY(roll, pitch, yaw);

  geometry_msgs::Pose2D msg_out;
  msg_out.x = goal_walk.pose.position.x;
  msg_out.y = goal_walk.pose.position.y;
  msg_out.theta = yaw;

  return walking_controller_->walkToGoal(msg_out);
}

void WalkToManipulate::modifyGoalForWalking(geometry_msgs::PoseStamped& goal_walk)
{
  // double yaw = tf::getYaw(goal_walk.pose.orientation);
  goal_walk.pose.position.x = goal_walk.pose.position.x - 0.5;
  goal_walk.pose.position.y = goal_walk.pose.position.y + 0.3;

  // goal_walk.pose.position.x += (-0.3 * (sin(yaw)) - 0.5 * (cos(yaw)));
  // goal_walk.pose.position.y -= (-0.3 * (cos(yaw)) + 0.5 * (sin(yaw)));
}

bool WalkToManipulate::walkRobotForManipulation(const geometry_msgs::PoseStamped& valve_center_world)
{
  geometry_msgs::PoseStamped valve_center;
  ROS_INFO_STREAM("Walk Goal World: "<<valve_center_world);
  valve_center.header.frame_id = rd_->getPelvisFrame();
  state_informer_->transformPose(valve_center_world.pose, valve_center.pose, valve_center_world.header.frame_id,
                                 rd_->getPelvisFrame());
ROS_INFO_STREAM("Walk Goal Pelvis: "<<valve_center_world);
  ROS_INFO("Modifying Goal.");
  modifyGoalForWalking(valve_center);
  ROS_INFO_STREAM("Walking Robot "<<valve_center);
  return walkRobot(valve_center);
}