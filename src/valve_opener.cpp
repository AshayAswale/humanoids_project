#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <humanoids_project/manipulate_valve.h>
#include <humanoids_project/walk_to_manipulate.h>

geometry_msgs::PoseStamped valve_position;
bool goal_available_ = false;
RobotStateInformer* state_informer_;
RobotDescription* rd_;

void goalMessageCB(const geometry_msgs::PointStamped& goal)
{
  ROS_INFO_ONCE("Starting task");
  valve_position.pose.position = goal.point;
  valve_position.pose.orientation.w = 1;
  valve_position.header = goal.header;
  goal_available_ = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "valve_opener_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
  rd_ = RobotDescription::getRobotDescription(nh);

  ros::Subscriber subscribe = nh.subscribe("/clicked_point", 1000, goalMessageCB);
  ManipulateValve manipulate_valve(nh);
  WalkToManipulate walk_robot(nh);
  ROS_INFO("All configurations done!");

  ros::Rate loop_rate(10.0);
  while (ros::ok() && !goal_available_)
  {
    loop_rate.sleep();
  }

  // valve_position.pose.position.z = 1.0;
  geometry_msgs::PoseStamped valve_position_world = valve_position;
  // valve_position_world.header.frame_id = rd_->getWorldFrame();
  // state_informer_->transformPose(valve_position.pose, valve_position_world.pose, valve_position.header.frame_id,
  //                                rd_->getWorldFrame());

  // bool robot_walked = walk_robot.walkRobotForManipulation(valve_position_world);
bool robot_walked = true;
  if(robot_walked)
  {
  float radius = 0.15;
  manipulate_valve.operateValve(valve_position_world, radius);

  ros::Duration(5.0f).sleep();
  }
  return 0;
}
