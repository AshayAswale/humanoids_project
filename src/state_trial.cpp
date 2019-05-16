#include <humanoids_project/state_trial.h>

StateMachine::StateMachine(ros::NodeHandle nh)
{
  manipulation = ManipulateValvePtr(new ManipulateValve(nh));
  bag_picker = new PickBag(nh);
  robot_walker = new WalkToManipulate(nh);

}

StateMachine::~StateMachine()
{
  // if(manipulation != nullptr)   delete manipulation;   
  if(bag_picker != nullptr)     delete bag_picker; 
  if(robot_walker != nullptr)   delete robot_walker;   
}

void StateMachine::runStateMachine()
{
  // if 
}