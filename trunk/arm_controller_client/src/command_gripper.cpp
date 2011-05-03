#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{
private:
  GripperClient* gripper_client_;

public:
  //Action client initialization
  Gripper(char side){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    std::string client = (side == 'r')?("r_gripper_controller/gripper_action"):
                                       ("l_gripper_controller/gripper_action");
    gripper_client_ = new GripperClient(client.c_str(), true);
    
    //wait for the gripper action server to come up
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }
  }

  ~Gripper(){
    delete gripper_client_;
  }

  //Open the gripper
  void open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)

    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
//    gripper_client_->waitForResult();
//    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//      ROS_INFO("The gripper opened!");
//    else
//      ROS_INFO("The gripper failed to open.");
  }

  //Close the gripper
  void close(){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0;
    squeeze.command.max_effort = -1.0;  // Close strong

    ROS_INFO("Sending squeeze goal");
    gripper_client_->sendGoal(squeeze);
//    gripper_client_->waitForResult();
//    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//      ROS_INFO("The gripper closed!");
//    else
//      ROS_INFO("The gripper failed to close.");
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_gripper");
  
  if(argc < 3) ROS_ERROR("Usage: command_gripper <open/close> <r/l>");

  Gripper gripper(*argv[2]);

  if(!strcmp(argv[1], "open"))  gripper.open();
  if(!strcmp(argv[1], "close")) gripper.close();

  return 0;
}
