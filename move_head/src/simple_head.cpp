#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class RobotHead
{
private:
  PointHeadClient* point_head_client_;

public:
  //! Action client initialization 
  RobotHead()
  {
    //Initialize the client for the Action interface to the head controller
    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

    //wait for head controller action server to come up 
    while(!point_head_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the point_head_action server to come up");
    }
  }

  ~RobotHead()
  {
    delete point_head_client_;
  }

  //! Points the hi-def camera at a point in real-time
  void followPoint( std::string pointing_frame, std::string frame_id, double x, double y, double z)
  {
       //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;

    //we are pointing the high-def camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = pointing_frame;

    //take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.2);

    //and go no faster than 1 rad/s
    goal.max_velocity = 1.0;

    //send the goal
    point_head_client_->sendGoal(goal);

    //wait for it to get there
    //point_head_client_->waitForResult();

  }

  //! Points the high-def camera frame at a point in a given frame  
  void lookAt(std::string pointing_frame, std::string frame_id, double x, double y, double z)
  {
    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;

    //we are pointing the high-def camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = pointing_frame;

    //take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.5);

    //and go no faster than 1 rad/s
    goal.max_velocity = 1.0;

    //send the goal
    point_head_client_->sendGoal(goal);

    //wait for it to get there
    point_head_client_->waitForResult();
  }

  //! Shake the head from left to right n times  
  void shakeHead(int n)
  {
    int count = 0;
    while (ros::ok() && ++count <= n )
    {
      //Looks at a point forward (x=5m), slightly left (y=1m), and 1.2m up
      lookAt("narrow_stereo_link", "base_link", 5.0, 1.0, 1.2);

      //Looks at a point forward (x=5m), slightly right (y=-1m), and 1.2m up
      lookAt("narrow_stereo_link", "base_link", 5.0, -1.0, 1.2);
    }
    lookAt("narrow_stereo_link", "base_link", 5.0, 0.0, 1.2);
  }

  //! Nod the head up and down n times  
  void nodHead(int n)
  {
    int count = 0;
    while (ros::ok() && ++count <= n )
    {
      //Looks at a point forward (x=5m), slightly left (y=1m), and 1.2m up
      lookAt("narrow_stereo_link", "base_link", 5.0, 0.0, 0.0);

      //Looks at a point forward (x=5m), slightly right (y=-1m), and 1.2m up
      lookAt("narrow_stereo_link", "base_link", 5.0, 0.0, 1.4);
    }
    lookAt("narrow_stereo_link", "base_link", 5.0, 0.0, 1.2);
  }
  
};

int main(int argc, char** argv)
{
  std::string mode;
  //bool right_arm= true;
  std::string pointing_frame = "narrow_stereo_optical_frame";
  std::string follow_frame = "r_gripper_tool_frame";

  if(argc < 2 ) {
    ROS_ERROR("Unspecified mode, using 'shake'. Modes are follow, nod, shake, look");
    ROS_ERROR("Usage: simple_head [mode]"); 
    mode = "shake";
  }
  else mode = argv[1];
  
  if( ! strcmp(mode.c_str(), "follow") )
  {
    if( argc < 3 )
    //if(*argv[2] == 'r') right_arm = true;
    //else if (*argv[2] == 'l') right_arm = false;
      ROS_ERROR("For follow mode, must specify the frame to point and the frame to follow");
    pointing_frame = argv[2];
    follow_frame = argv[3];
  }
  //init the ROS node
  ros::init(argc, argv, "robot_driver");

  RobotHead head;

  if( !strcmp( mode.c_str() , "follow") )
  {
    ROS_INFO("Following %s", follow_frame.c_str());
    //std::string frame = (right_arm)?("r_gripper_tool_frame"):("l_gripper_tool_frame");
    while ( ros::ok() )
    {
      
      head.followPoint(pointing_frame.c_str(), follow_frame.c_str(), 0, 0, 0);
      usleep(50000);
    }
  }
  else if( !strcmp( mode.c_str() , "shake" ))
  {
    ROS_INFO("Shaking head!");
    head.shakeHead(3);
  }
  else if( !strcmp( mode.c_str() , "nod" ))
  {
    ROS_INFO("Nodding head!");
    head.nodHead(3);
  }
  else if( !strcmp( mode.c_str() , "look" ))
  {
    if(argc < 6)
    {
      ROS_ERROR("arguments: follow <pointing_frame> <target_frame> <x> <y> <z>");
      exit(-1);
    }
    ROS_INFO("Looking with %s at point in %s!", argv[2], argv[3]);
    head.lookAt(argv[2], argv[3], atof(argv[4]), atof(argv[5]), atof(argv[6]) );
  }


}
