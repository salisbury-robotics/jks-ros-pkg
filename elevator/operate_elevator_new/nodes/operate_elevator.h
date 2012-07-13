#ifndef OPERATE_ELEVATOR_H
#define OPERATE_ELEVATOR_H

#include <iostream>
#include <sstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include <ctype.h>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "fit_plane.h"
#include "joint_path.h"

#include "ros/node.h"
#include "stair_msgs/Button.h"
#include "stair_msgs/AllButtons.h"
#include "stair_msgs/ButtonRequest.h"
#include "katana450/KatanaIK.h"
#include "katana450/KatanaPose.h"
#include "katana450/StringString.h"
#include "katana450/ArmCSpaceString.h"
#include "katana450/StringArmCSpace.h"
#include "katana450/ArmCSpaceSeqString.h"
#include "katana450/UInt32String.h"
#include "katana/ik_hook.h"
#include "borg/Filepaths.h"
#include "elevator/definitions.h"

#define PI 3.14159265358979323846


using namespace std;

struct compare_row_functor {
  bool operator() (const vector<double> first, const vector<double> second)
  {
    return (first[0] < second[0]);
  }
};

struct compare_col_functor {
  bool operator() (const vector<double> first, const vector<double> second)
  {
    return (first[1] < second[1]);
  }
};


class OperateElevator : public IKHook
{
 public:
    
  OperateElevator();
  ~OperateElevator();
  void init();
  void shutdown();
  void run();
  bool executeFindButtonSequence();
  bool executePressButton();
  void handleReceivedButtonData();

 private:
   
  stair_msgs::Button button_location;	// response from button classifier
  stair_msgs::ButtonRequest button_request; // request to locate a button
  stair_msgs::ButtonRequest begin_operate_elevator; // request to begin operate elevator sequence

  bool ONLINE_PROCESS;
  bool isCallPanel;    
  string panel_side;  // side of elevator door panel is on -- "left" or "right"
  string buttonCategory; // "call" or "interior"
  string buttonLabel;
  JointPath *armPathGen;
  FitPlane *planeFitter;
  string timestamp;  // timestamp for images and point clouds
  bool bVerbose;
  vector<double> button3DCoord; // 3D coordinates of button in robot frame
  vector<double> button2DCoord; // image coordinates of button
  string operate_elevator_pkg_path;
  string srcImageFile, srcImageFile_canon;
  string cloudFilename;
  string TF_borg2ArmFile;
  bool classifier_done; // flag to indicate if button classifier is done running
  bool button_is_found; // flag to indicate if button was found by classifier
  bool no_arm_move, use_fake_arm_data;
  bool use_fake_images;
  double fake_arm_data[3];
  static const double initWristOrientation = PI/2; //desired orientation of wrist for button-pressing
  static const int nJoints = 6; // number of joints on Katana arm (including gripper)
  double readyPosition[6];
  bool useMP;

  vector<vector<double> > ptCloud_sensor; // 3D pt cloud in sensor frame
  static const double searchDist = 5; // search region to find corresponding 3D point in cloud for 2D image coordinates
    
  // Coordinate transformations.
  vector<vector<double> > R_sensor2arm; // borg to arm rotation matrix 
  vector<double> T_sensor2arm;  // borg to arm translation (in arm frame)
    
  bool initTransformations(); 
  bool getPointCloud();
  bool getHighResImage();
  bool saveCloudToFile(vector<vector<double> > &cloud, string stamp);
  void findButton();
  void findCallPanelButton();
  bool loadDebugData();
  bool getButton3DCoord();
  bool getPlaneNormal(vector<double> &normal, vector<double> &centroid, 
     double windowSize);
  string getTimeStamp();

  bool pressButton(vector<double> goal);
  bool ik_joint_solution(double x, double y, double z, double theta_init,
			 double psi, double max_theta_dev, vector<double> &solution);
  bool moveSingleJoint(int joint, int degree);    
  bool moveRobotJoint(double degrees[]);    
  bool moveArmToHome();
  bool moveArmForCamera();
  bool moveArmBackToHome();
  bool moveArmToButton(vector<double> goal);
  bool reqLinearMoveService(vector<double> goalPose); 
  bool reqMoveArmService(vector<vector<double> > &joints); 
  bool reqGripperService(bool gripperOpen);
  bool reqCurJointAnglesService(vector<double> &joints);
  bool reqCurPoseService(vector<double> &pose); 
};

#endif
