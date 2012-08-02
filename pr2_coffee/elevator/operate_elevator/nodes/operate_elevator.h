#ifndef OPERATE_ELEVATOR_H
#define OPERATE_ELEVATOR_H

#include <iostream>
#include <vector>
#include <ctime>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "fit_plane.h"
#include "joint_path.h"
#include "warp_perspective.h"

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
   
  stair_msgs::Button button_2d_location;	// response from button classifier
  stair_msgs::ButtonRequest button_request; // request to locate a button
  stair_msgs::ButtonRequest begin_operate_elevator; // request to begin operate elevator sequence


  IplImage *cv_image;
  CvMat unwarp_matrix; 
  CvMat warp_matrix; 
  bool isCallPanel;    
  JointPath *armPathGen;
  FitPlane *planeFitter;
  WarpPerspective *pWarp;
  double button_x_modifier;
  double coord_modifier;
  bool useMP; // flag to indicate if motion planner should be used
  string timestamp;  // timestamp for images and point clouds
  bool bVerbose;
  vector<double> button3DCoord; // 3D coordinates of button in robot frame
  vector<double> button2DCoord; // image coordinates of button
  string operate_elevator_pkg_path;
  string panel_side;  // side of elevator door panel is on -- "left" or "right"
  string buttonCategory; // "call" or "interior"
  string buttonLabel;
  string srcImageFile, unwarpedImageFile;
  string cloudFilename;
  string TF_borg2ArmFile, TF_arm2MPKFile;
  bool classifier_done; // flag to indicate if button classifier is done running
  bool button_is_found; // flag to indicate if button was found by classifier
  bool arm_debug;
  bool borg_debug;
  bool wait_for_start_msg; // set false to begin operate elevator sequence when executable is launched
  static const double initWristOrientation = PI/2; //desired orientation of wrist for button-pressing
  static const int cloudSampleRate = 50; // pt cloud sample rate for motion planner
  static const int nJoints = 6; // number of joints on Katana arm (including gripper)
  double readyPosition[nJoints];   

  vector<vector<double> > ptCloud_sensor; // 3D pt cloud in sensor frame
  vector<vector<double> > ptCloud_MPK; // 3D pt cloud in MPK (motion planner) frame
  static const double searchDist = 5; // search region to find corresponding 3D point in cloud for 2D image coordinates
    
  // Coordinate transformations.
  vector<vector<double> > R_sensor2arm; // borg to arm rotation matrix 
  vector<double> T_sensor2arm;  // borg to arm translation (in arm frame)
  vector<vector<double> > R_arm2MPK;  // arm to motion planner (MPK) rotation 
  vector<double> T_arm2MPK; // arm to motion planner translation (in MPK frame)
    
  // Points from test image of checkerboard to use for mapping.
  const static float src_x[];
  const static float src_y[];
  const static float dst_x[];
  const static float dst_y[];
    
  bool initTransformations(); 
  bool getPointCloud();
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
  bool moveArmToButton(vector<double> MPgoal);
  bool reqLinearMoveService(vector<double> goalPose); 
  bool reqMoveArmService(vector<vector<double> > &joints); 
  bool reqGripperService(bool gripperOpen);
  bool reqCurJointAnglesService(vector<double> &joints);
  bool reqCurPoseService(vector<double> &pose); 
};

#endif
