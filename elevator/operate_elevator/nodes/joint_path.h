#pragma once

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>
#include <time.h>

#include "katana/ik_hook.h"
//#include "ArmMotionPlanner/motionplanner.h"
#include "ros/node.h"

#define PI 3.14159265358979323846

using namespace std;

/*
 * This class computes a sequence of joint angles for the Katana arm to press buttons. 
 * End effector goal locations must be in the robot arm frame (in mm).
 */

class JointPath
{
 public:
  JointPath(IKHook *ik_hook);
  ~JointPath();
  /*
    bool callMotionPlanner(vector<vector<double> > &ptCloud, int sample_rate, 
    vector<double> src, vector<double> goal, double wristOrientation,
    vector<vector<double> > &jointAnglePath);
  */
  bool generateLinearPath(vector<double> eeFinal, double delta, vector<double> curPose, 
			  vector<double> curJoints, vector<vector<double> > &jointAngles); 
				 												 
 private:
  IKHook *ik_hook;
  //MotionPlanner *planner;
  /*
    void sendSceneToMP(vector<vector<double> > &ptCloud, int sample_rate);
    bool callPlanSingle(bool do_search, int soft_obstacles, 
    vector<vector<double> >&jointAnglePath);
  */
  void recordJointAngles(vector<double> joints,
			 vector<vector<double> > &jointAngles);

  static const double maxWristDev = 60*PI/180;
  string filepath;
};

JointPath::JointPath(IKHook *_ik_hook) : ik_hook(_ik_hook)
{
  filepath = ros::getPackagePath("operate_elevator");
  //planner = new MotionPlanner();
  //planner->InitRobot(string("katana450"));
}

JointPath::~JointPath() {
  //delete planner;
}

// Call motion planner to plan joint trajectory. Input point cloud must be in MPK
// frame.
/*
  bool JointPath::callMotionPlanner(vector<vector<double> > &ptCloud, int sample_rate, 
  vector<double> src, vector<double> goal, double wristOrientation, 
  vector<vector<double> > &jointAnglePath)
  {
  if (goal.size() != 3) {
  cout << "Error: Need 3D coordinates for goal location." << endl;
  }
  cout << "Sending point cloud to planner." << endl;
  sendSceneToMP(ptCloud, sample_rate);

  cout << "Sending source to planner: ";
  for (size_t k=0; k<src.size(); k++) {
  cout << src[k] << " ";
  }
  cout << endl;
  planner->SetSource(src);
  
  vector<double> dst;
  double psi = 0.0; // wrist rotation angle
  if (!ik_hook->ik_joint_solution(goal[0], goal[1], goal[2], wristOrientation, 
  psi, maxWristDev, dst)) return false;
	
  cout << "Sending destination to planner." << endl;
  planner->SetDest(dst);
	
  bool do_search = false;
  int soft_obstacles = 0;
  cout << "Calling Motion Planner PlanSingle" << endl;
  return (callPlanSingle(do_search, soft_obstacles, jointAnglePath));
  }
*/

// Compute joint angles to  move end effector along a line.
bool JointPath::generateLinearPath(vector<double> eeFinal, double delta, 
				   vector<double> curPose, vector<double> curJoints, 
				   vector<vector<double> > &jointAngles)
{
  cout << "Computing joint angles to move along a line...." << endl;
	
  jointAngles.clear();
  vector<double> eeInit;
  cout << "Initial end effector location: ";
  for (size_t i=0; i<eeFinal.size(); i++) {
    eeInit.push_back(curPose[i]);
    cout << eeInit[i] << " ";
  }
  cout << endl;

  // Compute unit vector along desired direction of linear motion.
  vector<double> unitVec;
  double distance = 0;
  for (size_t i=0; i<eeFinal.size(); i++) {
    unitVec.push_back(eeFinal[i] - eeInit[i]);
    distance += ((eeFinal[i] - eeInit[i]) * (eeFinal[i] - eeInit[i]));
  }	
  distance = sqrt(distance);
  for (unsigned int i=0; i<unitVec.size(); i++) {
    unitVec[i] = unitVec[i]/distance;
  }
  cout << "distance from cur to goal = " << distance << endl;

  // Number of joint configurations in linear path.
  int numMove = ceil(distance/delta);
  cout << "number of moves to goal = " << numMove << " with delta = " << delta << endl;
  delta = distance/numMove;
  
  vector<double> eeGoal = eeInit;
  vector<double> joints;
  bool success = false;
  double wristOrientation = curPose[4];
  double wristRotation = curJoints[4];
  double psi = 0;

  for (int k=0; k<numMove; k++) 
    {
      cout << "eeGoal: ";
      for (int j=0; j<3; j++) {	
	eeGoal[j] = eeGoal[j] + unitVec[j]*delta;
	cout << eeGoal[j] << " ";
      }	
      cout << endl;
						
      success = ik_hook->ik_joint_solution(eeGoal[0], eeGoal[1], eeGoal[2], 
					   wristOrientation, psi, maxWristDev, joints);
      if (!success) {
	cout << "No joint solution found for waypoint " << k << "." << endl;
	break;
      }
      joints[4] = wristRotation;
      recordJointAngles(joints, jointAngles);
      joints.clear();
    }	

  if (success) {
    cout << endl << "Following joint angles computed: " << endl;
    for (size_t i= 0; i<jointAngles.size(); i++) {
      for (unsigned int j = 0; j<jointAngles[i].size(); j++) {
	cout << jointAngles[i][j] << " ";
      }
      cout << endl;
    }
  } else {
    cout << "Error: No solution found for all waypoints." << endl << endl;
  }

  return success;
}

/*
  void JointPath::sendSceneToMP(vector<vector<double> > &ptCloud, int sample_rate)
  {	
  // Write ptCloud to required data format for motion planner.
  vector<vector<double>*> *pts = new vector<vector<double>*>;
  int numPts = ptCloud.size();

  for (int i=0; i<numPts; i++) {
  // Don't include region where arm is in view of camera.
  if (ptCloud[i][0] < 400) {
  vector<double> *tempv = new vector<double>;
  tempv->push_back(ptCloud[i][2]);
  tempv->push_back(ptCloud[i][3]);
  tempv->push_back(ptCloud[i][4]);
  pts->push_back(tempv);
  }
  }

  // Send scene to motion planner.
  bool use_static_model = false;
  planner->InitScene(pts, use_static_model, sample_rate);
  }
*/

void JointPath::recordJointAngles(vector<double> joints, 
				  vector<vector<double> > &jointAngles) 
{
  /*
    cout << "joints: ";
    for (size_t i=0; i<joints.size(); i++) {
    cout << joints[i] << " ";
    }
    cout << endl << "=============" << endl;
  */
  jointAngles.push_back(joints);				
}

/*
  bool JointPath::callPlanSingle(bool do_search, int soft_obstacles,
  vector<vector<double> >&jointAnglePath)
  {
  vector<vector<double>*> *path = planner->PlanSingle(do_search, soft_obstacles);

  if (path == NULL) {
  // param problems (src/dst not given), or no plan found
  cout << "Motion Planner could not find a path." << endl;  
  return (false);
  }

  cout << "Path received from planner: " << endl;
  for (size_t i=0; i<path->size(); i++) {
  vector<double> temp;
  for (size_t j=0; j<path->at(i)->size(); j++) {
  temp.push_back(path->at(i)->at(j));
  cout << temp[j] << " ";
  }
  cout << endl;
  jointAnglePath.push_back(temp);
  }

  cout << "DONE: Plan" << endl;
  return (true);
  }
*/

