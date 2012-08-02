#ifndef KATANA_IK_BRIDGE_H
#define KATANA_IK_BRIDGE_H

#include "ros/node.h"
#include "std_msgs/Point3DFloat32.h"
#include "std_msgs/ArmCSpace.h"
#include "katana/KatanaIK.h"

#include <iostream>

using std::cout;
using std::endl;
using std::vector;
using std_msgs::Point3DFloat32;
using std_msgs::ArmCSpace;

#define PI 3.14159265358979323846

class KatanaIKBridge : public ros::node 
{
	public:
		KatanaIKBridge();
		~KatanaIKBridge();
    bool getIKSolution(double x, double y, double z, double theta, double psi, double max_theta_dev, vector<double> &solution);
	
	private:
		
};

#endif
