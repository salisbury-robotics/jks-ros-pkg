#include "katana_ik_bridge.h"

KatanaIKBridge::KatanaIKBridge() : ros::node("katana_ik_bridge",false)
{
  cout << "Initializing KatanaIKBridge." << endl;
}
	
KatanaIKBridge::~KatanaIKBridge()
{
    
}

/********************************Public methods***********************************/

bool KatanaIKBridge::getIKSolution(double x, double y, double z, double theta, double psi, double max_theta_dev, vector<double> &joints) 
{
  katana::KatanaIK::request req;
  katana::KatanaIK::response res;
  req.pose.x = x;
  req.pose.y = y;
  req.pose.z = z;
  req.theta = theta;
  req.psi = psi;
  req.max_theta_dev = max_theta_dev;
  
  cout << "Requesting katana_ik_calculate_service." << endl;
  if (!ros::service::call("katana_ik_calculate_service.", req, res)) {
    cout << "Error using ik_calculate_service." << endl;
    return (false);
  }
  cout << "Received joint solution: ";
  for (size_t i=0; i<res.solution.get_angles_size(); i++) {
    joints.push_back(res.solution.angles[i]); 
    cout << joints[i] << " ";
  }
  cout << endl;

	return (true);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv);
	KatanaIKBridge ik_bridge;

	ik_bridge.spin();
	ros::fini();
	return 0;
}
