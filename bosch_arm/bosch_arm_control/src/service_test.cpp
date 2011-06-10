#include "ros/ros.h"
#include "bosch_arm_srvs/SetJointAngles.h"
#include<vector>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_test");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<bosch_arm_srvs::SetJointAngles>("set_joint_angles");
  bosch_arm_srvs::SetJointAngles srv;
  double des[]={1,0,1,0};
  srv.request.joint_angles=std::vector<double>(des,des+4);
  client.call(srv);
  return 0;
}
