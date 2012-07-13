#include <string>
#include "stair_msgs/ButtonRequest.h"
#include "ros/ros.h"
using std::string;

ros::Publisher *g_button_pub = NULL;
string g_button_category, g_button_label;

void operate_connect(const ros::SingleSubscriberPublisher &pub)
{
  printf("operate connect\n");
  stair_msgs::ButtonRequest br;
  br.button_category = g_button_category;
  br.button_label = g_button_label;
  pub.publish(br);
}

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    printf("usage: begin_operate_button CATEGORY LABEL\n  e.g., ./begin_operate_button exterior up\n");
    return 1;
  }
  g_button_category = string(argv[1]);
  g_button_label = string(argv[2]);
  ros::init(argc, argv, "begin_operate_elevator");
  ros::NodeHandle n;
  // ros message to receive request to run operate_elevator sequence
  ros::Publisher button_pub = n.advertise<stair_msgs::ButtonRequest>("begin_operate_elevator", 1, operate_connect);
  g_button_pub = &button_pub;
  ros::spin();
  return 0;
}

