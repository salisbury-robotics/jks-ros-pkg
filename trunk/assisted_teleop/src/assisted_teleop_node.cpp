#include <QApplication>
#include <ros/ros.h>
#include <assisted_teleop/assisted_teleop.h>

//using namespace moveit_visualization_ros;
using namespace assisted_teleop;

AssistedTeleop* at_;

void sigHandler(int x) {
  ROS_INFO_STREAM("Getting sig handler");
  delete at_;
  exit(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_visualizer", ros::init_options::NoSigintHandler);

  // CAN'T SPIN AS RVIZ ALREADY IS!!!!
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);

  QApplication app( argc, argv );

  at_ = new AssistedTeleop();
  app.exec();

}
