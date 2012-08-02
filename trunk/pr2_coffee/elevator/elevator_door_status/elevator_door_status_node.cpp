#include "ros/ros.h"
#include "laser_scan/LaserScan.h"
#include "elevator_door_status.h"
#include "world_db/Query.h"
#include <string>
using std::string;

int g_num_caught = 0, g_num_to_skip = 1;
string g_cloudfile;

ElevatorDoorStatus *g_detector = NULL;
ros::Publisher *g_open_door_pub = NULL;
ElevatorDoorStatus::DoorState g_ds = ElevatorDoorStatus::ALL_CLOSED;
bool g_at_panel = false;

void scan_cb(const laser_scan::LaserScan::ConstPtr &msg)
{
  if (!g_at_panel)
    return;
  ElevatorDoorStatus::DoorState ds = g_detector->detect(msg->ranges);
  if (ds != g_ds)
  {
    g_ds = ds;
    const string DOOR_CLOSED = "<door state='closed' />";
    const string DOOR_OPEN   = "<door state='open' />";
    string left_door_state = DOOR_CLOSED, right_door_state = DOOR_CLOSED;
    if (ds == ElevatorDoorStatus::RIGHT_OPEN)
      right_door_state = DOOR_OPEN;
    else if (ds == ElevatorDoorStatus::LEFT_OPEN)
      left_door_state = DOOR_OPEN;
    world_db::Query::Request req;
    world_db::Query::Response res;
    req.transaction.push_back(string("update replace $x in doc('worlddb')/world/map/place[@name=\"first floor left elevator\"]/door with ")+left_door_state);
    req.transaction.push_back(string("update replace $x in doc('worlddb')/world/map/place[@name=\"first floor right elevator\"]/door with ")+right_door_state);
    if (!ros::service::call("world_db", req, res))
      ROS_ERROR("query fail");
  }
}

int main(int argc, char **argv)
{
  ElevatorDoorStatus eds;
  g_detector = &eds;
  ros::init(argc, argv, "elevator_door_status");
  ros::NodeHandle n;
  ros::Subscriber laser_sub = n.subscribe<laser_scan::LaserScan>("base_scan", 1, scan_cb);
  bool active = false;
  ros::Time last_position_time = ros::Time::now();
  while (n.ok())
  {
    if (ros::Time::now() - last_position_time > ros::Duration(1.0))
    {
      // see if we are in a position to observe the door
      world_db::Query::Request req;
      world_db::Query::Response res;
      req.transaction.push_back("import module namespace math = \"math\";\n"
          "let $robot_pos := doc('worlddb')/world/robot/position\n"
          "let $tgt_pos   := doc('worlddb')/world/map/place[@name=\"first floor elevator call panel\"]/position\n"
          "return (math:dist_sqr_2d($robot_pos, $tgt_pos) < math:square(0.2))\n"
          "  and  (abs($robot_pos/@theta - $tgt_pos/@theta) < number(0.3))");
//      req.transaction.push_back("let $pos := doc('worlddb')/world/robot/position\nreturn concat($pos/@x,' ',$pos/@y,' ',$pos/@theta)");
      if (!ros::service::call("world_db", req, res) ||
          !res.result.size())
        ROS_ERROR("query failed to decide if the robot is at the call panel");
      else
      {
        ROS_INFO("at call panel: %s", res.result[0].c_str());
        g_at_panel = (res.result[0] == string("true"));
      }
      last_position_time = ros::Time::now();
    }
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  return 0;
}

