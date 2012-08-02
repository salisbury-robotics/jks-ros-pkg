#ifndef FIND_CALL_PANEL_BUTTON_H
#define FIND_CALL_PANEL_BUTTON_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <assert.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include "ros/node.h"
#include "deadreckon/DriveDeadReckon.h"
#include "stair_msgs/DetectOpenDoorRequest.h"
#include "stair_msgs/ElevatorDoorStatus.h"
#include "laser_scan/LaserScan.h"

#define PI 3.14159265

using namespace std;

class FindCallPanelButton
{
  public:
    stair_msgs::DetectOpenDoorRequest detect_open_door;
    stair_msgs::ElevatorDoorStatus open_door_status;

    FindCallPanelButton();
    ~FindCallPanelButton();
    void init();
    void shutdown();

  private:

    bool bVerbose;

};

#endif
