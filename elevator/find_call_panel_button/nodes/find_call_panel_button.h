#ifndef FIND_CALL_PANEL_BUTTON_H
#define FIND_CALL_PANEL_BUTTON_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <assert.h>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "../SvlClassify/svlElevatorClassifier.h"
#include "../SvlClassify/svlElevatorOverlayer.h"
#include "ros/node.h"
#include "stair_msgs/Button.h"
#include "stair_msgs/ButtonRequest.h"

#define PI 3.14159265

using namespace std;

class FindCallPanelButton
{
 public:
  stair_msgs::Button single_button;
  stair_msgs::ButtonRequest button_request;

  FindCallPanelButton();
  ~FindCallPanelButton();
  void init();
  void shutdown();
  void findButtons();

 private:
  svlObject2dFrame svlDetections;
  string image_name;
  string image_file;
  bool bVerbose;

};

#endif
