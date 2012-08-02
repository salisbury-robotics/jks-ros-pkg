#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include <assert.h>

#include "ros/node.h"
#include "stair_msgs/Button.h"
#include "stair_msgs/AllButtons.h"
#include "stair_msgs/ButtonRequest.h"

#include "map_pixels.h"
#include "main/ElevClassifier.h"


using namespace std;

class FindElevatorButtons
{
  public:
    stair_msgs::Button single_button;
    stair_msgs::AllButtons all_buttons;
    stair_msgs::ButtonRequest button_request;
    void init();
    bool loadCloud(vector<vector<double> > & cloud);
    void findButtons();

  private:

    ElevClassifier * classifier;
    ElevPanel * panel;
    ElevDataFrame swodData;
    ElevDataFrame emData;
    ElevDataFrame new_emData;
    ElevDataFrame ocrData;
    ElevDataFrame hmmData;
    ElevClassification buttons;

    string find_button_pkg_path;
    string image_file, cloud_file;
    string button_category, button_label;

    string unwarpedImageFile;
    bool ONLINE_PROCESS;
};
