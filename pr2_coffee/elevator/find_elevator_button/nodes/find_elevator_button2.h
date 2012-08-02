#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include "../lib_find_buttons/button_struct.h"
#include "ros/node.h"
#include "stair_msgs/Button.h"
#include "stair_msgs/AllButtons.h"
#include "stair_msgs/ButtonRequest.h"

#include "../lib_find_buttons/EMGridFit/EMGridFit.h"
#include "../lib_find_buttons/HMMLabel/HMMLabel.h"
#include "../lib_find_buttons/LabelClassify/label_classify.h"
#include "../lib_find_buttons/SvlClassify/svl_classify.h"
#include "../lib_find_buttons/SvlClassify/svl_overlay.h"

#define WINDOW_NAME "Result"

using namespace std;

class FindElevatorButtons
{
  public:
    stair_msgs::Button single_button;
    stair_msgs::AllButtons all_buttons;
    stair_msgs::ButtonRequest button_request;
    void init();
    void shutdown();
    void findButtons();

  private:

    // grid parameters (gx, gy, dx, dy, nrows, ncols)
    vector<grid_param_struct> gridParams; 
	
    // final output of location and label for all buttons from EM
//    vector<vector<button_struct> > finalDetections;

	 // observations about buttons 
	 vector< vector<button_struct> > obsDetections;

    // final buttons locations with labels 
	 vector<button_struct> labeledButtons;
    
    // ADD instance of class to run svl classifier and prune results
    EMGridFit *emg;
    HMMLabel *hmml;
    
//    const int nPixels = 307200;
    vector<CvPoint> gridPoints;
    int loadImageAs;
    IplImage* source_image;
    string imageFile, imageName;
    string detectionsFile;
    string requestedButton;
    string find_button_pkg_path;
    bool bVerbose;
    svlObject2d object_;
    svlObject2dFrame svlDetections;
    svlObject2dFrame EMCandidates;

    void getSvlDetections();  
    void fitGridToDetections(int nPixels);
    void generateGridPoints();
    void labelDetections();
    //    void labelDetections(vector<grid_param_struct> grid_params, vector< vector<button_struct> > obs_detections);
    void labelHMM();
    // Display results on image and save to file.
    void displayResults();
	 void getButtonInfo();

	 bool getDir(string dir, vector<string> &files);
};
