
#ifndef SVLELEVATOROVERLAY_H
#define SVLELEVATOROVERLAY_H

#include "../definitions.h"
#include "CallButtonEnhancer.h"
class svlElevatorOverlayer
{
public:
  svlElevatorOverlayer(); // constructor
  ~svlElevatorOverlayer();  //deconstructor
  svlElevatorOverlayer(string& image_file, svlObject2dFrame& raw_detections,bool isCallPanel);
  bool saveOverlays(string detection_overlays_path);
  void processDetections();
  svlObject2dFrame getFinalDetections();
  svlObject2dSequence processed_detections;
private:
  bool isCallPanel;
  IplImage* image;
  string image_file;
  string image_name;
  svlObject2dFrame final_detections;
  svlObject2dFrame getThresholdDetections(svlObject2dFrame& detections);
  svlObject2dFrame getValidDetections(svlObject2dFrame& detections);
  svlObject2dFrame getPrunedDetections(svlObject2dFrame& detections);
  svlObject2dFrame getHighOverlapDetections(svlObject2dFrame& detections);
};


double getSizeMin(svlObject2dFrame& detections);
double getPrMax(svlObject2dFrame& detections);
double getSizeMean(svlObject2dFrame& detections);
double getSizeSTD(svlObject2dFrame& detections);
double getPrMean(svlObject2dFrame& detections);
double getPrSTD(svlObject2dFrame& detections);
bool isButtonDetection(svlObject2d& detect, svlObject2d& gt);
bool  saveFalsePositives(svlObject2dFrame& detections,svlObject2dFrame& gt, string& image_name,string& neg_dir);
bool createNegatives(svlObject2dFrame& gt, string& image_name,string& neg_dir,int numNegatives);
int bwareaopen(IplImage *image, int size);



#endif
