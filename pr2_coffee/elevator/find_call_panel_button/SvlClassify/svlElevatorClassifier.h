#ifndef SVLELEVATORCLASSIFIER_H
#define SVLELEVATORCLASSIFIER_H

#include "../definitions.h"
#include "svlElevatorOverlayer.h"

class svlElevatorClassifier
{
 public:

  svlElevatorClassifier(string& file, string model, bool isCallPanel = false,  bool save_overlays=true,string gt_file = "");
  ~svlElevatorClassifier(){};
  
  void classify();
  svlObject2dFrame processDetections(svlObject2dFrame& detections);
  svlObject2dFrame classifications;
  bool createFalsePositives();
 private:
  bool isCallPanel;
  string overlays_path;
  string test_image_file;
  string test_image_name;
  string gt_file;
  string model;
  bool save_overlays;
};

#endif
