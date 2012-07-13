#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <string>
#include <vector>
#include "svlVision.h"

#include <math.h>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>  
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include "ros/node.h"
using namespace std;


struct statistics {
  double precision;
  double p_exist_G_det;
  double p_not_exist_G_det;
  double detection_size_mean;
  double detect_std[2];
  double gt_std[2];
};

const string BASE_PATH = ros::getPackagePath("find_call_panel_button") + "/";	   
const string DATA_PATH = BASE_PATH+"data/";
const string DEFAULT_MODEL="plastic_retrain";

const double MIN_RATIO = 0.00;
const double MAX_RATIO = 0.009;
const double FALSE_POSITIVE_THRESHOLD = 0.7;

const string TRAIN_BASE_PATH = DATA_PATH + "classify/";
const string SCRIPTS_PATH = BASE_PATH + "SvlClassify/scripts/";

const string FALSE_POSITIVES_PATH = TRAIN_BASE_PATH + "false_positives/";
const string FINAL_DETECTIONS_PATH = TRAIN_BASE_PATH + "final_detections/";
const string RAW_DETECTIONS_PATH = TRAIN_BASE_PATH + "raw_detections/";
const string GT_DETECTIONS_PATH = TRAIN_BASE_PATH + "ground_truth/";
const string MODELS_PATH = TRAIN_BASE_PATH + "models/";
const string TEST_IMAGES_PATH =  TRAIN_BASE_PATH + "test_images/";
const string TEST_SETS_PATH =  TRAIN_BASE_PATH + "test_sets/";
const string DETECTION_OVERLAYS_PATH =  TRAIN_BASE_PATH + "detection_overlays/";
const string TRAIN_DATA_PATH =  TRAIN_BASE_PATH + "train_data/";
#endif
