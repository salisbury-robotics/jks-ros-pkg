#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include "elevdata/ElevData.h"
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include "warp_perspective.h"

using namespace std;

const double w_scale_factor = 10;
const double h_scale_factor = 10;
const double img_width = 3456;
const double img_height = 2592;

//Extrinsic parameters (position of canon camera frame wrt flea frame):
const double T[3] = {-131.343650419688, -33.1208749103514, -37.931353990237};  // mm
const double R[3][3] = {{0.952942582514526,        0.0912651046099517,        -0.289086691342028},
                        {0.0523166398657014,          0.88978888972131,         0.453363762250227},
                        {0.298602417310558,        -0.447153678738141,         0.843143038853573}};

//Intrinsic parameters of canon camera:
double fc[2] = {3570.75689,   3582.80926};
double cc[2] = {1721.29735,   1342.12499};
double alpha = 0;
double kc[5] = {-0.18830,   0.18230,   -0.00486,   0.00036,  0.00000};


struct compare_row_functor {
  bool operator() (const vector<double> first, const vector<double> second)
  {
    return (first[0] < second[0]);
  }
};
 
struct compare_col_functor {
  bool operator() (const vector<double> first, const vector<double> second)
  {
    return (first[1] < second[1]);
  }
};

bool get3DPoint(int pixel_row, int pixel_col, vector<vector<double> > &cloud, 
     double searchDist, vector<double> & point);
vector<double> project(vector<double> &Xw);
bool mapPixels(ElevDetection &detection, vector<vector<double> > &cloud);
void unwarpImage(IplImage * srcImage, string unwarpedImageFile);
