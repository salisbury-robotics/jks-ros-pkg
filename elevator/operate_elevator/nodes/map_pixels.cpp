#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace std;

// NOTE: I think 3D points are in laser stage frame not flea frame -- need to look into this!!!

  // position of right(flea) camera wrt left camera (canon)
 //Rotation vector:             om = [ 0.44461   0.14230  -0.03101 ] � [ 0.12480   0.05876  0.01319 ]
 //Translation vector:           T = [ -91.27664   207.89630  8.10682 ] � [ 6.02304   21.09011  44.07893 ]
double T_canon2flea[3] = {-91.27664, 207.89630, 8.10682};  // mm
double R_canon2flea[3][3] = {{0.989587, 0.060952, 0.130396},
                             {0.001170, 0.902479, -0.430733},
                             {-0.143934, 0.426400, 0.893009}};
  
  // position of right(canon) camera wrt left camera (flea)
  //Rotation vector:             om = [ -0.20721   -0.29357  0.03675 ]
  //Translation vector:           T = [ 81.21237   -185.71510  -5.60391 ]
double T_flea2canon[3] = { 81.21237, -185.71510, -5.60391};  // mm
double R_flea2canon[3][3] = {{0.956707, -0.005870, -0.290994},
                             {0.066042, 0.978096, 0.197398},
                             {0.283462, -0.208069, 0.936139}};

// flea intrinsics
// Focal Length:          fc_left = [ 569.41186   569.03058 ] � [ 5.88022   5.60798 ]
// Principal point:       cc_left = [ 298.53754   255.84447 ] � [ 6.52504   5.88328 ]
// Skew:             alpha_c_left = [ 0.00000 ] � [ 0.00000  ]   => angle of pixel axes = 90.00000 � 0.00000 degrees
// Distortion:            kc_left = [ -0.34072   0.18975   0.00109   -0.00155  0.00000 ] � [ 0.01717   0.03676   0.00155   0.00194  0.00000 ]
//
// canon intrinsics
// Focal Length:          fc_right = [ 3642.82642   3646.14996 ] � [ 49.37854   49.79061 ]
// Principal point:       cc_right = [ 1674.51375   1308.37720 ] � [ 38.00273   37.21275 ]
// Skew:             alpha_c_right = [ 0.00000 ] � [ 0.00000  ]   => angle of pixel axes = 90.00000 � 0.00000 degrees
// Distortion:            kc_right = [ -0.12267   0.40854   0.00734   0.00097  0.00000 ] � [ 0.03908   0.35019   0.00275   0.00220  0.00000 ]

double fc[2] = {3642.82642,   3646.14996};
double cc[2] = {1674.51375,   1308.37720};
double alpha_c = 0;
double kc[5] = {-0.12267,   0.40854,   0.00734,   0.00097,  0.00000};


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


int main(int argc, char *argv[])
{
  vector<vector<double> > cloud;
  int pixel_row = 1;
  int pixel_col = 1;
  double searchDist = 5;

  if (argc == 3) {
    pixel_row = atoi(argv[1]);
    pixel_col = atoi(argv[2]);
    printf("pixel row/col %d %d\n", pixel_row, pixel_col);
  }

  // Read in images.
  IplImage* flea_image = cvLoadImage("test_flea.pgm"); 
  IplImage* canon_image = cvLoadImage("test_canon.jpg"); 
  
  // Read in point cloud from file and sort.
  FILE* f = fopen("cloudrowcol.txt", "r");
  if(f == NULL) {
    printf("Couldn't read point cloud file.\n");
    return false;
  }
  char buf[512];
  char* p = NULL;
  char delim[] = "\t\n ";
  while (fgets(buf, 512, f) != NULL) {
    vector<double> point;
    p = strtok(buf, delim);
    point.push_back(atof(p));
    for (unsigned int i=1; i<5; i++) {
      p = strtok(NULL,delim);
      point.push_back(atof(p));
    }
    cloud.push_back(point);
  }
  size_t numPts = cloud.size();
  cout << "Number of pts in point cloud: " << numPts << endl;
  fclose(f);

   // Sort point cloud by increasing pixel row number and increasing column number.
  cout << "sorting point cloud by row and column...." << endl;

  sort(cloud.begin(), cloud.end(), compare_row_functor());

  vector<vector<double> > temp_cloud;
  vector<vector<double> > sub_cloud;
  int i=0;
  for (int k=0; k<cloud[numPts-1][0]; k++) {
    while(cloud[i][0] == k) {
      sub_cloud.push_back(cloud[i]);
      i++;
    }
    sort(sub_cloud.begin(), sub_cloud.end(), compare_col_functor());
    for (size_t j=0; j<sub_cloud.size(); j++) {
      temp_cloud.push_back(sub_cloud[j]);
    }
    sub_cloud.clear();
  }
  cloud = temp_cloud;

  // find 3D points corresponding to input pixel
  vector<double> candidate(cloud[0].size(),0);
  vector<double> row_candidates, col_candidates, x_candidates, y_candidates, z_candidates;
  double dist;

  // Find pixels within searchDist of desired location.
  cout << "Looking for pixels within searchDist....." << endl;
  for (size_t i=0; i<numPts; i++) {
    dist = sqrt((pixel_row - cloud[i][0])*(pixel_row - cloud[i][0]) +
        (pixel_col - cloud[i][1])*(pixel_col - cloud[i][1]));
    if (dist <= searchDist) {
      row_candidates.push_back(cloud[i][0]);
      col_candidates.push_back(cloud[i][1]);
      x_candidates.push_back(cloud[i][2]);
      y_candidates.push_back(cloud[i][3]);
      z_candidates.push_back(cloud[i][4]);
    }
  }

  // Choose median of these points.
  cout << "Finding median of all candidates....." << endl;
  if (x_candidates.size() > 0) {
    sort(row_candidates.begin(), row_candidates.end());
    sort(col_candidates.begin(), col_candidates.end());
    sort(x_candidates.begin(), x_candidates.end());
    sort(y_candidates.begin(), y_candidates.end());
    sort(z_candidates.begin(), z_candidates.end());
    candidate[0] = row_candidates[int(row_candidates.size()/2)];
    candidate[1] = col_candidates[int(col_candidates.size()/2)];
    candidate[2] = x_candidates[int(x_candidates.size()/2)] * 1000;
    candidate[3] = y_candidates[int(y_candidates.size()/2)]* 1000;
    candidate[4] = z_candidates[int(z_candidates.size()/2)] * 1000;
    cerr << "point: " << endl;
    for (size_t j=0; j<candidate.size(); j++) {
      cerr << candidate[j] << " ";
    }
    cerr << endl;
  } else {
    cout << "Point cloud does not contain 3D coordinates for pixel." << endl;
  } 

  // Transform 3D point from flea frame to canon frame.
  vector<double> point;
  point.push_back(R_flea2canon[0][0]*candidate[2] + R_flea2canon[0][1]*candidate[3] + R_flea2canon[0][2]*candidate[4] + T_flea2canon[0]);
  point.push_back(R_flea2canon[1][0]*candidate[2] + R_flea2canon[1][1]*candidate[3] + R_flea2canon[1][2]*candidate[4] + T_flea2canon[1]);
  point.push_back(R_flea2canon[2][0]*candidate[2] + R_flea2canon[2][1]*candidate[3] + R_flea2canon[2][2]*candidate[4] + T_flea2canon[2]);

  // Project 3D point in canon frame onto image plane.
  double Xn[2];
  Xn[0] = point[0]/point[2];
  Xn[1] = point[1]/point[2];
  double r_sq = pow(Xn[0],2) + pow(Xn[1],2);
  double dx[2];
  dx[0] = 2*kc[2]*Xn[0]*Xn[1] + kc[3]*(r_sq + 2*pow(Xn[0],2));
  dx[1] = kc[2]*(r_sq + 2*pow(Xn[1],2)) + 2*kc[3]*Xn[0]*Xn[1];
  double Xd[2];
  double c = 1+kc[0]*r_sq + kc[1]*pow(r_sq,2) + kc[4]*pow(r_sq,3);
  Xd[0] = c*Xn[0] + dx[0];
  Xd[1] = c*Xn[1] + dx[1];
  double Xp[2];
  Xp[0] = fc[0]*(Xd[0]+alpha_c*Xd[1]) + cc[0];
  Xp[1] = fc[1]*Xd[1] + cc[1];

  cerr << "pixels in canon image: " << Xp[0] << " " << Xp[1] << endl;

  // Mark pixel on images.
  cvCircle(flea_image, cvPoint(pixel_col, pixel_row), 4, CV_RGB(0, 0, 255), 3);
  cvCircle(canon_image, cvPoint(Xp[0], Xp[1]), 12, CV_RGB(0, 0, 255), 3);

  // Save images to file.
  cvSaveImage("flea_result.jpg", flea_image);
  cvSaveImage("canon_result.jpg", canon_image);
  
/*
  // Display images.
  cvNamedWindow("flea image", 1);
  cvShowImage("flea image", flea_image);
  cvWaitKey(-1);
  cvDestroyAllWindows();
  
  cvNamedWindow("canon image", 1);
  cvShowImage("canon image", canon_image);
  cvWaitKey(-1);
  cvDestroyAllWindows();
*/

  return 0;
}

