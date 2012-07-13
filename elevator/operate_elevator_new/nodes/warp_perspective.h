#pragma once

#include <iostream>
#include <vector>
#include <time.h>

#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "opencv/highgui.h"

using namespace std;

class WarpPerspective {
  public:
    WarpPerspective(bool bVerbose = false);
  	~WarpPerspective();
    void getWarpingMatrix(CvMat &map_matrix, CvMat &map_matrix_inv, const float* src_x, 
                           const float* src_y, const float* dst_x, const float* dst_y);
    IplImage* warpPerspectiveImage(IplImage* src_image, CvMat &map_matrix); 
    vector<double> warpPerspectivePoint(vector<double> imageCoord, const CvMat &map_matrix); 
    private:
      bool bVerbose_;
};


WarpPerspective::WarpPerspective(bool bVerbose) {
  bVerbose_ = bVerbose;
}


WarpPerspective::~WarpPerspective() {
}


void WarpPerspective::getWarpingMatrix(CvMat &map_matrix, CvMat &map_matrix_inv, const float* src_x, 
                                        const float* src_y, const float* dst_x, const float* dst_y) 
{
  // Get perspective transformation (unwarping) matrix.
  CvPoint2D32f src[4], dst[4];
  for (int i=0; i<4; i++) {
    src[i].x = src_x[i];
    src[i].y = src_y[i];
    dst[i].x = dst_x[i];
    dst[i].y = dst_y[i];
  }
  cvWarpPerspectiveQMatrix(dst, src, &map_matrix);
  
  if (bVerbose_) {
    cout << "map matrix: " << endl;
    for (int i=1; i<10; i++) {
      cout << map_matrix.data.db[i-1] << " ";
      if (i%3 == 0) cout << endl;
    }
    cout << endl;
  }
  
  // Get perspective transformation inverse (warping) matrix.
  cvWarpPerspectiveQMatrix(src, dst, &map_matrix_inv);
  
  if (bVerbose_) {
    cout << "map matrix inverse: " << endl;
    for (int i=1; i<10; i++) {
      cout << map_matrix_inv.data.db[i-1] << " ";
      if (i%3 == 0) cout << endl;
    }
    cout << endl;
  }
  return;
}


IplImage* WarpPerspective::warpPerspectiveImage(IplImage* src_image, CvMat &map_matrix) 
{
  IplImage* unwarped_image = cvCloneImage(src_image);

  // Apply perspective warping.
  cvWarpPerspective(src_image, unwarped_image, &map_matrix, 
                    CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS);

  // Verify that if image is warped back, we get the original.
  IplImage* warped_image = cvCloneImage(src_image);
  cvWarpPerspective(unwarped_image, warped_image, &map_matrix, 
                    CV_WARP_INVERSE_MAP+CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS);

  if (bVerbose_) {
    // Display original image.
    cvNamedWindow("Image", 1);
    cvShowImage("Image", src_image);
    cvWaitKey(-1);
    cvDestroyAllWindows();

    // Display unwarped image.
    cvNamedWindow("Unwarped Image", 1);
    cvShowImage("Unwarped Image", unwarped_image);
    cvWaitKey(-1);
    cvDestroyAllWindows();
    
    /*// Display warped image (should be same as original).
    cvNamedWindow("Warped (Orig) Image", 1);
    cvShowImage("Warped (Orig) Image", warped_image);
    cvWaitKey(-1);
    cvDestroyAllWindows();
    */
  }

  cvReleaseImage(&warped_image);
//  cvReleaseImage(&unwarped_image);
  
  return unwarped_image;
}


// Get corresponding coordinates of pixel in warped image to unwarped.
// src_coord  = (x, y) coordinates for a single pixel in the unwarped image.
vector<double> WarpPerspective::warpPerspectivePoint(vector<double> imageCoord, const CvMat &map_matrix) 
{
  double src[2], dst[2];
  vector<double> unwarpedCoord;
  if (int(imageCoord.size()) != 2) {
    cout << "Error: src_coord should be size 2 -- (x, y) pixel coordinate." << endl;
    return unwarpedCoord;
  }
  for (size_t i=0; i<imageCoord.size(); i++) {
    src[i] = imageCoord[i];
    dst[i] = 0;
  }
  
  if (bVerbose_) {
    cout << "map matrix: " << endl;
    for (int i=1; i<10; i++) {
      cout << map_matrix.data.db[i-1] << " ";
      if (i%3 == 0) cout << endl;
    }
    cout << endl;
  }

  CvMat src_arr, dst_arr;
  cvInitMatHeader(&src_arr, 1, 1, CV_64FC2, src);
  cvInitMatHeader(&dst_arr, 1, 1, CV_64FC2, dst);

  cvPerspectiveTransform(&src_arr, &dst_arr, &map_matrix);
  
  CvScalar scal;
  scal = cvGet2D(&src_arr, 0, 0);
  if (bVerbose_) {
    printf("pixels in warped (original) image: \n");
    printf( "(%f,%f,%f) \n", scal.val[0], scal.val[1], scal.val[2] );
  }

  scal = cvGet2D(&dst_arr, 0, 0);
  if (bVerbose_) {
    printf("corresponding pixels in unwarped image: \n");
    printf( "(%f,%f,%f) \n", scal.val[0], scal.val[1], scal.val[2] );
  }

  unwarpedCoord.push_back(scal.val[0]);
  unwarpedCoord.push_back(scal.val[1]);
  
  return unwarpedCoord;
}
