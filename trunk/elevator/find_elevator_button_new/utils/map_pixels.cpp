#include "map_pixels.h"

bool get3DPoint(int pixel_row, int pixel_col, vector<vector<double> > &cloud, double searchDist,
                         vector<double> & point)
{
  vector<double> candidate(cloud[0].size(),0);
  vector<double> row_candidates, col_candidates, x_candidates, y_candidates, z_candidates;
  double dist;
  size_t numPts = cloud.size();

  // Find pixels within searchDist of desired location.
  //cerr << "Looking for pixels within searchDist....." << endl;
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
  cerr << "num candidates = " << x_candidates.size() << endl;

  // Choose median of these points.
  //cerr << "Finding median of all candidates....." << endl;
  if (x_candidates.size() > 0) {
    sort(row_candidates.begin(), row_candidates.end());
    sort(col_candidates.begin(), col_candidates.end());
    sort(x_candidates.begin(), x_candidates.end());
    sort(y_candidates.begin(), y_candidates.end());
    sort(z_candidates.begin(), z_candidates.end());
    candidate[0] = row_candidates[int(row_candidates.size()/2)];
    candidate[1] = col_candidates[int(col_candidates.size()/2)];
    candidate[2] = x_candidates[int(x_candidates.size()/2)];
    candidate[3] = y_candidates[int(y_candidates.size()/2)];
    candidate[4] = z_candidates[int(z_candidates.size()/2)];
    cerr << "point in laser frame: " << candidate[0] << " " << candidate[1] << " " << candidate[2]*1000
      << " " << candidate[3]*1000 << " " << candidate[4]*1000 << endl;
  } else {
    cerr << "Point cloud does not contain 3D coordinates for pixel." << endl << endl;
    return false;
  } 

  point.push_back(candidate[2]);
  point.push_back(candidate[3]);
  point.push_back(candidate[4]);
  return true;
}

vector<double> project(vector<double> &Xw)
{
  // Transform 3D points from flea camera frame to canon frame and project onto canon image.
  vector<double> Xc, xp;
  double xn[2], xd[2], dx[2];
  double r_sq, c;
  Xc.push_back(R[0][0]*Xw[0] + R[0][1]*Xw[1] + R[0][2]*Xw[2] + T[0]);
  Xc.push_back(R[1][0]*Xw[0] + R[1][1]*Xw[1] + R[1][2]*Xw[2] + T[1]);
  Xc.push_back(R[2][0]*Xw[0] + R[2][1]*Xw[1] + R[2][2]*Xw[2] + T[2]);
  cerr << "point in canon frame: " << Xc[0] << " " << Xc[1] << " " << Xc[2] << endl;
 
  xn[0] = Xc[0]/Xc[2];
  xn[1] = Xc[1]/Xc[2];
  r_sq = pow(xn[0],2) + pow(xn[1],2);
  dx[0] = 2*kc[2]*xn[0]*xn[1] + kc[3]*(r_sq + 2*pow(xn[0],2));
  dx[1] = kc[2]*(r_sq + 2*pow(xn[1],2)) + 2*kc[3]*xn[0]*xn[1];
  c = 1+kc[0]*r_sq + kc[1]*pow(r_sq,2) + kc[4]*pow(r_sq,3);
  xd[0] = c*xn[0] + dx[0];
  xd[1] = c*xn[1] + dx[1];
  xp.push_back(fc[0]*(xd[0]+alpha*xd[1]) + cc[0]);
  xp.push_back(fc[1]*xd[1] + cc[1]);
  cerr << "pixels in canon image: " << xp[0] << " " << xp[1] << endl;
  return xp;
}
  

//  ADD 3D coordinates to ElevDetection structure
bool mapPixels(ElevDetection &detection, vector<vector<double> > &cloud)
{
  WarpPerspective pWarp(false);
  CvMat unwarp_matrix;
  CvMat warp_matrix;
  
  // Generate unwarping matrix.
  const float src_x[4] = {156, 467, 467, 201};
  const float src_y[4] = {159, 145,363 , 348};
  const float dst_x[4] = {201, 467, 467, 201};
  const float dst_y[4] = {159, 159, 348, 348};
  double mat1[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double mat2[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  cvInitMatHeader(&unwarp_matrix, 3, 3, CV_64FC1, mat1);
  cvInitMatHeader(&warp_matrix, 3, 3, CV_64FC1, mat2);
  pWarp.getWarpingMatrix(unwarp_matrix, warp_matrix, src_x, src_y, dst_x, dst_y);

  double searchDist = 3;
  
  cerr << "mapPixels::mapping flea detection [x y w h] = " << detection.toString() << endl;

  // Map 2D pixel coordinates from unwarped image to original warped image.
      // 3d point cloud data corresponds with original warped image.
  vector<double> temp;
  //temp.push_back(detection.y);
  //temp.push_back(detection.x);
  temp.push_back(detection.x + detection.w/2);
  temp.push_back(detection.y + detection.h/2);
  temp = pWarp.warpPerspectivePoint(temp, unwarp_matrix);
  double px = temp[0];
  double py = temp[1];
  cout << "Warped 2D button coords (original image): " << px << " " << py << endl;

  // Get 3D coordinates for center of detection in canon camera frame.
  vector<double> Xworld;
  if (!get3DPoint(py, px, cloud, searchDist, Xworld)) {
    return false;
  }
  // Save 3D coordinates for detection.
  detection.Xw = Xworld[0];
  detection.Yw = Xworld[1];
  detection.Zw = Xworld[2];
 
 // Convert 3D point to mm.
  for (size_t i=0; i<Xworld.size(); i++) {
    Xworld[i] *= 1000;
  }  
  // find corresponding pixels in canon image
  vector<double> xp = project(Xworld);
  
  // get 3D coordinates for corners to figure out new w,h
  // FOR NOW just used fixed scale factor of flea image width / height.
  /*
  vector<double> XupperL = get3DPoint(detection.x, detection.y, searchDist);
  vector<double> XupperR = get3DPoint(detection.x+detection.w, detection.y, searchDist);
  vector<double> XlowerL = get3DPoint(detection.x, detection.y+detection.h, searchDist);
  vector<double> XlowerR = get3DPoint(detection.x+detection.w, detection.y+detection.h, searchDist);
  vector<double> xp_ul = project(XupperL);
  vector<double> xp_ur = project(XupperR);
  vector<double> xp_ll = project(XlowerL);
  vector<double> xp_lr = project(XlowerR);
  */

  detection.w = detection.w * w_scale_factor;
  detection.h = detection.h * h_scale_factor;
  detection.x = xp[0] - detection.w/2;
  if (detection.x < 1) {
    detection.x = 1;
  } else if (detection.x + detection.w > img_width) {
    detection.w = img_width - detection.x;
  }
  detection.y = xp[1] - detection.h/2;
  if (detection.y < 1) {
    detection.y = 1;
  } else if (detection.y + detection.h > img_height) {
    detection.y = img_height - detection.y;
  }
  cerr << "mapPixels::new detection data: [x y w h] = " << detection.toString() << endl << endl;

  return true;
}


void unwarpImage(IplImage * srcImage, string unwarpedImageFile) 
{
  WarpPerspective pWarp(false);
  CvMat unwarp_matrix;
  CvMat warp_matrix;
  // Generate unwarping matrix.
  const float src_x[4] = {156, 467, 467, 201};
  const float src_y[4] = {159, 145,363 , 348};
  const float dst_x[4] = {201, 467, 467, 201};
  const float dst_y[4] = {159, 159, 348, 348};
  double mat1[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double mat2[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  cvInitMatHeader(&unwarp_matrix, 3, 3, CV_64FC1, mat1);
  cvInitMatHeader(&warp_matrix, 3, 3, CV_64FC1, mat2);
  pWarp.getWarpingMatrix(unwarp_matrix, warp_matrix, src_x, src_y, dst_x, dst_y);
  cerr << "unwarpImage::warp_matrix " << endl;
  for (int i=0; i<3; i++) {
    for (int j=0; j<3; j++) {
      cerr << cvmGet(&warp_matrix, i, j) << " ";
    }
    cerr << endl;
  }
  //Apply perspective warping to unwarp the image into the wall plane.
  IplImage* unwarped_image = pWarp.warpPerspectiveImage(srcImage, warp_matrix);
  cvSaveImage(unwarpedImageFile.c_str(), unwarped_image);
  cvReleaseImage(&unwarped_image);
  return;
}
