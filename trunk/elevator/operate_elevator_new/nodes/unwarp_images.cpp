#include "warp_perspective.h"
#include <dirent.h>
#include <errno.h>

using namespace std;

bool readUnwarpingPoints(string dataFile, float* src_x, float* src_y, float* dst_x, float* dst_y)
{
  FILE* f;
  char delim[] = ",\n";
  char *p = NULL;
  char buffer[512];
  
  f = fopen(dataFile.c_str(), "r");

  if (f == NULL) {
    cout << "Couldn't read unwarping points text file.....skipping to next image." << endl;
    return false;
  }

  cout << "Reading values into src array...." << endl;
  for (int i=0; i<4; i++) {
    if (fgets(buffer, 512, f) == NULL) {
      cout << "Couldn't read unwarping points text file.....skipping to next image." << endl;
      return false;
    }
    p = strtok(buffer, delim);
    src_x[i] = (atof(p));    
    p = strtok(NULL, delim);
    src_y[i] = (atof(p));    
    cout << "(" << src_x[i] << "," << src_y[i] << ")" << endl;
  }
  
  cout << "Reading values into dst array...." << endl;
  for (int i=0; i<4; i++) {
    if (fgets(buffer, 512, f) == NULL) {
      cout << "Couldn't read unwarping points text file.....skipping to next image." << endl;
      return false;
    }
    p = strtok(buffer, delim);
    dst_x[i] = (atof(p));    
    p = strtok(NULL, delim);
    dst_y[i] = (atof(p));    
    cout << "(" << dst_x[i] << "," << dst_y[i] << ")" << endl;
  }
  fclose(f);
  return true;
}

bool readTruthData(string dataFile, vector<vector<double> > &truthData)
{
  FILE* f;
  char delim[] = ",\n";
  char *p = NULL;
  char buffer[512];
  vector<double> point(4, 0);
  
  f = fopen(dataFile.c_str(), "r");

  if (f == NULL) {
    cout << "Couldn't read truth data text file.....skipping to next image." << endl;
    return false;
  }

  cout << "Reading truth values into vector...." << endl;
  while (fgets(buffer, 512, f) != NULL) {
    p = strtok(buffer, delim);
    point[0] = atof(p);
    for (int i=1; i<4; i++) {
      p = strtok(NULL, delim);
      point[i] = atof(p);    
    }
    //cout << "(" << point[0] << "," << point[1] << "," << point[2] << ",";
    //  cout << point[3] << ")" << endl;
    truthData.push_back(point);
  }
  //cout << endl << endl;

  fclose(f);
  return true;
}

void saveUnwarpedTruth(string dataFile, vector<vector<double> > &data)
{
  FILE* f;
  f = fopen(dataFile.c_str(), "w");
  for (size_t j=0; j<data.size(); j++) {
			fprintf(f,"%f,%f,%f,%f\n", data[j][0], data[j][1], data[j][2], data[j][3]);
  }
  fclose(f);
}

bool getDir(string dir, vector<string> &files)
{
  DIR *dp;
  struct dirent *dirp;
  string filename;

  if((dp  = opendir(dir.c_str())) == NULL) {
    cout << "Error(" << errno << ") opening " << dir << endl;
    return false;
  }

  while ((dirp = readdir(dp)) != NULL) {  
    filename = string(dirp->d_name); 
    if (filename.length() > 2) {
      files.push_back(filename);
    }
  }

  closedir(dp);
  return true;
}

int main()
{
	WarpPerspective* pWarp = new WarpPerspective(false);

  string srcImageDir = "/home/stair/ellen/stair/perception/find_elevator_button/Data/images/stair_images";
  string unwarpedImageDir = "/home/stair/ellen/stair/perception/find_elevator_button/Data/images/stair_images_unwarped/";
  string unwarpDataDir = "/home/stair/ellen/stair/perception/operate_elevator/data/data4Kristen/unwarping_points/";
  string truthDir = "/home/stair/ellen/stair/perception/operate_elevator/data/data4Kristen/truth_stair_images/";
  string unwarpedTruthDir = "/home/stair/ellen/stair/perception/operate_elevator/data/data4Kristen/unwarped_truth_stair_images/";
  vector<string> srcImageFiles = vector<string>();
  string srcImageFile, truthFile;
  string unwarpDataFile, unwarpedImageFile, unwarpedTruthFile;
  float src_x[4], src_y[4];
  float dst_x[4], dst_y[4];
  vector<vector<double> > truthData, unwarpedTruthData;
  vector<double> coordinates(2, 0);
  vector<double> unwarpedCoord;
  vector<double> truthPoint(4, 0);

  // Initialize unwarping matrix.
  double Ident[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  CvMat unwarp_matrix;  // matrix to unwarp image -- result is image warped to vertical plane.
  CvMat warp_matrix;  // matrix to warp image -- result is original image.
  cvInitMatHeader(&unwarp_matrix, 3, 3, CV_64FC1, Ident);
  cvInitMatHeader(&warp_matrix, 3, 3, CV_64FC1, Ident);

  // Get list of all images.
  getDir(srcImageDir, srcImageFiles);
  size_t numImages = srcImageFiles.size();

  // Loop over all images in sourceImageDir.
  for (size_t k=0; k<numImages; k++) 
  {
	
    // Read in source image.
    srcImageFile = srcImageDir + "/" + srcImageFiles[k]; 
    cout << "Reading in src image: " << srcImageFile << endl;
    IplImage* source_image = cvLoadImage(srcImageFile.c_str(), CV_LOAD_IMAGE_COLOR); 

    // Read in unwarping data points text file corresponding to current image.
    unwarpDataFile = unwarpDataDir + srcImageFiles[k].substr(0, srcImageFiles[k].size()-4) + "_unwarp.txt";
    cout << "Reading in unwarping points: " << unwarpDataFile << endl;
    if (!readUnwarpingPoints(unwarpDataFile, src_x, src_y, dst_x, dst_y)) continue;

    // Get unwarping matrix.
    pWarp->getWarpingMatrix(unwarp_matrix, warp_matrix, src_x, src_y, dst_x, dst_y); 

    // Apply perspective warping to unwarp the image into the wall plane.
    cout << "Applying perspective warping to image... " << endl;
    IplImage* unwarped_image = pWarp->warpPerspectiveImage(source_image, unwarp_matrix);

    // Read in truth data. 
    truthFile = truthDir + srcImageFiles[k].substr(0, srcImageFiles[k].size()-4) + "_coords.txt";
    cout << "Reading in truth data: " << truthFile << endl;
    if(!readTruthData(truthFile, truthData)) continue;

    // Apply perspective warping to truth data.
    // Transform image coord from unwarped image to original (warped) image.

		unwarpedTruthData.clear(); // Clear unwarped truth data for each image

    for (size_t j=0; j<truthData.size(); j++) {
      coordinates[0] = truthData[j][0];
      coordinates[1] = truthData[j][1];

      unwarpedCoord = pWarp->warpPerspectivePoint(coordinates, unwarp_matrix); 

/*
      cout << "(" << coordinates[0] << "," << coordinates[1] << ")" << endl;
      cout << "(" << unwarpedCoord[0] << "," << unwarpedCoord[1] << ")" << endl;
      cout << endl;
*/     

      truthPoint[0] = unwarpedCoord[0];
      truthPoint[1] = unwarpedCoord[1];
      for (size_t i=2; i<truthData[j].size(); i++) {
        truthPoint[i] = truthData[j][i];
      }
      unwarpedTruthData.push_back(truthPoint);
    }
    cout << endl;
  
    // Save unwarped image to file.
    unwarpedImageFile = unwarpedImageDir + srcImageFiles[k].substr(0, srcImageFiles[k].size()-4) + "_unwarped.jpg";
    cout << "Saving unwarped image file: " << unwarpedImageFile << endl;
    cvSaveImage(unwarpedImageFile.c_str(), unwarped_image);
   
    // Save unwarped truth data to file.
    unwarpedTruthFile = unwarpedTruthDir + srcImageFiles[k].substr(0, srcImageFiles[k].size()-4) + 
                        "_coords_unwarped.txt";
    cout << "Saving unwarped truth data: " << unwarpedTruthFile << endl;
    saveUnwarpedTruth(unwarpedTruthFile, unwarpedTruthData); 

    cvReleaseImage(&unwarped_image);
    cvReleaseImage(&source_image);
    truthData.clear();
  }

  delete pWarp;
  return 0;
}

