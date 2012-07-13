#include "find_elevator_button.h"
#define DEBUG 1
#define ONLINE_PROCESS 0 // If we are working on the robot, set to 1
#define USE_TRAINING_DATA 1 // For offline processes

void FindElevatorButtons::init()
{

  bVerbose = true;
  emg = new EMGridFit(bVerbose);
  hmml = new HMMLabel();
  find_button_pkg_path = ros::getPackagePath("find_elevator_button");

  ros::Node *node = ros::Node::instance();
  node->subscribe("elevator_buttons_request", button_request,
                  &FindElevatorButtons::findButtons, this, 10);
  node->advertise<stair_msgs::Button>("single_button", 10);
  node->advertise<stair_msgs::AllButtons>("all_buttons", 10);
	
	// Find buttons the normal way
	if (ONLINE_PROCESS == 1) {
		findButtons();	
	
	// Use previously svl classification and loop over all images
	} else {
		string detectionsPath = find_button_pkg_path + "/Data/classify/final_detections";
		string srcImageDir; 
		vector<string> srcImageFiles;
		
		if (USE_TRAINING_DATA == 1) 
			srcImageDir= "/home/stair/ellen/stair/perception/find_elevator_button/Data/images/unwarped_train_low/";
		else 
			srcImageDir= "/home/stair/ellen/stair/perception/find_elevator_button/Data/images/unwarped_test_low/";

  		getDir(srcImageDir, srcImageFiles);

  		size_t numImages = srcImageFiles.size();
		srcImageFiles.clear();
		srcImageFiles.resize(2);
		srcImageFiles[0] = "DSCN5058_unwarped_low.jpg";
		srcImageFiles[1] = "DSCN5099_unwarped_low.jpg";

 	 	// Loop over all images in sourceImageDir, getting imageName, imageFile and detections
  	   for (size_t k=0; k<2; k++) //numImages; k++) 
  		{
			svlDetections.clear();
			imageName = srcImageFiles[k].substr(0,srcImageFiles[k].length()-4);
			imageFile = srcImageDir + srcImageFiles[k];
		//	imageName = "DSCN5099_unwarped_low";
		//	imageFile = srcImageDir + imageName + ".jpg";

			if (USE_TRAINING_DATA == 1) 
				detectionsFile = "train_" + imageName + "_train-pos-1700";			
			else 
				detectionsFile = "test_" + imageName + "_train-pos-1700";	
						
			// Find buttons if detection file exists
	 		cerr << "Reading in detections file: Path: " << detectionsPath << "  File: " << detectionsFile << endl;
	  		if (!readCacheObject2dFrame(detectionsPath.c_str(),detectionsFile.c_str(),svlDetections)) {
		 		cout << "Error reading svl detections file." << endl;
	  		} else {
		  		cout << "Number of detections " << svlDetections.size() << endl;
				findButtons();
			} 
		} 
	}

  // For debugging
//   labelHMM();
}


void FindElevatorButtons::findButtons()
{
	// Get ismage name and path
	if (ONLINE_PROCESS==1) {
	  requestedButton = button_request.button_label;
  	  //imageFile = button_request.source_image_filename;
		imageFile = button_request.image_filename;

		// Hard coded imageFile and name for debugging
		imageFile = "/home/stair/ellen/stair/perception/find_elevator_button/Data/images/stair_images_unwarped/stair_image_031309_01_unwarped.jpg";

	  // Extract image name from full image path.
	  size_t beginPos = imageFile.find_last_of("/") + 1;
	  size_t len = imageFile.find_last_of(".") - beginPos;
	  imageName = imageFile.substr(beginPos, len);
		

	}

  cout << "Image name: " << imageName << endl;

  // Load source_image from file.
  source_image = cvLoadImage(imageFile.c_str(), CV_LOAD_IMAGE_COLOR);
  assert(source_image != NULL);
  int nPixels = source_image->width*source_image->height;
  if (bVerbose) {
    cerr << "Loaded " << source_image->width << "x" << source_image->height << " from " << imageFile << endl;
  }

  // Get detections from svl.
	cout << "\n\n\n*** SVL DETECTIONS ***\n\n\n";
	getSvlDetections();

  // Run EM algortithm to fit grids to svl detections and output revised detections.
  cout << "\n\n\n*** EM GRID FIT ***\n\n\n";
  fitGridToDetections(nPixels);

  // Call tesseract to label detections.
  cout << "\n\n\n*** LABEL DETECTIONS ***\n\n\n";
  labelDetections();

  // Run Viterbi algorithm to determine labels
  cout << "\n\n\n*** LABEL VITERBI ***\n\n\n";
  labelHMM();

  // Find desired button
  cout << "\n\n\n*** FIND REQUESTED BUTTON ***\n\n\n";
  getButtonInfo();
}


void FindElevatorButtons::getSvlDetections()
{

	if (ONLINE_PROCESS == 1) {
	  svlDetections.clear();
	  SvlClassify classify = SvlClassify(this->imageFile);
	  classify.classify1();
	  classify.train2();
	  classify.classify2();
	  this->svlDetections = classify.train2_classifications;
	}

  // Display pruned SVL candidates on the image and save to file.
  IplImage* svl_image = cvCloneImage(source_image);
  for (size_t k=0; k<svlDetections.size(); k++) {
    cvRectangle(svl_image, cvPoint(int(svlDetections[k].x), int(svlDetections[k].y)),
      cvPoint(int(svlDetections[k].x + svlDetections[k].w - 1), 
        int(svlDetections[k].y + svlDetections[k].h - 1)), CV_RGB(0, 255, 0), 1);
  }
  string debugFile = find_button_pkg_path + "/Data/debug/" + imageName + "_svl.jpg";
  cvSaveImage(debugFile.c_str(), svl_image);
}




void FindElevatorButtons::fitGridToDetections(int nPixels)
{
  gridParams.clear();
  obsDetections.clear();
  emg->computeGridParams(svlDetections, gridParams, obsDetections, nPixels, imageName, imageFile);  // nPixels = 50;

  // Adjust top left coord of grid.
  for (size_t i=0; i<gridParams.size(); i++) {
    gridParams[i].gx += gridParams[i].dx;
    gridParams[i].gy += gridParams[i].dy;
  }


  // Display grid output by EM algorithm on image and save to file.
  IplImage* em_image = cvCloneImage(source_image);
  CvPoint pt1, pt2;
  int line_width = 4;
  
  // Assume no more than 5 grids.
  vector<CvScalar> colors;
  colors.push_back(CV_RGB(255, 0, 0));  // red
  colors.push_back(CV_RGB(255, 153, 18)); // orange
  colors.push_back(CV_RGB(155, 48, 255)); // purple
  colors.push_back(CV_RGB(0, 0 ,255));  // blue
  colors.push_back(CV_RGB(0, 255, 0)); // green

  // Draw vertical lines.
  for (size_t i=0; i<gridParams.size(); i++) {
    cout << "Drawing grid with parameters: " << endl;
    cout << "gx: " << gridParams[i].gx << endl;
    cout << "gy: " << gridParams[i].gy << endl;
    cout << "dx: " << gridParams[i].dx << endl;
    cout << "dy: " << gridParams[i].dy << endl;
    cout << "ncols: " << gridParams[i].ncols << endl;
    cout << "nrows: " << gridParams[i].nrows << endl;

    // Draw horizontal lines.
    for (int row=0; row<=gridParams[i].nrows; row++) {
      for (int col=0; col<gridParams[i].ncols; col++) {
        pt1.x = int(gridParams[i].gx + gridParams[i].dx*(col-0.75));
        pt1.y = int(gridParams[i].gy + gridParams[i].dy*(row-0.5));
        pt2.x = int(gridParams[i].gx + gridParams[i].dx*(col+0.25));
        pt2.y = pt1.y;
					if (DEBUG > 1) {
					  cout << "pt1: " << pt1.x << "," << pt1.y << endl;
					  cout << "pt2: " << pt2.x << "," << pt2.y << endl;
					}
			if (nPixels > 500000)
	        cvLine(em_image, pt1, pt2, colors[i], line_width, 8); 
			else 
	        cvLine(em_image, pt1, pt2, colors[i], line_width, 2); 
      }
    }
  
    // Draw vertical lines.
    for (int col=0; col<=gridParams[i].ncols; col++) {
      for (int row=0; row<gridParams[i].nrows; row++) {
        pt1.x = int(gridParams[i].gx + gridParams[i].dx*(col-0.75));
        pt1.y = int(gridParams[i].gy + gridParams[i].dy*(row-0.5));
        pt2.x = pt1.x;
        pt2.y = int(gridParams[i].gy + gridParams[i].dy*(row+0.5));
					if (DEBUG > 1) {
					  cout << "pt1: " << pt1.x << "," << pt1.y << endl;
					  cout << "pt2: " << pt2.x << "," << pt2.y << endl;
					}
			if (nPixels > 500000)
	        cvLine(em_image, pt1, pt2, colors[i], line_width, 8);
			else 
 	        cvLine(em_image, pt1, pt2, colors[i], line_width, 2);
      }
    }

    // display button locations from EM algorithm
    for (size_t k=0; k<obsDetections[i].size(); k++) {
      if (obsDetections[i][k].isButton) {
			if (nPixels > 500000) {
		     cvCircle(em_image, cvPoint(int(obsDetections[i][k].x), int(obsDetections[i][k].y)),
		              15, colors[i], -1);
			} else {
     			cvCircle(em_image, cvPoint(int(obsDetections[i][k].x), int(obsDetections[i][k].y)),
		              3, colors[i], -1);
			}
      }
    }
  }
  string debugFile = find_button_pkg_path + "/Data/debug/" + imageName + "_em.jpg";
  cvSaveImage(debugFile.c_str(), em_image);
}



/*void FindElevatorButtons::labelDetections()
{
  return;
}
*/


void FindElevatorButtons::labelDetections()
{
  LabelClassify label(this->imageFile,gridParams,obsDetections);
  label.cropLabels();
  label.binarize();
  label.tesseractOCR();
  return;
  
}


void FindElevatorButtons::labelHMM()
{

  //vector<grid_param_struct> gridParams;
  //vector< vector<button_struct> > obsDetections;
  //vector<button_struct> labeledButtons;

/*	gridParams.resize(3);
	gridParams[0].ncols = 1;
	gridParams[0].nrows = 1;
	gridParams[1].ncols = 1;
	gridParams[1].nrows = 5;
	gridParams[2].ncols = 2;
	gridParams[2].nrows = 3;

	obsDetections.resize(3);
	obsDetections[0].resize(1);
	obsDetections[1].resize(5);
	obsDetections[2].resize(6);
	obsDetections[0][0].isButton = 1;
	obsDetections[1][0].isButton = 1;
	obsDetections[1][1].isButton = 1;
	obsDetections[1][2].isButton = 1;
	obsDetections[1][3].isButton = 1;
	obsDetections[1][4].isButton = 1;
	obsDetections[2][0].isButton = 1;
	obsDetections[2][1].isButton = 1;
	obsDetections[2][2].isButton = 1;
	obsDetections[2][3].isButton = 0;
	obsDetections[2][4].isButton = 1;
	obsDetections[2][5].isButton = 1;
   imageName = "DSCN5068_unwarped";
*/

  hmml->getButtonLabels(gridParams,obsDetections,labeledButtons,imageName,imageFile);
}


void FindElevatorButtons::getButtonInfo() 
{
	stair_msgs::Button tmpButton;
	// Get all buttons
	cout << "\nDetermining all buttons" << endl;
	for (size_t i=0; i<labeledButtons.size(); i++) {
		if (labeledButtons[i].isButton == 1) {
			tmpButton.x = labeledButtons[i].x;
			tmpButton.y = labeledButtons[i].y;
			tmpButton.label = labeledButtons[i].label;
			all_buttons.data.push_back(tmpButton);
		}
	}
	
	if (DEBUG > 0) {
		cout << "ALL BUTTON DATA" << endl;
		for (size_t i=0; i<all_buttons.data.size(); i++) {
			cout << "Button: " << i;
			cout << "\t x=" << all_buttons.data[i].x;
			cout << "\t y=" << all_buttons.data[i].y;
			cout << "\t label=" << all_buttons.data[i].label << endl;
		}
	}

	// Get single button if this is an online process

	if (ONLINE_PROCESS==1) {
		cout << "\n Searching for button request" << endl;
		int i = 0;
		while (i<all_buttons.data.size() && button_request.button_label.compare(all_buttons.data[i].label)!=0) {
			i++;
		}
		if (i!=all_buttons.data.size()) {
			single_button = all_buttons.data[i];		
			cout << "\n FOUND DESIRED BUTTON!" << endl;	
		} else {
			single_button.x = -1;
			single_button.y = -1;
			single_button.label = "ButtonDoesNotExist";
			cout << "\n DESIRED BUTTON DOES NOT EXIST!" << endl;	
		}		
	} 

	
}


bool FindElevatorButtons::getDir(string dir, vector<string> &files)
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


void FindElevatorButtons::shutdown()
{
  delete emg;
  delete hmml;
}

int main (int argc, char **argv)
{

  ros::init(argc, argv);
  ros::Node n("find_elevator_buttons");

  FindElevatorButtons buttonFinder;
  buttonFinder.init();
  
  n.spin();

  buttonFinder.shutdown();
  printf("Ros process find_elevator_buttons is shutting down. \n");
  
  return 0;
}
