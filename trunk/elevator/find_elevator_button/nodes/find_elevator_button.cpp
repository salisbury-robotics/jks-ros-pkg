#include "find_elevator_button.h"

#define DEBUG 1
#define ONLINE_PROCESS true // If we are working on the robot, set to true
#define IMAGE_TYPE "unwarped.jpg" 
#define IMAGE_TYPE2 "stair"  

int START_FROM_STEP = 0; // Start from given step in process
                         // (0=SWOD), 1=EM, 2=Tess, 3=HMM
void FindElevatorButtons::init()
{

  bVerbose = true;
  //  find_button_pkg_path = "/home/stair/ellen/stair/perception/find_elevator_button";
  find_button_pkg_path = ros::getPackagePath("find_elevator_button");
	  
  if (ONLINE_PROCESS) 
    {
      ros::Node *node = ros::Node::instance();
      node->subscribe("elevator_buttons_request", button_request,
		      &FindElevatorButtons::findButtons, this, 10);
      node->advertise<stair_msgs::Button>("single_button", 10);
      node->advertise<stair_msgs::AllButtons>("all_buttons", 10);

      // Find buttons the normal way
      cout << "Waiting for request... " << endl; 
    }  

  else // For offline process - to loop over set of pictures
    {
      cout << "Enter starting step > ";
      cin >> START_FROM_STEP; 

      string detectionsPath = find_button_pkg_path + 
	"/Data/classify/final_detections";
      string srcImageDir = TEST_IMAGES_PATH; 
      vector<string> srcImageFiles;
    
      getDir(TEST_IMAGES_PATH,srcImageFiles);

      size_t numImages = srcImageFiles.size();
      cout << "Number of Images = " << numImages << endl;

      size_t k = 0;
      cout << "Enter number of image to start with > ";
      cin >> k;

      for (; k<numImages; k++) // Loop through all images
	{	
	  imageName = srcImageFiles[k].substr(0,srcImageFiles[k].length()-4);
	  imageFile = srcImageDir + imageName + ".jpg";
      
	  cout << k << "    " << imageFile << endl;

	  // Reinitialize/clear variables		
	  gridParams.clear();
	  obsDetections.clear();			
	  labeledButtons.clear();
	  gridPoints.clear();
	  svlDetections.clear();

	  findButtons();
	  cvReleaseImage(&source_image); 
	  
	} 
    }

}


void FindElevatorButtons::findButtons()
{

  
  if (ONLINE_PROCESS)  
    {
      cout << "FIND BUTTONS" <<endl;
      gridParams.clear();
      obsDetections.clear();			
      labeledButtons.clear();
      gridPoints.clear();
      svlDetections.clear();

      requestedButton = button_request.button_label;
      cout << "FIND BUTTONS: REQUEST BUTTON FOUND" <<endl;
      this->imageFile = button_request.image_filename;
      cout << "FIND BUTTONS: Extract image Name" <<endl;
      // Extract image name from full image path.
      size_t beginPos = imageFile.find_last_of("/") + 1;
      size_t len = imageFile.find_last_of(".") - beginPos;
      this->imageName = imageFile.substr(beginPos, len);

      cout << "Image name: " << imageName << endl;

      // Load source_image from file.
      this->source_image = cvLoadImage(imageFile.c_str(), CV_LOAD_IMAGE_COLOR);
      assert(source_image != NULL);
      int nPixels = source_image->width*source_image->height;
      if (bVerbose) {
	cerr << "Loaded " << source_image->width << "x" << source_image->height 
	     << " from " << imageFile << endl;
      }

      // Run process
      cout << "\n\n\n*** SVL DETECTIONS ***\n\n\n";
      getSvlDetections();

      cout << "\n\n\n*** EM GRID FIT ***\n\n\n";
      fitGridToDetections(nPixels);

      cout << "\n\n\n*** LABEL DETECTIONS ***\n\n\n";
      labelDetections();

      cout << "\n\n\n*** LABEL VITERBI ***\n\n\n";
      labelHMM();

      cout << "\n\n\n*** FIND REQUESTED BUTTON ***\n\n\n";
      getButtonInfo();
    
    }

  else    // Run offline process on all images
    {

      cout << "Image name: " << imageName << endl;

      // Load source_image from file.
      source_image = cvLoadImage(imageFile.c_str(), CV_LOAD_IMAGE_COLOR);
      assert(source_image != NULL);
      int nPixels = source_image->width*source_image->height;
      if (bVerbose) {
	cerr << "Loaded " << source_image->width << "x" << source_image->height 
	     << " from " << imageFile << endl;
      }
  
      getGroundTruthData();

      // Run process starting from desired step
      if (START_FROM_STEP <= 0) {
	cout << "\n\n\n*** EM GRID FIT ***\n\n\n";
	getSvlDetections();
      }

      if (START_FROM_STEP <= 1) {
	cout << "\n\n\n*** EM GRID FIT ***\n\n\n";
	fitGridToDetections(nPixels);
      }

      if (START_FROM_STEP <= 2){
	cout << "\n\n\n*** LABEL DETECTIONS ***\n\n\n";
	labelDetections();
      }

      if (START_FROM_STEP <= 3) {
	cout << "\n\n\n*** LABEL VITERBI ***\n\n\n";
	labelHMM();

	cout << "\n\n\n*** FIND REQUESTED BUTTON ***\n\n\n";
	getButtonInfo();
      }

    }

}


void FindElevatorButtons::getSvlDetections()
{
  
  svlDetections.clear();
    
  //  string model = "plastic_retrain";
  //  string model = "gates_call_buttons";
  string model = DEFAULT_MODEL;
  string name = this->imageName;
  string ground_truths = "";
  bool isCallPanel = false;
  
  string file =this->imageFile;
  svlObject2dFrame frame;
  cout << "SVL: creating Classifier" <<endl;
  svlElevatorClassifier c = svlElevatorClassifier(file,model,isCallPanel,true,ground_truths);
  c.classify();
  
  
  
  bool l = readCacheObject2dFrame(FINAL_DETECTIONS_PATH.c_str(), 
				  (imageName + "_" + model).c_str(),
				  this->svlDetections);
  if (!l){
    cout << "Couldn't load new detections" << endl;
  }
  
  
  
  // Display pruned SVL candidates on the image and save to file.
  IplImage* svl_image = cvCloneImage(source_image);
  for (size_t k=0; k<svlDetections.size(); k++) {
    cvRectangle(svl_image, cvPoint(int(svlDetections[k].x), int(svlDetections[k].y)),
		cvPoint(int(svlDetections[k].x + svlDetections[k].w - 1), 
			int(svlDetections[k].y + svlDetections[k].h - 1)), CV_RGB(0, 255, 0), 1);
  }
  string debugFile = find_button_pkg_path + "/data/debug/" + imageName + "_svl.jpg";
  cvSaveImage(debugFile.c_str(), svl_image);
  cout << "Number of detections: " << svlDetections.size() << endl;
   

}



void FindElevatorButtons::fitGridToDetections(int nPixels)
{
  emg = new EMGridFit(bVerbose);
  gridParams.clear();
  obsDetections.clear();
  cout << "Number of detections: " << svlDetections.size() << endl;
  cout << "Image Name: " << imageName << endl;
  cout << "Image File: " << imageFile << endl;
  emg->computeGridParams(svlDetections, gridParams, obsDetections, nPixels, imageName, imageFile); 
  delete emg;

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
  string debugFile =DEBUG_PATH + imageName + "_em.jpg";
  cvSaveImage(debugFile.c_str(), em_image);
}




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
  hmml = new HMMLabel();
  hmml->getButtonLabels(gridParams,obsDetections,labeledButtons,imageName,imageFile,(START_FROM_STEP==3));
  delete hmml;
}


void FindElevatorButtons::getButtonInfo() 
{
  ros::Node* node = ros::Node::instance();

  stair_msgs::Button tmpButton;
	
  all_buttons.data.clear();

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
    cout << "\n\nALL BUTTON DATA" << endl;
    for (size_t i=0; i<all_buttons.data.size(); i++) {
      cout << "\t" << all_buttons.data[i].label;
      cout << "\t x=" << int(all_buttons.data[i].x);
      cout << "\t y=" << int(all_buttons.data[i].y) << endl;;

    }
  }

  // Get single button if this is an online process
  if (ONLINE_PROCESS==1) {
    cout << "\n Searching for button request" << endl;
    int i = 0;

    // Take last button that's a match - 
    // probably should use probability to find this...
    single_button.x = -1; 
    single_button.y = -1;
    single_button.label = "ButtonDoesNotExist";

    for (; i<all_buttons.data.size(); i++) {
      if (button_request.button_label.compare(all_buttons.data[i].label)==0) {
	if (single_button.x != -1)
	  cout << "Multiple buttons with desired label found. Choosing 'best'." << endl;
	single_button = all_buttons.data[i];	
      }
    }

    if (single_button.x != -1) 
      cout << "FOUND DESIRED BUTTON!" << endl;
    else
      cout << "DID NOT FIND DESIRED BUTTON!" << endl;
    
    /* // Take first button that's a match
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
    */

    node->publish("single_button", single_button);
  } 

	
}

void FindElevatorButtons::getGroundTruthData()
{
  string imageNameSave = imageName;
  string imageFileSave = imageFile;
  string temp_file;
  int isButtonTemp,tempBool,labelIndTemp;
  char labelTemp[10];
  FILE* fid;
  bool done = FALSE;
  float junk;
  grid_param_struct tempGridParams;
  svlObject2d temp_detections;
  button_struct tempObsDetections;

  if (START_FROM_STEP >= 1) { // Populate detections

    temp_file = GROUND_TRUTH_DETECTIONS_PATH + imageNameSave
      + "_obs.txt";
    fid = fopen(temp_file.c_str(),"r");
    assert(fid!=NULL);

    while (done==FALSE) {		
      done = (fscanf(fid,"%lf,%lf,%lf,%d,%f,%s\n",
		     &temp_detections.x,&temp_detections.y,
		     &temp_detections.w,
		     &isButtonTemp,&junk,labelTemp) == EOF);
      if (isButtonTemp == 1) {
	temp_detections.h = temp_detections.w;
	temp_detections.x -= temp_detections.w/2;
	temp_detections.y -= temp_detections.h/2;
	svlDetections.push_back(temp_detections);			
      }
    }
    fclose(fid);
  }

	
  if (START_FROM_STEP >= 2) {

    // Populate grid parameters
    temp_file = GROUND_TRUTH_GRID_PATH + imageNameSave + "_grid.txt";
    fid = fopen(temp_file.c_str(),"r");
    assert(fid!=NULL);
    done = FALSE;

    while(done==FALSE) {
      done = (fscanf(fid,"%d,%d,%d,%lf,%lf,%lf,%lf,%d\n",&tempGridParams.gridNum,
		     &tempGridParams.nrows,&tempGridParams.ncols,&tempGridParams.gx,
		     &tempGridParams.gy,&tempGridParams.dx,&tempGridParams.dy,
		     &tempGridParams.nDetections)==EOF);
      if (done == TRUE)
	break;
      tempGridParams.nDetections = tempGridParams.nrows*tempGridParams.ncols;
      tempGridParams.gridNum--;
      gridParams.push_back(tempGridParams);
    }
    fclose(fid);

    // Populate observationDetections 
    obsDetections.resize(gridParams.size());
    temp_file = GROUND_TRUTH_DETECTIONS_PATH + imageNameSave
      + "_obs.txt";
    fid = fopen(temp_file.c_str(),"r");		
    assert(fid!=NULL);

    for (size_t i=0; i<gridParams.size(); i++)
      {
	for (size_t j=0; j<gridParams[i].ncols*gridParams[i].nrows; j++)
	  {
	    fscanf(fid,"%lf,%lf,%f,%d,%d,%s\n",&tempObsDetections.x,&tempObsDetections.y,
		   &junk,&tempBool,&labelIndTemp,labelTemp);
	    tempObsDetections.isButton = (bool)tempBool;
	    if (START_FROM_STEP >= 3) { // Add label
	      tempObsDetections.label = labelTemp;
	      tempObsDetections.labelInd = labelIndTemp;
	    }

	    obsDetections[i].push_back(tempObsDetections);				

	  }
      }

    fclose(fid);
  }

  imageName = imageNameSave;
  imageFile = imageFileSave;
}


// Get all images in folder with IMAGE_TYPE and IMAGE_TYPE2 strings
// in their file names
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
    if (filename.length() > 2 && filename.find(IMAGE_TYPE)!=string::npos
	&& filename.find(IMAGE_TYPE2)!=string::npos) {
      files.push_back(filename);
    }
  }

  closedir(dp);
  return true;
}


void FindElevatorButtons::shutdown()
{

}

int main (int argc, char **argv)
{
  if (ONLINE_PROCESS) {
    ros::init(argc, argv);
    ros::Node n("find_elevator_buttons");
    FindElevatorButtons buttonFinder;
    buttonFinder.init();
    n.spin();
    buttonFinder.shutdown();
    printf("Ros process find_elevator_buttons is shutting down. \n");

  } else {

    FindElevatorButtons buttonFinder;
    buttonFinder.init();
    buttonFinder.shutdown();

  }

  return 0;
}
