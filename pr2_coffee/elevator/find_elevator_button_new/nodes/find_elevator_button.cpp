#include "find_elevator_button.h"


void FindElevatorButtons::init()
{
  ONLINE_PROCESS = true;

  captureOpenCVerrors();

  find_button_pkg_path = ros::getPackagePath("find_elevator_button_new");
  // if online process, these will be overwritten
  button_category = "interior";
  button_label = "1";
  //string debug_data_timestamp = "Sep-13-21-17-51";
  string debug_data_timestamp = "Sep-13-22-47-43";

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
  else // For offline process - run on test image.
    {
      // canon should be called "test_canon.jpg";
      image_file = ros::getPackagePath("operate_elevator_new") + string("/data/experiment/img_") + 
	debug_data_timestamp + "_flea.jpg"; 
      cloud_file = ros::getPackagePath("operate_elevator_new") + string("/data/experiment/cloud_") + 
	debug_data_timestamp + ".txt"; 
      findButtons();
    }

}


void FindElevatorButtons::findButtons()
{
  vector<vector<double> > cloud;

  cerr << "\n*** FIND BUTTONS ***\n" << endl;

  if (ONLINE_PROCESS)
    {
      image_file = button_request.image_filename;
      cloud_file = button_request.cloud_filename;
      button_category = button_request.button_category;
      button_label = button_request.button_label;
    } 
 

  // parse image file to extract image name without _flea.jpg or _canon.jpg
  size_t found = image_file.find_last_of("/\\");
  string img = image_file.substr(found+1);
  string path = image_file.substr(0, found+1);
  found = img.find_last_of("_");
  string image_name = img.substr(0, found);
  cerr << "FindElevatorButtons::findButtons::image_name: " << image_name << endl;

  // Apply perspective warping to unwarp the flea image into the wall plane and save to file.
  cerr << "opencv loading image_file: " << image_file << endl;
  IplImage * src_image = cvLoadImage(image_file.c_str());
  string unwarpedImageFile = path + image_name + "_unwarped_flea.jpg";
  unwarpImage(src_image, unwarpedImageFile);
  
  string orig_image_name = image_name;
  image_name = image_name + "_unwarped";
  
  // copy images to correct path
  string command_str = "cp " + unwarpedImageFile + " " + TEST_IMAGES_PATH + image_name + "_flea.jpg";
  cerr << "FindElevatorButtons::findButtons::command_str: " << command_str << endl;
  system(command_str.c_str());
  command_str = "cp " + path + orig_image_name  + "_canon.jpg " + TEST_IMAGES_PATH + image_name + "_canon.jpg";
  cerr << "FindElevatorButtons::findButtons::command_str: " << command_str << endl;
  system(command_str.c_str());
  cvReleaseImage(&src_image);

  // read in point cloud from file
  if (!loadCloud(cloud)) {
    cerr << "\n\n FindElevatorButtons::findButtons ERROR loading point cloud from file!";
    return;
  }

  panel = new ElevPanel(image_name, ELEVDATA_PATH, TEST_IMAGES_PATH); 
  classifier = new ElevClassifier(*panel);
  classifier->getPanel().imageFile = classifier->getPanel().fleaFile;

  system(("rm -Rf "+ classifier->getPanel().overlaysPath).c_str());
  system(("mkdir "+ classifier->getPanel().overlaysPath).c_str());

  cerr << "\n\n\n*** SVL DETECTIONS ***\n\n\n";
  swodData = classifier->runSwod();

  cerr << "\n\n\n*** EM GRID FIT ***\n\n\n";
  emData = classifier->runEm(swodData);
  
  classifier->getPanel().imageFile = classifier->getPanel().canonFile;
  
  //string canon_file = path + orig_image_name  + "_canon.jpg ";
  string canon_file = TEST_IMAGES_PATH + image_name + "_canon.jpg";
  IplImage * canon_img = cvLoadImage(canon_file.c_str(), CV_LOAD_IMAGE_COLOR);
  if (canon_img == NULL) {
    cerr << "error loading " << canon_file.c_str() << endl;
    return;
  }

  // map pixel locations in emData from flea image to canon image
  ElevDataFrame::iterator eIt = emData.begin();
  while(eIt != emData.end())
  { 
    ElevData& data = *eIt;
    ElevDetection& button = data.getButton();
    ElevDetection& label = data.getLabel();
    double button_label_separation = fabs(button.x - label.x);

    cerr << "mapping button... " << endl;
    if (mapPixels(button, cloud)) {
      cerr << "mapping label... " << endl;
      if (!mapPixels(label, cloud)) {
        // set label based on button image coords
        label.x = button.x - button_label_separation * w_scale_factor; 
        label.y = button.y; 
        label.w = label.w * w_scale_factor;
        label.h = label.h * h_scale_factor;
        cerr << "setting label data based on button...." << endl;
        cerr << "[x y w h] = " << label.toString() << endl << endl;
      }
      eIt++;
      
      // for debugging display mapped points on canon image
      cvRectangle(canon_img, cvPoint(int(button.x), int(button.y)),
            cvPoint(int(button.x+button.w), int(button.y+button.h)), CV_RGB(255, 0, 0), 4);
      cvRectangle(canon_img, cvPoint(int(label.x), int(label.y)),
            cvPoint(int(label.x+label.w), int(label.y+label.h)), CV_RGB(0, 0, 255), 4);

    } else {
      eIt = emData.erase(eIt);
    }
  } 
  string name = path + image_name  + "_canon_debug.jpg";
  cvSaveImage(name.c_str(), canon_img);
  cvReleaseImage(&canon_img);

  cerr << "\n\n\n*** LABEL DETECTIONS ***\n\n\n";

  ocrData = classifier->runOcr(emData);

  cerr << "\n\n\n*** RUN HMM ***\n\n\n";
  hmmData = classifier->runHmm(ocrData);

  IplImage* image = cvLoadImage(classifier->getPanel().imageFile.c_str());
  ElevClassification buttons = ElevClassification();

  for(int i=0; i < hmmData.size();i++){
    ElevDetection button = hmmData[i].getButton();
    if(button.pr > buttons[button.label].pr){
      buttons[button.label] = button;
    }
  }
  
  string classFile = classifier->getPanel().overlaysPath + "CLASSIFY: Panel.jpg";
  buttons.saveOverlay(image,classFile);

  cout << buttons.toString() << endl;

  // Write data to single_button!!
  ElevDetection button;
  cerr << "button_request.button_label = " << button_request.button_label << endl;
  cerr << "\n\n\n*** PUBLISHING RESULTS FOR REQUESTED BUTTON ***\n\n\n";
  if(buttons.find(button_request.button_label) != buttons.end()){
    button = buttons[button_request.button_label];
    single_button.px = button.getCenterX();
    single_button.py = button.getCenterY();
    single_button.X = button.Xw;
    single_button.Y = button.Yw;
    single_button.Z = button.Zw;
    single_button.label = button.label;
  }else{
    single_button.label = "NA";
  }

  if (ONLINE_PROCESS) {
    ros::Node *node = ros::Node::instance();
    node->publish("single_button", single_button);
  }

  delete classifier;
  delete panel;
}


  // Assumes point cloud file has already been sorted (in order of pixel row/cols)
  bool FindElevatorButtons::loadCloud(vector<vector<double> > & cloud)
  {
    FILE* f = fopen(this->cloud_file.c_str(), "r");
    if(f == NULL) {
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
    int numPts = cloud.size();
    cout << "Number of pts in point cloud: " << numPts << endl;
    fclose(f);

    return true;
  }


  int main (int argc, char **argv)
  {
//    if (ONLINE_PROCESS) 
//      {
	ros::init(argc, argv);
	ros::Node n("find_elevator_buttons");
	FindElevatorButtons buttonFinder;
	buttonFinder.init();
	n.spin();

  //    } else {

    //  FindElevatorButtons buttonFinder;
    //  buttonFinder.init();

    //}

    return 0;
  }
