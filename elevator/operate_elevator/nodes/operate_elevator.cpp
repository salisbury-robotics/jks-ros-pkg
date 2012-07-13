#include "operate_elevator.h"
#include <ctype.h>


// Intialize values for points to generate unwarping matrix.
const float OperateElevator::src_x[4] = {156, 467, 467, 201};
const float OperateElevator::src_y[4] = {159, 145,363 , 348};
const float OperateElevator::dst_x[4] = {201, 467, 467, 201};
const float OperateElevator::dst_y[4] = {159, 159, 348, 348};


/********************************Public methods************************/
OperateElevator::OperateElevator() {}

OperateElevator::~OperateElevator() {}

void OperateElevator::init()
{
  this->operate_elevator_pkg_path = ros::getPackagePath("operate_elevator");
  // debugging
  //this->borg_debug = true;
  //this->arm_debug = true;
  if(borg_debug){
    this->timestamp = "Jul-06-15-41-59";
    //    this->timestamp = "Jun-30-19-35-52";
    //    this->timestamp = "Jun-30-19-34-35";
    //    this->timestamp = "Jun-30-19-27-06";
    //    this->timestamp = "Jun-30-19-25-40";
    // Load "fake" image and cloud.
    this->srcImageFile = operate_elevator_pkg_path + "/data/images/image_" + this->timestamp + ".jpg";
    this->cloudFilename = operate_elevator_pkg_path + "/data/clouds/cloud_" + this->timestamp + ".txt"; 
  }
  
  // Initialize class variables.

  this->readyPosition[0] =180;
  this->readyPosition[1] =120;
  this->readyPosition[2] =70;
  this->readyPosition[3] =240;
  this->readyPosition[4] =180;
  this->readyPosition[5] =50;

  wait_for_start_msg = true;
  bVerbose = true;
  useMP = false;
  button2DCoord.resize(2);
  button3DCoord.resize(3);
  classifier_done = false;
  button_is_found = false;
  TF_borg2ArmFile = "TF_borg2katana.txt";
  TF_arm2MPKFile = "TF_katana2MPK.txt";
  if(!borg_debug){
    this->timestamp = getTimeStamp();
  }
  cout << "timestamp: " << timestamp << endl;

  this->panel_side = "right";
  
  armPathGen = new JointPath(this);
  planeFitter = new FitPlane();
  

  if (!initTransformations()) {
    printf("Error initializing operate elevator sequence. Exiting!\n");
    return;
  }

  ros::Node *node = ros::Node::instance();

  // ros message to request/receive elevator panel button data
  node->advertise<stair_msgs::ButtonRequest>("elevator_buttons_request", 10);
  node->advertise<stair_msgs::ButtonRequest>("elevator_call_panel_buttons_request", 10);
  node->subscribe("single_button", button_2d_location, 
                  &OperateElevator::handleReceivedButtonData, this, 10);
  // ros message to receive request to run operate_elevator sequence
  node->subscribe("begin_operate_elevator", begin_operate_elevator, 
		    &OperateElevator::run, this, 10);

  // run by launching executable instead of waiting for ros msg

}

void OperateElevator::shutdown()
{
  ros::Node *node = ros::Node::instance();
  node->unadvertise("elevator_buttons_request");
  node->unadvertise("elevator_call_panel_buttons_request");
  node->unsubscribe("single_button", &OperateElevator::handleReceivedButtonData, this);
  node->unsubscribe("begin_operate_elevator", &OperateElevator::run, this);
  delete armPathGen;
  delete planeFitter;
}

void OperateElevator::run()
{
  pWarp = new WarpPerspective(false);
  // Initialize unwarping matrix.
  double mat1[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double mat2[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  cvInitMatHeader(&this->unwarp_matrix, 3, 3, CV_64FC1, mat1);
  cvInitMatHeader(&this->warp_matrix, 3, 3, CV_64FC1, mat2);
  pWarp->getWarpingMatrix(this->unwarp_matrix, this->warp_matrix, src_x, src_y, dst_x, dst_y);
  cout << "PANEL SIDE = " << this->panel_side << endl;

  if (wait_for_start_msg) {
    buttonCategory = begin_operate_elevator.button_category;
    buttonLabel = begin_operate_elevator.button_label;
  }
  if(this->buttonCategory.compare("exterior") == 0){
    this->isCallPanel=true;
  }else{
    this->isCallPanel=false;
  }
  printf("Beginning press elevator button sequence. \n");
  
  if (!(executeFindButtonSequence())) {
    printf("Error encountered in operate elevator sequence.\n");
  } else {
    if (!executePressButton()) {
      printf("Error encountered in operate elevator sequence.\n");
    } else {
      printf("Successfully completed operate elevator sequence.\n");
    }
  }

  printf("Waiting for new request to begin press elevator button sequence.... \n");
  return;
}

bool OperateElevator::executeFindButtonSequence() 
{

  if (!moveArmToHome()) {
    return false; 
  }
  if (!moveArmForCamera()) {
    return false;
  }
  
  // Call borg point cloud service.
  if (!getPointCloud()){
    return false; 
  }


  // Publish request to find elevator button in 2D image.
  cout << buttonCategory << endl;
  if(buttonCategory.compare("exterior")==0){
    findCallPanelButton(); 
  }else{
    findButton(); 
  }

  // Wait until elevator button data is received from classifier.
  while(1) {
    if (classifier_done) break;
    usleep(1000);
  }
  
  // Exit if button was not found by classifier.
  if (!button_is_found) return false;
  
  cout << "2D button coords in unwarped image: ";
  cout << button2DCoord[0] << " " << button2DCoord[1] << endl << endl;

  // Reset flags
  classifier_done = false;
  button_is_found = false;
  
  // Transform image coord from unwarped image to original (warped) image.
  vector<double> temp;
  temp.push_back(button2DCoord[1]);
  temp.push_back(button2DCoord[0]);
  temp = pWarp->warpPerspectivePoint(temp, unwarp_matrix); 
  button2DCoord[0] = temp[1];
  button2DCoord[1] = temp[0];
  cout << "Warped 2D button coords (original image): ";
  cout << button2DCoord[0] << " " << button2DCoord[1] << endl << endl;

  // Get corresponding 3D coordinates for button pixel location.
  if(!getButton3DCoord()) {
    return false;
  } 

  if (!moveArmBackToHome()){
    return false;
  }
	
  return true;
}

bool OperateElevator::executePressButton() 
{
  //cout << "Sending command to close gripper." << endl;
  if (!reqGripperService(false)) return (false);
  
  // Find button/wall normal vector (inward) in sensor frame.
  vector<double> planeNormal, planeCentroid;
  double windowSize = 25.;
  if (!getPlaneNormal(planeNormal, planeCentroid, windowSize)) return false;
  if (planeNormal[0] < 0) {
    for (size_t i=0; i<planeNormal.size(); i++) {
      planeNormal[i] *= (-1.);
    }
  }

  // Transform normal vector from sensor frame into robot arm frame.
  vector<double> temp = planeNormal;
  for (size_t k=0; k<temp.size(); k++) {
    planeNormal[k] = R_sensor2arm[k][0]*temp[0] + R_sensor2arm[k][1]*temp[1] + 
      R_sensor2arm[k][2]*temp[2] + T_sensor2arm[k];
  }
  if (bVerbose) {
    cout << "Normal vector to plane in robot frame: ";
    for (size_t i=0; i<planeNormal.size(); i++) {
      cout << planeNormal[i] << " ";
    }
    cout << endl;
  }

  // Get 3D coordinates for point offset 2 cm from button along outward normal.
  vector<double> MPgoal;
  double offset = 20; // mm
  //for (size_t i=0; i<button3DCoord.size(); i++) {
  //MPgoal.push_back(button3DCoord[i] - (offset*planeNormal[i]));
  //}
  MPgoal.push_back(button3DCoord[0] - offset);
  MPgoal.push_back(button3DCoord[1]);
  MPgoal.push_back(button3DCoord[2]);
  cout << "MPgoal: " << MPgoal[0] << " " << MPgoal[1] << " " << MPgoal[2] << endl;

  // Use motion planner to move arm close to button.
  if (!moveArmToButton(MPgoal)) return false;

  // Get 3D coordinates for point inset from surface of button so arm will apply force.
  double inset = 20; // mm
  vector<double> goal;
  //for (size_t i=0; i<button3DCoord.size(); i++) {
  //  goal.push_back(button3DCoord[i] + (inset*planeNormal[i]));
  //}
  goal.push_back(button3DCoord[0] + inset);
  goal.push_back(button3DCoord[1]);
  goal.push_back(button3DCoord[2]);
	
  if (!pressButton(goal)) return false;


  moveRobotJoint(readyPosition);
  moveSingleJoint(0,90);
  moveSingleJoint(3,90);
  moveSingleJoint(2,90);
  moveSingleJoint(1,90);

  if (!moveArmToHome()) return false;

  return true;
}

bool OperateElevator::moveRobotJoint(double degrees[]){
  vector<double> joints;
  vector<vector<double> > jointAnglePath;
  for (int i=0; i<nJoints; i++) {
    joints.push_back(readyPosition[i]*PI/180.);
  }
  jointAnglePath.push_back(joints);
  
  cout << "Sending commands to move arm to ready position." << endl;
  if (!reqMoveArmService(jointAnglePath)) 
    return (false);
  return true;
}
bool  OperateElevator::moveSingleJoint(int joint, int degree){
  vector<double> joints;
  vector<vector<double> > jointAnglePath;
  vector<double> src;
  if (!reqCurJointAnglesService(src)) return (false);	
  src[joint]=PI*degree/180.;
  for (int i=0; i<nJoints; i++) {
    joints.push_back(src[i]);
  }
  jointAnglePath.push_back(joints);
  cout << "Moving Single Joint " << joint << " top degree: " << degree << endl;
  if (!reqMoveArmService(jointAnglePath)) 
    return (false);
  return true;
}
// find 2D image coordinates for requested button
void OperateElevator::findButton()
{
  cout << "Sending request to find_elevator_buttons..." << endl;
  ros::Node *node = ros::Node::instance();
  button_request.button_category = buttonCategory;
  button_request.button_label = buttonLabel;
  button_request.image_filename = unwarpedImageFile; 
  node->publish("elevator_buttons_request", button_request);
}

// find 2D image coordinates for requested button
void OperateElevator::findCallPanelButton()
{
  cout << "Sending request to find_elevator_call_panel_buttons..." << endl;
  ros::Node *node = ros::Node::instance();
  button_request.button_category = buttonCategory;
  button_request.button_label = buttonLabel;
  button_request.image_filename = unwarpedImageFile; 
  node->publish("elevator_call_panel_buttons_request", button_request);
}

void OperateElevator::handleReceivedButtonData()
{
  button2DCoord[0] = button_2d_location.y/this->coord_modifier;
  button2DCoord[1] = (button_2d_location.x)/this->coord_modifier;
  cout << "Response received from elevator button classifier" << endl;
  cout << "[Button.row (Y) Button.col (X) Button.label] = [" << button2DCoord[0];
  cout << " " << button2DCoord[1] << " " << button_2d_location.label <<"]"<< endl;
  if (button_2d_location.label == string("NA")) {
    cout << "Button not found." << endl;
  } else {
    button_is_found = true;  
  }
  classifier_done = true;
}

// Get 3D button location from borg scan data -- if no 3d data available, return false.
bool OperateElevator::getButton3DCoord()
{
  if (bVerbose) {
    cout << "Looking for corresponding 3D data for button pixel coordinates....." << endl;
  }
  
  vector<double> candidate(ptCloud_sensor[0].size(),0);
  vector<double> x_candidates, y_candidates, z_candidates;
  double dist;
  size_t numPts = ptCloud_sensor.size();
  
  if (bVerbose) {
    cout << "Looking for pixels within searchDist....." << endl;
  }
  // Find pixels within searchDist of desired location.
  for (size_t i=0; i<numPts; i++) {
    dist = sqrt((button2DCoord[0] - ptCloud_sensor[i][0])*(button2DCoord[0] - ptCloud_sensor[i][0]) +
                (button2DCoord[1] - ptCloud_sensor[i][1])*(button2DCoord[1] - ptCloud_sensor[i][1]));
    if (dist <= searchDist) {
      x_candidates.push_back(ptCloud_sensor[i][2]);
      y_candidates.push_back(ptCloud_sensor[i][3]);
      z_candidates.push_back(ptCloud_sensor[i][4]);
    }
  }

  if (bVerbose) {
    cout << "Finding median of all candidates....." << endl;
  }
  // Choose median of these points.
  if (x_candidates.size() > 0) {
    sort(x_candidates.begin(), x_candidates.end());
    sort(y_candidates.begin(), y_candidates.end());
    sort(z_candidates.begin(), z_candidates.end());
    candidate[2] = x_candidates[int(x_candidates.size()/2)];
    candidate[3] = y_candidates[int(y_candidates.size()/2)];
    candidate[4] = z_candidates[int(z_candidates.size()/2)];
    button3DCoord[0] = candidate[2];
    button3DCoord[1] = candidate[3];
    button3DCoord[2] = candidate[4];
    cout << "3D coord of button in borg frame: ";
    cout << button3DCoord[0] << " "<< button3DCoord[1] << " " << button3DCoord[2] << endl;
  } else {
    cout << "Point cloud does not contain 3D coordinates for button." << endl;
  }

  // transform from sensor frame to robot frame
  vector<double> temp = button3DCoord;
  for (size_t k=0; k<button3DCoord.size(); k++) {  
    button3DCoord[k] = R_sensor2arm[k][0]*temp[0] + R_sensor2arm[k][1]*temp[1] + 
      R_sensor2arm[k][2]*temp[2] + T_sensor2arm[k];
  }
  
  // Convert button coordinates from m to mm for Katana IK.
  for (size_t k=0; k<button3DCoord.size(); k++) {
    button3DCoord[k] *= 1000.;
  }

  cout << "3D coord of button in robot frame (mm): ";
  cout << button3DCoord[0] << " "<< button3DCoord[1] << " " << button3DCoord[2] << endl;

  return true;
}

/********************************Private methods******************************/

/* Reads from file coordinate transformation between sensor frame and robot 
 * frame. Assumes a specific format (see Rborg2katana.txt for example). */
bool OperateElevator::initTransformations() 
{
  // Read in arm to MPK (motion planner) transformation.
  R_arm2MPK.clear();
  T_arm2MPK.clear();
  char buffer[400];
  char buf[512];
  char* p = NULL;
  char delim[] = "\t\n ";

  sprintf(buffer, "%s%s%s", operate_elevator_pkg_path.c_str(), "/coordTransforms/", 
          TF_arm2MPKFile.c_str());	
  FILE* f = fopen(buffer, "r");
  if (f == NULL) {
    printf("Couldn't read arm to MPK transformation file.\n");
    return 1;
  }

  // Read R token
  if (fgets(buf, 512, f) == NULL) return false;
  p = strtok(buf, delim);
  // Read in rotation matrix
  for (unsigned int i=0; i<3; i++) {
    if (fgets(buf, 512, f) == NULL) return false;
    vector<double> v;
    p = strtok(buf, delim); 
    v.push_back(atof(p));
    for (unsigned int j=1; j<3; j++) {
      p = strtok(NULL, delim);
      v.push_back(atof(p));
    }
    R_arm2MPK.push_back(v);
  }
  cout << "R: " << endl;
  cout << R_arm2MPK[0][0] << " " << R_arm2MPK[0][1] << " " << R_arm2MPK[0][2] << endl; 
  cout << R_arm2MPK[1][0] << " " << R_arm2MPK[1][1] << " " << R_arm2MPK[1][2] << endl; 
  cout << R_arm2MPK[2][0] << " " << R_arm2MPK[2][1] << " " << R_arm2MPK[2][2] << endl;

  // Read T token
  if (fgets(buf, 512, f) == NULL) return false;
  p = strtok(buf, delim);
  // Read in translation vector. 
  if (fgets(buf, 512, f) == NULL) return false;
  p = strtok(buf, delim);
  T_arm2MPK.push_back(atof(p));
  for (unsigned int i=1; i<3; i++) { 
    p = strtok(NULL, delim);
    T_arm2MPK.push_back(atof(p));
  }
  cout << "T: " << endl;
  cout << T_arm2MPK[0] << " " << T_arm2MPK[1] << " " << T_arm2MPK[2] << endl; 
  fclose(f);
	
  // Read in sensor to arm transformation.
  R_sensor2arm.clear();
  T_sensor2arm.clear();
	
  p = NULL;
  sprintf(buffer, "%s%s%s", operate_elevator_pkg_path.c_str(), "/coordTransforms/", 
          TF_borg2ArmFile.c_str());	
  f = fopen(buffer, "r");
  if (f == NULL) {
    printf("Couldn't read arm sensor to arm file.\n");
    return 1;
  }

  // Read R token
  if (fgets(buf, 512, f) == NULL) return false;
  p = strtok(buf, delim);
  // Read in rotation matrix
  for (unsigned int i=0; i<3; i++) {
    if (fgets(buf, 512, f) == NULL) return false;
    vector<double> v;
    p = strtok(buf, delim); 
    v.push_back(atof(p));
    for (unsigned int j=1; j<3; j++) {
      p = strtok(NULL, delim);
      v.push_back(atof(p));
    }
    R_sensor2arm.push_back(v);
  }
  cout << "R: " << endl;
  cout << R_sensor2arm[0][0] << " " << R_sensor2arm[0][1] << " " << R_sensor2arm[0][2] << endl; 
  cout << R_sensor2arm[1][0] << " " << R_sensor2arm[1][1] << " " << R_sensor2arm[1][2] << endl; 
  cout << R_sensor2arm[2][0] << " " << R_sensor2arm[2][1] << " " << R_sensor2arm[2][2] << endl; 

  // Read T token
  if (fgets(buf, 512, f) == NULL) return false;
  p = strtok(buf, delim);
  // Read in translation vector. 
  if (fgets(buf, 512, f) == NULL) return false;
  p = strtok(buf, delim);
  T_sensor2arm.push_back(atof(p));
  for (unsigned int i=1; i<3; i++) { 
    p = strtok(NULL, delim);
    T_sensor2arm.push_back(atof(p));
  }
  cout << "T: " << endl;
  cout << T_sensor2arm[0] << " " << T_sensor2arm[1] << " " << T_sensor2arm[2] << endl; 
  fclose(f);

  return true;
}



bool OperateElevator::getPointCloud() 
{
  ptCloud_sensor.clear();
  ptCloud_MPK.clear();
  
  if(!borg_debug){
    // Request borg scan service.
    borg::Filepaths::Request req;
    borg::Filepaths::Response res;
    cout << "Requesting the get_borg_scan service." << endl;
    if (!ros::service::call("get_borg_scan", req, res)) {
      cout << "Error calling get_borg_scan service." << endl; 
      return false;
    }
    cout << "Borg done scanning." << endl;
  
    this->cloudFilename = res.cloud_file;
    this->srcImageFile = res.image_file;
  }
  cout << "Borg point cloud file: " << this->cloudFilename << endl;
  
  // Read in point cloud from file.
  FILE* f = fopen(this->cloudFilename.c_str(), "r");
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
    //point[0] = floor(point[0] + 0.5);
    //point[1] = floor(point[1] + 0.5);
    ptCloud_sensor.push_back(point);
  }
  
  int numPts = ptCloud_sensor.size();
  cout << "Number of pts in point cloud: " << numPts << endl;
  fclose(f);

  // Sort point cloud by increasing pixel row number and increasing column number.
  cout << "sorting point cloud by row and column...." << endl;
  
  sort(ptCloud_sensor.begin(), ptCloud_sensor.end(), compare_row_functor());
  
  vector<vector<double> > cloud;
  vector<vector<double> > sub_cloud;
  int i=0; 
  for (int k=0; k<ptCloud_sensor[numPts-1][0]; k++) {
    while(ptCloud_sensor[i][0] == k) {
      sub_cloud.push_back(ptCloud_sensor[i]);
      i++;
    }
    sort(sub_cloud.begin(), sub_cloud.end(), compare_col_functor());
    for (size_t j=0; j<sub_cloud.size(); j++) {
      cloud.push_back(sub_cloud[j]);
    }
    sub_cloud.clear();
  }
  ptCloud_sensor = cloud;

  numPts = ptCloud_sensor.size();
  cout << "Number of pts in point cloud: " << numPts << endl;
  
  cout << "Transforming point cloud to MPK frame..." << endl;

  vector<double> temp(3,0);  // x, y, z
  vector<double> point(5,0);
  for (int i=0; i<numPts; i++) 
    {
      point[0] = ptCloud_sensor[i][0];
      point[1] = ptCloud_sensor[i][1];
   
      // Transform from borg to katana frame.
      temp[0] = R_sensor2arm[0][0]*ptCloud_sensor[i][2] + R_sensor2arm[0][1]*ptCloud_sensor[i][3] + 
	R_sensor2arm[0][2]*ptCloud_sensor[i][4] + T_sensor2arm[0];
      temp[1] = R_sensor2arm[1][0]*ptCloud_sensor[i][2] + R_sensor2arm[1][1]*ptCloud_sensor[i][3] + 
	R_sensor2arm[1][2]*ptCloud_sensor[i][4] + T_sensor2arm[1];
      temp[2] = R_sensor2arm[2][0]*ptCloud_sensor[i][2] + R_sensor2arm[2][1]*ptCloud_sensor[i][3] + 
	R_sensor2arm[2][2]*ptCloud_sensor[i][4] + T_sensor2arm[2];
   
      // Transorm from katana frame to MPK frame.
      point[2] = R_arm2MPK[0][0]*temp[0] + R_arm2MPK[0][1]*temp[1] + R_arm2MPK[0][2]*temp[2] + T_arm2MPK[0];
      point[3] = R_arm2MPK[1][0]*temp[0] + R_arm2MPK[1][1]*temp[1] + R_arm2MPK[1][2]*temp[2] + T_arm2MPK[1];
      point[4] = R_arm2MPK[2][0]*temp[0] + R_arm2MPK[2][1]*temp[1] + R_arm2MPK[2][2]*temp[2] + T_arm2MPK[2];
    
      ptCloud_MPK.push_back(point);
    }
 


  // Save point cloud in sensor frame to time-stamped file.
  if (!saveCloudToFile(ptCloud_sensor, this->timestamp)) return false;

  cout << "Borg image file: " << this->srcImageFile << endl;
  
  // Load borg image and save to time-stamped file.
  cv_image = cvLoadImage(this->srcImageFile.c_str());
  
  srcImageFile = operate_elevator_pkg_path + "/data/images/image_" + 
    this->timestamp + string(".jpg");

  // Save image to file.
  cout << "Saving image to file...." << endl;
  cvSaveImage(srcImageFile.c_str(), cv_image);


  // Apply perspective warping to unwarp the image into the wall plane.
  IplImage* unwarped_image = pWarp->warpPerspectiveImage(cv_image, warp_matrix);
  cvSaveImage(( operate_elevator_pkg_path + "/data/images/image_" + "TEST_SAVE_1.jpg").c_str(), unwarped_image);
  IplImage* fp;
  this->coord_modifier = this->isCallPanel ? 1.5 : 2;
  fp = cvCreateImage(cvSize(unwarped_image->width*this->coord_modifier, unwarped_image->height*this->coord_modifier), unwarped_image->depth, unwarped_image->nChannels);
  cvResize(unwarped_image, fp); 
  cvSaveImage(( operate_elevator_pkg_path + "/data/images/image_" + "TEST_SAVE_2.jpg").c_str(), fp);
  if(this->isCallPanel){ 
    cout << "modifying image" << endl;
    CvScalar s;
    for(int n=0; n<fp->nChannels;n++){
      s.val[n] =0;
    }
    this->button_x_modifier = fp->width/3;
    for(int x =0; x<this->button_x_modifier;x++){
      for(int y =0; y<fp->height;y++){
	cvSet2D(fp,y,x,s);
      }
    }
    for(int x =fp->width-this->button_x_modifier;x<fp->width;x++){
      for(int y =0; y<fp->height;y++){
	cvSet2D(fp,y,x,s);
      }
    }
  }
  // Save unwarped image to file.
  unwarpedImageFile = operate_elevator_pkg_path + "/data/images/image_unwarped_" + 
    this->timestamp + ".jpg";


  /*
    cvSetImageROI(unwarped_image, cvRect(unwarped_image->width,0, unwarped_image->width, unwarped_image->height));
    fp = cvCreateImage(cvSize(unwarped_image->width, unwarped_image->height), unwarped_image->depth, unwarped_image->nChannels);
    cvCopy(unwarped_image, fp);

    cvSaveImage(unwarpedImageFile.c_str(), fp);
  
  */
  cvSaveImage(unwarpedImageFile.c_str(), fp);
  cvReleaseImage(&fp);

  cvReleaseImage(&cv_image);

  cvReleaseImage(&unwarped_image);
  return true;
}

 
string OperateElevator::getTimeStamp()
{
  time_t rawtime;
  time(&rawtime);
  struct tm* timeinfo = localtime(&rawtime);
  char* imageStamp = asctime(timeinfo);
 
  cout << "local time: " << imageStamp << endl;

  string imageStampStr = "";
  for (int i=4; i<7; i++) {
    imageStampStr += imageStamp[i];
  }
  for (int j=0; j<4; j++) {
    imageStampStr += string("-");
    for (int i=8+j*3; i<8+(j*3)+2; i++) {
      if (isspace(imageStamp[i])) {
	imageStampStr += "0";
      } else {
	imageStampStr += imageStamp[i];
      }
    }
  }
  return imageStampStr;
}


bool OperateElevator::saveCloudToFile(vector<vector<double> > &cloud, string stamp)
{
  cloudFilename = operate_elevator_pkg_path + string("/data/clouds/cloud_") + stamp + string(".txt");
  FILE *f;
  f = fopen(cloudFilename.c_str(), "w");
  if (f == NULL) {
    cout << "Error saving point cloud to file." << endl;
    return false;
  }
  size_t numPts = cloud.size();
  for (size_t i=0; i<numPts; i++) {
    fprintf(f, "%f %f %f %f %f\n", cloud[i][0], cloud[i][1], cloud[i][2], cloud[i][3], cloud[i][4]);
  }
  fclose(f);
  return true;
}

bool OperateElevator::moveArmToButton(vector<double> MPgoal)
{
  double wristOrientation = initWristOrientation;
  vector<double> joints;
  vector<vector<double> > jointAnglePath;


  // Move arm to ready position.
  moveSingleJoint(0,90);
  this->readyPosition[0]= 90;
  moveRobotJoint(this->readyPosition);
  this->readyPosition[0]= 180;
  moveSingleJoint(0,180);


  if (useMP) {
    // Call motion planner.
    cout << "Requesting motion plan....";
    //    armPathGen->callMotionPlanner(ptCloud_MPK, cloudSampleRate, src, MPgoal, 
    //			  wristOrientation, jointAnglePath);
  } else {
    vector<double> curPose, curJoints;
    double delta = 50.; // Linear distance to move between joint angle configurations in mm.

    if(!reqCurPoseService(curPose)) return false;
    if (!reqCurJointAnglesService(curJoints)) return false;
    
    cout << "Generating linear path....";
    if (!armPathGen->generateLinearPath(MPgoal, delta, curPose, curJoints, jointAnglePath))
      return false;
  }
	
  cout << "Sending commands to move arm to button: ";
  cout << MPgoal[0] << " " << MPgoal[1] << " " << MPgoal[2] << endl;
  if (!reqMoveArmService(jointAnglePath)) 
    return (false);
   
  /*
    vector<double> curPose;
    if(!reqCurPoseService(curPose)) return false;
    vector<double> goal_pose;
    for (size_t i=0; i<MPgoal.size(); i++) {
    goal_pose.push_back(MPgoal[i]);
    }
    goal_pose.push_back(curPose[3]);
    goal_pose.push_back(curPose[4]);
    goal_pose.push_back(curPose[5]);
    cout << "Sending pose to move arm to button: ";
    for (size_t i=0; i<goal_pose.size(); i++) {
    cout << goal_pose[i] << " ";
    }
    cout << endl;
    if(!reqLinearMoveService(goal_pose))
    return false;
  */

  return true;
}

// Get normal vector to the wall (button) plane in sensor frame.
bool OperateElevator::getPlaneNormal(vector<double> &normal, 
				     vector<double> &centroid, double windowSize) 
{
  vector<vector<double> > plane_points;
  double dist;
  double pixel_row = button2DCoord[0];
  double pixel_col = button2DCoord[1];
  double drow, dcol;

  // Find points within windowSize pixels of desired location.
  for (size_t i=0; i<ptCloud_sensor.size(); i++) {
    drow = pixel_row - ptCloud_sensor[i][0];
    dcol = pixel_col - ptCloud_sensor[i][1];
    dist = sqrt((drow*drow) + (dcol*dcol));
    if (dist <= windowSize) {
      vector<double> temp;
      temp.push_back(ptCloud_sensor[i][2]);
      temp.push_back(ptCloud_sensor[i][3]);
      temp.push_back(ptCloud_sensor[i][4]);
      plane_points.push_back(temp);
    }
  }

  if (plane_points.size() < 3) {
    cout << "Error: Not enought points to fit plane." << endl;
    return false;
  }

  planeFitter->fit_plane(plane_points, normal, centroid);
  return true;
}

// Move end effector along a linear path with wrist orientation fixed to press button.
// Retract from button along reverse path.
bool OperateElevator::pressButton(vector<double> goal)
{
  // Generate joint sequence to move end effector along button surface normal and
  // apply some force.
  vector<vector<double> > jointAnglePath;
  vector<double> curPose, curJoints;
  double delta = 25.; // Linear distance to move between joint angle configurations in mm. 

  if(!reqCurPoseService(curPose)) return false;
  if (!reqCurJointAnglesService(curJoints)) return false;
  if (!armPathGen->generateLinearPath(goal, delta, curPose, curJoints, jointAnglePath))
    return false;

  cout << "Moving end effector to: ";
  for (size_t k=0; k<goal.size(); k++) {
    cout << goal[k]<< " "; 
  }
  cout << endl;
  if (!reqMoveArmService(jointAnglePath)) return false;

  // Retract from button -- reverse order of joint angle commands for pressing button.
  vector<vector<double> > prevPath = jointAnglePath;
  int len1 = prevPath.size();
  int len2 = prevPath[0].size();
  for (size_t k=0; k<len1; k++) {
    for (size_t j=0; j<len2; j++) {
      jointAnglePath[k][j] = prevPath[(len1-1)-k][j]; 
    }
  }
  
  cout << "Moving end effector to: ";
  for (size_t k=0; k<goal.size(); k++) {
    cout << goal[k]<< " "; 
  }
  cout << endl;
  if (!reqMoveArmService(jointAnglePath)) return false;

  return true;
}

bool OperateElevator::moveArmToHome() 
{
  if(!arm_debug){
    katana450::StringString::Request req;
    katana450::StringString::Response res;
    cout << "Requesting the katana_move_upright service." << endl;
    bool success = ros::service::call("katana_move_upright_service", req, res);
    printf("Katana says: [%s]\n", res.str.c_str());
    return success;
  }else{
    return true;
  }
}

bool OperateElevator::moveArmForCamera() 
{
  if(!arm_debug){
    katana450::StringString::Request req;
    katana450::StringString::Response res;
    // want arm to move away from wall
    if (panel_side == string("left")) {
      req.str = "right";
    } else if (panel_side == string("right")) {
      req.str = "left";
    }
    cout << "Requesting the katana_move_for_camera service." << endl;
    bool success = ros::service::call("katana_move_for_camera_service", req, res);
    printf("Katana says: [%s]\n", res.str.c_str());
    return success;
  }else{
    return true;
  }
}

bool OperateElevator::moveArmBackToHome() 
{
  if(!arm_debug){
    katana450::StringString::Request req;
    katana450::StringString::Response res;
    if (panel_side == string("left")) {
      req.str = "right";
    } else if (panel_side == string("right")) {
      req.str = "left";
    }
    cout << "Requesting the katana_back_to_upright_service service." << endl;
    bool success = ros::service::call("katana_back_to_upright_service", req, res);
    printf("Katana says: [%s]\n", res.str.c_str());
    return success;
  }else{
    return true;
  }
}

bool OperateElevator::reqLinearMoveService(vector<double> goalPose) 
{
  if(!arm_debug){
    katana450::KatanaPose::Request req;
    katana450::KatanaPose::Response res;
    req.pose.x = goalPose[0];
    req.pose.y = goalPose[1];
    req.pose.z = goalPose[2];
    req.phi = goalPose[3];
    req.theta = goalPose[4];
    req.psi = goalPose[5];

    // Call Katana linear move.
    cout << "Requesting katana_move_linear_service." << endl;
    if (!ros::service::call("katana_move_linear_service", req, res)) {
      cout << "Linear move could not find kni solution!" << endl;
    }

    return (true);
  }else{
    return true;
  }
}

// Gripper commands can be 0 (closed) or 1 (open) only.
bool OperateElevator::reqGripperService(bool gripperOpen) 
{
  if(!arm_debug){
    katana450::UInt32String::Request req;
    katana450::UInt32String::Response res;
    req.value = gripperOpen;
	
    cout << "Requesting the katana_gripper_cmd_service." << endl;
    bool success = ros::service::call("katana_gripper_cmd_new_service", req, res);
    printf("Katana says: [%s]\n", res.str.c_str());
    return (success);
  }else{
    return true;
  }
}
  
bool OperateElevator::reqMoveArmService(vector<vector<double> > &joints) 
{
  if(!arm_debug){
    katana450::ArmCSpaceSeqString::Request req;
    katana450::ArmCSpaceSeqString::Response res;
  
    req.jointAngles.set_configs_size(joints.size());
    for (size_t i=0; i<joints.size(); i++) {
      req.jointAngles.configs[i].set_angles_size(5);
    }
    for (size_t i=0; i<joints.size(); i++) {
      for (size_t j=0; j<5; j++) {
	req.jointAngles.configs[i].angles[j] = joints[i][j];
      }
    }

    cout << "Requesting the katana_move_joint_sequence_rad_service." << endl;
    bool success = ros::service::call("katana_move_joint_sequence_rad_service", req, res);
    printf("Katana says: [%s]\n", res.str.c_str());
    return (success);
  }else{
    return true;
  }
}

bool OperateElevator::reqCurJointAnglesService(vector<double> &joints) 
{
  if(!arm_debug){
    joints.clear();
    katana450::StringArmCSpace::Request req;
    katana450::StringArmCSpace::Response res;
    cout << "Requesting the katana_get_joint_angles service." << endl;
    if (!ros::service::call("katana_get_joint_angles_service", req, res)) {
      printf("Error using the get_current_joint_angles service.\n");
      return (false);
    }
    cout << "Current joint angles: ";
    for (size_t i=0; i<res.jointAngles.get_angles_size(); i++) {
      joints.push_back(res.jointAngles.angles[i]*PI/180.);
      cout << res.jointAngles.angles[i]*PI/180. << " ";
    }
    cout << endl;
    return (true);
  }else{
    cout << "Current joint angles: ";
    for (size_t i=0; i<this->nJoints; i++) {
      joints.push_back(this->readyPosition[i]);
      cout << joints[i] << " ";
    }
    cout << endl;

    return true;
  }
}

bool OperateElevator::reqCurPoseService(vector<double> &pose) 
{
  if(!arm_debug){
    pose.clear();
    katana450::KatanaPose::Request req;
    katana450::KatanaPose::Response res;
    // Get current end-effector xyz coordinates from Katana server.
    cout << "Requesting katana_get_pose_service." << endl;
    if (!ros::service::call("katana_get_pose_service", req, res)) {
      cout << "Error using get_pose_service. Exiting!" << endl;
      return (false);
    }
    pose.push_back(res.pose.x);
    pose.push_back(res.pose.y);
    pose.push_back(res.pose.z);
    pose.push_back(res.phi);
    pose.push_back(res.theta);
    pose.push_back(res.psi);
    cout << "Current pose: ";
    for (size_t i=0; i<pose.size(); i++) {
      cout << pose[i] << " ";
    }
    cout << endl;

    return (true);
  }else{
    pose.push_back(250);
    pose.push_back(0);
    pose.push_back(100);
    pose.push_back(10);
    pose.push_back(180);
    pose.push_back(0);
    cout << "Current pose: ";
    for (size_t i=0; i<pose.size(); i++) {
      cout << pose[i] << " ";
    }
    cout << endl;

    return true;
  }
}

bool OperateElevator::ik_joint_solution(double x, double y, double z, double theta_init,
					double psi, double max_theta_dev, vector<double> &solution)
{
  katana450::KatanaIK::Request  req;
  katana450::KatanaIK::Response res;
  req.pose.x = x;
  req.pose.y = y;
  req.pose.z = z;
  req.theta = theta_init;
  req.psi = psi;
  req.max_theta_dev = max_theta_dev;
  cout << "Requesting the katana ik service." << endl;
  if (!ros::service::call("katana_ik_calculate_service", req, res)) {
    printf("Error using the katana_ik_calculate service.\n");
    return (false);
  }
  res.solution.get_angles_vec(solution);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("operate_elevator");
	
  OperateElevator operate_elevator_proc;
  operate_elevator_proc.init();
	
  n.spin();

  operate_elevator_proc.shutdown();
  printf("operate_elevator ros node is shutting down.\n");

  return 0;
}
