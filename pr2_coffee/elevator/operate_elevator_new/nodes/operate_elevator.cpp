#include "operate_elevator.h"


/********************************Public methods************************/
OperateElevator::OperateElevator() {}

OperateElevator::~OperateElevator() {}

void OperateElevator::init()
{
  ifstream input;
  input.open("input.txt");
  input >> this->panel_side;
  cerr << "panel side " << panel_side << endl;
  input >> buttonCategory;
  cerr << "category " << buttonCategory << endl;
  input >> buttonLabel;
  cerr << "label " << buttonLabel << endl;

  useMP = false;
  isCallPanel = false;
  ONLINE_PROCESS = true;
//  this->panel_side = "right";
//  buttonCategory = "interior";
//  buttonLabel = "5";

  // debugging
  this->use_fake_images = true;
  this->no_arm_move = true;
  this->use_fake_arm_data = true;
  // fake button coords in borg frame.
  fake_arm_data[0] = 0.269248;
  fake_arm_data[1] = -0.413521; 
  fake_arm_data[2] = 0.710658;
  
  this->operate_elevator_pkg_path = ros::getPackagePath("operate_elevator_new");
  
  if(use_fake_images) {
    // Load "fake" image and cloud.
    string num = "5";
    this->srcImageFile = operate_elevator_pkg_path + "/data/test/test" + num + "_flea.jpg";
    this->cloudFilename = operate_elevator_pkg_path + "/data/test/cloudrowcol" + num + ".txt"; 
    this->srcImageFile_canon = operate_elevator_pkg_path + "/data/test/test" + num + "_canon.jpg";
  }
  
  // Initialize class variables.
  bVerbose = true;
  button2DCoord.resize(2);
  button3DCoord.resize(3);
  classifier_done = false;
  button_is_found = false;
  TF_borg2ArmFile = "TF_borg2katana.txt";
  this->readyPosition[0] =180;
  this->readyPosition[1] =120;
  this->readyPosition[2] =70;
  this->readyPosition[3] =240;
  this->readyPosition[4] =180;
  this->readyPosition[5] =50;
  
  armPathGen = new JointPath(this);
  planeFitter = new FitPlane();  

  if (!initTransformations()) {
    printf("Error initializing operate elevator sequence. Exiting!\n");
    return;
  }

  if (ONLINE_PROCESS) {
    ros::Node *node = ros::Node::instance();

    // ros message to request/receive elevator panel button data
    node->advertise<stair_msgs::ButtonRequest>("elevator_buttons_request", 10);
    node->advertise<stair_msgs::ButtonRequest>("elevator_call_panel_buttons_request", 10);
    node->subscribe("single_button", button_location, 
                   &OperateElevator::handleReceivedButtonData, this, 10);
    // ros message to receive request to run operate_elevator sequence
    //node->subscribe("begin_operate_elevator", begin_operate_elevator, 
		//    &OperateElevator::run, this, 10);
    run();
  } else {
    run();
  }

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
  this->timestamp = getTimeStamp();
  cout << "timestamp: " << timestamp << endl;
  
  cout << "PANEL SIDE = " << this->panel_side << endl;

  //if (ONLINE_PROCESS) {
  //  buttonCategory = begin_operate_elevator.button_category;
  //  buttonLabel = begin_operate_elevator.button_label;
  //}
  
  if(this->buttonCategory.compare("exterior") == 0){
    this->isCallPanel=true;
  }else{
    this->isCallPanel=false;
  }

  printf("Beginning press elevator button sequence. \n");
  
  if (!(executeFindButtonSequence())) {
    printf("Error encountered in operate elevator sequence.\n");
  }
  else {
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

  // Get high-res image.
  if (!getHighResImage()) {
    return false; 
  }

  // Publish request to find elevator button in 2D image.
  cout << buttonCategory << endl;
  if(buttonCategory.compare("exterior") == 0){
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
  if (!button_is_found) 
    return false;
  
  cout << "2D button coords in unwarped image: ";
  cout << button2DCoord[0] << " " << button2DCoord[1] << endl << endl;
  cout << "3D button coords: ";
  cout << button3DCoord[0] <<  " " << button3DCoord[1] << " " << button3DCoord[2] << endl;
 
  if (use_fake_arm_data) {
    button3DCoord[0] = fake_arm_data[0];
    button3DCoord[1] = fake_arm_data[1];
    button3DCoord[2] = fake_arm_data[2];
  }

  // convert 3D coords from borg frame to katana frame.
  vector<double> temp (3,0);
  temp[0] = (R_sensor2arm[0][0]*button3DCoord[0] + R_sensor2arm[0][1]*button3DCoord[1] + 
      R_sensor2arm[0][2]*button3DCoord[2] + T_sensor2arm[0]) * 1000.;
  temp[1] = (R_sensor2arm[1][0]*button3DCoord[0] + R_sensor2arm[1][1]*button3DCoord[1] + 
      R_sensor2arm[1][2]*button3DCoord[2] + T_sensor2arm[1]) * 1000;
  temp[2] = (R_sensor2arm[2][0]*button3DCoord[0] + R_sensor2arm[2][1]*button3DCoord[1] + 
      R_sensor2arm[2][2]*button3DCoord[2] + T_sensor2arm[2]) * 1000;
  button3DCoord = temp;
  
  cout << "3D button coords in arm frame: ";
  cout << button3DCoord[0] <<  " " << button3DCoord[1] << " " << button3DCoord[2] << endl;

  // Reset flags
  classifier_done = false;
  button_is_found = false;
  
  if (!moveArmBackToHome()){
    return false;
  }
	
  return true;
}

bool OperateElevator::executePressButton() 
{
  //cout << "Sending command to close gripper." << endl;
  //if (!reqGripperService(false)) return (false);
  
  // Find button/wall normal vector (inward) in sensor frame.
  /*
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
  */

  // Get 3D coordinates for point offset 2 cm from button along outward normal.
  vector<double> goal;
  double offset = 20; // mm
  //for (size_t i=0; i<button3DCoord.size(); i++) {
  //MPgoal.push_back(button3DCoord[i] - (offset*planeNormal[i]));
  //}
  goal.push_back(button3DCoord[0] - offset);
  goal.push_back(button3DCoord[1]);
  goal.push_back(button3DCoord[2]);
  cout << "goal: " << goal[0] << " " << goal[1] << " " << goal[2] << endl;

  // Use motion planner to move arm close to button.
  if (!moveArmToButton(goal)) return false;

  // Get 3D coordinates for point inset from surface of button so arm will apply force.
  double inset = 20; // mm
  goal.clear();
  //for (size_t i=0; i<button3DCoord.size(); i++) {
  //  goal.push_back(button3DCoord[i] + (inset*planeNormal[i]));
  //}
  goal.push_back(button3DCoord[0] + inset);
  goal.push_back(button3DCoord[1]);
  goal.push_back(button3DCoord[2]);
	
  if (!pressButton(goal)) 
    return false;

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
  button_request.image_filename = this->srcImageFile; 
  button_request.cloud_filename = this->cloudFilename; 
  node->publish("elevator_buttons_request", button_request);
}

// find 2D image coordinates for requested button
void OperateElevator::findCallPanelButton()
{
  cout << "Sending request to find_elevator_call_panel_buttons..." << endl;
  ros::Node *node = ros::Node::instance();
  button_request.button_category = buttonCategory;
  button_request.button_label = buttonLabel;
  button_request.image_filename = srcImageFile; 
  node->publish("elevator_call_panel_buttons_request", button_request);
}

void OperateElevator::handleReceivedButtonData()
{
  button2DCoord[0] = button_location.px;
  button2DCoord[1] = button_location.py;
  button3DCoord[0] = button_location.X;
  button3DCoord[1] = button_location.Y;
  button3DCoord[2] = button_location.Z;

  cout << "Response received from elevator button classifier" << endl;
  cout << "[Button.row (Y) Button.col (X) Button.label] = [" << button2DCoord[0];
    cout << " " << button2DCoord[1] << " " << button_location.label <<"]"<< endl;
  cout << "[Button.X Button.Y Button.Z] = [" << button3DCoord[0];
    cout << " " << button3DCoord[1] << " " << " " << button3DCoord[1] << endl;
  
  if (button_location.label == string("NA")) {
    cout << "Button not found." << endl;
  } else {
    button_is_found = true;  
  }
  classifier_done = true;
}


/********************************Private methods******************************/

/* Reads from file coordinate transformation between sensor frame and robot 
 * frame. Assumes a specific format (see Rborg2katana.txt for example). */
bool OperateElevator::initTransformations() 
{
  // Read in sensor to arm transformation.
  R_sensor2arm.clear();
  T_sensor2arm.clear();

  char buffer[400];
  char buf[512];
  char delim[] = "\t\n ";
  char* p = NULL;
  
  sprintf(buffer, "%s%s%s", operate_elevator_pkg_path.c_str(), "/coordTransforms/", 
          TF_borg2ArmFile.c_str());	
  FILE* f = fopen(buffer, "r");
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

bool OperateElevator::getHighResImage()
{
  string command_str;

  if(!use_fake_images) 
  {
    system("gphoto2 --auto-detect");
    system("gphoto2 --capture-image");

    // hacky way of getting only the last captured image
    system("gphoto2 --list-files > tmp.txt");
    system("tail -n 1 tmp.txt > tmp1.txt");
    ifstream tmp_in;
    tmp_in.open("tmp1.txt");
    char pound;
    tmp_in >> pound;
    int num;
    tmp_in >> num;
    string fileStr;
    tmp_in >> fileStr;
    char buffer[20];
    std::ostringstream sin;
    sin << num;
    command_str = "gphoto2 --get-file " + sin.str();
    system(command_str.c_str());

    // move file to data folder
    command_str = string("mv ") + fileStr + " " + operate_elevator_pkg_path + 
      "/data/experiment/img_" + this->timestamp + "_canon.jpg";
    cerr << "command_str " << command_str << endl;
    system(command_str.c_str());  
  } 
  else 
  {
    command_str = string("cp ") + this->srcImageFile_canon + " " + operate_elevator_pkg_path + 
      "/data/experiment/img_" + this->timestamp + "_canon.jpg";
    cerr << "command_str " << command_str << endl;
    system(command_str.c_str());  
  }

  return true;
}

bool OperateElevator::getPointCloud() 
{
  ptCloud_sensor.clear();
  
  if(!use_fake_images){
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
  
  /*********************** Process borg point cloud ****************************/
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
  
  // Save point cloud in sensor frame to time-stamped file.
  if (!saveCloudToFile(ptCloud_sensor, this->timestamp)) return false;

  /*********************** Process borg image ********************************/
  cout << "Borg image file: " << this->srcImageFile << endl;
  
  // Copy src image to data folder.
  string command_str = "cp " + srcImageFile + " " + operate_elevator_pkg_path + 
    "/data/experiment/img_" + this->timestamp + "_flea.jpg";
  cerr << "OperateElevator::getPointCloud::command_str: " << command_str << endl;
  system(command_str.c_str());
  this->srcImageFile = operate_elevator_pkg_path + "/data/experiment/img_" + this->timestamp + "_flea.jpg";

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
  this->cloudFilename = operate_elevator_pkg_path + "/data/experiment/cloud_" + stamp + ".txt";
  cerr << "point cloud file: " << cloudFilename << endl;
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

bool OperateElevator::moveArmToButton(vector<double> goal)
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
    //    armPathGen->callMotionPlanner(ptCloud_MPK, cloudSampleRate, src, goal, 
    //			  wristOrientation, jointAnglePath);
  } else {
    vector<double> curPose, curJoints;
    double delta = 50.; // Linear distance to move between joint angle configurations in mm.

    if(!reqCurPoseService(curPose)) return false;
    if (!reqCurJointAnglesService(curJoints)) return false;
    
    cout << "Generating linear path....";
    if (!armPathGen->generateLinearPath(goal, delta, curPose, curJoints, jointAnglePath))
      return false;
  }
	
  cout << "Sending commands to move arm to button: ";
  cout << goal[0] << " " << goal[1] << " " << goal[2] << endl;
  if (!reqMoveArmService(jointAnglePath)) 
    return (false);
   
  /*
    vector<double> curPose;
    if(!reqCurPoseService(curPose)) return false;
    vector<double> goal_pose;
    for (size_t i=0; i<goal.size(); i++) {
    goal_pose.push_back(goal[i]);
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
  if(!no_arm_move){
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
  if(!no_arm_move){
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
  if(!no_arm_move){
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
  if(!no_arm_move){
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
  if(!no_arm_move){
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
  if(!no_arm_move){
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
  if(!no_arm_move){
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
  if(!no_arm_move){
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
