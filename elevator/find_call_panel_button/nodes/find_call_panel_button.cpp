#include "find_call_panel_button.h"


void FindCallPanelButton::init()
{
  bVerbose = false;

  ros::Node *node = ros::Node::instance();
  node->subscribe("elevator_call_panel_buttons_request", button_request,
		  &FindCallPanelButton::findButtons, this, 10);
  node->advertise<stair_msgs::Button>("single_button", 10);
  /*
    cout << "creating own start msg" << endl;
    node->advertise<stair_msgs::ButtonRequest>("begin_operate_elevator", 10);
    stair_msgs::ButtonRequest b;
    b.button_label="up";
    b.button_category = "exterior";
    sleep(5);
    node->publish("begin_operate_elevator",b);
  */

}

void FindCallPanelButton::findButtons() 
{
  cout << "find_call_panel_button: searching for buttons" << endl;
  this->image_file = this->button_request.image_filename;

  size_t beginPos = this->image_file.find_last_of("/") + 1;
  size_t len = this->image_file.find_last_of(".") - beginPos;
  this->image_name = this->image_file.substr(beginPos, len);

  string model = "gates_call_buttons";
  string name = this->image_name;
  string gt_file ="";
  bool isCallPanel = true;
  string ground_truths = "";
  string file = this->image_file;
  svlObject2dFrame frame;
  svlElevatorClassifier c = svlElevatorClassifier(file,model,isCallPanel,true,ground_truths);
  cout << "classifying image" << endl;
  c.classify();
  cout << "Done Classifying" << endl;
  bool l = readCacheObject2dFrame(FINAL_DETECTIONS_PATH.c_str(), 
				  (name + "_" + model).c_str(),
				  this->svlDetections);


  if (!l){
    cout << "Couldn't load new detections" << endl;
  }
  string processed_image_file;

  if (bVerbose) {
    // Display received image.
    processed_image_file = DETECTION_OVERLAYS_PATH + name +"_"+model +  "/5_Call_Enhanced.jpg";
    IplImage* cv_image1  = cvLoadImage(processed_image_file.c_str());
    cvNamedWindow("5_Call_Enhanced", 1);
    cout << "DISPLAYING WINDOW of " << processed_image_file << endl;
    cvShowImage("5_Call_Enhanced", cv_image1);
    cvWaitKey(-1);
    cvDestroyAllWindows();
    cvReleaseImage(&cv_image1);
  }


  if(this->svlDetections.size() <1){
    this->single_button.label = "NA";
  }else if(this->button_request.button_label.compare("up") !=0){
    this->single_button.x = this->svlDetections[0].x + this->svlDetections[0].w/2;
    this->single_button.y = this->svlDetections[0].y + this->svlDetections[0].h/2;
    this->single_button.label = "up";
  }else{
    this->single_button.x = this->svlDetections[this->svlDetections.size()>1?1:0].x + this->svlDetections[this->svlDetections.size()>1?1:0].w/2;
    this->single_button.y = this->svlDetections[this->svlDetections.size()>1?1:0].y + this->svlDetections[this->svlDetections.size()>1?1:0].h/2;
    this->single_button.label = "down";
  }
  ros::Node *node = ros::Node::instance();

  node->publish("single_button",this->single_button);
  
}



FindCallPanelButton::FindCallPanelButton() {}

FindCallPanelButton::~FindCallPanelButton() {}



void FindCallPanelButton::shutdown()
{
  ros::Node *node = ros::Node::instance();
  node->unadvertise("door_status");
  node->unsubscribe("elevator_call_panel_buttons_request",
		    &FindCallPanelButton::findButtons,this);
}


int main (int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("find_call_panel_button");
  FindCallPanelButton callPanelButton;
  callPanelButton.init();
  n.spin();
  callPanelButton.shutdown();
  printf("Ros process find_call_panel_button is shutting down.\n");

  return 0;
}
