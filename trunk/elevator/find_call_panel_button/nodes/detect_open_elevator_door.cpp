#include "detect_open_elevator_door.h"

// .outside_elevator = true/false
// detect_open_door.stairIsOutside = true/false
// node->publish("detect_open_door_request",detect_open_door);
// node->advertise<stair_msgs::DetectOpenDoorRequest>("detect_open_door_request",10);


void DetectOpenElevatorDoor::init()
{
  bVerbose = true;
	   
  ros::Node *node = ros::Node::instance();
  node->subscribe("detect_open_door_request",detect_open_door,
                  &DetectOpenElevatorDoor::detectOpenDoor, this, 10);
  node->advertise<stair_msgs::ElevatorDoorStatus>("door_status", 10);

}


/* Goes to scan position from call panel using dead reckoning */
void DetectOpenElevatorDoor::goToScanPosition(void)
{
  if (detect_open_door.stairIsOutside) {
    reqDeadReckonService(0,0,0); // Back up from call panel
  } else {
    reqDeadReckonService(0,0,0); // Back up and face left-hand wall
    reqDeadReckonService(0,0,0); // Go forward and face door
  }
  
}

/* Dead reckoning function */
bool DetectOpenElevatorDoor::reqDeadReckonService(double distance, double heading,
                                           double finalHeading)
{
  
  deadreckon::DriveDeadReckon::Request req;
  deadreckon::DriveDeadReckon::Response res;
  req.dist = distance;
  req.bearing = heading;
  req.finalRelHeading = finalHeading;
  cout << "Deadreckoning distance = " << req.dist << endl;
  cout << "Deadreckoning bearing = " << req.bearing*180./PI << endl;

  cout << "Requesting the DriveDeadReckon service." << endl;
  if (ros::service::call("DriveDeadReckon", req, res))
  {
    printf("Drive deadreckon succeeded. Nav says: [%s]\n", res.status.c_str());
  } else {
    printf("Error using the DriveDeadReckon service.\n");
    return (false);
  }
  
  return (true);
}


/* Checks whether door is open based on values of current scan */
bool DetectOpenElevatorDoor::doorIsOpen()
{
  int max_angle = 45;
  int ind = (90-max_angle)*180/PI*scan_msg.angle_increment;
  int end_index = (90+max_angle)*scan_msg.angle_increment;

  float angle = -PI/4;
  float angle_inc = scan_msg.angle_increment*180/PI;

  int diff_threshold = 80;
  int dist_thresh = 200;
  int nAboveThresh = 0;
  int minFractionAbove = .3*(end_index-ind);

  vector <float> values;
  values.clear();

  // Get values from last scan
  int nValues = (scan_msg.angle_max-scan_msg.angle_max)/scan_msg.angle_increment;
  for (int i=0; i<nValues; i++) {
    values.push_back(scan_msg.ranges[i]*cos(angle));
    angle += angle_inc;
  }
  
  // When STAIR is outside the elevator,
  // we look for a jump greater than difference threshold to check open door
  if (detect_open_door.stairIsOutside) {
    for (; ind < end_index; ind++) {
      if (abs(values[ind]-values[ind+1])>diff_threshold) {
	return(true);
      }
    }
  }

  // When STAIR is inside the elevator,
  // we look for 30% of values to be greater than min_distance threshold
  else {

    for (; ind < end_index; ind++) {
      if (values[ind]>dist_thresh) {
	nAboveThresh++;
      }
    }

    if (nAboveThresh > minFractionAbove) {
      return(true);
    }

  }

  return(false); // Door is still closed...

}


DetectOpenElevatorDoor::DetectOpenElevatorDoor() {}

DetectOpenElevatorDoor::~DetectOpenElevatorDoor() {}

/* Function called when request to check for open elevator door is received */
void DetectOpenElevatorDoor::detectOpenDoor()
{
  bool done = false;
  int secToWait = 5;
  useconds_t usecToWait = secToWait*1000000;
  unsigned int MAX_ITERATIONS = 180/secToWait; // wait 3 minutes max
  unsigned int nIter = 0;
  vector <float> values;

  goToScanPosition();

  ros::Node *node = ros::Node::instance();

  node->subscribe("sicklms",scan_msg,&DetectOpenElevatorDoor::checkForOpenDoor,this,1);

}

/* Function called when new scan is ready to check if door is open */
void DetectOpenElevatorDoor::checkForOpenDoor() 
{
  int secToWait = 5;
  useconds_t usecToWait = secToWait*1000000;
  unsigned int MAX_ITERATIONS = 180/secToWait; // wait 3 minutes max

  // Check if door is open
  if (doorIsOpen()) { 

    // Unsubscribe from sick laser and publish door open message
    ros::Node *node = ros::Node::instance();
    node->unsubscribe("sicklms",&DetectOpenElevatorDoor::checkForOpenDoor,this);

    open_door_status.isOpen = true; 
    node->publish("open_door_status",open_door_status);
  
  } else { // Wait a while until checking the next scan
    usleep(usecToWait);
  }   
}

void DetectOpenElevatorDoor::shutdown()
{
  ros::Node *node = ros::Node::instance();
  node->unadvertise("door_status");
  node->unsubscribe("detect_open_door_request",
		    &DetectOpenElevatorDoor::detectOpenDoor,this);

}


int main (int argc, char **argv)
{
    ros::init(argc, argv);
    ros::Node n("detect_open_elevator_door");
    DetectOpenElevatorDoor openDoorCheck;
    openDoorCheck.init();
    n.spin();
    openDoorCheck.shutdown();
    printf("Ros process detect_open_elevator_door is shutting down.\n");

  return 0;
}
