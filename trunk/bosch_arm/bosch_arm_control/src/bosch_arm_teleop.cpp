
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_SPACE 0x20

#define KEYCODE_MINUS 0x2d
#define KEYCODE_EQUAL 0x3d
#define KEYCODE_PLUS 0x2b


double max_effort = 20;

class TBK_Node
{
  private:
    geometry_msgs::Twist cmdvel;
    ros::NodeHandle n_;
    ros::Publisher pub_;

  public:
    TBK_Node()
    {
      pub_ = n_.advertise<geometry_msgs::Twist>("base_controller/command",10);
    }
    ~TBK_Node() { }
    void keyboardLoop();
    void pub_cmdvel();
    void stopRobot()
    {
      cmdvel.linear.x = cmdvel.angular.z = 0.0;
      pub_.publish(cmdvel);
    }
};

TBK_Node* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int
main(int argc, char** argv)
{
  ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  TBK_Node tbk;

        // start two threads: waiting for keyboard and publishing velocity message
  boost::thread t = boost::thread::thread(boost::bind(&TBK_Node::keyboardLoop, &tbk));
  boost::thread t2 = boost::thread::thread(boost::bind(&TBK_Node::pub_cmdvel, &tbk));

  ros::spin();

  t.interrupt();
  t.join();
  tbk.stopRobot();
  tcsetattr(kfd, TCSANOW, &cooked);

  return(0);
}

void
TBK_Node::keyboardLoop()
{
  char c;
  bool dirty=false;

  int jointid = 1;
  double effort[4] = {0,0,0,0};
  // angular speed change resolution
  double effort_resolution = 1;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Press 1 - 4 to select a joint");
  puts("---------------------------");

  struct pollfd ufd;
  ufd.fd = kfd;
  ufd.events = POLLIN;
  for(;;)
  {
    boost::this_thread::interruption_point();
    
    // get the next event from the keyboard
    int num;
    if((num = poll(&ufd, 1, 250)) < 0)
    {
      perror("poll():");
      return;
    }
    else if(num > 0)
    {
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        return;
      }
    }
    else
      continue;


    switch(c)
    {
      case 1:
      case 2:
      case 3:
      case 4:
        jointid = (int)c;
        break;
      case KEYCODE_MINUS:
        effort[jointid] -= effort_resolution;
        dirty = true;
        break;
      case KEYCODE_EQUAL:
        effort[jointid] += effort_resolution;
        dirty = true;
        break;
      case KEYCODE_SPACE:
        // clear effort
        dirty = true;
        break;
    }
    if (dirty == true)
    {
        // publish the updated velocity
      cmdvel.linear.x = speed;
      cmdvel.angular.z = ang_speed;
      pub_.publish(cmdvel);

                        // reset the angular speed to 0, so that it won't keep turning
                        cmdvel.angular.z = ang_speed = 0;
    }
  }
}

// constantly publish velocity message
void
TBK_Node::pub_cmdvel()
{
        // set the publishing rate to 10Hz
  ros::Rate loop_rate(10);
  // publish velocity messages
  while (ros::ok())
        {
    pub_.publish(cmdvel);
                ros::spinOnce();
                loop_rate.sleep();
        }
}
