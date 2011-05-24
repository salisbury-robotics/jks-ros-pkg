#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <pthread.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_L 0x6c
#define KEYCODE_M 0x6d
#define KEYCODE_SPACE 0x20

double max_speed = 0.500; // maximum speed, m/second

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

        
  double speed = 0;
  double ang_speed = 0;
  // angular speed change resolution
  double ang_resolution = 20.0*M_PI/180.0;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("1. I,M: increase / decrease forward velocity");
  puts("2. J,L: bump the robot's angle to the left / right");
  puts("3. K, space: stop the robot");
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

                // change the velocity according to the message
    switch(c)
    {
      case KEYCODE_I:
        // the speed cannot exceed max_speed
        if (speed < max_speed) speed += 0.05;
        dirty = true;
        break;
      case KEYCODE_M:
        speed -= 0.05;
        dirty = true;
        break;
      case KEYCODE_J:
        ang_speed += ang_resolution;
        dirty = true;
        break;
      case KEYCODE_L:
        ang_speed -= ang_resolution;
        dirty = true;
        break;
      case KEYCODE_K:
        speed = 0;
        ang_speed = 0;
        dirty = true;
        break;
      case KEYCODE_SPACE:
        speed = 0;
        ang_speed = 0;
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
