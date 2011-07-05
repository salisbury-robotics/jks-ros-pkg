/** Calibration for gravity compensation.
  *
**/
#include "ros/ros.h"
#include "cc2.h"
#include <bosch_arm_control/ArmState.h>
#include <boost/lexical_cast.hpp>
#include <bosch_arm_control/PointCmd.h>

using namespace std;
ros::Publisher joint_cmd_pub;
bosch_arm_control::PointCmd cmd;
bosch_arm_control::ArmState js;

double rate=500;
ros::Rate* loop_rate;

void jointStateCallBack(const bosch_arm_control::ArmState& msg)
{
  js=msg;
  cout<<"receive joint states\n";
}
//TODO: look at pr2's trajectory generation and the trajectory controller.
//The trajectory generation can be quite general, given the coordinate, speed
//and acceleration of the start and end point, output a smooth trajectory
//connecting two points.
//the trajectory controller should take some sparse way points or motion constraints,
//and generate the best trajectory based on the dynamic property of the arm
//However, no matter how smooth it is, way points are still artificial.
//is there a controll mechanism that do not need any way points but
//uses the constraints directly?
void move_joints_relative(const double * rel,double t, double *torque)
{
  //set the first command as the current positon.
  ros::spinOnce();
  double init_cmd_pos[4];
  for (int i=0;i<4;i++)
  {
    init_cmd_pos[i]=js.position[i];
    cmd.coordinates[i]=js.position[i];
    cmd.velocity[i]=0;
    cmd.acceleration[i]=0;
  }
  //for the connection set up delay.
  for (int i=0;i<3;i++)
  {
    cmd.header.stamp=ros::Time::now();
    joint_cmd_pub.publish(cmd);
    loop_rate->sleep();
  }

  //a simple trajectory:accelerate, linear, decelerate, still
  int t_end=int(t*rate);
  int t_dec=t_end-5;
  int t_acc=t_dec/3;
  int t_lin=t_dec-t_acc;
  double acc[4];
  for (int i=0;i<4;i++)
  {
    acc[i]=rel[i]/(2*t_acc*t_acc);
  }

  for (int t=1;t<=t_end;t++)
  {
    if (!ros::ok())
    {
      cout<<"calibration interrupted."<<endl;
      break;
    }
    for (int i=0;i<4;i++)
    {
      if (t<=t_acc)
      {
        cmd.velocity[i]+=acc[i];
        cmd.coordinates[i]+=cmd.velocity[i];
        cmd.acceleration[i]=acc[i];
      }
      else if (t<=t_lin)
      {
        cmd.velocity[i]=t_acc*acc[i];
        cmd.coordinates[i]+=cmd.velocity[i];
        cmd.acceleration[i]=0;
      }
      else if (t<=t_end)
      {
        cmd.velocity[i]-=acc[i];
        cmd.coordinates[i]+=cmd.velocity[i];
        cmd.acceleration[i]=-acc[i];
      }
      else
      {
        cmd.coordinates[i]=init_cmd_pos[i]+rel[i];
        cmd.velocity[i]=0;
        cmd.acceleration[i]=0;
      }

    }
    cmd.header.stamp=ros::Time::now();
    joint_cmd_pub.publish(cmd);
    loop_rate->sleep();
  }
  cout<<js.header.stamp.sec<<','<<js.header.stamp.nsec<<endl;
  ros::spinOnce();
  cout<<js.header.stamp.sec<<','<<js.header.stamp.nsec<<endl;
  sleep(2);
  ros::spinOnce();
  //read the torques
  for (int i=0;i<4;i++)
  {
    torque[i]=js.effort[i];
    cout<<torque[i]<<',';
  }
  cout<<js.header.stamp.sec<<','<<js.header.stamp.nsec<<endl;
}

void joint_linear_motion(double *v, double t_acc)
{
  ros::spinOnce();
  double init_cmd_pos[4];
  for (int i=0;i<4;i++)
  {
    init_cmd_pos[i]=js.position[i];
    cmd.coordinates[i]=js.position[i];
    cmd.velocity[i]=0;
    cmd.acceleration[i]=0;
  }
  //for the connection set up delay.
  for (int i=0;i<3;i++)
  {
    cmd.header.stamp=ros::Time::now();
    joint_cmd_pub.publish(cmd);
    loop_rate->sleep();
  }
  double vstep[4];
  int n_acc=(int)(t_acc*rate);
  for(int i=0;i<4;i++)
    vstep[i]=(v[i]/rate)/n_acc;
  int n=0;
  while(ros::ok())
  {
    n++;
    for(int i=0;i<4;i++)
    {
      if(n<=n_acc)
      {
        cmd.acceleration[i]=vstep[i];
        cmd.velocity[i]+=cmd.acceleration[i];
        cmd.coordinates[i]+=cmd.velocity[i];
      }
      else
      {
        cmd.acceleration[i]=0;
        //cmd.velocity[i]+=cmd.acceleration[i];
        cmd.coordinates[i]+=cmd.velocity[i];
      }
    }
    cmd.header.stamp=ros::Time::now();
    joint_cmd_pub.publish(cmd);
    loop_rate->sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration");
  ros::NodeHandle n;
  joint_cmd_pub =  n.advertise<bosch_arm_control::PointCmd> ("/joint_point_cmd",100);
  ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 2, jointStateCallBack);
  loop_rate=new ros::Rate(rate);
  cmd.coordinates.resize(4);
  cmd.velocity.resize(4);
  cmd.acceleration.resize(4);
  cmd.header.frame_id="joint_space";
  cmd.header.stamp=ros::Time::now();
  //get current joint angles
  while (js.position.size() ==0)
    ros::spinOnce();

  //performe measurements for joint 4.
  //relative motion of pi/12

  double rel[]={0,0,0,constants::pi};
  double torque[4];
  //move_joints_relative(rel,60, torque);
  double v[]={0,0,0,constants::pi/180};
  joint_linear_motion(v,0.2);


  return 0;
}
