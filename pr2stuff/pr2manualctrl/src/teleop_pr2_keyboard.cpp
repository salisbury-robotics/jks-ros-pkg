/* License Info{{{
 * teleop_pr2_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Kevin Watts }}}

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm_msgs/MoveArmAction.h>
#include <move_arm_msgs/utils.h>

enum motionInfo{ R_ARM, L_ARM, XDIR, YDIR, ZDIR };

// Keycodes {{{ 
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_8 0x38
#define KEYCODE_9 0x39
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_Z 0x7a
#define KEYCODE_X 0x78
#define KEYCODE_C 0x63
#define KEYCODE_V 0x76
#define KEYCODE_B 0x62
#define KEYCODE_N 0x6e
#define KEYCODE_M 0x6d
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_L 0x6c
#define KEYCODE_U 0x75
#define KEYCODE_I 0x69
#define KEYCODE_O 0x6f
#define KEYCODE_F 0x66
#define KEYCODE_R 0x72

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45
// }}}

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> ArmClient;

class TeleopPR2Keyboard
{
  private:
    double walk_vel, run_vel, yaw_rate, yaw_rate_run;
    geometry_msgs::Twist cmd;
    GripperClient* gripper_client_l;
    GripperClient* gripper_client_r;
    ArmClient* arm_client_l;
    ArmClient* arm_client_r;

    ros::NodeHandle n_;
    ros::Publisher vel_pub_;
    ros::NodeHandle n_private;

    double r_cur_x;
    double r_cur_y;
    double r_cur_z;
    double l_cur_x;
    double l_cur_y;
    double l_cur_z;
  public:
    void init()  { 
      //init for base movement
      cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
      vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      ros::NodeHandle n_priv("~");
      n_private = n_priv;
      n_private.param("walk_vel", walk_vel, 0.5);
      n_private.param("run_vel", run_vel, 1.0);
      n_private.param("yaw_rate", yaw_rate, 1.0);
      n_private.param("yaw_run_rate", yaw_rate_run, 1.5);

      //init for grippers
      gripper_client_r = new GripperClient("r_gripper_controller/gripper_action", true);
      gripper_client_l = new GripperClient("l_gripper_controller/gripper_action", true);

      //init for arm movement
      arm_client_r = new ArmClient("move_right_arm", true);
      arm_client_r->waitForServer();
      arm_client_l = new ArmClient("move_left_arm", true);
      arm_client_l->waitForServer();
      //ROS_INFO("Connected to server");
      moveArm(R_ARM, XDIR, 0, true);
      moveArm(L_ARM, XDIR, 0.1, true);
      //moveArm(R_ARM, XDIR, 0, true);
    }

    ~TeleopPR2Keyboard()   { }

    void moveArm(motionInfo arm, motionInfo dir, double distance, bool moveToStart = false) { // {{{
      move_arm_msgs::MoveArmGoal goalA;
      ArmClient * arm_client;
      if(arm == R_ARM) {
        goalA.motion_plan_request.group_name = "right_arm";
        arm_client = arm_client_r;
      } else {
        goalA.motion_plan_request.group_name = "left_arm";
        arm_client = arm_client_l;
      }
      goalA.motion_plan_request.num_planning_attempts = 1;
      goalA.motion_plan_request.planner_id = std::string("");
      goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
      goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

      motion_planning_msgs::SimplePoseConstraint desired_pose;
      desired_pose.header.frame_id = "torso_lift_link";
      if(arm == R_ARM) {
        desired_pose.link_name = "r_wrist_roll_link";
        if(moveToStart) {
          desired_pose.pose.position.x = r_cur_x = 0.75;
          desired_pose.pose.position.y = r_cur_y = -0.188;
          desired_pose.pose.position.z = r_cur_z = 0;
        } else {
          //r_cur_x += distance;
          //desired_pose.pose.position.x = r_cur_x;
          //desired_pose.pose.position.y = r_cur_y;
          //d/esired_pose.pose.position.z = r_cur_z;
          if(dir == XDIR) desired_pose.pose.position.x = r_cur_x = r_cur_x + distance;
          else desired_pose.pose.position.x = r_cur_x;
          if(dir == YDIR) desired_pose.pose.position.y = r_cur_y = r_cur_y + distance;
          else desired_pose.pose.position.y = r_cur_y;
          if(dir == ZDIR) desired_pose.pose.position.z = r_cur_z = r_cur_z + distance;
          else desired_pose.pose.position.z = r_cur_z;
        }
      } else {
        desired_pose.link_name = "l_wrist_roll_link";
        if(moveToStart) {
          desired_pose.pose.position.x = l_cur_x = 0.75;
          desired_pose.pose.position.y = l_cur_y = 0.188;
          desired_pose.pose.position.z = l_cur_z = 0;
        } else {
          if(dir == XDIR) desired_pose.pose.position.x = l_cur_x = l_cur_x + distance;
          else desired_pose.pose.position.x = l_cur_x;
          if(dir == YDIR) desired_pose.pose.position.y = l_cur_y = l_cur_y + distance;
          else desired_pose.pose.position.y = l_cur_y;
          if(dir == ZDIR) desired_pose.pose.position.z = l_cur_z = l_cur_z + distance;
          else desired_pose.pose.position.z = l_cur_z;
        }
      }
     
      desired_pose.pose.orientation.x = 0.0;
      desired_pose.pose.orientation.y = 0.0;
      desired_pose.pose.orientation.z = 0.0;
      desired_pose.pose.orientation.w = 1.0;

      desired_pose.absolute_position_tolerance.x = 0.02;
      desired_pose.absolute_position_tolerance.y = 0.02;
      desired_pose.absolute_position_tolerance.z = 0.02;

      desired_pose.absolute_roll_tolerance = 0.04;
      desired_pose.absolute_pitch_tolerance = 0.04;
      desired_pose.absolute_yaw_tolerance = 0.04;

      move_arm_msgs::addGoalConstraintToMoveArmGoal(desired_pose, goalA);

      if(n_private.ok()) {
        bool finished_within_time = false;
        arm_client->sendGoal(goalA);
        finished_within_time = arm_client->waitForResult(ros::Duration(200.0));
        if(!finished_within_time) {
          arm_client->cancelGoal();
          ROS_INFO("Timed out achieving goalA");
        } else {
          actionlib::SimpleClientGoalState state = arm_client->getState();
          if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Action finished: %s", state.toString().c_str());
          } else {
            ROS_INFO("Action failed: %s", state.toString().c_str());
          }
        }
      }           
    } // }}}
        
    void keyboardLoop();
    
    void openGripper(int hand){ //{{{
      pr2_controllers_msgs::Pr2GripperCommandGoal open;
      open.command.position = 0.08;
      open.command.max_effort = -1.0;  // Do not limit effort (negative)

      if(hand == 0) {
        gripper_client_r->sendGoal(open);
        gripper_client_r->waitForResult();
      } else {
        gripper_client_l->sendGoal(open);
        gripper_client_l->waitForResult();
      } 
    } // }}}

    void closeGripper(int hand){ // {{{
      pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
      squeeze.command.position = 0.0;
      squeeze.command.max_effort = 50.0;  // Close gently

      if(hand == 0) {
        gripper_client_r->sendGoal(squeeze);
        gripper_client_r->waitForResult();
      } else {
        gripper_client_l->sendGoal(squeeze);
        gripper_client_l->waitForResult();
      }
    }
    // }}}
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig) { // {{{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
} // }}}

int main(int argc, char** argv) { // {{{
  ros::init(argc, argv, "pr2manualctrl");

  TeleopPR2Keyboard tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
} //}}}

void TeleopPR2Keyboard::keyboardLoop() { // {{{
  char c;
  bool dirty=false;
  bool dirty2=false;
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WASD' to translate");
  puts("Use 'QE' to yaw");
  puts("Press 'Shift' to run");


  for(;;)
  {
    dirty=false;
    dirty2=false;
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

    switch(c)
    {
      // Walking
      case KEYCODE_8:
        cmd.linear.x = walk_vel;
        dirty = true;
        break;
      case KEYCODE_5:
        cmd.linear.x = - walk_vel;
        dirty = true;
        break;
      case KEYCODE_4:
        cmd.linear.y = walk_vel;
        dirty = true;
        break;
      case KEYCODE_6:
        cmd.linear.y = - walk_vel;
        dirty = true;
        break;
      case KEYCODE_7:
        cmd.angular.z = yaw_rate;
        dirty = true;
        break;
      case KEYCODE_9:
        cmd.angular.z = - yaw_rate;
        dirty = true;
        break;

        //Grippers
      case KEYCODE_V:
        openGripper(1);
        break;
      case KEYCODE_N:
        openGripper(0);
        break;
      case KEYCODE_C:
        closeGripper(1);
        break;
      case KEYCODE_M:
        closeGripper(0);
        break;

        //arms
      case KEYCODE_W:
        //l_arm.pose.position.x = l_arm.pose.position.x+0.1;
        moveArm(L_ARM, XDIR, 0.1);
        dirty2 = true;
        break;
      case KEYCODE_S:
        //l_arm.pose.position.x = l_arm.pose.position.x-0.1;
        moveArm(L_ARM, XDIR, -0.1);
        dirty2 = true;
        break;
      case KEYCODE_A:
        //l_arm.pose.position.y = l_arm.pose.position.y+0.1;
        moveArm(L_ARM, YDIR, 0.1);
        dirty2 = true;
        break;
      case KEYCODE_D:
        //l_arm.pose.position.y = l_arm.pose.position.y-0.1;
        moveArm(L_ARM, YDIR, -0.1);
        dirty2 = true;
        break;
      case KEYCODE_Q:
        //l_arm.pose.position.z = l_arm.pose.position.z+0.1;
        moveArm(L_ARM, ZDIR, 0.1);
        dirty2 = true;
        break;
      case KEYCODE_E:
        moveArm(L_ARM, ZDIR, -0.1);
        //l_arm.pose.position.z = l_arm.pose.position.z-0.1;
        dirty2 = true;
        break;


      case KEYCODE_I:
        //r_arm.pose.position.x = r_arm.pose.position.x+0.1;
        moveArm(R_ARM, XDIR, 0.1);
        dirty2 = true;
        break;
      case KEYCODE_K:
        //r_arm.pose.position.x = r_arm.pose.position.x-0.1;
        moveArm(R_ARM, XDIR, -0.1);
        dirty2 = true;
        break;
      case KEYCODE_J:
        //r_arm.pose.position.y = r_arm.pose.position.y+0.1;
        moveArm(R_ARM, YDIR, 0.1);
        dirty2 = true;
        break;
      case KEYCODE_L:
        //r_arm.pose.position.y = r_arm.pose.position.y-0.1;
        moveArm(R_ARM, YDIR, -0.1);
        dirty2 = true;
        break;
      case KEYCODE_U:
        //r_arm.pose.position.z = r_arm.pose.position.z+0.1;
        moveArm(R_ARM, ZDIR, 0.1);
        dirty2 = true;
        break;
      case KEYCODE_O:
        //r_arm.pose.position.z = r_arm.pose.position.z-0.1;
        moveArm(R_ARM, ZDIR, -0.1);
        dirty2 = true;
        break;

        // Running 
      case KEYCODE_W_CAP:
        cmd.linear.x = run_vel;
        dirty = true;
        break;
      case KEYCODE_S_CAP:
        cmd.linear.x = - run_vel;
        dirty = true;
        break;
      case KEYCODE_A_CAP:
        cmd.linear.y = run_vel;
        dirty = true;
        break;
      case KEYCODE_D_CAP:
        cmd.linear.y = - run_vel;
        dirty = true;
        break;
      case KEYCODE_Q_CAP:
        cmd.angular.z = yaw_rate_run;
        dirty = true;
        break;
      case KEYCODE_E_CAP:
        cmd.angular.z = - yaw_rate_run;
        dirty = true;
        break;
    }


    if (dirty == true) {
      vel_pub_.publish(cmd);
    }
    if(dirty2 == true) {
      //pose_pub_l.publish(l_arm);
      //pose_pub_r.publish(r_arm); 
    }

  }
} // }}}
