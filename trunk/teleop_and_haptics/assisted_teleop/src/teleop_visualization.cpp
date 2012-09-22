/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

// Author: Adapted from planning_visualization.cpp by Adam Leeper

#include <moveit_visualization_ros/interactive_marker_helper_functions.h>
#include <assisted_teleop/teleop_visualization.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <kinematic_constraints/utils.h>
#include <planning_models/conversions.h>

namespace moveit_visualization_ros {

TeleopVisualization::TeleopVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                         const boost::shared_ptr<planning_pipeline::PlanningPipeline>& move_group_pipeline,
                                         boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                                         boost::shared_ptr<planning_models_loader::KinematicModelLoader>& kinematic_model_loader,
                                         ros::Publisher& marker_publisher)
  : PlanningVisualization(planning_scene, move_group_pipeline, interactive_marker_server, kinematic_model_loader, marker_publisher)
{
  collision_visualization_.reset(new CollisionVisualization(marker_publisher));

  pnh_.param("teleop_period", teleop_period_, 0.0333);
  teleop_timer_ =  nh_.createTimer(ros::Duration(teleop_period_), boost::bind( &TeleopVisualization::teleopTimerCallback, this ) );
  //pnh_.param("constraint_aware", constraint_aware_, false);
  pnh_.param("planner_type", planner_type_, std::string("IK"));
  pnh_.param("execute_trajectory", execute_trajectory_, true);
}

void TeleopVisualization::selectGroup(const std::string& group) {
  if(current_group_ == group) return;
  if(group_visualization_map_.find(group) == group_visualization_map_.end()) {
    ROS_WARN_STREAM("No group name " << group);
  }
  if(!current_group_.empty()) {
    group_visualization_map_[current_group_]->hideAllMarkers();
  }
  current_group_ = group;
  group_visualization_map_[current_group_]->showAllMarkers();

  KinematicsStartGoalVisualization* ksgv = group_visualization_map_[current_group_].get();
  ksgv->setGoalState(planning_scene_->getCurrentState());
  ksgv->setStartState(planning_scene_->getCurrentState());
}

bool TeleopVisualization::getProxyState(planning_models::KinematicState &kin_state)
{

  if(!last_trajectory_ok_){
    ROS_WARN("Last trajectory was invalid, can't get proxy state.");
    return false;
  }

  trajectory_msgs::JointTrajectory traj;
  getLastTrajectory(current_group_, traj);
  if(traj.points.size() == 0)
  {
    ROS_ERROR("For some reason there are no points in a supposedly valid trajectory.");
    return false;
  }

  std::map<std::string, double> update;
  for (std::size_t i = 0 ; i < traj.joint_names.size() ; ++i)
  {
    update[traj.joint_names[i]] = traj.points[0].positions[i];
  }
  kin_state.setStateValues(update);

  //ROS_INFO("Returning with Proxy state!");
  return true;
}

void TeleopVisualization::generateTeleopPlan(const std::string& name) {

  KinematicsStartGoalVisualization* ksgv = group_visualization_map_[current_group_].get();

  ksgv->hideGoalRegularMarkers();

  //ksgv->setStartState(planning_scene_->getCurrentState());

  // We shouldn't need to do this here, but I'm worried that something weird is going on in the background.
//  {
//    planning_models::KinematicState proxy(ksgv->getStartState());
//    if( getProxyState(proxy) )
//      ksgv->setStartState(proxy);
//  }




  if(planner_type_ == "local")  // TODO change how these are selected!
  {
    createTeleopStep(name); // In this mode we use local "gradients" to move the end effector closer to the goal.
  }
  else if( planner_type_ == "global")
  {
    generatePlan(name, false);  // In this mode we use full motion planners to get to the goal.
  }
  else if( planner_type_ == "IK")
  {
    createIKStep(name);  // This is a basic mode that just stores the current IK solution as a trajectory point.
  }
  else
  {
    ROS_ERROR("No planner type specified!");
  }

  // = = = = = Store the last trajectory solution as the new proxy state... = = = = = =

  // This is the WRONG thing to do. The robot can end up in collision, and then you are hosed.
  //ksgv->setStartState(planning_scene_->getCurrentState());

  // Instead, we set the start state to the last computed proxy state. Since, ideally, the
  // last proxy state is "always valid", we won't run into the dreaded "can't plan" problem.
  // (Though, if the planning scene updates such that the proxy is now in collision we are
  // still screwed. Need to figure out the right way to deal with that case...
  {
    planning_models::KinematicState proxy(ksgv->getStartState());
    if( getProxyState(proxy) )
      ksgv->setStartState(proxy);
  }
}


void TeleopVisualization::createIKStep(const std::string& name) {

  KinematicsStartGoalVisualization* ksgv = group_visualization_map_[current_group_].get();

  // Just use the goal state pose which was compute using constrained IK.
  // This should be wrapped in such a way that the user is *not allowed* to
  // go through an invalid pose to a valid pose. While this may be super annoying, it prevents
  // the user from flying through obstacles when IK is suddenly valid on the other side...
  sensor_msgs::JointState js;
  planning_models::kinematicStateToJointState( ksgv->getGoalState(),js);
  trajectory_msgs::JointTrajectoryPoint pt;
  trajectory_msgs::JointTrajectory traj;

  const planning_models::KinematicModel::JointModelGroup *jmg = planning_scene_->getKinematicModel()->getJointModelGroup(current_group_);

  for(size_t i = 0 ; i < js.name.size(); i++)
  {
    if( !jmg->hasJointModel(js.name[i]) ) continue;
    traj.joint_names.push_back(js.name[i]);
    pt.positions.push_back(js.position[i]);
    if(!js.velocity.empty()) pt.velocities.push_back(js.velocity[i]);
  }
  // TODO This is totally a magic number...
  pt.time_from_start = ros::Duration(teleop_period_*1.5);
  traj.points.push_back(pt);
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = "odom_combined";

  last_trajectory_ = traj;
  last_group_name_ = current_group_;
  last_trajectory_ok_ = true;
}


void TeleopVisualization::createTeleopStep(const std::string& name) {
  ROS_INFO_STREAM("Planning for " << name);
  if(group_visualization_map_.find(name) == group_visualization_map_.end()) {
    ROS_INFO_STREAM("No group " << name << " so can't plan");
    return;
  }

  KinematicsStartGoalVisualization* ksgv = group_visualization_map_[name].get();

  const planning_models::KinematicState& start_state = ksgv->getStartState();
  //const planning_models::KinematicState& start_state = planning_scene_->getCurrentState();
  const planning_models::KinematicState& goal_state = ksgv->getGoalState();

  // TODO this function is probably broken; ask Ioan?
  geometry_msgs::PoseStamped goal_pose = ksgv->getGoalInteractiveMarkerPose();

  moveit_msgs::MoveItErrorCodes error_code;
  trajectory_msgs::JointTrajectory traj;

  bool success = false;

  {
    moveit_msgs::GetMotionPlan::Request req;
    moveit_msgs::GetMotionPlan::Response res;

    req.motion_plan_request.group_name = name;
    planning_models::kinematicStateToRobotState(start_state, req.motion_plan_request.start_state);


    geometry_msgs::PoseStamped im_pose = ksgv->getGoalInteractiveMarkerPose();
    tf::Pose p;
    tf::poseMsgToTF(im_pose.pose, p);
    p.setOrigin(p.getOrigin()-0.1*p.getBasis().getColumn(0));
    tf::poseTFToMsg(p, im_pose.pose);
    //im_pose.pose.position

    std::string group_name = name;
    std::string ee_control_frame;
    if(group_name == "right_arm"){ ee_control_frame = "r_wrist_roll_link"; }
    if(group_name == "left_arm") { ee_control_frame = "l_wrist_roll_link"; }

    req.motion_plan_request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(ee_control_frame, im_pose));
    req.motion_plan_request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state.getJointStateGroup(name),
                                                                                                       .001, .001));
    //ROS_INFO_STREAM("Constraints are: \n" << req.motion_plan_request.goal_constraints[0]);

    req.motion_plan_request.num_planning_attempts = 1;
    req.motion_plan_request.allowed_planning_time = ros::Duration(teleop_period_*1.3);

    // Manually define what planner to use - DTC
    //req.motion_plan_request.planner_id = getCurrentPlanner();

    //ROS_INFO_STREAM("USING MENU PLANNER " << getCurrentPlanner());

    if(!move_group_pipeline_->generatePlan(planning_scene_, req, res)) {
      ROS_WARN_STREAM("Response traj " << res.trajectory.joint_trajectory);
      success = false;
    }
    else
    {
      traj = res.trajectory.joint_trajectory;
      success = true;
    }
  }

  if(success)
  {
    last_start_state_ = start_state;
    last_trajectory_ = traj;
    last_group_name_ = name;
    last_trajectory_ok_ = true;
    cycle_ok_ = true;
  } else {
    last_trajectory_ok_ = false;
    ROS_INFO_STREAM("Teleop planning failed");
  }

}

bool TeleopVisualization::generatePlanForScene(const planning_scene::PlanningSceneConstPtr& scene,
                                               const std::string& group_name,
                                               const planning_models::KinematicState* start_state,
                                               const planning_models::KinematicState* goal_state,
                                               trajectory_msgs::JointTrajectory& ret_traj,
                                               moveit_msgs::MoveItErrorCodes& error_code) const
{
  moveit_msgs::GetMotionPlan::Request req;
  moveit_msgs::GetMotionPlan::Response res;

  req.motion_plan_request.group_name = group_name;
  planning_models::kinematicStateToRobotState(*start_state,req.motion_plan_request.start_state);
  req.motion_plan_request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state->getJointStateGroup(group_name),
                                                                                                     .001, .001));

  req.motion_plan_request.num_planning_attempts = 1;
  req.motion_plan_request.allowed_planning_time = ros::Duration(0.45);

  // Manually define what planner to use - DTC
  req.motion_plan_request.planner_id = getCurrentPlanner();

  ROS_INFO_STREAM("USING MENU PLANNER " << getCurrentPlanner());

  if(!move_group_pipeline_->generatePlan(scene, req, res)) {
    ROS_WARN_STREAM("Response traj " << res.trajectory.joint_trajectory);
    return false;
  }
  ret_traj = res.trajectory.joint_trajectory;
  return true;
}


void TeleopVisualization::teleopTimerCallback() {

  static ros::Duration average_duration = ros::Duration(0);
  static ros::Duration average_cycle = ros::Duration(0);
  static ros::Time last_end_time = ros::Time::now();
  ros::Time start_time = ros::Time::now();

  ROS_INFO_STREAM("teleopTimerCallback: v v v v v v v v v v v v v v v v v v v v v v v");

  if(current_group_.empty()) {
    ROS_WARN("teleopTimerCallback: No group is active, returning...");
    return;
  }
  if(group_visualization_map_.find(current_group_) == group_visualization_map_.end()) {
    ROS_WARN_STREAM("teleopTimerCallback: No group name " << current_group_);
    return;
  }

  // Generate a trajectory through various means:
  generateTeleopPlan(current_group_);

  // Send command for next posture
  if(execute_trajectory_)
  {
    //ROS_INFO("teleopTimerCallback: Calling trajectory execution function!");
    trajectory_execution_fn_();
  }

  ros::Time end_time = ros::Time::now();
  ros::Duration elapsed = end_time - start_time;
  ros::Duration last_cycle = end_time - last_end_time;
  last_end_time = end_time;
  float lambda = 0.1;
  average_duration = ros::Duration(lambda*elapsed.toSec() + (1-lambda)*average_duration.toSec());
  average_cycle = ros::Duration(lambda*last_cycle.toSec() + (1-lambda)*average_cycle.toSec());
  ROS_INFO("teleopTimerCallback: ran in %.1f ms, average %.1f ms, cycle average %.1f ms ^ ^ ^ ^ ^ ^ ^",
           elapsed.toSec()*1000,
           average_duration.toSec()*1000,
           average_cycle.toSec()*1000);
  //ROS_INFO_STREAM("teleopTimerCallback: ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^");
}


} // namespace moveit_visualization_ros


// GRAVEYARD code snippets

//bool show_collisions = false;
//if(show_collisions) // it's a bummer to have to recompute this result
//{
//  collision_detection::CollisionRequest req;
//  req.max_contacts = 50;
//  req.contacts = true;
//  req.distance = false;
//  req.verbose = false;
//  collision_detection::CollisionResult res;
//  planning_scene_->checkCollision(req, res, ksgv->getGoalState());
//  collision_visualization_->drawCollisions(res, planning_scene_->getPlanningFrame());
//}
