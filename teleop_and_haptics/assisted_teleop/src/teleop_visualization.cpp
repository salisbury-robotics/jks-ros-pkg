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
  pnh_.param("constraint_aware", constraint_aware_, false);

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

//  moveit_msgs::RobotState robot_state;

//  last_trajectory_;
//  planning_models::jointStateToKinematicState();
//  //planning_models::robotTrajectoryPointToRobotState(last_robot_trajectory_, last_robot_trajectory_.joint_trajectory.points.size()-1, robot_state);
//  planning_models::robotStateToKinematicState(robot_state, kin_state);

  ROS_INFO("Returning with Proxy state!");
  return true;
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
  ros::Time start_time = ros::Time::now();
  ROS_INFO_STREAM("teleopTimerCallback: v v v v v v v v v v v v v v v v v v v v v v v");
  //ROS_INFO_STREAM("Teleop timer update! Moving group...");

  if(current_group_.empty()) {
    ROS_WARN("teleopTimerCallback: No group is active, returning...");
    return;
  }
  if(group_visualization_map_.find(current_group_) == group_visualization_map_.end()) {
    ROS_WARN_STREAM("teleopTimerCallback: No group name " << current_group_);
    return;
  }

  KinematicsStartGoalVisualization* ksgv = group_visualization_map_[current_group_].get();

  if(constraint_aware_)
  {
    generatePlan(current_group_, false);

    // TODO need to figure out the right way to maintain the proxy...
    // because this is NOT hat we want
    //ksgv->setStartState(planning_scene_->getCurrentState());

    // Set start state to the last computed proxy state:
    planning_models::KinematicState proxy(ksgv->getStartState());
    if( getProxyState(proxy) )
      ksgv->setStartState(proxy);

    bool show_collisions = false;
    if(show_collisions) // it's a bummer to have to recompute this result
    {
      collision_detection::CollisionRequest req;
      req.max_contacts = 50;
      req.contacts = true;
      req.distance = false;
      req.verbose = false;
      collision_detection::CollisionResult res;
      planning_scene_->checkCollision(req, res, ksgv->getGoalState());
      collision_visualization_->drawCollisions(res, planning_scene_->getPlanningFrame());
    }
  }
  else
  {
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
    pt.time_from_start = ros::Duration(teleop_period_);
    traj.points.push_back(pt);
    traj.header.stamp = ros::Time::now();
    traj.header.frame_id = "odom_combined";

    last_trajectory_ = traj;
    last_group_name_ = current_group_;
    last_trajectory_ok_ = true;
  }


  // Send command for next posture
  ROS_INFO("teleopTimerCallback: Calling trajectory execution function!");
  trajectory_execution_fn_();

  ros::Duration elapsed = ros::Time::now() - start_time;
  float lambda = 0.1;
  average_duration = ros::Duration(lambda*elapsed.toSec() + (1-lambda)*average_duration.toSec());
  ROS_INFO("teleopTimerCallback: Teleop update took %.1f ms, filtered average is %.1f ms!", elapsed.toSec()*1000, average_duration.toSec()*1000);
  ROS_INFO_STREAM("teleopTimerCallback: ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^");
}


} // namespace moveit_visualization_ros
