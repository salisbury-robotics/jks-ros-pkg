/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

// Author: Adapted from planning_visualization.h by Adam Leeper

#ifndef _TELEOP_VISUALIZATION_H_
#define _TELEOP_VISUALIZATION_H_

#include <ros/ros.h>
#include <moveit_visualization_ros/kinematics_start_goal_visualization.h>
#include <moveit_visualization_ros/joint_trajectory_visualization.h>
#include <ompl_interface_ros/ompl_interface_ros.h>
#include <boost/function.hpp>
#include <trajectory_processing/trajectory_smoother.h>
#include <trajectory_processing/trajectory_shortcutter.h>
#include <kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <assisted_teleop/collision_visualization.h>

#include <local_planners/potential_field_solver.h>

namespace moveit_visualization_ros
{
  //typedef boost::function<void(const std::string&, const trajectory_msgs::JointTrajectory&)> TeleopExecutionFunction;
  typedef boost::function<void(void)> TrajectoryExecutionFunction;

class TeleopVisualization
{
  
public:

  TeleopVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                        const std::map<std::string, std::vector<moveit_msgs::JointLimits> >& group_joint_limit_map,
                        boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                        boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader>& kinematics_plugin_loader,
                        ros::Publisher& marker_publisher);
  
  void updatePlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene);

  void resetAllStartStates();
  void resetAllStartAndGoalStates();

  void addMenuEntry(const std::string& name, 
                    const boost::function<void(const std::string&)>& callback);
  
  void selectGroup(const std::string& name);

  void hideAllGroups();

  bool getLastTrajectory(std::string& group_name,
                         trajectory_msgs::JointTrajectory& traj) const
  {
    if(!last_trajectory_ok_) return false;
    group_name = last_group_name_;
    traj = last_trajectory_;
    return true;
  }

  void setAllStartChainModes(bool chain);

  std::string getCurrentGroup() const {
    return current_group_;
  }

  void setGoalState(const std::string& group_name,
                    const planning_models::KinematicState& state);

  void setStartState(const std::string& group_name,
                     const planning_models::KinematicState& state);
  /* ael
  void setTeleopExecutionFunction(const TeleopExecutionFunction function)
  {
    teleop_execution_fn_.reset(function);
  }*/

  void setTrajectoryExecutionFunction(TrajectoryExecutionFunction function)
  {
    trajectory_execution_fn_ = function;
  }

 bool getProxyState(planning_models::KinematicState* kin_state);


protected:

  void generatePlan(const std::string& name, bool play=true);
  bool generatePlanForScene(const planning_scene::PlanningSceneConstPtr& scene,
                            const std::string& arm_name,
                            const planning_models::KinematicState* start_state,
                            const planning_models::KinematicState* goal_state,
                            trajectory_msgs::JointTrajectory& traj,
                            moveit_msgs::RobotTrajectory& robot_traj,
                            moveit_msgs::MoveItErrorCodes& error_code) const;

  void generateRandomStartEnd(const std::string& name);
  void resetStartGoal(const std::string& name);

  /** @brief Callback for commanding robot to move. */
  void teleopTimerCallback();

  planning_scene::PlanningSceneConstPtr planning_scene_;
  ompl_interface_ros::OMPLInterfaceROS ompl_interface_;
  local_planners::LinearJointStepper my_planner_;
  boost::shared_ptr<trajectory_processing::TrajectorySmoother> trajectory_smoother_;
  boost::shared_ptr<trajectory_processing::TrajectoryShortcutter> unnormalize_shortcutter_;
  std::map<std::string, std::vector<moveit_msgs::JointLimits> > group_joint_limit_map_;
  std::string current_group_;
  std::map<std::string, boost::shared_ptr<KinematicsStartGoalVisualization> > group_visualization_map_;
  boost::shared_ptr<JointTrajectoryVisualization> joint_trajectory_visualization_;
  boost::shared_ptr<moveit_visualization_ros::CollisionVisualization> collision_visualization_;
  ros::Publisher display_traj_publisher_;
  
  std::string last_group_name_;
  moveit_msgs::RobotTrajectory last_robot_trajectory_;
  trajectory_msgs::JointTrajectory last_trajectory_;
  bool last_trajectory_ok_;

  /* ael */
  ros::Timer teleop_timer_;
  //boost::shared_ptr<TeleopExecutionFunction> teleop_executution_fn_;


  TrajectoryExecutionFunction trajectory_execution_fn_;
};

}

#endif
