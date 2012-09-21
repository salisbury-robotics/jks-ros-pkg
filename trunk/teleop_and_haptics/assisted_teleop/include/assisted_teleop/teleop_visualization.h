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

// Author: Adam Leeper

#ifndef _TELEOP_VISUALIZATION_H_
#define _TELEOP_VISUALIZATION_H_

#include <local_planners/linear_joint_stepper.h>

#include <moveit_visualization_ros/planning_visualization.h>

#include <moveit_visualization_ros/kinematics_start_goal_visualization.h>
#include <moveit_visualization_ros/joint_trajectory_visualization.h>
#include <moveit_visualization_ros/collision_visualization.h>

#include <planning_models_loader/kinematic_model_loader.h>
#include <planning_pipeline/planning_pipeline.h>

#include <ros/ros.h>
#include <boost/function.hpp>

namespace moveit_visualization_ros
{
  typedef boost::function<void(void)> TrajectoryExecutionFunction;

class TeleopVisualization : public PlanningVisualization
{
  
public:

  TeleopVisualization(const planning_scene::PlanningSceneConstPtr& planning_scene,
                      const boost::shared_ptr<planning_pipeline::PlanningPipeline>& move_group_pipeline,
                      boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& interactive_marker_server,
                      boost::shared_ptr<planning_models_loader::KinematicModelLoader>& kinematic_model_loader,
                      ros::Publisher& marker_publisher);
  

  void selectGroup(const std::string& group);

  void setTrajectoryExecutionFunction(TrajectoryExecutionFunction function)
  {
    trajectory_execution_fn_ = function;
  }

 bool getProxyState(planning_models::KinematicState &kin_state);

// virtual bool getLastJointState(std::string& group_name,
//                        trajectory_msgs::JointTrajectory& traj) const
// {
//   if(!last_trajectory_ok_) return false;
//   group_name = last_group_name_;
//   traj = last_trajectory_;
//   return true;
// }


protected:

  //! This is the top-level planning call
  virtual void generateTeleopPlan(const std::string& name);

  //! This calls planners that expect *pose* constraints.
  virtual void createTeleopStep(const std::string& name);

  //! Just puts the IK state into a trajectory point.
  virtual void createIKStep(const std::string& name);

  //! Call "regular" planners.
  //virtual void generatePlan(const std::string& name, bool play=true);
  virtual bool generatePlanForScene(const planning_scene::PlanningSceneConstPtr& scene,
                           const std::string& arm_name,
                           const planning_models::KinematicState* start_state,
                           const planning_models::KinematicState* goal_state,
                           trajectory_msgs::JointTrajectory& traj,
                           moveit_msgs::MoveItErrorCodes& error_code) const;


  /** @brief Callback for commanding robot to move. */
  void teleopTimerCallback();

  //ompl_interface_ros::OMPLInterfaceROS ompl_interface_;
  //local_planners::LinearJointStepper my_planner_;

  boost::shared_ptr<moveit_visualization_ros::CollisionVisualization> collision_visualization_;

  /** Used to store the last valid proxy state */
  moveit_msgs::RobotTrajectory last_robot_trajectory_;

  /** Timer for teleop callback */
  ros::Timer teleop_timer_;

  /** Timer period for teleop callback */
  double teleop_period_;

  /** Parameter for trying out different planner types... */
  std::string planner_type_;

  /** Parameter for bypassing trajectory execution... */
  bool execute_trajectory_;

  // TODO do I still need this?
  TrajectoryExecutionFunction trajectory_execution_fn_;
};

}

#endif
