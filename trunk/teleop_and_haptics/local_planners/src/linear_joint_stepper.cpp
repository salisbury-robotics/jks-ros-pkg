#include <local_planners/util.h>
#include <local_planners/linear_joint_stepper.h>
#include <pluginlib/class_list_macros.h>
#include <planning_models/kinematic_model.h>
#include <planning_interface/planning_interface.h>


namespace local_planners{

bool LinearJointStepper::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::GetMotionPlan::Request &req,
                   moveit_msgs::GetMotionPlan::Response &res) const
{
  ROS_WARN("Solving using LinearJointStepper!");

  planning_models::KinematicState start_state = planning_scene->getCurrentState();
  planning_models::robotStateToKinematicState(*(planning_scene->getTransforms()), req.motion_plan_request.start_state, start_state);

  planning_models::KinematicState goal_state = start_state;
  const moveit_msgs::Constraints &c = req.motion_plan_request.goal_constraints[0];
  std::map<std::string, double> update;
  for (std::size_t i = 0 ; i < c.joint_constraints.size() ; ++i)
  {
    update[c.joint_constraints[i].joint_name] = c.joint_constraints[i].position;
  }
  goal_state.setStateValues(update);
  unsigned int steps = 1  + (start_state.distance(goal_state) / 0.03);

  std::vector<planning_models::KinematicStatePtr> path;
  if(planning_scene->isStateValid(start_state))
    path.push_back(planning_models::KinematicStatePtr(new planning_models::KinematicState(start_state)));
  else
  {
    ROS_ERROR("Can't plan, start state is not valid.");

    printCollisionInfo(*planning_scene, start_state);

    return false;
  }


  if (steps < 3)
  {
    //planning_models::KinematicStatePtr point = planning_models::KinematicStatePtr(new planning_models::KinematicState(goal_state));

    if (planning_scene->isStateValid(goal_state))
      path.push_back(planning_models::KinematicStatePtr(new planning_models::KinematicState(goal_state)));
  }
  else
  {
    for (size_t s = 1 ; s <= steps ; ++s)
    {
      planning_models::KinematicStatePtr point = planning_models::KinematicStatePtr(new planning_models::KinematicState(start_state));
      createKinematicStatePoint(planning_scene, start_state, goal_state, update, s, steps, point);
      if (planning_scene->isStateValid(*point))
        path.push_back(point);
      else
      {
        //printCollisionInfo(*planning_scene, *point);
        break;
      }
    }
  }

  // convert path to result trajectory
  moveit_msgs::RobotTrajectory rt;
  trajectory_msgs::JointTrajectory traj;

  // Just take the last point
  //for( size_t path_index = path.size()-1; path_index < path.size(); path_index++)
  {

    sensor_msgs::JointState js;
    planning_models::kinematicStateToJointState(*(path.back()), js);
    trajectory_msgs::JointTrajectoryPoint pt;

    const planning_models::KinematicModel::JointModelGroup *jmg = planning_scene->getKinematicModel()->getJointModelGroup(req.motion_plan_request.group_name);

    // getJointNames
    for(size_t i = 0 ; i < js.name.size(); i++)
    {
      std::string name = js.name[i];
      if( !jmg->hasJointModel(name) ) continue;

      //if( path_index == 0)
        traj.joint_names.push_back(js.name[i]);
      pt.positions.push_back(js.position[i]);
      if(js.velocity.size()) pt.velocities.push_back(js.velocity[i]);
    }
    pt.time_from_start = ros::Duration(0.1);
    traj.points.push_back(pt);
  }
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = "odom_combined";

  res.trajectory.joint_trajectory = traj;

  return true;
}

} // namespace local_planners

PLUGINLIB_DECLARE_CLASS(local_planners, LinearJointStepper,
                        local_planners::LinearJointStepper,
                        planning_interface::Planner)

