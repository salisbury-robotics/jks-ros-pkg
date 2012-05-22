#include <local_planners/convex_constraint_solver.h>
#include <pluginlib/class_list_macros.h>
#include <planning_models/kinematic_model.h>

namespace local_planners{

  double normalizeAngle( double angle )
  {
    if(angle < -M_PI || angle > M_PI)
    {
      return angle - copysign(1.0, angle)*2*M_PI*ceil(fabs(angle)/ (2*M_PI));
    }
    return angle;
  }

  void printCollisionInfo(const planning_scene::PlanningScene& ps, const planning_models::KinematicState& ks )
  {

    collision_detection::CollisionRequest req;
    req.max_contacts = 50;
    req.contacts = true;
    req.distance = false;
    req.verbose = false;
    collision_detection::CollisionResult res;
    ps.checkCollision(req, res, ks);

    ROS_INFO("Contact count: %zd", res.contact_count);
    if(res.collision)
      for( collision_detection::CollisionResult::ContactMap::iterator it = res.contacts.begin(); it != res.contacts.end(); ++it)
      {
        std::string contact1 = it->first.first;
        std::string contact2 = it->first.second;
        std::vector<collision_detection::Contact>& vec = it->second;

        for(size_t contact_index = 0; contact_index < vec.size(); contact_index++)
        {
          Eigen::Vector3d pos =     vec[contact_index].pos;
          Eigen::Vector3d normal =  vec[contact_index].normal;
          double depth = vec[contact_index].depth;
          ROS_INFO("Contact between [%s] and [%s] point: %.2f %.2f %.2f normal: %.2f %.2f %.2f depth: %.3f",
                   contact1.c_str(), contact2.c_str(),
                   pos(0), pos(1), pos(2),
                   normal(0), normal(1), normal(2),
                   depth);
        }

      }
  }

bool ConvexConstraintSolver::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::GetMotionPlan::Request &req,
                   moveit_msgs::GetMotionPlan::Response &res) const
{

  // Need to add in joint limit constraints
  // Get the position constraints from the collision detector


  //  1. Compute unconstrained motion to go from proxy to goal.
  //  2. Get the current contact set by moving the proxy toward the goal by some epsilon and get all collision points.
  //  3. Compute constrained motion (convex solver).
  //  4. Compute collisions along the constrained motion path.
  //  5. Set proxy to stop at the first new contact along path.



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
    for (size_t s = 1 ; s <= 5 ; ++s)
    {
      planning_models::KinematicStatePtr point = planning_models::KinematicStatePtr(new planning_models::KinematicState(start_state));
      for (std::map<std::string, double>::const_iterator it = update.begin() ; it != update.end() ; ++it)
      {
        double sv = start_state.getJointState(it->first)->getVariableValues()[0];
        double gv = goal_state.getJointState(it->first)->getVariableValues()[0];
        std::vector<moveit_msgs::JointLimits> limits = planning_scene->getKinematicModel()->getJointModel(it->first)->getJointLimits();
        double u = 0;
        moveit_msgs::JointLimits &limit = limits[0];
        bool continuous = !limit.has_position_limits;


        //ROS_INFO("Joint %s is continuous: %d, start = %.2f, goal = %.2f ", limit.joint_name.c_str(), continuous, sv, gv);

        gv = normalizeAngle(gv);
        sv = normalizeAngle(sv);
        // Handle wrap around
        double delta = gv - sv;
        if( delta >= M_PI || delta <= -M_PI )
          u = sv  - s * delta/(double)steps;
        else
          u = sv  + s * delta/(double)steps;

        //ROS_INFO("Now start = %.2f, goal = %.2f, update = %.2f ", sv, gv, u);

        //planning_models::KinematicModel::JointModel jm = planning_scene->getKinematicModel()->getJointModel(it->first)->getType();
        point->getJointState(it->first)->setVariableValues(&u);
      }
      if (planning_scene->isStateValid(*point))
        path.push_back(point);
      else
      {
        printCollisionInfo(*planning_scene, *point);
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

PLUGINLIB_DECLARE_CLASS(local_planners, ConvexConstraintSolver,
                        local_planners::ConvexConstraintSolver,
                        planning_interface::Planner);

