#include <local_planners/convex_constraint_solver.h>
#include <pluginlib/class_list_macros.h>
#include <planning_models/kinematic_model.h>
//#include <planning_models/kinematic_model/joint_model_group-inc.h>
#include <planning_models/kinematic_state.h>
//#include <planning_models/kinematic_state/joint_state_group-inc.h>
#include <local_planners/util.h>

#define HAVE_CVX

namespace local_planners{

  // TODO can we pick the epsilon size more intelligently for each joint?

// This is "global" storage for the last collision state of the joint state group. It is
// a *TERRIBLE HACK* but I don't know how else to re-use old collision info...
collision_detection::CollisionResult last_collision_result;

// --------------------------------------------------------------------------
#ifdef HAVE_CVX
// this implementation uses the problem-specific QP solver generated by CVXGEN

// Load and instantiate global namespaces with relevant data and working space.

extern "C" {
#include "cvxgen/solver.h"
}
Vars vars;
Params params;
Workspace work;
Settings settings;
#endif


bool ConvexConstraintSolver::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::GetMotionPlan::Request &req,
                   moveit_msgs::GetMotionPlan::Response &res) const
{

  // Need to add in joint limit constraints
  // Get the position constraints from the collision detector

  // The raw algorithm laid out in Chan et. al. is as follows:
  //  1. Compute unconstrained motion to go from proxy to goal.
  //  2. Get the current contact set by moving the proxy toward the goal by some epsilon and get all collision points.
  //  3. Compute constrained motion (convex solver).
  //  4. Compute collisions along the constrained motion path.
  //  5. Set proxy to stop at the first new contact along path.


  // In our framework this will look more like this:
  // Outside ths planner:
  //  1. Compute error between cartesian end effector and cartesian goal.
  //  2. Cap pose error (based on some heuristic?) because we are about to make a linear approximation.
  //  3. Use Jinverse to compute the joint deltas for the pose error (or perhaps we should frame this as a velocity problem).
  //  4. Add the joint deltas to the start state to get a goal state.

  // Inside the planner:
  //  1. Get the "current" contact set by moving the proxy toward the goal by some epsilon and get all collision points.
  //  2. Compute constrained motion (convex solver).
  //  3. Subdivide motion subject to some sort of minimum feature size.
  //  4. Move proxy in steps, checking for colliding state along the way. Optionally use interval bisection to refine.

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -/

  // We "getCurrentState" just to populate the structure with auxiliary info, then copy in the transform info from the planning request.
  planning_models::KinematicState start_state = planning_scene->getCurrentState();
  planning_models::robotStateToKinematicState(*(planning_scene->getTransforms()), req.motion_plan_request.start_state, start_state);

  // constrained_goal_state is the optimization output before interval stepping, proxy_state will be used as we step around.
  planning_models::KinematicState proxy_state = start_state;
  planning_models::KinematicState constrained_goal_state = start_state;







  // ======== Extract all contact points and normals from previous collision state, get associated Jacobians ========
    const planning_models::KinematicState::JointStateGroup * jsg = proxy_state.getJointStateGroup(req.motion_plan_request.group_name);
    const planning_models::KinematicModel::JointModelGroup * jmg = planning_scene->getKinematicModel()->getJointModelGroup(req.motion_plan_request.group_name);

    std::vector<Eigen::MatrixXd> contact_jacobians;
    std::vector<Eigen::Vector3d> contact_normals;

    for( collision_detection::CollisionResult::ContactMap::const_iterator it = last_collision_result.contacts.begin(); it != last_collision_result.contacts.end(); ++it)
    {
      std::string contact1 = it->first.first;
      std::string contact2 = it->first.second;
      const std::vector<collision_detection::Contact>& vec = it->second;

      for(size_t contact_index = 0; contact_index < vec.size(); contact_index++)
      {
        Eigen::Vector3d point =     vec[contact_index].pos;
        Eigen::Vector3d normal =  vec[contact_index].normal;
        double depth = vec[contact_index].depth;
        ROS_INFO("Contact between [%s] and [%s] point: %.2f %.2f %.2f normal: %.2f %.2f %.2f depth: %.3f",
                 contact1.c_str(), contact2.c_str(),
                 point(0), point(1), point(2),
                 normal(0), normal(1), normal(2),
                 depth);
        // Contact point needs to be expressed with respect to the link; normals should stay in the common frame
        planning_models::KinematicState::LinkState *link_state = start_state.getLinkState(contact1);
        Eigen::Affine3d link_T_world = link_state->getGlobalCollisionBodyTransform().inverse();
        point = link_T_world*point;

        Eigen::MatrixXd jacobian;
        if(jsg->getJacobian(contact1, point, jacobian))
        {
          contact_jacobians.push_back(jacobian);
          contact_normals.push_back(normal);
        }
      }
    }


// ======== Extract goal "constraints" ========

  const moveit_msgs::Constraints &c = req.motion_plan_request.goal_constraints[0];


  // Position and Orientation
  if(c.position_constraints.size() != 1 || c.orientation_constraints.size() != 1)
  {
    ROS_ERROR("Currently require exactly one position and orientation constraint. Aborting...");
    return false;
  }
  moveit_msgs::PositionConstraint    pc = c.position_constraints[0];
  moveit_msgs::OrientationConstraint oc = c.orientation_constraints[0];
  if(pc.link_name != oc.link_name)
  {
    ROS_ERROR("Currently can't support position and orientation goals that are not for the same link. Aborting...");
    return false;
  }
  if(pc.constraint_region.primitive_poses.size() == 0)
  {
    ROS_ERROR("Need to specify a single primitive_pose for position constraint region. Aborting...");
    return false;
  }
  if(pc.constraint_region.primitive_poses.size() != 1)
  {
    ROS_ERROR("Need exactly one 'pose' for the end-effector goal region. Aborting...");
  }


  std::string planning_frame = planning_scene->getPlanningFrame();
  if(pc.header.frame_id != planning_frame)
    ROS_WARN("The position goal header [%s] and planning_frame [%s] don't match, things are probably all wrong!",
             pc.header.frame_id.c_str(), planning_frame.c_str() );
  if(oc.header.frame_id != planning_frame)
    ROS_WARN("The orientation goal header [%s] and planning_frame [%s] don't match, things are probably all wrong!",
             oc.header.frame_id.c_str(), planning_frame.c_str() );

  Eigen::Vector3d goal_point;
  Eigen::Quaterniond goal_orientation;
  { // scoped so we don't pollute function scope with these message temps
    geometry_msgs::Point msg_goal_point = pc.constraint_region.primitive_poses[0].position;
    geometry_msgs::Quaternion msg_goal_orientation = oc.orientation;
    goal_point = Eigen::Vector3d(msg_goal_point.x, msg_goal_point.y, msg_goal_point.z);
    goal_orientation = Eigen::Quaterniond(msg_goal_orientation.w, msg_goal_orientation.x, msg_goal_orientation.y, msg_goal_orientation.z);
  }

  planning_models::KinematicState::LinkState *link_state = start_state.getLinkState(pc.link_name);
  Eigen::Affine3d planning_T_link = link_state->getGlobalLinkTransform();

  Eigen::Vector3d ee_point_in_ee_frame = Eigen::Vector3d(pc.target_point_offset.x, pc.target_point_offset.y, pc.target_point_offset.z);
  Eigen::Vector3d ee_point_in_planning_frame = planning_T_link*ee_point_in_ee_frame;

  // TODO need to make sure these are expressed in the same frame.
  Eigen::Vector3d x_error = goal_point - ee_point_in_planning_frame;
  //Eigen::Quaterniond


  // Get end-effector Jacobian
  std::string ee_link = "r_wrist_roll_link";
  if(jmg->isChain())
    ee_link = planning_scene->getKinematicModel()->getJointModelGroup(jmg->getAttachedEndEffectorGroupName())->getEndEffectorParentGroup().second;
  else
    ROS_WARN("Uising r_wrist_roll_link as HARD_CODED value.");
  if(ee_link != pc.link_name)
    ROS_WARN("ee_link [%s] and position_goal link [%s] aren't the same, this could be bad!", ee_link.c_str(), pc.link_name.c_str());

  Eigen::MatrixXd ee_jacobian;
  if(!jsg->getJacobian(ee_link , ee_point_in_ee_frame , ee_jacobian))
  {
    ROS_ERROR("Unable to get end-effector Jacobian! Can't plan, exiting...");
    return false;
  }


// ======== Pack into solver data structure, run solver. ========

//  Vars vars;
//  Params params;
//  Workspace work;
//  Settings settings;

  // CVX Settings
  set_defaults();
  setup_indexing();
  settings.verbose = 0;

  // - - - - - - - load all problem instance data - - - - - - - //

  // end-effector Jacobian
  // TODO magic numbers (though I suppose the CVX solver is already hard-coded)
  for(unsigned int index = 0; index < 6*7; index++ )
    params.J_goal[index] = ee_jacobian(index%7, index/7);

  // set up constraints from contact set
  unsigned constraint_count = std::min<size_t>(50, contact_normals.size());
  for(unsigned int constraint = 0; constraint < 50; constraint++)
  {
    if(constraint < constraint_count)
    {
      //printf("has constraint\n");
      for (int j = 0; j < 6*7; j++) {
        params.Jac[constraint][j] = contact_jacobians[constraint](j%7, j/7);
      }
      for (int j = 0; j < 3; j++) {
        params.normal[constraint][j] = contact_normals[constraint][j];
      }
    }
    else{
      //printf("setting to zero\n");
      for (int j = 0; j < 6*7; j++) {
        params.Jac[constraint][j] = 0;
      }
      for (int j = 0; j < 3; j++) {
        params.normal[constraint][j] = 0;
      }
    }
  }





//  // Set mass, inertia, acceleration, and angular acceleration params
//  for(int i = 0; i < 3; i++)
//  {
//    //printf("params index: %02d", i);
//    params.M[i] = m_mass[i];
//    params.I[3*i+i] = m_moment[i];
//    params.a_u[i] = a[i];
//    params.alpha_u[i] = alpha[i];
//  }
//  // set up constraints from contact set
//  for(int i = 0; i < 11; i++)
//  {
//    //printf("constraint index %02d: \n", i);
//    if(i < m_contactNormals.size())
//    {
//      //printf("has constraint\n");
//      vector3d &n = m_contactNormals[i];
//      vector3d rxn = cml::cross(m_contactPoints[i], n);
//      for (int j = 0; j < 3; j++) {
//        params.n[i][j] = n[j];
//        params.rxn[i][j] = rxn[j];
//      }
//    }
//    else{
//      //printf("setting to zero\n");
//      for (int j = 0; j < 3; j++) {
//        params.n[i][j] = 0;
//        params.rxn[i][j] = 0;
//      }
//    }
//  }

  // - - - - - - - Solve our problem at high speed! - - - - - - - //
//  long num_iters = solve();
//  if(work.converged == 1)
//  {
//    for(int i = 0; i < 3; i++)
//    {
//      ac[i] = vars.a[i];
//      alphac[i] = vars.alpha[i];
//    }
//  }
//  else
//  {
//    printf("solving failed to converge in %d iterations.\n", num_iters);
//    ac.zero();
//    alphac.zero();
//  }



// ======== Unpack solver result into constrained goal state. ========
  std::map<std::string, double> goal_update;
  // TODO populate with results...
  constrained_goal_state.setStateValues(goal_update);


// ======== Step toward constrained goal, checking for new collisions along the way ========
  double proxy_goal_tolerance = 0.1;
  double interpolation_progress = 0.0;
  double interpolation_step = 0.1;
  while(interpolation_progress <= 1.0)
  {
    interpolation_progress += interpolation_step;
//    if(proxy_state.distance(constrained_goal_state) < proxy_goal_tolerance)
//    {
//      ROS_INFO("Reached constrained goal state without any collisions!");
//      last_collision_result = collision_detection::CollisionResult();
//      break;
//    }

//    planning_models::KinematicStatePtr point = planning_models::KinematicStatePtr(new planning_models::KinematicState(start_state));
//    // TODO 0.001 is a magic number...
//    createKinematicStatePoint(planning_scene, proxy_state, constrained_goal_state, goal_update, 0.001, point);

    // TODO this interpolation scheme might not allow the arm to slide along contacts very well...
    planning_models::KinematicStatePtr point;
    start_state.interpolate(constrained_goal_state, interpolation_progress, *point);

    // get contact set
    collision_detection::CollisionRequest collision_request;
    // TODO magic number
    collision_request.max_contacts = 50;
    collision_request.contacts = true;
    collision_request.distance = false;
    collision_request.verbose = false;
    collision_detection::CollisionResult collision_result;
    planning_scene->checkCollision(collision_request, collision_result, *point);
    if(collision_result.collision)
    {
      last_collision_result = collision_result;
      break;
    }
    else
    {
      proxy_state = *point;
    }
  }

// ======== Convert proxy state to a "trajectory" ========
  moveit_msgs::RobotTrajectory rt;
  trajectory_msgs::JointTrajectory traj;
  trajectory_msgs::JointTrajectoryPoint pt;
  sensor_msgs::JointState js;

  planning_models::kinematicStateToJointState(proxy_state, js);
  //const planning_models::KinematicModel::JointModelGroup *jmg = planning_scene->getKinematicModel()->getJointModelGroup(req.motion_plan_request.group_name);

  // getJointNames
  for(size_t i = 0 ; i < js.name.size(); i++)
  {
    std::string name = js.name[i];
    if( !jmg->hasJointModel(name) ) continue;

    traj.joint_names.push_back(js.name[i]);
    pt.positions.push_back(js.position[i]);
    if(js.velocity.size()) pt.velocities.push_back(js.velocity[i]);
  }
  pt.time_from_start = ros::Duration(0.1);
  traj.points.push_back(pt);

  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = "odom_combined";

  res.trajectory.joint_trajectory = traj;

  return true;
}

} // namespace local_planners

PLUGINLIB_DECLARE_CLASS(local_planners, ConvexConstraintSolver,
                        local_planners::ConvexConstraintSolver,
                        planning_interface::Planner);
