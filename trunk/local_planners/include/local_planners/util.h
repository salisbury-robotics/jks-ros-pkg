#ifndef LOCAL_PLANNERS_UTIL_H
#define LOCAL_PLANNERS_UTIL_H

#include <planning_scene/planning_scene.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/MotionPlanDetailedResponse.h>
#include <planning_interface/planning_interface.h>
#include <planning_models/conversions.h>

namespace local_planners {

  inline double normalizeAngle( double angle )
  {
    if(angle < -M_PI || angle > M_PI)
    {
      return angle - copysign(1.0, angle)*2*M_PI*ceil(fabs(angle)/ (2*M_PI));
    }
    return angle;
  }

  inline void printCollisionInfo(const planning_scene::PlanningScene& ps, const planning_models::KinematicState& ks )
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

  inline void createKinematicStatePoint(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                 planning_models::KinematicState &start_state, planning_models::KinematicState &goal_state,
                                 std::map<std::string, double> &name_map,
                                 double step_size,
                                 planning_models::KinematicStatePtr &point)
  {
    for (std::map<std::string, double>::const_iterator it = name_map.begin() ; it != name_map.end() ; ++it)
    {
      double sv = start_state.getJointState(it->first)->getVariableValues()[0];
      double gv = goal_state.getJointState(it->first)->getVariableValues()[0];
      std::vector<moveit_msgs::JointLimits> limits = planning_scene->getKinematicModel()->getJointModel(it->first)->getJointLimits();
      double u = 0;
      moveit_msgs::JointLimits &limit = limits[0];

      gv = normalizeAngle(gv);
      sv = normalizeAngle(sv);
      // Handle wrap around
      double delta = gv - sv;
      if( delta >= M_PI || delta <= -M_PI )
        u = sv - step_size;
      else
        u = sv + step_size;

      point->getJointState(it->first)->setVariableValues(&u);
    }
  }

  inline void createKinematicStatePoint(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                 planning_models::KinematicState &start_state, planning_models::KinematicState &goal_state,
                                 std::map<std::string, double> &name_map,
                                 unsigned int step_index, unsigned int num_steps,
                                 planning_models::KinematicStatePtr &point)
  {
    for (std::map<std::string, double>::const_iterator it = name_map.begin() ; it != name_map.end() ; ++it)
    {
      double sv = start_state.getJointState(it->first)->getVariableValues()[0];
      double gv = goal_state.getJointState(it->first)->getVariableValues()[0];
      std::vector<moveit_msgs::JointLimits> limits = planning_scene->getKinematicModel()->getJointModel(it->first)->getJointLimits();
      double u = 0;
      moveit_msgs::JointLimits &limit = limits[0];

      gv = normalizeAngle(gv);
      sv = normalizeAngle(sv);
      // Handle wrap around
      double delta = gv - sv;
      if( delta >= M_PI || delta <= -M_PI )
        u = sv - step_index * delta/(double)num_steps;
      else
        u = sv + step_index * delta/(double)num_steps;

      point->getJointState(it->first)->setVariableValues(&u);
    }
  }

}

#endif
