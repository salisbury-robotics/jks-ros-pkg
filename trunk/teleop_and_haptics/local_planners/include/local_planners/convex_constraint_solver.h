#ifndef LOCAL_PLANNERS_CONVEX_CONSTRAINT_SOLVER_H
#define LOCAL_PLANNERS_CONVEX_CONSTRAINT_SOLVER_H

#include <planning_scene/planning_scene.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/MotionPlanDetailedResponse.h>
#include <planning_interface/planning_interface.h>
#include <planning_models/conversions.h>

namespace local_planners
{

class ConvexConstraintSolver : public planning_interface::Planner
{
  public:
    ConvexConstraintSolver() {}
    virtual ~ConvexConstraintSolver() {};

    /*********************************************************/
    /// Subclass may implement methods below
    virtual void init(const planning_models::KinematicModelConstPtr& model) {}

    /// Get a short string that identifies the planning interface
    virtual std::string getDescription(void) const { return "Adam's potential field planner/solver."; }

    /// Get the names of the known planning algorithms (values that can be filled as planner_id in the planning request)
    virtual void getPlanningAlgorithms(std::vector<std::string> &algs) const { algs.clear(); }

    /**********************************************************/
    /// Subclass must implement methods below
    virtual bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                       const moveit_msgs::GetMotionPlan::Request &req,
                       moveit_msgs::GetMotionPlan::Response &res) const;


    virtual bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
               const moveit_msgs::GetMotionPlan::Request &req,
               moveit_msgs::MotionPlanDetailedResponse &res) const
    {
      moveit_msgs::GetMotionPlan::Response res2;
      if (solve(planning_scene, req, res2))
      {
        res.trajectory_start = res2.trajectory_start;
        res.trajectory.push_back(res2.trajectory);
        res.description.push_back("Constrained motion.");
        res.processing_time.push_back(res2.planning_time);
        return true;
      }
      return false;
    }

    /// Determine whether this plugin instance is able to represent this planning request
    virtual bool canServiceRequest(const moveit_msgs::GetMotionPlan::Request &req,
                                   planning_interface::PlannerCapability& capabilities) const
    {
      ROS_WARN("This planner is allowing all motion planning requests, and may not be doing the right thing...");
      return true;
    }

    /// Request termination, if a solve() function is currently computing plans
    virtual void terminate(void) const {
    }

};


} // local_planners

#endif // LOCAL_PLANNERS_CONVEX_CONSTRAINT_SOLVER_H
