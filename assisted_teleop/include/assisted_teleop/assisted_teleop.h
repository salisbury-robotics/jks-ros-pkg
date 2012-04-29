#ifndef _ASSISTED_TELEOP_H_
#define _ASSISTED_TELEOP_H_

#include <ros/ros.h>
#include <moveit_visualization_ros/interactive_object_visualization_qt_wrapper.h>
#include <moveit_visualization_ros/interactive_object_visualization_widget.h>
#include <moveit_visualization_ros/planning_group_selection_menu.h>
#include <moveit_visualization_ros/planning_scene_file_menu.h>
//#include <moveit_visualization_ros/planning_visualization_qt_wrapper.h>
#include <assisted_teleop/teleop_visualization_qt_wrapper.h>
#include <moveit_visualization_ros/kinematic_state_joint_state_publisher.h>
#include <trajectory_execution_ros/trajectory_execution_monitor_ros.h>

#include <OGRE/OgreLogManager.h>
#include <rviz/visualization_panel.h>
#include <rviz/visualization_manager.h>
#include <QMenu>

namespace assisted_teleop {

class AssistedTeleop {

public:

  AssistedTeleop();

  ~AssistedTeleop();

  virtual void updatePlanningScene(planning_scene::PlanningSceneConstPtr planning_scene);

  void updateToCurrentState();

  bool doneWithExecution();

  void executeLastTrajectory();

protected:

  void updateSceneCallback();

  void publisherFunction(bool joint_states);

  bool first_update_;

  ros::Publisher vis_marker_array_publisher_;
  ros::Publisher vis_marker_publisher_;

  rviz::VisualizationPanel* rviz_frame_;
  QWidget* main_window_;
  moveit_visualization_ros::PlanningGroupSelectionMenu* planning_group_selection_menu_;
  QMenu* coll_object_menu_;
  boost::shared_ptr<tf::TransformListener> transformer_;

  bool allow_trajectory_execution_;

  planning_scene::PlanningSceneConstPtr current_diff_;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  boost::shared_ptr<KinematicStateJointStatePublisher> joint_state_publisher_;
  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  boost::shared_ptr<moveit_visualization_ros::TeleopVisualizationQtWrapper> tv_;
  boost::shared_ptr<moveit_visualization_ros::InteractiveObjectVisualizationQtWrapper> iov_;
  boost::shared_ptr<trajectory_execution::TrajectoryExecutionMonitor> trajectory_execution_monitor_;
  boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader> kinematics_plugin_loader_;


};

}

#endif
