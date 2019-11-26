#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>

using namespace ros;
using namespace std;
using namespace moveit::planning_interface;
using namespace moveit_visual_tools;

class Roscovery {
public:
  Roscovery();

private:
  static const vector<string> JOINT_NAMES;
  static const std::string PLANNING_GROUP;

  MoveGroupInterface* moveGroup;
  PlanningSceneInterface* planningSceneInterface;
  MoveItVisualTools* visualTools;

  NodeHandle nodeHandle;

  Publisher instructionsPublisher;

  void addFloor();
};