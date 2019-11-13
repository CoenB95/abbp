#include "abbp_robot_node.h"

namespace rvt = rviz_visual_tools;

int main(int argc, char** argv) {
  init(argc, argv, "abbp_robot_node");
  AsyncSpinner spinner(1);
  spinner.start();
  Roscovery roscovery;
}

const vector<string> Roscovery::JOINT_NAMES = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
const std::string Roscovery::PLANNING_GROUP = "manipulator";

Roscovery::Roscovery() {
  ROS_INFO("ABBP Robot started");

  moveGroup = new MoveGroupInterface(PLANNING_GROUP);
  planningSceneInterface = new PlanningSceneInterface();
  visualTools = new MoveItVisualTools("world");

  visualTools->deleteAllMarkers();
  visualTools->loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visualTools->publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visualTools->trigger();

  // Getting Basic Information
  ROS_INFO_STREAM("Planning frame: " << moveGroup->getPlanningFrame());
  ROS_INFO_STREAM("End effector link: " << moveGroup->getEndEffectorLink());
  ROS_INFO("Available Planning Groups:");
  std::copy(moveGroup->getJointModelGroupNames().begin(), moveGroup->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  visualTools->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  addFloor();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  moveGroup->setPoseTarget(target_pose1);

  ROS_INFO("Visualizing target pose");
  visualTools->deleteAllMarkers();
  visualTools->publishAxisLabeled(target_pose1, "Target");
  visualTools->trigger();
  visualTools->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  ROS_INFO("Planning movement...");
  MoveGroupInterface::Plan my_plan;
  MoveItErrorCode planResult = moveGroup->plan(my_plan);
  ROS_INFO("Planning %s", planResult == MoveItErrorCode::SUCCESS ? "SUCCEEDED" : "FAILED");

  ROS_INFO("Visualizing plan as trajectory line");
  visualTools->publishTrajectoryLine(my_plan.trajectory_, moveGroup->getCurrentState()->getJointModelGroup(PLANNING_GROUP));
  visualTools->trigger();
  visualTools->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  ROS_INFO("Moving robot according to plan...");
  MoveItErrorCode executionResult = moveGroup->execute(my_plan);
  ROS_INFO("Execution %s", planResult == MoveItErrorCode::SUCCESS ? "SUCCEEDED" : "FAILED");
}

void Roscovery::addFloor() {
  // Define a box to add to the world.
  shape_msgs::SolidPrimitive objectShape;
  objectShape.type = shape_msgs::SolidPrimitive::BOX;
  objectShape.dimensions.resize(3);
  objectShape.dimensions[0] = 2.0;
  objectShape.dimensions[1] = 2.0;
  objectShape.dimensions[2] = 0.1;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose objectPose;
  objectPose.orientation.w = 1.0;
  objectPose.position.x = 0.0;
  objectPose.position.y = 0.0;
  objectPose.position.z = -0.05;

  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collisionObject;
  collisionObject.header.frame_id = moveGroup->getPlanningFrame();
  collisionObject.id = "box1";
  collisionObject.primitives.push_back(objectShape);
  collisionObject.primitive_poses.push_back(objectPose);
  collisionObject.operation = collisionObject.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collisionObject);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planningSceneInterface->addCollisionObjects(collision_objects);

  // Show text in RViz of status
  visualTools->trigger();
  visualTools->prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");
}
