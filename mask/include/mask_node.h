#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>

using namespace ros;
using namespace std;

class MaskNode {
public:
  MaskNode();

private:
  cv_bridge::CvImagePtr colorImagePtr = nullptr;
  cv_bridge::CvImagePtr depthImagePtr = nullptr;

  NodeHandle nodeHandle;

  Publisher instructionsPublisher;

  void loop();
  void onColorImage(const sensor_msgs::ImageConstPtr& msg);
  void onDepthImage(const sensor_msgs::ImageConstPtr& msg);
};