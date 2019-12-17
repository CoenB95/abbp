#pragma once

#include <camera_node/prop.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>

using namespace ros;
using namespace std;

class MaskNode {
public:
  MaskNode();

  void loop();

private:
  cv_bridge::CvImagePtr colorImagePtr = nullptr;
  cv_bridge::CvImagePtr colorImageSnapshotPtr = nullptr;
  cv_bridge::CvImagePtr depthImagePtr = nullptr;
  cv_bridge::CvImagePtr depthImageSnapshotPtr = nullptr;

  NodeHandle nodeHandle;

  Subscriber colorImageListener;
  Subscriber depthImageListener;
  Subscriber maskDetectionListener;

  Publisher colorImagePublisher;
  Publisher propPublisher;

  vector<camera_node::prop> props;

  void onColorImage(const sensor_msgs::ImageConstPtr& msg);
  void onDepthImage(const sensor_msgs::ImageConstPtr& msg);
  void onMaskDetection(const mask_rcnn_ros::RectArrayConstPtr& msg);
};