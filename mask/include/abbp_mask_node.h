#pragma once

#include <abbp_mask/DepthPose.h>
#include <cv_bridge/cv_bridge.h>
#include <mask_rcnn_ros/RectArray.h>
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
  bool hideCircleDepth;
  bool hideMaskDepth;

  cv_bridge::CvImagePtr colorImagePtr = nullptr;
  cv_bridge::CvImagePtr colorImageSnapshotPtr = nullptr;
  cv_bridge::CvImagePtr depthImagePtr = nullptr;
  cv_bridge::CvImagePtr depthImageSnapshotPtr = nullptr;

  NodeHandle nodeHandle;

  Subscriber colorImageListener;
  Subscriber depthImageListener;
  Subscriber maskDetectionListener;

  Publisher circlePosePublisher;
  Publisher objectImagePublisher;
  Publisher objectPosePublisher;

  vector<abbp_mask::DepthPose> props;

  void onColorImage(const sensor_msgs::ImageConstPtr& msg);
  void onDepthImage(const sensor_msgs::ImageConstPtr& msg);
  void onMaskDetection(const mask_rcnn_ros::RectArrayConstPtr& msg);
};