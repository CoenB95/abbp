#pragma once

#include <abbp_mask/DepthPose.h>
#include <abbp_mask/DepthPoseService.h>
#include <cv_bridge/cv_bridge.h>
#include <mask_rcnn_ros/RectArray.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>

#include "colors.h"
#include "images.h"
#include "utils.h"

using namespace avl;
using namespace ros;
using namespace std;

class MaskResult {
public:
  const int id;
  const String className;
  const abbp_mask::DepthPose pose;

  MaskResult(int id, String className, abbp_mask::DepthPose pose)
   : id(id), className(className), pose(pose) { }
  static MaskResult of (int id, String className, Point position, double depth) {
    abbp_mask::DepthPose pose;
    pose.x = position.x;
    pose.y = position.y;
    pose.depth = depth;
    return MaskResult(id, className, pose);
  }
};

class MaskNode {
public:
  MaskNode();

  void loop();

private:
  bool hideCircleDepth;
  bool hideMaskDepth;
  bool maskingDone = false;

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

  ServiceServer maskServiceListener;

  vector<MaskResult> maskingResults;

  vector<Window*> windows;
  Window* liveResultWindow;
  Window* circleResultWindow;
  Window* maskResultWindow;
  ImageView* liveImageView = new ImageView();
  ImageView* circleImageView = new ImageView();
  ImageView* maskImageView = new ImageView();
  IndeterminateOverlay* progressCircle = nullptr;

  void onColorImage(const sensor_msgs::ImageConstPtr& msg);
  void onDepthImage(const sensor_msgs::ImageConstPtr& msg);
  void onMaskDetection(const mask_rcnn_ros::RectArrayConstPtr& msg);
  bool onMaskServiceCall(abbp_mask::DepthPoseService::Request& request, abbp_mask::DepthPoseService::Response& response);
  void mask();
};