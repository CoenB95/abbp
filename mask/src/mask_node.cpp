#include <cv_bridge/cv_bridge.h>
#include <mask_rcnn_ros/RectArray.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "avans-vision-lib/utils.h"

#include "mask_node.h"

using namespace std;

int main(int argc, char** argv) {
  init(argc, argv, "mask_node");
  MaskNode masknode;
  masknode.loop();
}

MaskNode::MaskNode() {
  ROS_INFO("Masking init");
  colorImageListener = nodeHandle.subscribe("/camera/color/image_raw", 10, &MaskNode::onColorImage, this);
  depthImageListener = nodeHandle.subscribe("/camera/aligned_depth_to_color/image_raw", 10, &MaskNode::onDepthImage, this);
  maskDetectionListener = nodeHandle.subscribe("/object_detector/rects", 10, &MaskNode::onMaskDetection, this);
}

void MaskNode::loop() {
  ROS_INFO("Masking started");
  Rate fastRate(10);
  Rate slowRate(1);

  while(ros::ok()) {
    //Needed to let ROS update its pubs 'n subs.
    spinOnce();

    //Needed to let OpenCV update its windows.
    cv::waitKey(1);

    if (colorImagePtr == nullptr || depthImagePtr == nullptr) {
      ROS_INFO("Waiting for images...");
      slowRate.sleep();
      continue;
    }

    fastRate.sleep();
  }
  
  ROS_INFO("Masking stopped");
}

void MaskNode::onColorImage(const sensor_msgs::ImageConstPtr& msg) {
  try {
    colorImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Error reading color image: %s", e.what());
    colorImagePtr = nullptr;
    return;
  }
}

void MaskNode::onDepthImage(const sensor_msgs::ImageConstPtr& msg) {
  try {
    depthImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Error reading depth image: %s", e.what());
    depthImagePtr = nullptr;
    return;
  }
}

void MaskNode::onMaskDetection(const mask_rcnn_ros::RectArrayConstPtr& msg) {
  ROS_INFO("Mask produced!");
  if (msg->indices.size() < 1) {
    ROS_INFO("  Nothing");
    return;
  }

  ROS_INFO("  First item:");
  ROS_INFO_STREAM("  class_id: " << msg->labels[0] << " (" << msg->names[0] << ")");
  ROS_INFO_STREAM("  score: " << msg->likelihood[0]);
  geometry_msgs::Point32 tl = msg->polygon[0].polygon.points[0];
  geometry_msgs::Point32 br = msg->polygon[0].polygon.points[1];

  cv::Mat masked_image(colorImagePtr->image);
  cv::rectangle(masked_image, cv::Point2f(tl.x, tl.y), cv::Point2f(br.x, br.y), Colors::RED);
  ostringstream s;
  s << msg->names[0] << " " << fixed << setprecision(3) << msg->likelihood[0];
  ROS_INFO("%s", s.str().c_str());
  cv::putText(masked_image, s.str(), cv::Point(tl.x, tl.y - 8), cv::FONT_HERSHEY_SIMPLEX , 0.5, Colors::WHITE);
  ImageUtils::window(masked_image, "Result", true);
}