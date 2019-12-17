#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "avans-vision-lib/colors.h"
#include "avans-vision-lib/images.h"
#include "avans-vision-lib/utils.h"

#include "abbp_mask_node.h"

using namespace avl;
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

  objectImagePublisher = nodeHandle.advertise<sensor_msgs::Image>("/abbp_mask_node/object/image_raw", 1);
  objectPosePublisher = nodeHandle.advertise<abbp_mask::DepthPose>("/abbp_mask_node/object/pose", 1);

  ROS_INFO_STREAM("Found param /mask_node/disable_circle_depth: " << nodeHandle.hasParam("/mask_node/disable_circle_depth") ? "true" : "false");
  ROS_INFO_STREAM("Found param /disable_circle_depth: " << nodeHandle.hasParam("/disable_circle_depth") ? "true" : "false");
  ROS_INFO_STREAM("Found param disable_circle_depth: " << nodeHandle.hasParam("disable_circle_depth") ? "true" : "false");

  nodeHandle.param("/mask_node/disable_circle_depth", hideCircleDepth, true);
  nodeHandle.param("/mask_node/disable_mask_depth", hideMaskDepth, false);

  WindowUtils::window("Live");
  if (!hideCircleDepth) WindowUtils::window("Circle Result");
  if (!hideMaskDepth) WindowUtils::window("Mask Result");
}

void MaskNode::loop() {
  ROS_INFO("Masking started");

  while(ros::ok()) {
    //Needed to let ROS update its pubs 'n subs.
    spinOnce();

    //Needed to let OpenCV update its windows.
    int k = cv::waitKey(1);

    if (colorImagePtr == nullptr || depthImagePtr == nullptr) {
      ROS_INFO("Waiting for images...");
      cv::waitKey(1000);
      continue;
    }

    if (k == 'a') {
      ROS_INFO("Making snapshot of camera..");
      colorImageSnapshotPtr = colorImagePtr;
      depthImageSnapshotPtr = depthImagePtr;
      objectImagePublisher.publish(colorImageSnapshotPtr->toImageMsg());
      ROS_INFO("  Published color image");
    } else if (k > 48 && k < 58) {
      int i = k - 48;
      ROS_INFO_STREAM("Choose object #" << i);
      if (i < 1 || i > props.size()) {
        ROS_INFO_STREAM("  Not existing (max " << props.size() << ")");
      } else {
        objectPosePublisher.publish(props[i - 1]);
        ROS_INFO("  Published prop");
      }
    }
  }
  
  ROS_INFO("Masking stopped");
}

void MaskNode::onColorImage(const sensor_msgs::ImageConstPtr& msg) {
  try {
    colorImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    WindowUtils::showColor(colorImagePtr->image, "Live");
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Error reading color image: %s", e.what());
    colorImagePtr = nullptr;
    return;
  }

  Mat grayImage;
  cvtColor(colorImagePtr->image, grayImage, CV_RGB2GRAY);

  // Setup SimpleBlobDetector parameters.
  SimpleBlobDetector::Params params;
  params.minThreshold = 60;
  params.maxThreshold = 150;
  params.filterByArea = true;
  params.minArea = 600;
  params.maxArea = 16000;
  params.filterByCircularity = true;
  params.minCircularity = 0.4;
  params.filterByConvexity = false;
  params.minConvexity = 0.87;
  params.filterByInertia = false;
  params.minInertiaRatio = 0.01;
  Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

  // Detect blobs
  vector<KeyPoint> keypoints;
  detector->detect(grayImage, keypoints);

  // Draw detected blobs as red circles.
  // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
  // the size of the circle corresponds to the size of blob
  Mat resultImage;
  colorImagePtr->image.copyTo(resultImage);
  if (keypoints.size() > 0) {
    drawKeypoints(resultImage, keypoints, resultImage, Colors::RED * 0.5, DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    drawMarker(resultImage, keypoints[0].pt, Colors::RED, MarkerTypes::MARKER_CROSS);
  } else {
    ImageUtils::simpleOverlay(resultImage, "NOT FOUND");
  }

  WindowUtils::showColor(resultImage, "Circle Result", hideCircleDepth);
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
  props.clear();

  if (msg->indices.size() < 1) {
    ROS_INFO("  Nothing");
    return;
  }

  cv::Mat masked_image(colorImageSnapshotPtr->image);

  int imageWidth = colorImageSnapshotPtr->image.cols;
  int imageHeight = colorImageSnapshotPtr->image.rows;
  Point textOffset(0, - 8);
  vector<Color> randomColors = ColorUtils::randomColors(msg->labels.size());

  for (int i = 0; i < msg->labels.size(); i++) {
    Point tl = Point(msg->polygon[i].polygon.points[0].x, msg->polygon[i].polygon.points[0].y);
    Point br = Point(msg->polygon[i].polygon.points[1].x, msg->polygon[i].polygon.points[1].y);
    Rect objectRect(tl, br);
    Point objectPosition(tl);
    Point objectCenter((br - tl) / 2 + tl);
    float objectDepth = ImageUtils::getPixel<float>(depthImageSnapshotPtr->image, objectCenter);

    int si = i * imageWidth * imageHeight;
    int ei = (i + 1) * imageWidth * imageHeight - 1;
    vector<int16_t> sub16S;
    for (int i = si; i < ei; i++) {
      sub16S.push_back(msg->indices[i]);
    }
    cv::Mat objectMask(imageHeight, imageWidth, CV_16S, (void*)(sub16S.data()));

    ImageUtils::forEachPixel<Color>(masked_image, masked_image, [&](cv::Point pc, Color px) {
       return ImageUtils::getPixel<int16_t>(objectMask, pc) == 0 ? px : px * (1 - 0.5) + randomColors[i] * 0.5;
    });

    cv::rectangle(masked_image, objectRect, randomColors[i]);
    cv::drawMarker(masked_image, objectCenter, Colors::BLACK, cv::MarkerTypes::MARKER_CROSS, 8);

    abbp_mask::DepthPose prop;
    prop.x = objectCenter.x;
    prop.y = objectCenter.y;
    prop.depth = objectDepth;
    props.push_back(prop);

    ostringstream s;
    s << "#" << i + 1 << " " << msg->names[i] << " (" << fixed << setprecision(3) << msg->likelihood[i] << ") ";
    cv::putText(masked_image, s.str(), objectPosition + textOffset, cv::FONT_HERSHEY_SIMPLEX , 0.5, Colors::WHITE);

    ROS_INFO("  %s", s.str().c_str());
  }

  WindowUtils::showColor(masked_image, "Mask Result", hideMaskDepth);
}