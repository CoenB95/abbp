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
  namedWindow("Live", cv::WINDOW_AUTOSIZE);
  namedWindow("Result", cv::WINDOW_AUTOSIZE);
  masknode.loop();
}

MaskNode::MaskNode() {
  ROS_INFO("Masking init");
  colorImageListener = nodeHandle.subscribe("/camera/color/image_raw", 10, &MaskNode::onColorImage, this);
  depthImageListener = nodeHandle.subscribe("/camera/aligned_depth_to_color/image_raw", 10, &MaskNode::onDepthImage, this);
  maskDetectionListener = nodeHandle.subscribe("/object_detector/rects", 10, &MaskNode::onMaskDetection, this);

  colorImagePublisher = nodeHandle.advertise<sensor_msgs::Image>("/mask_node/camera/color/image_raw", 1);
}

void MaskNode::loop() {
  ROS_INFO("Masking started");
  Rate fastRate(10);
  Rate slowRate(1);

  while(ros::ok()) {
    //Needed to let ROS update its pubs 'n subs.
    spinOnce();

    //Needed to let OpenCV update its windows.
    int k = cv::waitKey(1);

    if (colorImagePtr == nullptr || depthImagePtr == nullptr) {
      ROS_INFO("Waiting for images...");
      slowRate.sleep();
      continue;
    }

    if (k == 'a') {
      savedColorImagePtr = colorImagePtr;
      ROS_INFO("Published photo!");
      colorImagePublisher.publish(savedColorImagePtr->toImageMsg());
    }

    fastRate.sleep();
  }
  
  ROS_INFO("Masking stopped");
}

void MaskNode::onColorImage(const sensor_msgs::ImageConstPtr& msg) {
  try {
    colorImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    ImageUtils::window(colorImagePtr->image, "Live", true);
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

  //ROS_INFO("  First item:");
  //ROS_INFO_STREAM("  class_id: " << msg->labels[0] << " (" << msg->names[0] << ")");
  //ROS_INFO_STREAM("  score: " << msg->likelihood[0]);

  int w = colorImagePtr->image.cols;
  int h = colorImagePtr->image.rows;
  cv::Mat masked_image(savedColorImagePtr->image);

  vector<Color> randomColors = ColorUtils::randomColors(msg->labels.size());

  for (int i = 0; i < msg->labels.size(); i++) {
    //ROS_INFO_STREAM("Image (mask) size: " << mask.rows * mask.cols << " = " << mask.rows << "x" << mask.cols);
    //ROS_INFO_STREAM("Indices size     : " << msg->indices.size());

    geometry_msgs::Point32 tl = msg->polygon[i].polygon.points[0];
    geometry_msgs::Point32 br = msg->polygon[i].polygon.points[1];

    cv::rectangle(masked_image, cv::Point2f(tl.x, tl.y), cv::Point2f(br.x, br.y), randomColors[i]);

    int si = i * w * h;
    int ei = (i + 1) * w * h - 1;
    ROS_INFO_STREAM("Masking #" << i << ", index " << si << " till " << ei << " of " << msg->indices.size());
    vector<int16_t> sub16S;
    for (int i = si; i < ei; i++) {
      sub16S.push_back(msg->indices[i]);
    }
    //vector<int64_t> sub(msg->indices.begin() + si, msg->indices.begin() + ei);
    cv::Mat mask(h, w, CV_16S, (void*)(sub16S.data()));

    //ImageUtils::forEachPixel<Color>(masked_image, masked_image, [&](cv::Point pc, Color px) { return ImageUtils::getPixel16S(mask, pc) == 0 ? px : Colors::ORANGE; });
    ImageUtils::forEachPixel<Color>(masked_image, masked_image, [&](cv::Point pc, Color px) {
       return ImageUtils::getPixel<int16_t>(mask, pc) == 0 ? px : px * (1 - 0.5) + randomColors[i] * 0.5;
       //image[:, :, c] * (1 - alpha) + alpha * color[c] * 255
    });
    /*ImageUtils::forEachPixel<Color>(masked_image, masked_image, [&](cv::Point pc, Color px) {
      int pv = ImageUtils::getPixel16S(mask, pc);
      if (pv == 0) {
        //ROS_INFO("Pox ");
        return px;
      } else {
        ROS_INFO("Pix ");
        return Colors::ORANGE;
      }
    });*/

    ostringstream s;
    s << msg->names[i] << " " << fixed << setprecision(3) << msg->likelihood[i];
    ROS_INFO("%s", s.str().c_str());
    cv::putText(masked_image, s.str(), cv::Point(tl.x, tl.y - 8), cv::FONT_HERSHEY_SIMPLEX , 0.5, Colors::WHITE);
  }

  ImageUtils::window(masked_image, "Result", true);
}