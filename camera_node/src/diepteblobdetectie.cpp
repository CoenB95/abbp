#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <iostream>
#include <string>
#include <stdint.h>
#include <vector>
#include "camera_node/prop.h"

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

int32_t x,y,h,w;
float d;
Mat im_with_keypoints_;

void imageCb(const sensor_msgs::ImageConstPtr& msg);
void Depth_imageCb(const sensor_msgs::ImageConstPtr& msg);
int vindBal(Mat input, int32_t & x_, int32_t & y_, int32_t & h_, int32_t & w_, Mat & mask1);
void Callback_diepte(const sensor_msgs::ImageConstPtr& msg);

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber depth_image_sub_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;


public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
      &ImageConverter::imageCb, this);
    //Subsribe to depth image en depth image callback functie aanroepen
    depth_image_sub_ = it_.subscribe("/camera/aligned_depth_to_color/image_raw", 1,
      &ImageConverter::Depth_imageCb, this);
    image_pub_ = it_.advertise("/camerabeeld", 1);
   

    namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }

  
    Mat input = cv_ptr->image;
    vindBal(cv_ptr->image, x, y, h, w, im_with_keypoints_);
    imshow(OPENCV_WINDOW, cv_ptr->image);



    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void Depth_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptrD;
    try
    {
      cv_ptrD = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }

    Mat depth = cv_ptrD->image;
    d = depth.at<float>(y, x);
    ROS_INFO("Depth at %3d, %3d: %3.2f", x, y, d);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_node");
  ros::NodeHandle n;
  ros::Subscriber depth_sub = n.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 25, Callback_diepte);
  ros::Publisher  prop_pub = n.advertise<camera_node::prop>("blob_properties", 25);

  ImageConverter ic;

  ros::Rate loop_rate(50);
  while(ros::ok())
  {
    camera_node::prop data;

    data.x = x;
    data.y = y;
    data.h = h;
    data.w = w;
    data.d = d;

    prop_pub.publish(data);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

int vindBal(Mat input, int32_t & x_, int32_t & y_, int32_t & h_, int32_t & w_, Mat & im_with_keypoints)
{
  Mat GrayImage;
  vector<KeyPoint> keypoints;

  cvtColor(input, GrayImage, CV_RGB2GRAY);

  // Setup SimpleBlobDetector parameters.
  SimpleBlobDetector::Params params;

  // Change thresholds
  params.minThreshold = 60;
  params.maxThreshold = 150;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 600;
  params.maxArea = 16000;

  // Filter by Circularity
  params.filterByCircularity = true;
  params.minCircularity = 0.4;

  // Filter by Convexity
  params.filterByConvexity = false;
  params.minConvexity = 0.87;

  // Filter by Inertia
  params.filterByInertia = false;
  params.minInertiaRatio = 0.01;
  // Set up detector with params
  Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);



  // Detect blobs
  detector->detect(GrayImage, keypoints);

  // Draw detected blobs as red circles.
  // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
  // the size of the circle corresponds to the size of blob

 
  drawKeypoints(input, keypoints, input, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  int x1 = 0;
  int y1 = 0;
    if (keypoints.size() > 0)
    {
      x1 = keypoints[0].pt.x;
      y1 = keypoints[0].pt.y;
    }
  circle(input, Point(x1, y1), 2, Scalar(0, 0, 255), -1);
  x_ = x1;
  y_ = y1;


  h_ = input.cols;
  w_ = input.rows;

  return 0;
}

void Callback_diepte(const sensor_msgs::ImageConstPtr& msg) {
  return;
}