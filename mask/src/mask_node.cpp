#include <cv_bridge/cv_bridge.h>
#include <mask_rcnn_ros/RectArray.h>
#include <ros/ros.h>

#include "mask_node.h"

using namespace std;

const static string ROOT_DIR = "/Users/kjwdamme/School/jaar4/Project/Fase2/abbp";
const static string PYREALSENSE_DIR = "/Users/kjwdamme/School/jaar4/Project/Fase2/librealsense/build/wrappers/python";
//Directory to save logs and trained model
const static string MODEL_DIR = ROOT_DIR + "/logs";

//Path to Objects trained weights
const static string OBJECT_WEIGHTS_PATH = "mask_rcnn_abbp_0030.h5";

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
  //image_pub_ = it_.advertise("/camerabeeld", 1);
}

void MaskNode::loop() {
  ROS_INFO("Masking started");
  Rate fastRate(10);
  Rate slowRate(1);

  while(ros::ok()) {
    spinOnce();

    if (colorImagePtr == nullptr || depthImagePtr == nullptr) {
      ROS_INFO("Waiting for images...");
      slowRate.sleep();
      continue;
    }

    fastRate.sleep();
  }
  
  ROS_INFO("Masking stopped");
  /*

    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        results = model.detect([color_image], verbose=1)

        # Display results
        ax = get_ax(1)
        r = results[0]
        print(r['class_ids'])
        image = visualize_mask(color_image, r['rois'], r['masks'], r['class_ids'],
                                        class_names, r['scores'], title="Predictions")

        images = np.hstack((color_image, image))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    pipeline.stop()
    cv2.destroyAllWindows()*/
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
  ROS_INFO_STREAM("  index: " << msg->indices[0]);
  ROS_INFO_STREAM("  label: " << msg->labels[0]);
  ROS_INFO_STREAM("  likey: " << msg->likelihood[0]);
  ROS_INFO_STREAM("  name : " << msg->names[0]);
}