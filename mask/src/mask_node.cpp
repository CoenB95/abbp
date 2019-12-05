#include <cv_bridge/cv_bridge.h>
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
  //roscovery.loop();
}

MaskNode::MaskNode() {

  nodeHandle.subscribe("/camera/color/image_raw", 1, &MaskNode::onColorImage, this);
  nodeHandle.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &MaskNode::onDepthImage, this);

  //image_pub_ = it_.advertise("/camerabeeld", 1);
}

void MaskNode::loop() {
  ROS_INFO("Masking started");
  Rate fastRate(10);
  Rate slowRate(1);

  while(ros::ok()) {
    if (colorImagePtr == nullptr || depthImagePtr == nullptr) {
      ROS_INFO("Waiting for images...");
      slowRate.sleep();
    } else {
      fastRate.sleep();
    }
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
    depthImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Error reading depth image: %s", e.what());
    depthImagePtr = nullptr;
    return;
  }
}