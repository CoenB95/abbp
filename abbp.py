import sys

ROOT_DIR = '/Users/kjwdamme/School/jaar4/Project/Fase2/abbp'
PYREALSENSE_DIR = '/Users/kjwdamme/School/jaar4/Project/Fase2/librealsense/build/wrappers/python'
sys.path.append(ROOT_DIR)
sys.path.append(PYREALSENSE_DIR)

import pyrealsense2 as rs
import numpy as np
import cv2

import os
import sys
import random
import math
import re
import time
import tensorflow as tf
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import skimage.io

# Import Mask RCNN
from mrcnn import utils
from mrcnn import visualize
from mrcnn.visualize import display_images
import mrcnn.model as modellib
from mrcnn.model import log

import training

# Directory to save logs and trained model
MODEL_DIR = os.path.join(ROOT_DIR, "logs")

# Path to Ballon trained weights
# You can download this file from the Releases page
# https://github.com/matterport/Mask_RCNN/releases
OBJECT_WEIGHTS_PATH = "mask_rcnn_abbp_0030.h5"

config = training.ABBPConfig()
IMAGES_DIR = os.path.join(ROOT_DIR, "test_images")


# Override the training configurations with a few
# changes for inferencing.
class InferenceConfig(config.__class__):
    # Run detection on one image at a time
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

config = InferenceConfig()
config.display()

# Device to load the neural network on.
# Useful if you're training a model on the same
# machine, in which case use CPU and leave the
# GPU for training.
DEVICE = "/cpu:0"  # /cpu:0 or /gpu:0

# Inspect the model in training or inference modes
# values: 'inference' or 'training'
# TODO: code for 'training' test mode not ready yet
TEST_MODE = "inference"


def get_ax(rows=1, cols=1, size=16):
    """Return a Matplotlib Axes array to be used in
    all visualizations in the notebook. Provide a
    central point to control graph sizes.

    Adjust the size attribute to control how big to render images
    """
    _, ax = plt.subplots(rows, cols, figsize=(size * cols, size * rows))
    return ax


with tf.device(DEVICE):
    model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR,
                              config=config)

model.load_weights(OBJECT_WEIGHTS_PATH, by_name=True)

class_names = [
    "BG",
    "one",
    "two",
    "three",
    "four",
    "five",
    "fix"
]


pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#
# # Start streaming
pipeline.start(config)
#
time.sleep(5)


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
    image = visualize.display_instances(color_image, r['rois'], r['masks'], r['class_ids'],
                                class_names, r['scores'], ax=ax,
                                title="Predictions")

    images = np.hstack((color_image, image))


    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


pipeline.stop()
cv2.destroyAllWindows()


