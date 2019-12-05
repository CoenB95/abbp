import sys

ROOT_DIR = '/Users/kjwdamme/School/jaar4/Project/Fase2/abbp'
PYREALSENSE_DIR = '/Users/kjwdamme/School/jaar4/Project/Fase2/librealsense/build/wrappers/python'
sys.path.append(ROOT_DIR)
sys.path.append(PYREALSENSE_DIR)

import pyrealsense2 as rs
import numpy as np
import cv2
import skimage.io

import os
import tensorflow as tf
import matplotlib.pyplot as plt
import time

# Import Mask RCNN
from mrcnn import visualize
import mrcnn.model as modellib

import training

# Directory to save logs and trained model
MODEL_DIR = os.path.join(ROOT_DIR, "logs")

# Path to Objects trained weights
OBJECT_WEIGHTS_PATH = "mask_rcnn_abbp_0030.h5"

config = training.ABBPConfig()


# Override the training configurations with a few
# changes for inferencing.
class InferenceConfig(config.__class__):
    # Run detection on one image at a time
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
    IMAGE_CHANNEL_COUNT = 4
    MEAN_PIXEL = np.array([123.7, 116.8, 103.9, 0.5])


config = InferenceConfig()
config.display()

# Device to load the neural network on.
# Useful if you're training a model on the same
# machine, in which case use CPU and leave the
# GPU for training.
DEVICE = "/cpu:0"  # /cpu:0 or /gpu:0

#with tf.device(DEVICE):
#     model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR,
#                              config=config)

# model.load_weights(OBJECT_WEIGHTS_PATH, by_name=True)

class_names = [
    "BG",
    "one",
    "two",
    "three",
    "four",
    "five",
    "fix"
]


def visualize_mask(image, boxes, masks, class_ids, class_names, scores=None,
                   title=""):
    # Number of instances
    N = boxes.shape[0]
    if not N:
        print("No instances to dislay")
    else:
        assert boxes.shape[0] == masks.shape[-1] == class_ids.shape[0]

    colors = visualize.random_colors(N)
    
    masked_image = image.copy()
    for i in range(N):
        color = colors[i]

        # Bouding box
        if not np.any(boxes[i]):
            # Skip this instance because it does not have a bounding box
            continue
        y1, x1, y2, x2 = boxes[i]

        cv2.rectangle(masked_image, (x1, y1), (x2, y2), rgb(color), 2)

        mask = masks[:, :, i]

        alpha = 0.5

        # Visualizing the mask
        # 3 Because 3 color channels
        for c in range(3):
            masked_image[:, :, c] = np.where(mask == 1, image[:, :, c] * (1 -
                                                                          alpha)
                                             + alpha * color[c] * 255,
                                             masked_image[:, :, c])

        # Visualizing class and score
        class_id = class_ids[i]
        score = scores[i] if scores is not None else None
        label = class_names[class_id]
        caption = "{} {:.3f}".format(label, score) if score else label
        cv2.putText(masked_image, caption, (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX , 0.5, (255, 255, 255))

    return masked_image

def rgb(color):
    rgb = []
    for v in list(color):
       rgb.append(v * 255)
    return rgb

def test():
    image = cv2.imread("test.png")

    results = model.detect([image], verbose=1) 

    r = results[0]

    masked_image = visualize_mask(image, r['rois'], r['masks'], r['class_ids'],
                           class_names, r['scores'], title="Predictions")

    cv2.imshow("predictions", masked_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    #
    # # Start streaming
    pipeline.start(config)
    #

    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        colorizer = rs.colorizer()
        colorizer.set_option(rs.option.color_scheme, 2)
        colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        align = rs.align(rs.stream.color)
        frameset = align.process(frames)

        aligned_depth_frame = frameset.get_depth_frame()
        colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
        converted_depth = skimage.color.rgb2gray(colorized_depth)
        stacked = np.dstack((color_image, colorized_depth))

        print(stacked.shape)

        # images = np.hstack((stacked[:, :, :3], colorized_depth[:, :, 3]))

        #results = model.detect([color_image], verbose=1)

        # Display results
        #ax = get_ax(1)
        #r = results[0]
        #print(r['class_ids'])
        #image = visualize_mask(color_image, r['rois'], r['masks'], r['class_ids'],
        #                                class_names, r['scores'], title="Predictions")

        # images = np.hstack((converted_depth, colorized_depth))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('converted', converted_depth)
        cv2.imshow('Not converted', colorized_depth)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    pipeline.stop()
    cv2.destroyAllWindows()

main()


