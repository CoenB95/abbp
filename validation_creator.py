import sys
import os
import numpy as np
import skimage.io
import cv2
import keyboard

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
print("ROOT: " + str(ROOT_DIR))
sys.path.insert(0, ROOT_DIR)

# PYREALSENSE_DIR = ROOT_DIR + '/librealsense/build/wrappers/python'
PYREALSENSE_DIR = '/Users/kjwdamme/School/jaar4/Project/Fase2/librealsense/build/wrappers/python'
sys.path.append(PYREALSENSE_DIR)
print(PYREALSENSE_DIR)

import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

#
# # Start streaming
pipeline.start(config)
#
number = 0
img_class = input("Press image class ID: ")

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

    align = rs.align(rs.stream.color)
    frameset = align.process(frames)

    aligned_depth_frame = frameset.get_depth_frame()
    colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

    cv2.imshow("test", color_image)



    try:  # used try so that if user pressed other than the given key error will not be shown
        if keyboard.is_pressed('a'):
            # print("doe het eens")
            # print('depthimage-' + str(img_class) + '_' + str(number) + '.png')
            # print('image-' + str(img_class) + '_' + str(number) + '.png')
            cv2.imwrite(os.path.join('datasets/val_images3/depth', 'depthimage-' + str(img_class) + '_' + str(number) + '.png'), colorized_depth)
            cv2.imwrite(os.path.join('datasets/val_images3/color', 'image-' + str(img_class) + '_' + str(number) + '.png'), color_image)
            number += 1
            print("Saved picture!")
        elif keyboard.is_pressed('s'):
            number = 0
            img_class = input("Press image class ID: ")
    except:
        pass

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipeline.stop()
cv2.destroyAllWindows()