import sys
import os
import numpy as np
import skimage.io
import json
import cv2
import glob

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
print("ROOT: " + str(ROOT_DIR))
sys.path.insert(0, ROOT_DIR)

from mrcnn.config import Config
from mrcnn import utils, model as modellib, visualize
import annotation_generator

# PYREALSENSE_DIR = ROOT_DIR + '/librealsense/build/wrappers/python'
PYREALSENSE_DIR = '/Users/kjwdamme/School/jaar4/Project/Fase2/librealsense/build/wrappers/python'
sys.path.append(PYREALSENSE_DIR)
print(PYREALSENSE_DIR)

import pyrealsense2 as rs
from mrcnn import utils
import annotation_generator

MODEL_DIR = os.path.join(ROOT_DIR, "logs")

# Directory of images to run detection on
IMAGE_DIR = os.path.join(ROOT_DIR, "images")

DEFAULT_LOGS_DIR = os.path.join(ROOT_DIR, "logs")

COCO_WEIGHTS_PATH = os.path.join(ROOT_DIR, "coco.h5")

class_names = [
    "BG",
    "one",
    "two",
    "three",
    "four",
    "five",
]


# CONFIGURATION
class ABBPConfig(Config):
    """Configuration for training on the objects dataset.
    Derives from the base Config class and overrides some values.
    """
    # Give the configuration a recognizable name
    NAME = "ABBP"

    # Adjust down if you use a smaller GPU.
    IMAGES_PER_GPU = 2

    # Number of classes (including background)
    NUM_CLASSES = 1 + 5  # Background + balloon
    # Number of training steps per epoch
    STEPS_PER_EPOCH = 100

    # Skip detections with < 90% confidence
    DETECTION_MIN_CONFIDENCE = 0.9

    # NUMBER OF GPUs to use. When using only a CPU, this needs to be set to 1.
    GPU_COUNT = 2


# Dataset class ABBPDataset(utils.Dataset):
class ABBPDataset(utils.Dataset):
    def load_object(self, dataset_dir):
        """Load a subset of the Objects dataset.
        dataset_dir: Root directory of the dataset.
        subset: Subset to load: train or val
        """
        # Add classes. 6 Objects to be detected so 6 classes.
        self.add_class("ABBP", 1, "one")
        self.add_class("ABBP", 2, "two")
        self.add_class("ABBP", 3, "three")
        self.add_class("ABBP", 4, "four")
        self.add_class("ABBP", 5, "five")

        # Create annotations
        if not os.path.isfile(os.path.join(dataset_dir, "annotations.json")):
            annotation_generator.generate_annotations(dataset_dir)

        # Create annotations
        if not os.path.isfile(os.path.join(dataset_dir, "annotations.json")):
            annotation_generator.generate_annotations(dataset_dir)

        # Load annotations
        annotations = json.load(open(os.path.join(dataset_dir, "annotations.json")))
        annotations = list(annotations.values())  # don't need the dict keys

        # Add images
        for a in annotations:
            x_points = a['region']['all_points_x']
            y_points = a['region']['all_points_y']

            self.add_image(
                "ABBP",
                image_id=a['filename'],  # use file name as a unique image id
                path=os.path.join(dataset_dir, a['filename']),
                depth_path=os.path.join(dataset_dir, "depth" + a['filename']),
                width=a['width'], height=a['height'],
                polygon=[x_points, y_points],
                class_id=int(a['class']))

    def load_mask(self, image_id):
        """Generate instance masks for an image.
       Returns:
        masks: A bool array of shape [height, width, instance count] with
            one mask per instance.
        class_ids: a 1D array of class IDs of the instance masks.
        """

        # Convert polygons to a bitmap mask of shape
        # [height, width, instance_count]
        info = self.image_info[image_id]

        mask = np.zeros([info["height"], info["width"], 1], dtype=np.uint8)

        xpoints = info["polygon"][0]
        ypoints = info["polygon"][1]

        rr, cc = skimage.draw.polygon(xpoints, ypoints)

        mask[cc, rr, 0] = 1

        # Return mask, and array of class IDs of each instance. Since we have
        # one class ID only, we return an array of 1s
        return mask.astype(np.bool), np.array([info['class_id']])

    def image_reference(self, image_id):
        return self.image_info[image_id]


def validate():
    amount_correct = 0

    for file in glob.glob("datasets/val_images3/color/*.png"):
        img_array = cv2.imread(file)

        # Use model to predict classes from validation images
        results = model.detect([img_array], verbose=1)

        r = results[0]

        idx = len(file.split('/'))
        class_id = int(file.split('/')[idx - 1].split('-')[1].split('_')[0])


        print("Detected class id: " + str(r))
        print("Real class id:   : " + str(class_id))
        #
        # # Check wether there is only 1  class detected (because every
        # # validation image only contains 1 object) and if the detected class is
        # # the same as the class retrieved from the annotations
        if len(r['class_ids']) == 1 and class_id == r['class_ids'][0]:
            amount_correct += 1
            print("Correct 👌")
        else:
            print("Wrong 😡")

    # Calculate correct percentage
    accuracy = amount_correct / len(glob.glob("datasets/val_images3/color/*.png"))

    print(accuracy)


def rgb(color):
    rgb = []
    for v in list(color):
        rgb.append(v * 255)
    return rgb


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
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

    return masked_image


def train(model):
    """Train the model."""
    # Training dataset.
    dataset_train = ABBPDataset()
    dataset_train.load_object("datasets/images")
    dataset_train.prepare()

    dataset_val = ABBPDataset()
    dataset_val.load_object("datasets/val_images")
    dataset_val.prepare()

    # Since we're using a very small dataset, and starting from
    # COCO trained weights, we don't need to train too long. Also,
    # no need to train all layers, just the heads should do it.
    print("Training network heads")
    model.train(dataset_train, dataset_val,
                learning_rate=0.001,
                epochs=30,
                layers='heads')


def inference(model):
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

        align = rs.align(rs.stream.color)
        frameset = align.process(frames)

        aligned_depth_frame = frameset.get_depth_frame()
        colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

        # depth_gray = cv2.cvtColor(colorized_depth, cv2.COLOR_BGR2GRAY)
        #
        # x, mask = cv2.threshold(depth_gray, 15, 255, cv2.THRESH_BINARY_INV)
        #
        # dst = cv2.inpaint(depth_gray, mask, 3, cv2.INPAINT_NS)
        #
        # dst = cv2.cvtColor(dst, cv2.COLOR_GRAY2RGB)

        inference_results = model.detect([color_image], verbose=1)

        # Display results
        r = inference_results[0]
        print(r['class_ids'])
        result_image = visualize_mask(color_image,
                                      r['rois'], r['masks'],
                                      r['class_ids'],
                                      class_names, r['scores'],
                                      title="Predictions")

        # Show images
        cv2.imshow('RealSense', result_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    pipeline.stop()
    cv2.destroyAllWindows()


def image(model):
    image = cv2.imread("img.png")

    results = model.detect([image], verbose=1)

    r = results[0]

    image = visualize_mask(image, r['rois'], r['masks'], r['class_ids'], class_names, r['scores'], title="Predictions")

    cv2.imshow('dst', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


import argparse

# Parse command line arguments
parser = argparse.ArgumentParser(description='Train Mask R-CNN to detect custom objects')

parser.add_argument("command", metavar="<command>", help="'train' or 'inference'")

parser.add_argument('--dataset', required=False, metavar="/path/to/dataset/",
                    help='Directory of the Balloon dataset')

parser.add_argument('--logs', required=False, default=DEFAULT_LOGS_DIR,
                    metavar="path/to/logs/", help='Logs and checkpoints directory (default=logs/)')

parser.add_argument('--weights', required=True, metavar="/path/to/weights.h5",
                    help="Path to weights .h5 file or 'coco'")

args = parser.parse_args()

if args.command == "train":
    config = ABBPConfig()
else:
    class InferenceConfig(ABBPConfig):
        GPU_COUNT = 1
        IMAGES_PER_GPU = 1


    config = InferenceConfig()
config.display()

if args.command == "train":
    model = modellib.MaskRCNN(mode="training", config=config,
                              model_dir=args.logs)

    model.load_weights(args.weights, by_name=True,
                       exclude=["mrcnn_class_logits", "mrcnn_bbox_fc", "mrcnn_bbox", "mrcnn_mask"])
else:
    model = modellib.MaskRCNN(mode="inference", config=config,
                              model_dir=args.logs)

    model.load_weights(args.weights, by_name=True)


if args.command == "train":
    train(model)
elif args.command == "validate":
    validate()
elif args.command == "inference":
    inference(model)
else:
    print("'{}' is not recognized. Use 'train', 'validate' or 'inference'".format(args.command))
