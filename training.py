import sys
import os

ROOT_DIR = '/Users/kjwdamme/School/jaar4/Project/Fase2/abbp'
sys.path.insert(0, ROOT_DIR)

import random
import math
import numpy as np
import skimage.io
import json
import h5py
import cv2

from mrcnn.config import Config
from mrcnn import model as modellib, utils

MODEL_DIR = os.path.join(ROOT_DIR, "logs")

# Directory of images to run detection on
IMAGE_DIR = os.path.join(ROOT_DIR, "images")

DEFAULT_LOGS_DIR = os.path.join(ROOT_DIR, "logs")

COCO_WEIGHTS_PATH = os.path.join(ROOT_DIR, "coco.h5") 


# CONFIGURATION 
class ABBPConfig(Config):
    """Configuration for training on the toy  dataset.
    Derives from the base Config class and overrides some values.
    """
    # Give the configuration a recognizable name
    NAME = "ABBP"

    # Adjust down if you use a smaller GPU.
    IMAGES_PER_GPU = 1

    # Number of classes (including background)
    NUM_CLASSES = 1 + 6  # Background + balloon
    # Number of training steps per epoch
    STEPS_PER_EPOCH = 100

    # Skip detections with < 90% confidence
    DETECTION_MIN_CONFIDENCE = 0.9


# Dataset class ABBPDataset(utils.Dataset):
class ABBPDataset(utils.Dataset):
    def load_object(self, dataset_dir):
        """Load a subset of the Balloon dataset.
        dataset_dir: Root directory of the dataset.
        subset: Subset to load: train or val
        """
        # Add classes. 6 Objects to be detected so 6 classes.
        self.add_class("ABBP", 1, "one")
        self.add_class("ABBP", 2, "two")
        self.add_class("ABBP", 3, "three")
        self.add_class("ABBP", 4, "four")
        self.add_class("ABBP", 5, "five")
        self.add_class("ABBP", 6, "six")

        # Train or validation dataset?
        # assert subset in ["train", "val"]
        # dataset_dir = os.path.join(dataset_dir, subset)

        # Load annotations
        annotations = json.load(open(os.path.join(dataset_dir, "annotations.json")))
        annotations = list(annotations.values())  # don't need the dict keys

        # Add images
        for a in annotations:
            x_points = a['region']['all_points_x']
            y_points = a['region']['all_points_y']

            # for x, y in zip(x_points, y_points):
            #     polygon.append(np.array([x, y]).transpose())

            self.add_image(
                "ABBP",
                image_id=a['filename'],  # use file name as a unique image id
                path=os.path.join(dataset_dir, a['filename']),
                width=a['width'], height=a['height'],
                polygon=[x_points, y_points],
                class_id=int(a['region']['class']))

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
        # info = next(filter(lambda x: x['id'] == image_id, self.image_info))
        
        mask = np.zeros([info["height"], info["width"], 1], dtype=np.uint8)
        rr, cc = skimage.draw.polygon(info["polygon"][0], info["polygon"][1])

        mask[cc, rr, 0] = 1

        # Return mask, and array of class IDs of each instance. Since we have
        # one class ID only, we return an array of 1s
        return mask.astype(np.bool), np.array([info['class_id']])

    def image_reference(self, image_id):
        info = next(filter(lambda x: x['id'] == image_id, self.image_info))

        return info['path']


def train(model):
    """Train the model."""
    # Training dataset.
    dataset_train = ABBPDataset()
    dataset_train.load_object("images")
    dataset_train.prepare()

    # Validation dataset
    dataset_val = ABBPDataset()
    dataset_val.load_object("val_images")
    dataset_val.prepare()

    # *** This training schedule is an example. Update to your needs ***
    # Since we're using a very small dataset, and starting from
    # COCO trained weights, we don't need to train too long. Also,
    # no need to train all layers, just the heads should do it.
    print("Training network heads")
    model.train(dataset_train, dataset_val,
                learning_rate=.7,
                epochs=30,
                layers='heads')


def tests():
    dataset = ABBPDataset()
    dataset.load_object("images")

    info = dataset.image_info[0]

    print(info['id'])
    mask, class_ids = dataset.load_mask(5)

    _idx = np.sum(mask, axis=(0, 1)) > 0
    mask = mask[:, :, _idx]
    class_ids = class_ids[_idx]

    cv2.imshow('mask', np.array(mask * 255, dtype=np.uint8))

    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main():
    import argparse

    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Train Mask R-CNN to detect objects.')
    parser.add_argument("command",
                        metavar="<command>",
                        help="'train' or 'splash'")
    parser.add_argument('--dataset', required=False,
                        metavar="/path/to/balloon/dataset/",
                        help='Directory of the Balloon dataset')
    parser.add_argument('--weights', required=True,
                        metavar="/path/to/weights.h5",
                        help="Path to weights .h5 file or 'coco'")
    parser.add_argument('--logs', required=False,
                        default=DEFAULT_LOGS_DIR,
                        metavar="/path/to/logs/",
                        help='Logs and checkpoints directory (default=logs/)')
    parser.add_argument('--image', required=False,
                        metavar="path or URL to image",
                        help='Image to apply the color splash effect on')
    parser.add_argument('--video', required=False,
                        metavar="path or URL to video",
                        help='Video to apply the color splash effect on')
    args = parser.parse_args()

    # Validate arguments
    if args.command == "train":
        assert args.dataset, "Argument --dataset is required for training"
    elif args.command == "splash":
        assert args.image or args.video, \
            "Provide --image or --video to apply color splash"

    print("Weights: ", args.weights)
    print("Dataset: ", args.dataset)
    print("Logs: ", args.logs)

    # Configurations
    if args.command == "train":
        config = ABBPConfig()
    else:
        class InferenceConfig(ABBPConfig):
            # Set batch size to 1 since we'll be running inference on
            # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
            GPU_COUNT = 1
            IMAGES_PER_GPU = 1

        config = InferenceConfig()
    config.display()

    # Create model
    if args.command == "train":
        model = modellib.MaskRCNN(mode="training", config=config,
                                  model_dir=args.logs)
    else:
        model = modellib.MaskRCNN(mode="inference", config=config,
                                  model_dir=args.logs)

    # Select weights file to load
    if args.weights.lower() == "coco":
        weights_path = COCO_WEIGHTS_PATH
        # Download weights file
        if not os.path.exists(weights_path):
            utils.download_trained_weights(weights_path)
    elif args.weights.lower() == "last":
        # Find last trained weights
        weights_path = model.find_last()
    elif args.weights.lower() == "imagenet":
        # Start from ImageNet trained weights
        weights_path = model.get_imagenet_weights()
    else:
        weights_path = args.weights

    # Load weights
    print("Loading weights ", weights_path)

    # number of classes
    model.load_weights(weights_path, by_name=True, exclude=[
        "mrcnn_class_logits", "mrcnn_bbox_fc",
        "mrcnn_bbox", "mrcnn_mask"])
    # Train or evaluate
    train(model)


main()
