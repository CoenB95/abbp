import sys
import os
import numpy as np
import skimage.io
import json

ROOT_DIR = '/Users/kjwdamme/School/jaar4/Project/Fase2/abbp'
sys.path.insert(0, ROOT_DIR)

from mrcnn.config import Config
from mrcnn import utils, model as modellib

MODEL_DIR = os.path.join(ROOT_DIR, "logs")

# Directory of images to run detection on
IMAGE_DIR = os.path.join(ROOT_DIR, "images")

DEFAULT_LOGS_DIR = os.path.join(ROOT_DIR, "logs")

COCO_WEIGHTS_PATH = os.path.join(ROOT_DIR, "coco.h5") 


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
    NUM_CLASSES = 1 + 6  # Background + balloon
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
        self.add_class("ABBP", 6, "six")

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
        
        mask = np.zeros([info["height"], info["width"], 1], dtype=np.uint8)

        xpoints = info["polygon"][0]
        ypoints = info["polygon"][1]

        for xs, ys in zip(xpoints, ypoints):
            rr, cc = skimage.draw.polygon(xs, ys)

            mask[cc, rr, 0] = 1

        # Return mask, and array of class IDs of each instance. Since we have
        # one class ID only, we return an array of 1s
        return mask.astype(np.bool), np.array([info['class_id']])

    def image_reference(self, image_id):
        return self.image_info[image_id]


def train(model):
    """Train the model."""
    # Training dataset.
    dataset_train = ABBPDataset()
    dataset_train.load_object("images")
    dataset_train.prepare()

    # Since we're using a very small dataset, and starting from
    # COCO trained weights, we don't need to train too long. Also,
    # no need to train all layers, just the heads should do it.
    print("Training network heads")
    model.train(dataset_train, dataset_val,
                learning_rate=0.001,
                epochs=30,
                layers='heads')

def validate(model):
    dataset_val = ABBPDataset()
    dataset_val.load_object("val_images")
    dataset_val.prepare()

    amount_correct = 0

    for image in self.image_info:
        print(image)

        img_array = skimage.imread(image.path, as_gray=True)

        # Use model to predict classes from validation images
        results = model.detect([img_array], verbose=1)

        r = results[0]

        # Check wether there is only 1  class detected (because every
        # validation image only contains 1 object) and if the detected class is
        # the same as the class retrieved from the annotations
        if len(r['class_ids']) == 1 and  image['class'] == r['class_ids'][0]:
            amount_correct += 1

    # Calculate correct percentage
    return amount_correct / len(self.image_info) * 100


import argparse

# Parse command line arguments
parser = argparse.ArgumentParser(description='Train Mask R-CNN to detect custom
                                 objects')

parser.add_argument("command", metavar="<command>", help="'train' or
                    'inference'")

parser.add_argument('--dataset', required=True, metavar="/path/to/dataset/",
                    help='Directory of the Balloon dataset')

parser.add_argument('--logs', required=False, default=DEFAULT_LOGS_DIR,
                    metavar="path/to/logs/", help='Logs and checkpoints
                    directory (default=logs/)')

parser.add_argument('--weights', required=True, metavar="/path/to/weights.h5",
                    help="Path to weights .h5 file or 'coco'")

args = parser.parse_args()

if args.command == "train":
    config = ABBPConfig()
else:
    class InferenceConfig(ABBPConfig):
        GPU_COUNT = 1
        IMAGES_PER_GPU = 1
        IMAGE_CHANNEL_COUNT = 1
        MEAN PIXEL = 114.8
    config = InferenceConfig()
config.display()

if args.command == "train":
    model = modellib.MaskRCNN(mode="training", config=config,
                              model_dir=args.logs)
else:
    model = modellib.MaskRCNN(mode="inference", config=config,
                              model_dir=args.logs)

model.load_weights(args.weights, by_name=True, exclude=["conv1", "mrcnn_class_logits","mrcnn_bbox_fc", "mrcnn_bbox", "mrcnn_mask"])

if args.command == "train":
    train(model)
elif args.command == "validate":
    validate(model)
elif args.command == "inference":
    # TODO
