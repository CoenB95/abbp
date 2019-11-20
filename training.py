import sys
import os
import numpy as np
import skimage.io
import json

ROOT_DIR = '/Users/kjwdamme/School/jaar4/Project/Fase2/abbp'
sys.path.insert(0, ROOT_DIR)

from mrcnn.config import Config
from mrcnn import utils

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
        # info = next(filter(lambda x: x['id'] == image_id, self.image_info))
        
        mask = np.zeros([info["height"], info["width"], 1], dtype=np.uint8)
        rr, cc = skimage.draw.polygon(info["polygon"][0], info["polygon"][1])

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

    # Validation dataset
    dataset_val = ABBPDataset()
    dataset_val.load_object("val_images")
    dataset_val.prepare()

    # Since we're using a very small dataset, and starting from
    # COCO trained weights, we don't need to train too long. Also,
    # no need to train all layers, just the heads should do it.
    print("Training network heads")
    model.train(dataset_train, dataset_val,
                learning_rate=0.001,
                epochs=30,
                layers='heads')
