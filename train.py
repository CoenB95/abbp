from mrcnn.config import Config
from mrcnn import model as modellib, utils

from training import ABBPDataset, ABBPConfig

config = ABBPConfig()
config.display()

model = modellib.MaskRCNN(mode="training", config=config, model_dir="./")

model.load_weights("mask_rcnn_coco.h5", by_name=True, exclude=[ "conv1", "mrcnn_class_logits", "mrcnn_bbox_fc", "mrcnn_bbox", "mrcnn_mask"])

dataset_train = ABBPDataset()
dataset_train.load_object("./datasets/images")
dataset_train.prepare()

# Validation dataset
dataset_val = ABBPDataset()
dataset_val.load_object("./datasets/val_images")
dataset_val.prepare()



# *** This training schedule is an example. Update to your needs ***
# Since we're using a very small dataset, and starting from
# COCO trained weights, we don't need to train too long. Also,
# no need to train all layers, just the heads should do it.
print("Training network heads")
model.train(dataset_train, dataset_val,
            learning_rate=.001,
            epochs=30,
            layers='heads')



