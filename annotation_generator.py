import numpy as np
import cv2
import glob
import json
import os


def find_contours(img):
    # Grayscale image
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Threshold the image
    ret, thresh = cv2.threshold(imgray, 127, 255, 0)

    # Find the contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    xpoints = np.array(contours[0]).transpose()[0][0]
    ypoints = np.array(contours[0]).transpose()[1][0]

    return xpoints, ypoints


class Image(object):
    def __init__(self, file):
        print(file)
        print("this is the path", os.path)
        self.image = cv2.imread(file)
        idx = len(file.split('/'))
        self.name = file.split('/')[idx - 1]
        self.classification = self.name.split('-')[1].split('_')[0]
        self.contours = find_contours(cv2.imread(file))


def generate_annotations():
    # Save the points to a json file
    images = []

    for file in glob.glob("./datasets/images/*.png"):
        images.append(Image(file))

    data = {}

    for image in images:
        region = {}

        x_points, y_points = image.contours

        region['all_points_x'] = x_points.tolist()
        region['all_points_y'] = y_points.tolist()

        image_information = {}
        height, width, _ = image.image.shape
        image_information['filename'] = image.name
        image_information['class'] = image.classification
        image_information['width'] = width
        image_information['height'] = height
        image_information['region'] = region

        data['image_' + image.name] = image_information

    with open('datasets/images/annotations.json', 'w') as json_file:
        json.dump(data, json_file)