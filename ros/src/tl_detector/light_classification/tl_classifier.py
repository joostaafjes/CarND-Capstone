import os
import time

import tensorflow as tf
from keras.preprocessing.image import array_to_img, img_to_array, load_img
from PIL import Image, ImageOps
import numpy as np

from object_classifier import ObjectClassifier
from color_classifier import ColorClassifier

class TLClassifier(object):
    def __init__(self):
        # init object classifier (step 1)
        self.object_classifier = ObjectClassifier()

        # init traffic light color classifier (step 2)
        self.color_classifier = ColorClassifier()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        # step 1
        traffic_light_images = self.object_classifier.get_traffic_light_images(image)

        traffic_light_color = self.color_classifier.predict_images(traffic_light_images)

        return traffic_light_color

