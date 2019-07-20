import os
import time

import tensorflow as tf
from keras.preprocessing.image import array_to_img, img_to_array, load_img
from PIL import Image, ImageOps
import numpy as np

from object_classifier import ObjectClassifier
from color_classifier import ColorClassifier
#from traffic_light_colors import TrafficLight #had isseus around the emun import so useing styx.msg below
from styx_msgs.msg import TrafficLight
import rospy

class TLClassifier(object):
    #john  added boolean is_site to indicate site or sim classifer to load
    def __init__(self, is_site): 
        # init object classifier (step 1)
        rospy.loginfo('TLClassifier start inititialization')
        self.object_classifier = ObjectClassifier()

        # init traffic light color classifier (step 2)
        #john added is_site log output 
        if is_site:
            rospy.loginfo('is_site TRUE using site classifer')
        else :
            rospy.loginfo('is_site FALSE using sim classifer')
            
        self.color_classifier = ColorClassifier(is_site)#john ...now sending boolena to color classifier
        
        rospy.loginfo('TLClassifier inititialized')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        rospy.loginfo('TLClassifier received image')

        # step 1
        traffic_light_images = self.object_classifier.get_traffic_light_images(image)
        
        #john there might be a probelm here what if there are 1 or 3 predictions how does that effect below?
        # eventually this gets passed back to tl_detector and is a singluar state maybe this sia problem?
        traffic_light_color = self.color_classifier.predict_images(traffic_light_images)

        rospy.loginfo('Traffic light detected {}'.format(traffic_light_color))
        #changed bacue now using styx msg #TrafficLight(traffic_light_color)))

        return traffic_light_color

