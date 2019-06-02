#!/usr/bin/env python

import rospy
import cv2
import sys
import os
import csv

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pynput.keyboard import KeyCode, Listener

class DatasetBuilder():
    def __init__(self, dataset_name):
        self.bridge = CvBridge()
        self.dataset_name = dataset_name
        self.images_captured = 0
        self.csv_data = [['file', 'label']]

        self.image = None
        self.traffic_lights = None

        rospy.Subscriber('/image_color', Image, self.image_callback)

        if os.path.exists(dataset_name):
            rospy.logerr('Cannot create dataset with name: {}. Directory already exists.'.format(dataset_name))
            exit()

        os.mkdir(dataset_name)
        os.mkdir(dataset_name + '/imgs')

        rospy.loginfo('Created new dataset in directory: ' + dataset_name + '.')

        with Listener(on_press=self.on_press) as listener:
            listener.join()

    def image_callback(self, msg):
        self.image = msg

    def traffic_lights_callback(self, msg):
        self.traffic_lights = msg

    def on_press(self, key):
        if self.image is None:
            rospy.logerr('No image captured')

        if   key == KeyCode.from_char('1'):
            label = 'red'
        elif key == KeyCode.from_char('2'):
            label = 'yellow'
        elif key == KeyCode.from_char('3'):
            label = 'green'
        elif key == KeyCode.from_char('4'):
            label = 'none'
        else: return

        rospy.loginfo('Capturing image with label: {}.'.format(label))

        img_file = 'imgs/{:03d}.png'.format(self.images_captured)
        cv_image = self.bridge.imgmsg_to_cv2(self.image, 'bgr8')
        cv2.imwrite('{}/{}'.format(self.dataset_name, img_file), cv_image)

        self.csv_data.append([img_file, label])
        with open('{}/dataset.csv'.format(self.dataset_name), 'w') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(self.csv_data)

        self.images_captured += 1

if __name__ == '__main__':
    rospy.init_node('dataset_builder', anonymous=True)
    builder = DatasetBuilder(sys.argv[1])
    rospy.spin()
