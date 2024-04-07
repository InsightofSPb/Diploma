import os
import cv2
import rosbag
from cv_bridge import CvBridge
import numpy as np

# Specify the path to your ROS bag file
BAG_FILE = '/home/s/dipl/src/hector_quadrotor/controller/b.bag'

# Specify the image topics you want to extract
IMAGE_TOPIC_RGB = '/asus_camera/rgb/camera_info'

with rosbag.Bag(BAG_FILE, 'r') as bag:
    # Extract and save RGB images
    for topic, msg, t in bag.read_messages(topics=[IMAGE_TOPIC_RGB]):
        print(msg)
        print('---------------------------------')
        break

print("Image extraction complete.")