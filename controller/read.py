import os
import cv2
import rosbag
from cv_bridge import CvBridge
import numpy as np

BAG_FILE = '/home/s/dipl/src/hector_quadrotor/controller/results/19_56_12_05_circle_separate.bag'

IMAGE_TOPIC_RGB = '/downward_cam/downward_camera/image'

OUTPUT_DIR_RGB = 'results/19_56_12/base'

if not os.path.exists(OUTPUT_DIR_RGB):
    os.makedirs(OUTPUT_DIR_RGB)

bridge = CvBridge()

with rosbag.Bag(BAG_FILE, 'r') as bag:
    i = 0
    for topic, msg, t in bag.read_messages(topics=[IMAGE_TOPIC_RGB]):
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(os.path.join(OUTPUT_DIR_RGB, f"frame{i}.png"), cv_image)
        i += 1
        print(i)
    

print("Image extraction complete.")