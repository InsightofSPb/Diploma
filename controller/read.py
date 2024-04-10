import os
import cv2
import rosbag
from cv_bridge import CvBridge
import numpy as np

# Specify the path to your ROS bag file
BAG_FILE = '/home/s/dipl/src/hector_quadrotor/controller/solid_small35.bag'

# Specify the image topics you want to extract
IMAGE_TOPIC_RGB = '/asus_camera/rgb/image_raw'
IMAGE_TOPIC_DEPTH = '/asus_camera/depth/image_raw'

# Specify the output directories for the extracted images
OUTPUT_DIR_RGB = 'controller/solid/small/image'
OUTPUT_DIR_DEPTH = 'controller/solid/small/depth'

# Create the output directories if they don't exist
if not os.path.exists(OUTPUT_DIR_RGB):
    os.makedirs(OUTPUT_DIR_RGB)
if not os.path.exists(OUTPUT_DIR_DEPTH):
    os.makedirs(OUTPUT_DIR_DEPTH)

# Initialize the CvBridge class
bridge = CvBridge()

# Open the ROS bag file
with rosbag.Bag(BAG_FILE, 'r') as bag:
    i = 0
    for topic, msg, t in bag.read_messages(topics=[IMAGE_TOPIC_RGB]):
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(os.path.join(OUTPUT_DIR_RGB, f"frame{i}.png"), cv_image)
        i += 1
    
    # Extract and save Depth images
    j = 0
    for topic, msg, t in bag.read_messages(topics=[IMAGE_TOPIC_DEPTH]):
        try:
            # Attempt to convert depth images to OpenCV format
            cv_image = bridge.imgmsg_to_cv2(msg, "passthrough")
            # Normalize the depth image for visualization
            cv_image_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
            cv_image_normalized = np.uint8(cv_image_normalized)
            colored_depth_image = cv2.applyColorMap(cv_image_normalized, cv2.COLORMAP_WINTER)
            cv2.imwrite(os.path.join(OUTPUT_DIR_DEPTH, f"frame{j}.png"), colored_depth_image)
            j += 1
        except Exception as e:
            print(f"Error processing depth image: {e}")

print("Image extraction complete.")