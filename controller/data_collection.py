import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from hector_uav_msgs.srv import EnableMotors
from hector_uav_msgs.msg import Altimeter
from sensor_msgs.msg import PointCloud2, Image, Range
import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from statistics import mean


Kz = 0.2
Bz = 0.5

Kx = 0.1
Bx = 0.3

Ky = 0.1
By = 0.3


des_range = 3


start_pos = (25, -30)


class PathNode:
    def __init__(self, rate=10) -> None:

        rospy.init_node("controller_node")
        rospy.Subscriber("/ground_truth/state", Odometry, self.state_callback)  
        rospy.Subscriber("/asus_camera/color/image_raw", Image, self.image_callback_rgb)
        #rospy.Subscriber("/asus_camera/depth/image_raw", Image, self.image_callback_depth)
        rospy.Subscriber("/downward_cam/downward_camera/image", Image, self.define_borders)
        rospy.Subscriber("/sonar_height", Range, self.read_laser_data_alt)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 
        self.rate = rospy.Rate(rate)
        self.enable_motors()

        self.position = Point()
        self.twist = Twist()
        self.current_range = 0.0

        self.bridge = CvBridge()
    
    @staticmethod
    def enable_motors():
        try:
            rospy.wait_for_service('/enable_motors')
            call_em = rospy.ServiceProxy('/enable_motors', EnableMotors)
            resp1 = call_em(True)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    def image_callback_rgb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("RGB window", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)

    def image_callback_depth(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            cv_image_normalized = np.uint8(cv_image_normalized)
            colored_depth_image = cv2.applyColorMap(cv_image_normalized, cv2.COLORMAP_WINTER)
            cv2.imshow("Depth Image", colored_depth_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

    def define_borders(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                epsilon = 0.01 * cv2.arcLength(largest_contour, True)
                approx = cv2.approxPolyDP(largest_contour, epsilon, True)
                distances = [cv2.norm([self.position.x, self.position.y] - point[0]) for point in approx]
                closest_indices = np.argsort(distances)[:3]
                
                closest_points = [tuple(approx[index][0]) for index in closest_indices]
                
                offset_points = list(closest_points)
                mean_first_elements = mean(x[0] for x in offset_points)
                mean_second_elements = mean(x[1] for x in offset_points)
                moved_points = [(x[0] + 0.1 * mean_first_elements, x[1] - 2 * mean_second_elements) for x in offset_points]

                for point in closest_points:
                    cv2.circle(cv_image, point, 5, (0, 0, 255), -1)
                
                for pt in moved_points:
                    int_point = np.round(pt).astype("int")
                    cv2.circle(cv_image, tuple(int_point), 5, (255, 255, 0), -1)
                                 
                cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 3)
            
        except CvBridgeError as e:
            print(e)
        
        finally:
            cv2.imshow("Downward Image", cv_image)
            cv2.waitKey(3)
            #cv2.imshow("Grey Image", gray)

    def read_laser_data_alt(self,msg):
        self.current_range = msg.range

    def state_callback(self, msg):
        self.position = msg.pose.pose.position
        self.twist = msg.twist.twist

    def spin(self):
        time_st = rospy.get_time()
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            elapsed_time = current_time - time_st
            if elapsed_time < 100:
                uz = Kz * (des_range - self.position.z) - Bz * self.twist.linear.z
                ux = 0.2
                uy = 0
                
            elif elapsed_time > 100:

                ux = Kx * (10 - self.position.x) - Bx * self.twist.linear.x
                uy = Ky * (10 - self.position.y) - By * self.twist.linear.y
                uz = Kz * (des_range - self.current_range) - Bz * self.twist.linear.z

                print(f'X= {self.position.x}, Y= {self.position.y}, Z= {self.current_range}')
                print(f'DES_X= {10}, DES_Y= {10}, DES_Z = {des_range}')

            cmd_msg = Twist() 
            cmd_msg.linear.z = uz
            cmd_msg.linear.x = ux
            cmd_msg.linear.y = uy
            self.cmd_pub.publish(cmd_msg)

            self.rate = rospy.sleep(0.1)


def main():
    ctrl = PathNode()
    ctrl.spin()


if __name__=='__main__':
    main()