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

Kz = 0.2
Bz = 0.5

Ky = 0.05
By = 0.01

Kw = 0.1
Bw = 0.0001


des_range = 3


start_pos = (25, -30)


class PathNode:
    def __init__(self, rate=10) -> None:

        rospy.init_node("controller_node")
        rospy.Subscriber("/ground_truth/state", Odometry, self.state_callback)  
        #rospy.Subscriber("/asus_camera/rgb/image_raw", Image, self.image_callback_rgb)
        #rospy.Subscriber("/asus_camera/depth/image_raw", Image, self.image_callback_depth)
        rospy.Subscriber("/downward_cam/downward_camera/image", Image, self.define_borders)
        rospy.Subscriber("/sonar_height", Range, self.read_laser_data_alt)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 
        self.rate = rospy.Rate(rate)
        self.enable_motors()

        self.position = Point()
        self.twist = Twist()
        self.current_range = 0.0
        self.omega_error = 0
        self.omega_error_prev = 0
        self.y_error = 0
        self.y_error_prev = 0

        self.dot_position = None
        
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
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Filter contours based on area
            min_area = 100  # Adjust this value based on your requirements
            filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
            
            if filtered_contours:
                largest_contour = max(filtered_contours, key=cv2.contourArea)
                topmost_points = sorted(largest_contour, key=lambda x: x[0][1])[:1]
                avg_x = int(sum(point[0][0] for point in topmost_points) / 1)
                avg_y = int(sum(point[0][1] for point in topmost_points) / 1)
                self.dot_position = (avg_x, avg_y)
                
                # Implement Kalman filter or Median Flow tracking here
                # Update the tracked position based on the Kalman filter prediction
                
                top_points = np.where(thresh[0] >= 0)
                mid_points = np.where(thresh[int(msg.height / 2)] >= 0)
                if (not np.isnan(np.average(top_points)) and not np.isnan(np.average(mid_points))):
                    top_line_point = int(np.average(top_points))
                    mid_line_point = int(np.average(mid_points))
                    self.omega_error = avg_x - mid_line_point
                
                _, cy_list = np.where(thresh >= 0)
                if not np.isnan(np.average(cy_list)):
                    cy = int(np.average(cy_list))
                    self.y_error = msg.width / 2 - cy

                cv2.circle(cv_image, (top_line_point, 10), 5, (0, 0, 255), -1)
                cv2.circle(cv_image, (mid_line_point, int(msg.height/2)), 5, (0, 255, 0), -1)

                cv2.circle(cv_image, self.dot_position, 5, (255, 0, 0), -1)
                cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 3)

                cv2.imshow("Downward Image", cv_image)
                cv2.waitKey(3)
                # cv2.imshow("Grey Image", gray)

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
            if elapsed_time < 5:
                uz = Kz * (des_range - self.position.z) - Bz * self.twist.linear.z
                ux = 0.5
                uy = 0
                uw = 0
                
            else:
                ux = 1
                uy = Ky * self.y_error - By * (self.y_error - self.y_error_prev) / (1.0 / 10.0)
                self.y_error_prev = self.y_error

                uz = Kz * (des_range - self.current_range) - Bz * self.twist.linear.z
                uw = Kw * self.omega_error - Bw * (self.omega_error - self.omega_error_prev) / (1.0 / 50.0)
                self.omega_error_prev = self.omega_error

            cmd_msg = Twist() 
            cmd_msg.linear.z = uz
            cmd_msg.linear.x = ux
            cmd_msg.linear.y = uy
            cmd_msg.angular.z = -uw
            self.cmd_pub.publish(cmd_msg)

            self.rate = rospy.sleep(0.1)


def main():
    ctrl = PathNode()
    ctrl.spin()


if __name__=='__main__':
    main()