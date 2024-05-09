import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import rospy
import time
import os

from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge, CvBridgeError

Kz = 0.2
Bz = 0.5

Ky = 0.05
By = 0.01

Kx = 0.05
Bx = 0.01

Kx_1 = 0.03
Bx_1 = 0.03

Ky_1 = 0.03
By_1 = 0.03

Kw = 0.1
Bw = 0.0001


des_range = 7


class PathNode:
    def __init__(self, rate=10) -> None:

        rospy.init_node("controller_node")
        self.pile_info = PileInfromation()

        self.pile_info = PileInfromation()

        rospy.Subscriber("/ground_truth/state", Odometry, self.state_callback)  
        #rospy.Subscriber("/asus_camera/rgb/image_raw", Image, self.image_callback_rgb)
        #rospy.Subscriber("/asus_camera/depth/image_raw", Image, self.image_callback_depth)
        rospy.Subscriber("/downward_cam/downward_camera/image", Image, self.follow_border)
        rospy.Subscriber("/downward_cam/downward_camera/image", Image, self.follow_border)
        rospy.Subscriber("/sonar_height", Range, self.read_laser_data_alt)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 
        self.rate = rospy.Rate(rate)
        self.enable_motors()
        self.bridge = CvBridge()

        self.position = Point()
        self.twist = Twist()
        self.current_range = 0.0
        self.omega_error = 0
        self.omega_error_prev = 0
        self.y_error = 0
        self.y_error_prev = 0

        self.dot_position = None
        self.line_offset = True
        self.offset_dist = 0.5

        self.counter = 0
        self.offset_dist = 0.5

        self.counter = 0

    
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

    def follow_border(self, msg):
    def follow_border(self, msg):
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
        pile_height = []
        coordinates = 'coordinates.txt'
        height = 'height.txt'
        time_st = rospy.get_time()
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            elapsed_time = current_time - time_st
            point_index = 0
            
            if elapsed_time < 10:
                uz = Kz * (des_range - self.position.z) - Bz * self.twist.linear.z
                ux = 0
                uy = 0
                uw = 0

            elif elapsed_time >= 10 and elapsed_time  <= 14:
                uz = Kz * (des_range - self.position.z) - Bz * self.twist.linear.z
                ux = 0.5
                uy = 0
                uw = 0
                
            else:
                if self.line_offset:
                    uy = Ky * self.y_error - By * (self.y_error - self.y_error_prev) / (1.0 / 10.0) + self.offset_dist
                else:
                    uy = Ky * self.y_error - By * (self.y_error - self.y_error_prev) / (1.0 / 10.0)
                ux = 1.0
                self.y_error_prev = self.y_error

                uz = Kz * (des_range - self.position.z) - Bz * self.twist.linear.z
                # uz = Kz * (des_range - self.current_range) - Bz * self.twist.linear.z
                uw = Kw * self.omega_error - Bw * (self.omega_error - self.omega_error_prev) / (1.0 / 50.0)
                self.omega_error_prev = self.omega_error
                pile_height.append(self.current_range)
                print(f'Длина pile_height = {len(pile_height)}')
                self.pile_info.contour_info(coordinates=self.dot_position)
                self.pile_info.drone_positions(pose_x=self.position.x,
                                               pose_y=self.position.y,
                                               pose_z=self.position.z)

                if elapsed_time > 30:
                    self.pile_info.define_loop(finish_pose=self.position)
                    if self.pile_info.flag:

                        if not os.path.exists(coordinates):
                            with open(coordinates, 'w') as file:
                                for item in self.pile_info.exploration_trajectory:
                                    file.write(str(item) + '\n')
                            print(f"Список сохранен в файл {coordinates}")

                        uy = Ky * self.y_error - By * (self.y_error - self.y_error_prev) / (1.0 / 10.0)
                        ux = 1.0
                        self.y_error_prev = self.y_error

                        uz = Kz * (des_range - self.position.z) - Bz * self.twist.linear.z
                        # uz = Kz * (des_range - 1 - self.current_range) - Bz * self.twist.linear.z
                        uw = Kw * self.omega_error - Bw * (self.omega_error - self.omega_error_prev) / (1.0 / 50.0)
                        self.omega_error_prev = self.omega_error

                        min_x = min(coord[0] for coord in self.pile_info.exploration_trajectory[:len(self.pile_info.contour_coordinates)])
                        max_x = max(coord[0] for coord in self.pile_info.exploration_trajectory[:len(self.pile_info.contour_coordinates)])
                        min_y = min(coord[1] for coord in self.pile_info.exploration_trajectory[:len(self.pile_info.contour_coordinates)]) 
                        max_y = max(coord[1] for coord in self.pile_info.exploration_trajectory[:len(self.pile_info.contour_coordinates)])
                        print(f'max_x = {max_x}, min_x = {min_x}')
                        print(f'max_y = {max_y}, min_y = {min_y}')


                        # points = [[-5, -7], 
                        #           [-5, 3], 
                        #           [3, 3], 
                        #           [-5, 3],
                        #           [0, 0]]
                        # y_error_prev_p = 0
                        # x_error_prev_p = 0
                        # if self.counter < len(points):
                        #     target_x, target_y = points[self.counter]
                        #     # Calculate errors based on current position and target point
                        #     x_error_p = target_x - self.position.x
                        #     y_error_p = target_y - self.position.y

                        #     # Calculate control inputs based on errors
                        #     ux = Kx_1 * x_error_p - Bx_1 * self.twist.linear.x
                        #     uy = Ky_1 * y_error_p - By_1 * self.twist.linear.y
                        #     print('----------------')
                        #     uz = Kz * (des_range - self.current_range) - Bz * self.twist.linear.z
                        #     uw = 0
                        #     y_error_prev_p = y_error_p
                        #     x_error_prev_p = x_error_p
                        #     print(f'Errors: {x_error_p}, {y_error_p}')

                        #     if np.allclose([x_error_p, y_error_p], [0, 0], rtol=2, atol=2):
                        #         self.counter += 1
                        # else:
                        #     ux = 0
                        #     uy = 0
                        #     uz = Kz * (des_range - self.current_range) - Bz * self.twist.linear.z
                        #     uw = 0
                            


            cmd_msg = Twist() 
            cmd_msg.linear.z = uz
            cmd_msg.linear.x = ux
            cmd_msg.linear.y = uy
            cmd_msg.angular.z = -uw
            self.cmd_pub.publish(cmd_msg)
            self.rate = rospy.sleep(0.1)



class PileInfromation():
    def __init__(self) -> None:
        print(f'Instance of {self.__class__.__name__} was created!')
        self.middle_point = []
        self.flag = False
        self.contour_coordinates = []
        self.exploration_start_point = tuple()
        self.exploration_end_point = list()
        self.pile_height = []
        self.exploration_trajectory = []

    def __str__(self) -> str:
        return f"Storage class for infomation about the pile's coordinates"

    def contour_info(self, coordinates: tuple = None):
        if not self.flag:
            self.contour_coordinates.append(coordinates)
    
    def drone_positions(self, pose_x: float = None, pose_y: float = None, pose_z: float = None):
        coords = tuple([pose_x, pose_y, pose_z])
        self.exploration_trajectory.append(coords)

    # TODO: make start_pose the real start pose, not the first point after time
    def define_loop(self, start_pose: list = None, finish_pose: list = None):
        start_pose = np.array([self.exploration_trajectory[0]], dtype=float)
        finish_pose = np.array([finish_pose.x, finish_pose.y, finish_pose.z], dtype=float)
        if np.allclose(start_pose, finish_pose, rtol=1, atol=1):
            self.flag = True
    
    def follow_exploration_trajectory(self):
        pass


class PileInfromation():
    def __init__(self) -> None:
        print(f'Instance of {self.__class__.__name__} was created!')
        self.middle_point = []
        self.flag = False
        self.contour_coordinates = []
        self.exploration_start_point = tuple()
        self.exploration_end_point = list()
        self.pile_height = []
        self.exploration_trajectory = []

    def __str__(self) -> str:
        return f"Storage class for infomation about the pile's coordinates"

    def contour_info(self, coordinates: tuple = None):
        if not self.flag:
            self.contour_coordinates.append(coordinates)
    
    def drone_positions(self, pose_x: float = None, pose_y: float = None, pose_z: float = None):
        coords = tuple([pose_x, pose_y, pose_z])
        self.exploration_trajectory.append(coords)

    # TODO: make start_pose the real start pose, not the first point after time
    def define_loop(self, start_pose: list = None, finish_pose: list = None):
        start_pose = np.array([self.exploration_trajectory[0]], dtype=float)
        finish_pose = np.array([finish_pose.x, finish_pose.y, finish_pose.z], dtype=float)
        if np.allclose(start_pose, finish_pose, rtol=1, atol=1):
            self.flag = True
    
    def follow_exploration_trajectory(self):
        pass

def main():
    ctrl = PathNode()
    ctrl.spin()


if __name__=='__main__':
    main()