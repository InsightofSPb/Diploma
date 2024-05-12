import cv2
import numpy as np
import matplotlib.pyplot as plt
import rospy
import time
import os
from math import cos, sin, radians
from datetime import datetime

from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion

Kz = 0.2
Bz = 0.5

Ky = 0.05
By = 0.01

Kx = 0.05
Bx = 0.01

Kx_1 = 0.1
Bx_1 = 0.03

Ky_1 = 0.1
By_1 = 0.1

Kw = 0.1
Bw = 0.0001


des_range = 7


class PathNode:
    def __init__(self, rate=10) -> None:

        rospy.init_node("controller_node")
        self.pile_info = PileInformation()

        rospy.Subscriber("/ground_truth/state", Odometry, self.state_callback)  
        #rospy.Subscriber("/asus_camera/rgb/image_raw", Image, self.image_callback_rgb)
        #rospy.Subscriber("/asus_camera/depth/image_raw", Image, self.image_callback_depth)
        rospy.Subscriber("/downward_cam/downward_camera/image", Image, self.follow_border)
        rospy.Subscriber("/sonar_height", Range, self.read_laser_data_alt)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 
        self.rate = rospy.Rate(rate)
        self.enable_motors()
        self.bridge = CvBridge()

        self.position = Point()
        self.twist = Twist()
        self.orientation = Point()
        self.current_range = 0.0
        self.omega_error = 0
        self.omega_error_prev = 0
        self.y_error = 0
        self.y_error_prev = 0

        self.dot_position = None
        self.line_offset = True
        self.offset_dist =  0.0

        self.counter = 0

        self.omega_error_circle = 0
        self.omega_error_circle_prev = 0

    
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

    def calculate_pid(self, setpoint, actual, error_sum, last_error, Kp, Ki, Kd, dt):
        error = setpoint - actual
        error_sum += error * dt
        d_error = (error - last_error) / dt
        last_error = error
        output = Kp * error + Ki * error_sum + Kd * d_error
        return output, error_sum, last_error


    def spin(self):
        pile_height = []
        current_time_sys = datetime.now().strftime("%H_%M_%d_%m")
        coordinates = f'contour_{current_time_sys}.txt'
        trajectory = f'trajectory_with_range_{current_time_sys}.txt'
        time_st = rospy.get_time()
        scale = 1
        final = False
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            elapsed_time = current_time - time_st

            self.pile_info.drone_positions_full(pose_x=self.position.x,
                                    pose_y=self.position.y,
                                    pose_z=self.position.z,
                                    z_range=self.current_range,
                                    time=elapsed_time)
            
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
                    uy = Ky * self.y_error - By * (self.y_error - self.y_error_prev) / (1.0 / 10.0)
                else:
                    uy = Ky * self.y_error - By * (self.y_error - self.y_error_prev) / (1.0 / 10.0)
                ux = 1.0
                self.y_error_prev = self.y_error

                uz = Kz * (des_range - self.position.z) - Bz * self.twist.linear.z
                uw = Kw * self.omega_error - Bw * (self.omega_error - self.omega_error_prev) / (1.0 / 50.0)
                self.omega_error_prev = self.omega_error

                pile_height.append(self.current_range)

                self.pile_info.contour_info(coordinates=self.dot_position)
                self.pile_info.drone_positions(pose_x=self.position.x,
                                               pose_y=self.position.y,
                                               pose_z=self.position.z)
                
                if elapsed_time > 60:
                    self.pile_info.define_loop(finish_pose=self.position)
                    if self.pile_info.flag:

                        if not os.path.exists(coordinates):
                            with open(coordinates, 'w') as file:
                                for item in self.pile_info.exploration_trajectory_full:
                                    file.write(str(item) + '\n')
                            print(f"Список сохранен в файл {coordinates}")

                        min_x = min(coord[0] for coord in self.pile_info.exploration_trajectory[:len(self.pile_info.contour_coordinates)])
                        max_x = max(coord[0] for coord in self.pile_info.exploration_trajectory[:len(self.pile_info.contour_coordinates)])
                        min_y = min(coord[1] for coord in self.pile_info.exploration_trajectory[:len(self.pile_info.contour_coordinates)]) 
                        max_y = max(coord[1] for coord in self.pile_info.exploration_trajectory[:len(self.pile_info.contour_coordinates)])

                        if not self.pile_info.circle_flag_1:
                            circle_information = self.pile_info.find_inscribed_circle(min_x=min_x,
                                                                                    min_y=min_y,
                                                                                    max_y=max_y,
                                                                                    max_x=max_x,
                                                                                    radius_step=0.2 * scale)
                            print(f'circle information = {circle_information}')
                        if not self.pile_info.circle_flag_2:
                            circle_trajectory = self.pile_info.generate_circle_points(center_x=circle_information[0],
                                                                                    center_y=circle_information[1],
                                                                                    radius=circle_information[2],
                                                                                    step_angle=30)
                            print(f'circle trajectory = {circle_trajectory}')
                        
                        if self.pile_info.circle_flag_2:
                            points = circle_trajectory
                            counter_visited = 0
                            for point in points:

                                x = point[0]
                                y = point[1]

                                while not np.allclose([x, y], [self.position.x, self.position.y], rtol=0.08, atol=0.15):
                                    ux = Kx_1 * (x - self.position.x) - Bx_1 * self.twist.linear.x
                                    uy = Ky_1 * (y - self.position.y) - By_1 * self.twist.linear.y
                                    uz = Kz * (des_range - self.position.z) - Bz * self.twist.linear.z

                                    print(f'x = {x}, y = {y}')
                                    print(f'x_act = {self.position.x}, y_act = {self.position.y}')

                                    cmd_msg = Twist() 
                                    cmd_msg.linear.z = uz
                                    cmd_msg.linear.x = -ux
                                    cmd_msg.linear.y = -uy
                                    cmd_msg.angular.z = 0
                                    self.cmd_pub.publish(cmd_msg)
                                    self.rate = rospy.sleep(0.1)

                                    current_time = rospy.get_time()
                                    elapsed_time = current_time - time_st
                                    self.pile_info.drone_positions_full(pose_x=self.position.x,
                                                                        pose_y=self.position.y,
                                                                        pose_z=self.position.z,
                                                                        z_range=self.current_range,
                                                                        time=elapsed_time)
                                counter_visited += 1
                                print(f'{counter_visited} points out of {len(points)} visited')

                                if counter_visited == len(points):
                                    print("All points visited, going to the next circle!")
                                    self.pile_info.circle_flag_1 = False
                                    self.pile_info.circle_flag_2 = False
                                    scale += 1
                                    if scale != 3:
                                        break
                                    else:
                                        final = True

                            if final:
                                print("All points visited, going to the middle!")
                                x_fin, y_fin = circle_information[0], circle_information[1]
                                while not np.allclose([x_fin, y_fin], [self.position.x, self.position.y], rtol=0.05, atol=0.1):
                                    ux = Kx_1 * (x_fin - self.position.x) - Bx_1 * self.twist.linear.x
                                    uy = Ky_1 * (y_fin - self.position.y) - By_1 * self.twist.linear.y
                                    uz = Kz * (des_range - self.position.z) - Bz * self.twist.linear.z
                                    print('Going to the middle')

                                    cmd_msg = Twist() 
                                    cmd_msg.linear.z = uz
                                    cmd_msg.linear.x = -ux
                                    cmd_msg.linear.y = -uy
                                    cmd_msg.angular.z = 0
                                    self.cmd_pub.publish(cmd_msg)
                                    self.rate = rospy.sleep(0.1)

                                    current_time = rospy.get_time()
                                    elapsed_time = current_time - time_st
                                    self.pile_info.drone_positions_full(pose_x=self.position.x,
                                                                        pose_y=self.position.y,
                                                                        pose_z=self.position.z,
                                                                        z_range=self.current_range,
                                                                        time=elapsed_time)

                                    if np.allclose([x_fin, y_fin], [self.position.x, self.position.y], rtol=0.05, atol=0.1):
                                        finish = True
                                        if not os.path.exists(trajectory):
                                            with open(trajectory, 'w') as file:
                                                for item in self.pile_info.exploration_trajectory_full:
                                                    file.write(str(item) + '\n')
                                            print(f"Список сохранен в файл {trajectory}")                            

            cmd_msg = Twist() 
            cmd_msg.linear.z = uz
            cmd_msg.linear.x = ux
            cmd_msg.linear.y = uy
            cmd_msg.angular.z = -uw
            self.cmd_pub.publish(cmd_msg)
            self.rate = rospy.sleep(0.1)



class PileInformation():
    def __init__(self) -> None:
        print(f'Instance of {self.__class__.__name__} was created!')
        self.middle_point = []
        self.flag = False
        self.contour_coordinates = []
        self.exploration_start_point = tuple()
        self.exploration_end_point = list()
        self.pile_height = []
        self.exploration_trajectory = []
        self.exploration_trajectory_full = []
        self.circle_flag_1 = False
        self.circle_flag_2 = False

    def __str__(self) -> str:
        return f"Storage class for infomation about the pile's coordinates"

    def contour_info(self, coordinates: tuple = None):
        if not self.flag:
            self.contour_coordinates.append(coordinates)
    
    def drone_positions(self, pose_x: float = None, pose_y: float = None, pose_z: float = None):
        coords = tuple([pose_x, pose_y, pose_z])
        self.exploration_trajectory.append(coords)

    def drone_positions_full(self, pose_x: float = None, pose_y: float = None, pose_z: float = None, 
                             z_range: float = None, time: float = None):
        coords = tuple([pose_x, pose_y, pose_z, z_range, time])
        self.exploration_trajectory_full.append(coords)

    # TODO: make start_pose the real start pose, not the first point after time
    def define_loop(self, start_pose: list = None, finish_pose: list = None):
        start_pose = np.array([self.exploration_trajectory[0]], dtype=float)
        finish_pose = np.array([finish_pose.x, finish_pose.y, finish_pose.z], dtype=float)
        if np.allclose(start_pose, finish_pose, rtol=0.25, atol=0.2):
            self.flag = True
        
    def find_inscribed_circle(self, min_x, min_y, max_x, max_y, radius_step = 0.3):
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        
        width = abs(max_x - min_x)
        height = abs(max_y - min_y)
        radius = min(width, height) / 2 - min(width, height) * radius_step
        self.circle_flag_1 = True
                
        return center_x, center_y, radius
    
    def generate_circle_points(self, center_x, center_y, radius, step_angle=10):
        points = []
        angle = 250
        while angle < 605:
            x = center_x + radius * cos(radians(angle))
            y = center_y + radius * sin(radians(angle))
            points.append((x, y))
            angle += step_angle

            if angle >= 605:
                self.circle_flag_2 = True

        return points

def main():
    ctrl = PathNode()
    ctrl.spin()


if __name__=='__main__':
    main()