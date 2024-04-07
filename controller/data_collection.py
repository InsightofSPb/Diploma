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

Kz = 0.4
Bz = 0.5

Kx = 0.1
Bx = 0.3

Ky = 0.1
By = 0.3



des_range = 2.5


start_pos = (-10, -10, des_range)
point_list = [start_pos, 
              (0, 0, des_range) ,
              (5, 3, des_range) ,
              (10, 0, des_range),
              (15, 3, des_range) , 
              (10, 6, des_range),
              (10, 15, des_range),
              (12, 10, des_range), 
              (6, 6, des_range), 
              (3, 3, des_range), 
              start_pos]

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
            cv2.imshow("Downward Image", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)


    def read_laser_data_alt(self,msg):
        self.current_range = msg.range

    def state_callback(self, msg):
        self.position = msg.pose.pose.position
        self.twist = msg.twist.twist

    def spin(self):
        point_list_real = []
        while not rospy.is_shutdown():
            for point in point_list:
                x_des, y_des = point[0], point[1]
                while not np.allclose([self.position.x, self.position.y, self.current_range], [x_des, y_des, des_range], rtol=0.2, atol=0.2):
                    ux = Kx * (x_des - self.position.x) - Bx * self.twist.linear.x
                    uy = Ky * (y_des - self.position.y) - By * self.twist.linear.y
                    uz = Kz * (des_range - self.current_range) - Bz * self.twist.linear.z

                    print(f'X= {self.position.x}, Y= {self.position.y}, Z= {self.current_range}')
                    print(f'DES_X= {x_des}, DES_Y= {y_des}, DES_Z = {des_range}')

                    cmd_msg = Twist() 
                    cmd_msg.linear.z = uz
                    cmd_msg.linear.x = ux
                    cmd_msg.linear.y = uy
                    self.cmd_pub.publish(cmd_msg)

                    self.rate = rospy.sleep(0.1)
                point_list_real.append((self.position.x, self.position.y))
                    

                if len(point_list) == len(point_list_real):
                    print(point_list_real)
                    #plot_trajectory(points_des=point_list, points_real=point_list_real)


def plot_trajectory(points_des = None, points_real = None):
    plt.figure(figsize=(20, 20)) 

    des_x = [coord[0] for coord in points_des]
    des_y = [coord[1] for coord in points_des]

    real_x = [coord[0] + 1 for coord in points_real]
    real_y = [coord[1] + 1 for coord in points_real]
    print(des_x, real_x)
    print(des_y, real_y)

    plt.plot(real_x, real_y, marker='x', label='Real Trajectory')
    plt.plot(des_x, des_y, marker='o', label='Desired Trajectory')

    plt.grid()
    plt.xlabel('X', fontsize=14)
    plt.ylabel('Y', fontsize=14)
    plt.title('Trajectory Plot', fontsize=16)
    plt.legend(fontsize=12) 

    plt.xticks(fontsize=8)
    plt.yticks(fontsize=8)

    ax = plt.gca()
    ax.set_xticks(np.arange(min(des_x + real_x), max(des_x + real_x) + 1, 1))

    plt.savefig('src/hector_quadrotor/controller/trajectory.png')
    #plt.show()

def main():
    ctrl = PathNode()
    ctrl.spin()


if __name__=='__main__':
    main()