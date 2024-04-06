import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from hector_uav_msgs.srv import EnableMotors
from hector_uav_msgs.msg import Altimeter
from sensor_msgs.msg import PointCloud2, Image, Range
import cv2
from cv_bridge import CvBridge, CvBridgeError

x_des = 10
y_des = 10
Kz = 0.5
Bz = 0.5


Kx = 0.1
Bx = 0.1

Ky = 0.1
By = 0.1

des_range = 5


class Controller:
    def __init__(self) -> None:
        rospy.init_node("controller_node")
        self.enable_motors()
        self.current_range = 0.0
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  
        rospy.Subscriber("/ground_truth/state", Odometry, self.state_callback)  
        self.position = Point()
        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.bridge = CvBridge()
        #rospy.Subscriber("/camera/depth/points", PointCloud2, self.read_laser_data)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        rospy.Subscriber("/sonar_height", Range, self.read_laser_data_alt)

    def enable_motors(self):
        try:
            rospy.wait_for_service('/enable_motors')
            call_em = rospy.ServiceProxy('/enable_motors', EnableMotors)
            resp1 = call_em(True)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def state_callback(self, msg):
        self.position = msg.pose.pose.position
        self.twist = msg.twist.twist
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # _, mask = cv2.threshold(grey_image, 8, 255, cv2.THRESH_BINARY_INV)
        # cv_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        cv2.imshow("Image window2", cv_image)
        cv2.waitKey(3)

    # def read_laser_data(self,msg):
    #     self.d = msg
    #     return self.a

    def read_laser_data_alt(self,msg):
        self.current_range = msg.range

    def spin(self):
        time_st = rospy.get_time()
        flag = 0
        time_man = 0
        while not rospy.is_shutdown():

            if rospy.get_time() - time_st < 10.0:
                uz = Kz * (des_range - self.current_range) - Bz * self.twist.linear.z
                ux = 0
                uy = 0
                az = 0
            else:
                uz = Kz * (des_range - self.current_range) - Bz * self.twist.linear.z
                ux = Kx * (x_des - self.position.x) - Bx * self.twist.linear.x
                uy = Ky * (y_des - self.position.y) - By * self.twist.linear.y

            cmd_msg = Twist() 
            cmd_msg.linear.z = uz
            cmd_msg.linear.x = ux
            cmd_msg.linear.y = uy
            cmd_msg.angular.z = az
            self.cmd_pub.publish(cmd_msg)


def main():
    ctrl = Controller()
    ctrl.spin()


if __name__=='__main__':
    main()