import rospy
# из ROS топиков импортируем необходимые типы сообщений
from nav_msgs.msg import Odometry  # тип сообщения
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors  # служба "Включить моторы" 

cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # необходимо в топик cmd_vel опубликовать сообщение, которое реализуется в функции ниже
K = 1.0  # задажим пропорциональный и дифференциальный коэффициенты ПД и желаемую высоту
B = 2.0
z_des = 3.0


def enable_motors():
    rospy.wait_for_service('/enable_motors')  # ожидаем появления сервиса
    try:
        call_em = rospy.ServiceProxy('/enable_motors', EnableMotors)  # создали клиента на этот сервис
        resp1 = call_em(True)  # в сервис нужно отправить 1 для запуска
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def callback(msg):

    z = msg.pose.pose.position.z  # из получаемого сообщения извлекаем величину координаты z  и скорость вдоль оси z (линейную)
    dotz = msg.twist.twist.linear.z

    uz = K * (z_des - z) - B * dotz  # зададим управляющий закон - ПД-регулятор
    

   
    cmd_msg = Twist()  # создаём пустое сообщение, в которое
    cmd_msg.linear.z = uz  # в поле линейной скорости по z будет подавать наш управляющий сигнал


    cmd_pub.publish(cmd_msg)

def main():
    rospy.init_node("test_takeoff_node")  # обозначаем, что создали новую ноду и даём ей название. Нужно, чтобы к системе подключить новую ноду и производить управление
    if enable_motors():  # проверяем, работают ли моторы, чтобы дрон мог взлететь
        print("Motors started!")
    else:
        print("Motors didn't start")    

    rospy.Subscriber("/ground_truth/state", Odometry, callback)  # необходимо реализовать подписку на топик ros`а. В первом поле - название топика, во втором - тип сообщения, в третьем - функция, которая будет что-то делать с данными

    rospy.spin()  # в ros master передаём информацию о том, что нода работает и запускается бесконечный цикл выполнения вплоть до получения останавливающего сигнала
    

if __name__ =="__main__":
    main()