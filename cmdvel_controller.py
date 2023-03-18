import rospy
from ros_connector import ROSConnector
from geometry_msgs.msg import Twist

class CmdVelController(ROSConnector):
    def __init__(self, master_uri='http://192.168.0.2:11311', node_name='cmd_vel_controller'):
        super().__init__(master_uri, node_name)
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def send_velocity_command(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        self.publisher.publish(twist)
        self.rate.sleep()

    def stop(self):
        self.send_velocity_command(0, 0)

if __name__ == '__main__':
    controller = CmdVelController()

    try:
        while not rospy.is_shutdown():
            # 以下の例では、前進速度 0.5 m/s、回転速度 0.5 rad/s で cmd_vel トピックを発行します。
            # 実際の使用時には、適切な速度コマンドに置き換えてください。
            controller.send_velocity_command(0.5, 0.5)
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.stop()
