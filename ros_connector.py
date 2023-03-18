import os
import socket
import rospy

class ROSConnector:
    def __init__(self, master_uri='http://192.168.0.2:11311', node_name='test_node'):
        self.master_uri = master_uri
        self.node_name = node_name
        self.local_ip = self._get_localip()

        os.environ['ROS_MASTER_URI'] = self.master_uri
        os.environ['ROS_IP'] = self.local_ip
        os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = ''

        rospy.init_node(self.node_name, disable_signals=True)

    def _get_localip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('www.google.com', 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip

    def spin(self):
        rospy.spin()


