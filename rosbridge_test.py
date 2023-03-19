import time
from rosbridge_tcp_client import ROSBridgeClient

class HzMonitor():
  def __init__(self, name):
    self.t = time.time()
    self.fps = 0.0
    self.name = name
    
  def callback(self, msg):
      dt = time.time() - self.t
      self.fps += 1.0 * ( 1.0/dt - self.fps ) * dt
      print(self.name + ': ' + str(int(self.fps)))
      self.t = time.time()
      time.sleep(0.01)


if __name__ == "__main__":
  address = "192.168.0.2"
  port = 9090

  client = ROSBridgeClient(address, port)
  client.connect()
    
  scanMonitor = HzMonitor('/front/scan')
  imuMonitor = HzMonitor('/imu')

  client.subscriber("/front/scan", "sensor_msgs/LaserScan", scanMonitor.callback)
  client.subscriber("/imu/data", "sensor_msgs/Imu", imuMonitor.callback)

  client.publisher("/any/publish_topic", "std_msgs/String", "Hello, ROS!")

  client.spin()
  client.disconnect()
