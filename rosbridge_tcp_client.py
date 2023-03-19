import json
import socket
import signal
import sys
import collections
import threading

class ROSBridgeClient:
  def __init__(self, address, port):
    self.address = address
    self.port = port
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.subscribers = {}
    self.buffer = bytearray()
    self.queues = {}
    self.lock = threading.Lock()

  def connect(self):
    self.sock.connect((self.address, self.port))

  def disconnect(self):
    self.sock.close()

  def send_request(self, request):
    self.sock.sendall(json.dumps(request).encode("utf-8"))
    
  def receive_response(self):
    while True:
      try:
        data = self.sock.recv(4096)
        self.buffer.extend(data)
        
        if b'}{' in self.buffer:
          det_pos = self.buffer.index(b'}{')
          response = json.loads(self.buffer[:det_pos+1].decode('utf-8'))
          self.buffer = self.buffer[det_pos+1:]
        else:
          response = json.loads(self.buffer.decode('utf-8'))
          self.buffer = bytearray()
        break
      except json.JSONDecodeError:
        continue

    return response

  def rostopic_list(self):
    request = {
      "op": "call_service",
      "service": "/rosapi/topics",
      "args": {},
      "id": "get_topics"
    }
    self.send_request(request)
    response = self.receive_response()

    if response["id"] == "get_topics":
      topics = response["values"]["topics"]
      return topics

  def rosnode_list(self):
    request = {
      "op": "call_service",
      "service": "/rosapi/nodes",
      "args": {},
      "id": "get_nodes"
    }
    self.send_request(request)
    response = self.receive_response()

    if response["id"] == "get_nodes":
      nodes = response["values"]["nodes"]
      return nodes

  def subscriber(self, topic, message_type, callback, queue_size=10):
    request = {
      "op": "subscribe",
      "topic": topic,
      "type": message_type,
    }
    self.send_request(request)
    self.subscribers[topic] = callback
    self.queues[topic] = collections.deque([], queue_size)

  def publisher(self, topic, message_type, message):
    request = {
      "op": "advertise",
      "topic": topic,
      "type": message_type,
    }
    self.send_request(request)

    pub_request = {
      "op": "publish",
      "topic": topic,
      "msg": {
        "data": message
      },
    }
    self.send_request(pub_request)

  def receive_loop(self):
    try:
      while True:
        response = self.receive_response()
        if response["op"] == "publish" and response["topic"] in self.subscribers:
          with self.lock:
            self.queues[response["topic"]].append(response["msg"])
    except KeyboardInterrupt:
      print('receive_loop stop')

  def spin(self):
    receive_thread = threading.Thread(target=self.receive_loop)
    receive_thread.start()
    
    try:
      while True:
        for key in self.queues.keys():
          msg = None
          with self.lock:
            que = self.queues[key]
            if len(que) > 0:
              msg = self.queues[key].popleft()
          if msg:
            self.subscribers[key](msg)
    except KeyboardInterrupt:
      receive_thread.join()
      print('spin stop')

if __name__ == "__main__":
  address = "192.168.0.2"
  port = 9090

  client = ROSBridgeClient(address, port)
  client.connect()

  nodes = client.rosnode_list()
  print("ノードリスト:")
  for node in nodes:
    print(node)

  print("")

  topics = client.rostopic_list()
  print("トピックリスト:")
  for topic in topics:
    print(topic)
  
  client.disconnect()
