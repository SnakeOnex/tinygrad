import zmq
from enum import IntEnum, auto

from config import tcp_config

def create_publisher_socket(port):
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(tcp_config["TCP_HOST"] + f":{port.value}")
    return socket

def create_subscriber_socket(port):
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(tcp_config["TCP_HOST"] + f":{port.value}")
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    return socket

def get_last_subscription_data(socket):
    data = None
    while socket.poll(0.):
        data = socket.recv()
    return data

class NodePorts(IntEnum):
    CAN1 = 10000
    CAN2 = 20000

class CAN1NodeMsgPorts(IntEnum):
    WHEEL_SPEED = NodePorts.CAN1
    MISSION = auto()
    START_BUTTON = auto()

class CAN2NodeMsgPorts(IntEnum):
    GO_SIGNAL = NodePorts.CAN2