import zmq
import pickle
from enum import IntEnum, auto

from config import tcp_config


def create_publisher_socket(port):
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(tcp_config["TCP_HOST"] + f":{port}")
    return socket


def create_subscriber_socket(port):
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(tcp_config["TCP_HOST"] + f":{port}")
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    return socket


def publish_data(socket, data):
    socket.send(pickle.dumps(data))


def get_last_subscription_data(socket):
    data = None
    while socket.poll(0.):
        data = socket.recv()

    if data is not None:
        data = pickle.loads(data)

    return data


def update_subscription_data(socket, old_data):
    data = get_last_subscription_data(socket)
    if data is None:
        data = old_data

    return data
