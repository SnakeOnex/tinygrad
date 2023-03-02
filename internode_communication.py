import zmq
import pickle
from enum import IntEnum, auto

from config import tcp_config


def create_publisher_socket(port):
    """Creates a publisher socket

    Args:
        port (str or int): TCP port number

    Returns:
        zmq.Context().socket(): TCP publisher socket
    """
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(tcp_config["TCP_HOST"] + f":{port}")
    return socket


def create_subscriber_socket(port):
    """Creates a subscriber socket

    Args:
        port (str or int): TCP port number

    Returns:
        zmq.Context().socket(): TCP subscriber socket
    """
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(tcp_config["TCP_HOST"] + f":{port}")
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    return socket


def publish_data(socket, data):
    """Publish data to a publisher-type socket

    Args:
        socket (zmq.Context().socket()): socket to sent from
        data (any): data to be sent
    """
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
