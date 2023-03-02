import multiprocessing.connection as connection
import socket
import select
import struct

def connect_client(address, port):
    remote_address = address, port
    listener = connection.Listener(remote_address)
    conn = listener.accept()
    return conn


def bind_udp_socket(host, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((host, port))

    poller = select.poll()
    poller.register(s, select.POLLIN)

    return s, poller

def get_socket_obj():
	return socket(socket.AF_INET, SOCK_DGRAM)