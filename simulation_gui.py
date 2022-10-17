from simulation import KEYBOARDS_PORT
from ursina import *
from ursina.shaders import lit_with_shadows_shader

import multiprocessing.connection as connection
from multiprocessing import shared_memory
import socket
import struct
import select

from enum import Enum
import sys
import numpy as np
import random
import socket
import pickle
import argparse

from objects.formula import Formula
from objects.cone import Cone
from state import State

from network_helpers import connect_client, bind_udp_socket
from math_helpers import angle_to_vector, vec_to_3d, rotate_around_point, local_to_global

HOST = '127.0.0.1'
VISUAL_PORT = 1337
CONTROLS_PORT = 1338
KEYBOARDS_PORT = 1339


class CameraMode(Enum):
    WORLD = 0
    FIRST_PERSON = 1
    THIRD_PERSON = 2

    def next(self):
        v = self.value
        return CameraMode((v + 1) % 3)


def world_setup():
    ground = Entity(
        model='cube',
        color=color.gray,
        texture='models/asp.jpg',
        texture_scale=(4, 4),
        position=(0, 0, 0),
        scale=(200, 0, 500),
        collider='box',
        shader=lit_with_shadows_shader
    )
    skybox = load_texture('models/sky.jpg')
    sky = Sky(
        texture=skybox,
        color=color.cyan
    )

    pivot = Entity()
    DirectionalLight(parent=pivot, position=(2., 10., 2.),
                     shadows=True, rotation=(90., 0., 0.))


def render_path(path_list, path_entity):
    path = np.array([[app.path_mem[i] for i in range(0, len(app.path_mem)//2)],
                    [app.path_mem[i] for i in range(len(app.path_mem)//2, len(app.path_mem))]]).T

    path[:, 0] *= -1
    path = local_to_global(path, state.car_pos, state.heading)
    path = [vec_to_3d(p, y=0.01) for p in path]

    # print("path: ", path)
    # path_entity = Entity(shader=lit_with_shadows_shader,color=color.red,model=Mesh(vertices=[[0., 0., 0.], [0., 0., 0.]], mode='line', thickness=5,colors=[color.red, color.red]))
    path_entity.model.vertices = path
    return path


def render_cones(state):
    [Cone(model='models/yellow_cone.obj', position=(c[0], 0.01, c[1]))
     for c in state.yellow_cones]
    [Cone(model='models/blue_cone.obj', position=(c[0], 0.01, c[1]))
     for c in state.blue_cones]
    [Cone(model='models/orange_cone.obj', position=(c[0], 0.01, c[1]))
     for c in state.orange_cones]
    [Cone(model='models/big_orange_cone.obj', position=(c[0], 0.01, c[1]))
     for c in state.big_cones]


def render_car(state, formula, driver):

    car_x = state[0]
    car_y = state[1]
    heading = state[2]
    steering_angle = state[3]

    heading_vec = angle_to_vector(heading)

    driver.position = Vec3(car_x, 0., car_y) - 1. * \
        vec_to_3d(heading_vec) + Vec3(0., 0.7, 0.)
    driver.rotation = formula.offset_rot + Vec3(0., 0., heading)

    formula.position = Vec3(car_x, 0., car_y)
    formula.rotation = formula.offset_rot + Vec3(0., 0, heading)

    formula.fl_wheel.rotation = (0., 0., steering_angle)
    formula.fr_wheel.rotation = (0., 0., steering_angle)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='maps/circle_map.json')
    parser.add_argument('--tcp', action='store_true')
    args = parser.parse_args()

    app = Ursina()

    # config
    communication = "udp"  # "udp", "shared_mem"
    camera.fov = 78
    window.title = "VirtualMilovice"
    window.size = (1280, 720)
    window.borderless = False
    window.fps_counter.enabled = True
    app.AS = False

    if random.random() < 0.01:
        Audio(sound_file_name='models/terrymusic.mp3', autoplay=True, loop=True)

    # 1. SETUP WORLD
    world_setup()

    # 2. SETUP STATE
    state = State(args.map)

    controls_addr = (HOST, CONTROLS_PORT)
    controls_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    app.controls_state = [0, 0, 0, 0]  # go_signal

    # keyboard
    keyboard_addr = (HOST, KEYBOARDS_PORT)
    keyboard_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    app.keyboard_state = [0, 0, 0, 0]

    if communication == "udp":
        visual_socket, visual_poller = bind_udp_socket(HOST, VISUAL_PORT)

        data = visual_socket.recvfrom(16)
        app.visual_state = struct.unpack('<4f', data[0])

    render_cones(state)
    formula = Formula()
    driver = Entity(model='sphere', scale=0.2)
    text_main = Text()
    Text.size = 0.05
    app.text_AS = Text(text="AS: OFF", origin=(3.5, -5.), color=color.red)
    app.cam_mode = CameraMode.WORLD

    app.update_count = 0
    app.frame_start = time.perf_counter()

    def input(key):
        # change camera mode
        if key == 'p':
            app.cam_mode = app.cam_mode.next()
            print(f"CAMERA MODE: {app.cam_mode}")

        if key == 'r':
            state.reset_state()

        if key == 'e':
            app.AS = True
            app.text_AS.color = color.lime
            app.text_AS.text = "AS: ON"

        if key == 'g':
            app.controls_state[0] = 1
            data = struct.pack('<4i', *app.controls_state)
            controls_socket.sendto(data, (HOST, CONTROLS_PORT))

    def update():
        while True and communication == "udp":
            app.evts = visual_poller.poll(0.)
            if len(app.evts) == 0:
                break
            sock, evt = app.evts[0]
            if evt:
                data = visual_socket.recvfrom(16)
                app.visual_state = struct.unpack('<4f', data[0])
                app.update_count += 1

        app.keyboard_state = [0, 0, 0, 0]
        # key handling
        if held_keys['w'] and held_keys['space']:
            pass
        elif held_keys['w']:
            # print("w is pressed")
            app.keyboard_state[0] = 1
            data = struct.pack('<4i', *app.keyboard_state)
            keyboard_socket.sendto(data, (HOST, KEYBOARDS_PORT))

        elif held_keys['space']:
            app.keyboard_state[1] = 1
            data = struct.pack('<4i', *app.keyboard_state)
            keyboard_socket.sendto(data, (HOST, KEYBOARDS_PORT))
        if held_keys['a'] and held_keys['d']:
            pass
        elif held_keys['a']:
            app.keyboard_state[2] = 1
            data = struct.pack('<4i', *app.keyboard_state)
            keyboard_socket.sendto(data, (HOST, KEYBOARDS_PORT))
        elif held_keys['d']:
            app.keyboard_state[3] = 1
            data = struct.pack('<4i', *app.keyboard_state)
            keyboard_socket.sendto(data, (HOST, KEYBOARDS_PORT))

        render_car(app.visual_state, formula, driver)
        # state.update_state(time.dt)
        # text = f"Speed: {state.speed:.2f}\nSteering angle: {state.steering_angle:.2f}\nHeading: {state.heading:.2f}\n"

        # text_main.text = text

        # update camera
        if app.cam_mode == CameraMode.WORLD:
            camera.position = Vec3(0, 15, -20)
            camera.look_at(formula)
        elif app.cam_mode == CameraMode.FIRST_PERSON:
            camera.position = driver.position
            # camera.rotation = (0.,-state.heading,0.)
            camera.rotation = (0., -app.visual_state[2], 0.)
        elif app.cam_mode == CameraMode.THIRD_PERSON:
            rot_x, rot_y = rotate_around_point(
                -app.visual_state[2], (0, 0), (10, 0))
            camera.position = driver.position + Vec3(-rot_x, 2, rot_y)
            camera.rotation = (0., -app.visual_state[2], 0.)

    app.run()
