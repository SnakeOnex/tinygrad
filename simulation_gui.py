from ursina import *
from ursina.shaders import lit_with_shadows_shader

import multiprocessing.connection as connection
from multiprocessing import shared_memory
import socket
import struct
import select
import zmq

from enum import Enum
import sys
import numpy as np
import random
import socket
import pickle
import argparse

from objects.formula import Formula
from objects.cone import Cone
from sim.state import State

from sim.network_helpers import connect_client, bind_udp_socket
from sim.math_helpers import angle_to_vector, vec_to_3d, rotate_around_point, local_to_global

HOST = '127.0.0.1'
VISUAL_PORT = 1337
CONTROLS_PORT = 1338

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
    cones = []
    # 0.01
    for c in state.yellow_cones:
        cone = Cone(model='models/yellow_cone.obj', position=Vec3(c[0], 0.01, c[1]))
        cones.append(cone)

    for c in state.blue_cones:
        cone = Cone(model='models/blue_cone.obj', position=Vec3(c[0], 0.01, c[1]))
        cones.append(cone)

    for c in state.orange_cones:
        cone = Cone(model='models/orange_cone.obj', position=(c[0], 0.01, c[1]))
        cones.append(cone)

    for c in state.big_cones:
        cone = Cone(model='models/big_orange_cone.obj', position=(c[0], 0.01, c[1]))
        cones.append(cone)

    start_points = [Vec3(x,0,y) for x, y in state.start_line]
    start_line = Entity(shader=lit_with_shadows_shader, color=color.red, model=Mesh(vertices=start_points, mode='line', thickness=2))

    finish_points = [Vec3(x,0,y) for x, y in state.finish_line]
    finish_line = Entity(shader=lit_with_shadows_shader, color=color.red, model=Mesh(vertices=finish_points, mode='line', thickness=2))

    car_pos = Entity(model='sphere', position=(state.car_pos[0], 0, state.car_pos[1]), scale=(0.1))

    return cones


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

    # 3. setup communication interfaces

    context = zmq.Context()
    controls_socket = context.socket(zmq.PUB)
    controls_socket.bind("tcp://127.0.0.1:50002")
    app.controls_state = [0, 0, 0, 0]  # go_signal, lateral, longitudinal

    # gui input
    gui_socket = context.socket(zmq.SUB)
    gui_socket.connect("tcp://127.0.0.1:50001")
    gui_socket.setsockopt(zmq.SUBSCRIBE, b"")
    gui_poller = zmq.Poller()
    gui_poller.register(gui_socket, zmq.POLLIN)

    data = gui_socket.recv()
    app.visual_state = pickle.loads(data)

    cones = render_cones(state)
    formula = Formula()
    driver = Entity(model='sphere', scale=0.2)
    text_main = Text()
    Text.size = 0.05
    app.text_AS = Text(text="", origin=(3.5, -5.), color=color.red)
    # app.text_hit_cones = Text(text="Hit cones: {0}".format(marshall.num_of_hit_cones), origin=(-2, -9.), color=color.red)
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
            # app.text_AS.text = "AS: ON"

        if key == 'g':
            app.controls_state[0] = 1

    def update():

        while gui_poller.poll(0.):
            data = gui_socket.recv()
            app.visual_state = pickle.loads(data)

        controls_socket.send(pickle.dumps(app.controls_state))


        # key handling

        ## longitudinal controls
        if held_keys['w']:
            app.controls_state[2] = 1
        elif held_keys['space']:
            app.controls_state[2] = -1
        else:
            app.controls_state[2] = 0

        ## lateral controls
        if held_keys['a']:
            app.controls_state[1] = -1;
        elif held_keys['d']:
            app.controls_state[1] = 1
        else:
            app.controls_state[1] = 0

        render_car(app.visual_state, formula, driver)

        # text = f"Speed: {state.speed:.2f}\nSteering angle: {state.steering_angle:.2f}\nHeading: {state.heading:.2f}\n"
        text_main.text = text

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
