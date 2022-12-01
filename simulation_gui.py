from ursina import *
from ursina.shaders import lit_with_shadows_shader

import multiprocessing.connection as connection
from multiprocessing import shared_memory
import socket
import struct
import select
import zmq

from enum import Enum
import json
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
from sim.simulation import GUIValues, ControlsValues


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
    dl = DirectionalLight(parent=pivot, position=(2., 10., 2.),
                          shadows=True, rotation=(90., 0., 0.))
    dl.disable()


def compute_as_state(as_debug, state):
    path = as_debug['path']
    cone_pos = as_debug['world_preds'][:,:2]

    car_x, car_y = state[GUIValues.car_pos]
    heading = state[GUIValues.car_heading]
    path[:, 0] *= -1
    path = local_to_global(path, (car_x, car_y), heading)
    cone_pos = local_to_global(cone_pos, (car_x, car_y), heading)
    path = [vec_to_3d(p, y=0.01) for p in path]
    cones = [vec_to_3d(cone, y=1.00) for cone in cone_pos]
    return path, cones


def render_cones(state):
    cones = []
    # 0.01
    for c in state.yellow_cones:
        cone = Cone(model='models/yellow_cone.obj',
                    position=Vec3(c[0], 0.01, c[1]))
        cones.append(cone)

    for c in state.blue_cones:
        cone = Cone(model='models/blue_cone.obj',
                    position=Vec3(c[0], 0.01, c[1]))
        cones.append(cone)

    for c in state.orange_cones:
        cone = Cone(model='models/orange_cone.obj',
                    position=(c[0], 0.01, c[1]))
        cones.append(cone)

    for c in state.big_cones:
        cone = Cone(model='models/big_orange_cone.obj',
                    position=(c[0], 0.01, c[1]))
        cones.append(cone)

    cone_mask = np.zeros((len(cones),))

    start_points = [Vec3(x, 0, y) for x, y in state.start_line]
    start_line = Entity(shader=lit_with_shadows_shader, color=color.red, model=Mesh(
        vertices=start_points, mode='line', thickness=2))

    finish_points = [Vec3(x, 0, y) for x, y in state.finish_line]
    finish_line = Entity(shader=lit_with_shadows_shader, color=color.red, model=Mesh(
        vertices=finish_points, mode='line', thickness=2))

    car_pos = Entity(model='sphere', position=(
        state.car_pos[0], 2.0, state.car_pos[1]), scale=(0.1))

    return cones, cone_mask


def render_car(state, formula, driver, car_rect):

    car_x, car_y = state[GUIValues.car_pos]
    heading = state[GUIValues.car_heading]
    steering_angle = state[GUIValues.steering_angle]

    heading_vec = angle_to_vector(heading)

    driver.position = Vec3(car_x, 0., car_y) - 1. * \
        vec_to_3d(heading_vec) + Vec3(0., 0.7, 0.)
    # driver.position = Vec3(car_x, 0., car_y) - 0. * vec_to_3d(heading_vec) + Vec3(0., 0.7, 0.)
    # driver.position = Vec3(car_x, 1., car_y)
    driver.rotation = formula.offset_rot + Vec3(0., 0., heading)

    formula.position = Vec3(car_x, 0., car_y)
    formula.rotation = formula.offset_rot + Vec3(0., 0, heading)

    formula.fl_wheel.rotation = (0., 0., steering_angle)
    formula.fr_wheel.rotation = (0., 0., steering_angle)

    formula.steering_wheel.rotation = (steering_angle, 0., 0.)

    # draw rectangle around car
    car_rect.position = Vec3(car_x, 0., car_y) - 0.5 * vec_to_3d(heading_vec)
    car_rect.rotation = formula.offset_rot + Vec3(0., 0., heading)
    # car_rect = Entity(model='quad', color=color.black, position=Vec3(car_x, 0., car_y))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='maps/circle_map.json')
    parser.add_argument('--config_json', type=str, default='sim_config.json')
    args = parser.parse_args()

    app = Ursina()

    # config
    camera.fov = 78
    window.title = "VirtualMilovice"
    window.size = (1280, 720)
    window.borderless = False
    window.fps_counter.enabled = True
    with open(Path(args.config_json).resolve(), 'r') as f:
        config = json.loads(f.read())
    app.AS = False

    if random.random() < 0.01:
        Audio(sound_file_name='models/terrymusic.mp3', autoplay=True, loop=True)

    # 1. SETUP WORLD
    world_setup()

    # 2. SETUP STATE
    state = State(0, args.map)

    # 3. setup communication interfaces

    # controls
    context = zmq.Context()
    controls_socket = context.socket(zmq.PUB)
    controls_socket.bind(
        config["TCP_HOST"]+":"+config["CONTROLS_PORT"])
    # go_signal, lateral, longitudinal
    app.controls_state = [0 for _ in range(len(ControlsValues))]

    # gui input
    gui_socket = context.socket(zmq.SUB)
    gui_socket.connect(
        config["TCP_HOST"]+":"+config["GUI_PORT"])
    gui_socket.setsockopt(zmq.SUBSCRIBE, b"")
    gui_poller = zmq.Poller()
    gui_poller.register(gui_socket, zmq.POLLIN)

    # autonomous debug socket

    as_debug_socket = context.socket(zmq.SUB)
    as_debug_socket.connect(
        config["TCP_HOST"]+":"+config["AS_DEBUG_PORT"])
    as_debug_socket.setsockopt(zmq.SUBSCRIBE, b"")
    as_debug_poller = zmq.Poller()
    as_debug_poller.register(as_debug_socket, zmq.POLLIN)

    data = gui_socket.recv()
    app.visual_state = pickle.loads(data)

    app.cones, app.cones_mask = render_cones(state)
    formula = Formula()
    driver = Entity(model='sphere', scale=0.2)
    car_rect = Entity(model='cube', color=color.black, position=Vec3(
        state.car_pos[0], 0., state.car_pos[1]), scale=Vec3(3, 1.5, 0.3))
    app.path_entity = Entity(color=color.white, model=Mesh(vertices=[
        [0., 0., 0.], [0., 0., 0.]], mode='line', thickness=50, colors=[color.white, color.white, color.white, color.white, color.white]))
    car_rect.enabled = False

    text_main = Text()
    Text.size = 0.05
    app.text_AS = Text(text="", origin=(3.5, -5.), color=color.red)
    app.track_marshall_text = Text(text="not loaded")
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
            app.controls_state[ControlsValues.go_signal] = 1

    def update():

        while gui_poller.poll(0.):
            data = gui_socket.recv()
            app.visual_state = pickle.loads(data)
            app.track_marshall_text.text = f"ksicht"

        controls_socket.send(pickle.dumps(app.controls_state))

        # TODO: Implement visualization of path planning, cone detections and autonomous debug info
        while as_debug_poller.poll(0.):
            as_debug_data = pickle.loads(as_debug_socket.recv())
            path, cones = compute_as_state(
                as_debug_data, app.visual_state)
            app.path_entity.model = Mesh(vertices=path, mode='line', thickness=10, colors=[
                                         color.white, color.white, color.white, color.white, color.white])

            for cone in cones:
                Entity(model='cube', color=color.black, position=cone, scale=Vec3(1, 1.0, 1.0))

        # key handling

        # longitudinal controls
        if held_keys['w']:
            app.controls_state[ControlsValues.long_control] = 1
        elif held_keys['space']:
            app.controls_state[ControlsValues.long_control] = -1
        else:
            app.controls_state[ControlsValues.long_control] = 0

        # lateral controls
        if held_keys['a']:
            app.controls_state[ControlsValues.lat_control] = -1
        elif held_keys['d']:
            app.controls_state[ControlsValues.lat_control] = 1
        else:
            app.controls_state[ControlsValues.lat_control] = 0

        render_car(app.visual_state, formula, driver, car_rect)

        # text = f"Speed: {state.speed:.2f}\nSteering angle: {state.steering_angle:.2f}\nHeading: {state.heading:.2f}\n"
        # update text
        received_cones_mask = app.visual_state[GUIValues.cones_mask]
        diff = np.where(received_cones_mask != app.cones_mask)[0]

        for cone_idx in diff:
            # cones[cone_idx].enabled = False
            app.cones[cone_idx].rotation = Vec3(90., 0., 0)

        app.cones_mask = received_cones_mask

        trackmarshall_text = ""
        trackmarshall_text += f"cones hit: {int(sum(app.visual_state[GUIValues.cones_mask]))}\n"
        trackmarshall_text += f"go_signal: {int(app.visual_state[GUIValues.go_signal])}\n"
        trackmarshall_text += f"time: {app.visual_state[GUIValues.race_time]:.2f}\n"
        trackmarshall_text += f"debug: {str(app.visual_state[GUIValues.debug])}\n"
        app.track_marshall_text.text = trackmarshall_text

        text_main.text = text

        # update camera
        if app.cam_mode == CameraMode.WORLD:
            camera.position = Vec3(0, 15, -20)
            camera.look_at(formula)
        elif app.cam_mode == CameraMode.FIRST_PERSON:
            camera.position = driver.position
            # camera.rotation = (0.,-state.heading,0.)
            camera.rotation = (
                0., -app.visual_state[GUIValues.car_heading], 0.)
        elif app.cam_mode == CameraMode.THIRD_PERSON:
            rot_x, rot_y = rotate_around_point(
                -app.visual_state[GUIValues.car_heading], (0, 0), (10, 0))
            camera.position = driver.position + Vec3(-rot_x, 2, rot_y)
            camera.rotation = (
                0., -app.visual_state[GUIValues.car_heading], 0.)

    app.run()
