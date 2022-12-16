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
    BIRD_VIEW = 3

    def next(self):
        v = self.value
        return CameraMode((v + 1) % 4)


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


def cone_pos_to_mesh(cone_pos, width=1.0, height=2.0):
    vertices = []

    x, y = cone_pos
    center = Vec3(x, 0., y)

    z_bottom = 0.01
    z_toppom = z_bottom + height

    vertices.append(center + Vec3(width/2, z_bottom, width/2))
    vertices.append(center + Vec3(width/2, z_toppom, width/2))
    vertices.append(center + Vec3(-width/2, z_toppom, width/2))
    vertices.append(center + Vec3(-width/2, z_bottom, width/2))
    vertices.append(center + Vec3(width/2, z_bottom, width/2))

    vertices.append(center + Vec3(width/2, z_bottom, -width/2))
    vertices.append(center + Vec3(width/2, z_toppom, -width/2))
    vertices.append(center + Vec3(width/2, z_toppom,  width/2))
    vertices.append(center + Vec3(width/2, z_toppom, -width/2))

    vertices.append(center + Vec3(-width/2, z_toppom, -width/2))
    vertices.append(center + Vec3(-width/2, z_bottom, -width/2))
    vertices.append(center + Vec3(width/2, z_bottom, -width/2))
    vertices.append(center + Vec3(-width/2, z_bottom, -width/2))
    vertices.append(center + Vec3(-width/2, z_bottom,  width/2))
    vertices.append(center + Vec3(-width/2, z_toppom,  width/2))
    vertices.append(center + Vec3(-width/2, z_toppom, -width/2))

    return vertices


def compute_as_state(world_preds, path, state):
    """
    given debug information from AS system, returns path and cone positions in global coordinates
    args:
      as_debug - AS debug dict
      state - visual_state list (info about current car_pos & heading)
    ret:
      path - path in local coord
      cones - Nx3, local cone positions + cone class
    """

    cone_pos = world_preds[:, :2]
    cone_cls = world_preds[:, 2:3]

    car_x, car_y = state[GUIValues.car_pos]
    heading = state[GUIValues.car_heading]
    path[:, 0] *= -1
    cone_pos[:, 0] *= -1

    path = local_to_global(path, (car_x, car_y), heading)
    cone_pos = local_to_global(cone_pos, (car_x, car_y), heading)

    path = [vec_to_3d(p, y=0.01) for p in path]
    cones = np.hstack((cone_pos, cone_cls))
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


def create_simulation_string(visual_state):
    simulation_text = "Simulation status:\n"
    simulation_text += f"Cones hit: {int(sum(visual_state[GUIValues.cones_mask]))}\n"
    simulation_text += f"AS Go signal: {'Active' if int(visual_state[GUIValues.go_signal]) else 'Inactive'}\n"
    simulation_text += f"Time elapsed: {visual_state[GUIValues.race_time]:.2f} s\n"
    simulation_text += f"Debug: {visual_state[GUIValues.debug][:4]}\n"
    return simulation_text


def create_car_string(speed, steering_angle):
    car_status_text = "Car status:\n"
    car_status_text += f"Speed: {speed:.2f} m/s\n"
    car_status_text += f"Steering angle: {steering_angle:.2f} rad/s\n"
    return car_status_text


def create_mission_string(id, values):
    mission_status_text = "Mission status:\n"
    mission_status_text += f"ID: {id}"
    for key, value in values.items():
        mission_status_text += f"\n{key}: {format_val_string(value)}"
    return mission_status_text


def format_val_string(val):
    return f"{val:.5f}" if type(val) != str and type(val) != int else val


def render_car(state, formula, driver, car_rect):
    car_x, car_y = state[GUIValues.car_pos]
    heading = state[GUIValues.car_heading]
    steering_angle = state[GUIValues.steering_angle]

    heading_vec = angle_to_vector(heading)

    driver.position = Vec3(car_x, 0., car_y) - 1. * \
        vec_to_3d(heading_vec) + Vec3(0., 0.7, 0.)
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
    cone_count = 20
    cone_detections = []
    camera.fov = 78
    line_height = 1.3
    text_x = -0.85
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

    app.cones, app.cones_mask = render_cones(state)

    # autonomous debug socket
    as_debug_socket = context.socket(zmq.SUB)
    as_debug_socket.connect(
        config["TCP_HOST"]+":"+config["AS_DEBUG_PORT"])
    as_debug_socket.setsockopt(zmq.SUBSCRIBE, b"")
    as_debug_poller = zmq.Poller()
    as_debug_poller.register(as_debug_socket, zmq.POLLIN)

    data = gui_socket.recv()
    app.visual_state = pickle.loads(data)

    formula = Formula()
    driver = Entity(model='sphere', scale=0.2)

    # AS debug init (creating empty path and cone mesh objects)
    car_rect = Entity(model='cube', color=color.black, position=Vec3(
        state.car_pos[0], 0., state.car_pos[1]), scale=Vec3(3, 1.5, 0.3))
    app.path_entity = Entity(shader=lit_with_shadows_shader, color=color.white, model=Mesh(
        vertices=[[0., 0., 0.], [0., 0., 0.]], mode='line'))

    for _ in range(cone_count):
        cone_detections.append(Entity(shader=lit_with_shadows_shader, color=color.red, model=Mesh(
            vertices=[[0., 0., 0.], [0., 0., 0.]], mode='line')))

    car_rect.enabled = False
    Text.size = 0.025
    app.simulation_text = Text(text=create_simulation_string(
        app.visual_state), color=color.turquoise)
    app.simulation_text.x = text_x
    app.simulation_text.y = 0.45
    app.simulation_text.line_height = line_height
    app.simulation_text.background = True

    app.car_status_text = Text(text=create_car_string(0, 0), color=color.azure)
    app.car_status_text.x = text_x
    app.car_status_text.y = 0.22
    app.car_status_text.line_height = line_height
    app.car_status_text.background = True

    app.mission_status_text = Text(
        text=create_mission_string("N/A", {}), color=color.violet)
    app.mission_status_text.x = text_x
    app.mission_status_text.y = 0.05
    app.mission_status_text.line_height = line_height
    app.mission_status_text.background = True

    app.cam_mode = CameraMode.FIRST_PERSON
    app.update_count = 0
    app.frame_start = time.perf_counter()

    def input(key):
        # change camera mode
        if key == 'p':
            app.cam_mode = app.cam_mode.next()
            print(f"CAMERA MODE: {app.cam_mode}")

        if key == 'r':
            state.reset_state()

        if key == 'g':
            app.controls_state[ControlsValues.go_signal] = 1

    def update():

        while gui_poller.poll(0.):
            data = gui_socket.recv()
            app.visual_state = pickle.loads(data)

        controls_socket.send(pickle.dumps(app.controls_state))

        while as_debug_poller.poll(0.):
            as_debug_data = pickle.loads(as_debug_socket.recv())
            path, cones = compute_as_state(
                as_debug_data["perception"], as_debug_data["path"], app.visual_state)
            app.path_entity.model = Mesh(
                vertices=path, mode='line', thickness=10)

            for i, cone_detection in enumerate(cone_detections):
                if cones.shape[0] <= i:
                    cone_detection.enabled = False
                else:
                    cone_detection.enabled = True

                    if cones[i, 2] == 0:
                        cone_color = color.yellow
                    elif cones[i, 2] == 1:
                        cone_color = color.blue
                    elif cones[i, 2] == 2:
                        cone_color = color.orange
                    elif cones[i, 2] == 3:
                        cone_color = color.red

                    if cones[i, 2] == 3:
                        cone_detection.model = Mesh(vertices=cone_pos_to_mesh(
                            cones[i, :2], width=0.3, height=0.6), mode='line', thickness=3)
                    else:
                        cone_detection.model = Mesh(vertices=cone_pos_to_mesh(
                            cones[i, :2], width=0.3, height=0.4), mode='line', thickness=3)

                    cone_detection.color = cone_color

            app.car_status_text.text = create_car_string(
                as_debug_data["speed"], as_debug_data["steering_angle"])
            app.car_status_text.background = True

            app.mission_status_text.text = create_mission_string(
                as_debug_data["mission_id"], as_debug_data["mission_status"])
            app.mission_status_text.background = True

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

        app.simulation_text.text = create_simulation_string(
            app.visual_state)
        app.simulation_text.background = True

        #text_main.text = text

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
        elif app.cam_mode == CameraMode.BIRD_VIEW:
            rot_x, rot_y = rotate_around_point(
                -app.visual_state[GUIValues.car_heading], (0, 0), (10, 0))
            camera.position = driver.position + Vec3(0, 18, 0)
            camera.rotation = (
                90., -app.visual_state[GUIValues.car_heading], 0.)

    app.run()
