from ursina import *
from ursina.shaders import lit_with_shadows_shader

import multiprocessing.connection as connection
from multiprocessing import shared_memory
import socket
import struct
import select
import zmq

from enum import IntEnum
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
from rendering_helpers import render_world, render_cones, render_car, cone_pos_to_mesh

MAX_PATH_LEN = 15

class CameraMode(IntEnum):
    WORLD = 0
    FIRST_PERSON = 1
    THIRD_PERSON = 2
    BIRD_VIEW = 3

    def next(self):
        v = self.value
        return CameraMode((v + 1) % 4)


def compute_as_state(world_preds, path, target, state):
    """
    given debug information from AS system, returns path and cone positions in global coordinates
    args:
      world_preds - AS's cone detections in car's coordinate system
      path - Nx2 array of AS's currently planned path in car's coordinate system
      target - 2D point representing AS controller current lookahead point 
      state - visual_state list (info about current car_pos & heading)
    ret:
      path - path in local coord
      cones - Nx3, local cone positions + cone class
    """

    cone_pos = world_preds[:, :2]
    cone_cls = world_preds[:, 2:3]
    car_x, car_y = state[GUIValues.car_pos]
    heading = state[GUIValues.car_heading]

    path = local_to_global(path, (car_x, car_y), heading)

    target = target.reshape((1, 2))
    target = local_to_global(target, (car_x, car_y), heading)
    cone_pos = local_to_global(cone_pos, (car_x, car_y), heading)

    path = [vec_to_3d(p, y=0.01) for p in path]
    target = vec_to_3d(target[0], y=-0.05)
    cones = np.hstack((cone_pos, cone_cls))
    return path, target, cones


def create_simulation_string(debug):
    simulation_text = "Simulation status:\n"
    for key, value in debug.items():
        simulation_text += f"{key}: {value}\n"
    return simulation_text


def create_car_string(speed, steering_angle, heading, car_pos):
    car_status_text = "Car status:\n"
    car_status_text += f"Speed: {speed:.2f} m/s\n"
    car_status_text += f"Steering angle: {steering_angle:.2f}\n"
    car_status_text += f"Heading: {heading:.2f}\n"
    # car_status_text += f"Car pos: {car_pos[0]:.2f}\n"
    return car_status_text


def create_mission_string(id, values):
    mission_status_text = "Mission status:\n"
    mission_status_text += f"ID: {id}"
    for key, value in values.items():
        mission_status_text += f"\n{key}: {format_val_string(value)}"
    return mission_status_text


def format_val_string(val):
    value_str = val
    if type(val) == float or type(val) == np.float_:
        value_str = f"{val: .2f}"
    elif type(val) == np.ndarray:
        value_str = np.array2string(val, precision=2, suppress_small=True)
    return value_str


if __name__ == '__main__':
    # 0. parsing arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='maps/circle_map.json')
    parser.add_argument('--config_json', type=str, default='sim_config.json')
    args = parser.parse_args()

    # 1. engine initialization
    app = Ursina()
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

    # 2. Render the environment & initializate engine structures
    render_world()
    state = State(0, args.map)
    app.cones, app.cones_mask = render_cones(state)

    formula = Formula()
    driver = Entity(model='sphere', scale=0.2)
    app.cam_mode = CameraMode.FIRST_PERSON

    # AS debug init (creating empty path and cone mesh objects)
    car_rect = Entity(model='cube', color=color.black, position=Vec3(state.car_pos[0], 0., state.car_pos[1]), scale=Vec3(3, 1.5, 0.3))
    app.path_entity = Entity(shader=lit_with_shadows_shader, color=color.rgb(0,255,0), model=Mesh(vertices=[[0., 0., 0.], [0., 0., 0.]], mode='line'))

    app.path_entities = [Entity(shader=lit_with_shadows_shader, color=color.rgb(0,255,0), model=Mesh(vertices=[[0., 0., 0.], [0., 0., 0.]], mode='line', thickness=8)) for _ in range(MAX_PATH_LEN)]

    app.target = Entity(model='cube', scale=0.2, color=color.green)

    for _ in range(cone_count):
        cone_detections.append(Entity(shader=lit_with_shadows_shader, color=color.red, model=Mesh(vertices=[[0.,0.,0.] for _ in range(16)], mode='line', thickness=3)))

    # 2. setup communication interfaces

    # 2.A - GUI -> Simulation socket for sending control commands
    context = zmq.Context()
    controls_socket = context.socket(zmq.PUB)
    controls_socket.bind(
        config["TCP_HOST"]+":"+config["CONTROLS_PORT"])
    # go_signal, lateral, longitudinal
    app.controls_state = [0 for _ in range(len(ControlsValues))]

    # 2.B - Simulation -> GUI socket for receiving simulation state needed for visualization
    gui_socket = context.socket(zmq.SUB)
    gui_socket.connect(
        config["TCP_HOST"]+":"+config["GUI_PORT"])
    gui_socket.setsockopt(zmq.SUBSCRIBE, b"")
    gui_poller = zmq.Poller()
    gui_poller.register(gui_socket, zmq.POLLIN)
    data = gui_socket.recv()
    app.visual_state = pickle.loads(data)

    # 2.C - AS -> GUI socket for receiving Autonomous System internal state debbuging messages for visualization
    as_debug_socket = context.socket(zmq.SUB)
    as_debug_socket.connect(
        config["TCP_HOST"]+":"+config["AS_DEBUG_PORT"])
    as_debug_socket.setsockopt(zmq.SUBSCRIBE, b"")
    as_debug_poller = zmq.Poller()
    as_debug_poller.register(as_debug_socket, zmq.POLLIN)

    # 3. GUI initialization
    car_rect.enabled = False
    Text.size = 0.025
    app.simulation_text = Text(text=create_simulation_string(
        app.visual_state[GUIValues.debug]), color=color.turquoise)
    app.simulation_text.x = text_x
    app.simulation_text.y = 0.45
    app.simulation_text.line_height = line_height
    app.simulation_text.background = True

    app.car_status_text = Text(text=create_car_string(0, 0, 0, 0), color=color.azure)
    app.car_status_text.x = text_x
    app.car_status_text.y = 0.2
    app.car_status_text.line_height = line_height
    app.car_status_text.background = True

    app.mission_status_text = Text(
        text=create_mission_string("N/A", {}), color=color.violet)
    app.mission_status_text.x = text_x
    # app.mission_status_text.y = 0.05
    app.mission_status_text.y = -0.05
    app.mission_status_text.line_height = line_height
    app.mission_status_text.background = True

    def input(key):
        """
        Ursina function for processing keyboard inputs
        """
        if key == 'p':
            app.cam_mode = app.cam_mode.next()
            print(f"CAMERA MODE: {app.cam_mode}")

        if key == 'r':
            state.reset_state()

        if key == 'g':
            app.controls_state[ControlsValues.go_signal] = 1

    def update():
        """
        Ursina engine update loop
        """

        # 1. send controls packet to simulation process
        controls_socket.send(pickle.dumps(app.controls_state))

        # 2. receive GUI state packet from simulation process
        while gui_poller.poll(0.):
            data = gui_socket.recv()
            app.visual_state = pickle.loads(data)

            app.car_status_text.text = create_car_string(
                app.visual_state[GUIValues.car_speed],
                app.visual_state[GUIValues.steering_angle],
                app.visual_state[GUIValues.car_heading],
                app.visual_state[GUIValues.car_pos])
            app.car_status_text.background = True

        # 3. receive internal state from BROS for visualization
        while as_debug_poller.poll(0.):
            as_debug_data = pickle.loads(as_debug_socket.recv())
            path, target, cones = compute_as_state(as_debug_data["perception"], as_debug_data["path"], as_debug_data["target"], app.visual_state)

            if len(path) > 1:
                for i in range(MAX_PATH_LEN):
                    if i <= len(path) - 2:
                        app.path_entities[i].color = color.rgb(i*25, 0, 0)
                        app.path_entities[i].model.vertices[0]=path[i:i+2][0]
                        app.path_entities[i].model.vertices[1]=path[i:i+2][1]
                        app.path_entities[i].model.generate()
                    else:

                        if app.path_entities[i].model.vertices[0] != [0., 0., 0.]:
                            app.path_entities[i].model.vertices[0]=[0., 0., 0.]
                            app.path_entities[i].model.vertices[1]=[0., 0., 0.]
                            app.path_entities[i].model.generate()

            # else:
            #     app.path_entity.model = Mesh()

            # render target
            app.target.position = target

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
                        cone_verts = cone_pos_to_mesh(cones[i, :2], width=0.3, height=0.6)
                        for j in range(len(cone_verts)):
                            cone_detection.model.vertices[j]=cone_verts[j]
                    else:
                        cone_verts = cone_pos_to_mesh(cones[i, :2], width=0.3, height=0.4)
                        for j in range(len(cone_verts)):
                            cone_detection.model.vertices[j]=cone_verts[j]

                    cone_detection.model.generate()
                    cone_detection.color = cone_color

            app.mission_status_text.text = create_mission_string(as_debug_data["mission_id"], as_debug_data["mission_status"])
            app.mission_status_text.background = True

        # key handling

        # longitudinal controls
        if held_keys['w']:
            print("forward")
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

        app.simulation_text.text = create_simulation_string(app.visual_state[GUIValues.debug])
        app.simulation_text.background = True

        #text_main.text = text

        # update camera
        if app.cam_mode == CameraMode.WORLD:
            camera.position = Vec3(0, 15, -20)
            camera.look_at(formula)
        elif app.cam_mode == CameraMode.FIRST_PERSON:
            camera.position = driver.position
            # camera.rotation = (0.,-state.heading,0.)
            camera.rotation = (0., -app.visual_state[GUIValues.car_heading] + 90., 0.)
        elif app.cam_mode == CameraMode.THIRD_PERSON:
            rot_x, rot_y = rotate_around_point(-app.visual_state[GUIValues.car_heading]+90., (0, 0), (10, 0))
            camera.position = driver.position + Vec3(-rot_x, 2, rot_y)
            camera.rotation = (0., -app.visual_state[GUIValues.car_heading]+90., 0.)
        elif app.cam_mode == CameraMode.BIRD_VIEW:
            rot_x, rot_y = rotate_around_point(-app.visual_state[GUIValues.car_heading] + 90., (0, 0), (10, 0))
            camera.position = driver.position + Vec3(0, 18, 0)
            camera.rotation = (90., -app.visual_state[GUIValues.car_heading], 0.)

    app.run()
