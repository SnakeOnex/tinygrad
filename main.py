from ursina import *
from ursina.shaders import lit_with_shadows_shader

import multiprocessing.connection as connection
from multiprocessing import shared_memory

from objects.formula import Formula
from objects.cone import Cone
from enum import Enum
import sys 
import numpy as np
import random
import socket
import pickle

from state import State
from math_helpers import angle_to_vector, vec_to_3d, rotate_around_point, local_to_global

CONNECTION = False

class CameraMode(Enum):
    WORLD = 0
    FIRST_PERSON = 1
    THIRD_PERSON = 2

    def next(self):
        v = self.value
        return CameraMode((v + 1) % 3)

def world_setup():
    ground = Entity(
        model = 'cube',
        color = color.gray,
        texture='asp.jpg',
        texture_scale = (4, 4),
        position=(0,0,0),
        scale = (200, 0, 500),
        collider = 'box',
        shader=lit_with_shadows_shader
    )
    skybox = load_texture('sky.jpg')
    sky = Sky(
        texture=skybox,
        color=color.cyan
    )
    
    pivot = Entity()
    DirectionalLight(parent=pivot, position=(2.,10.,2.), shadows=True, rotation=(90.,0., 0.))

def connect_client(address, port):
    remote_address = address, port
    listener = connection.Listener(remote_address)
    conn =  listener.accept()
    return conn

def render_path(path_list, path_entity):
    path = np.array([[app.path_mem[i] for i in range(0, len(app.path_mem)//2)], [app.path_mem[i] for i in range(len(app.path_mem)//2, len(app.path_mem))]]).T

    path[:,0] *= -1
    path = local_to_global(path, state.car_pos, state.heading)
    path = [vec_to_3d(p, y=0.01) for p in path]

    print("path: ", path)
    # path_entity = Entity(shader=lit_with_shadows_shader,color=color.red,model=Mesh(vertices=[[0., 0., 0.], [0., 0., 0.]], mode='line', thickness=5,colors=[color.red, color.red]))
    path_entity.model.vertices = path
    return path

def render_cones(state):
    [Cone(texture='models/yellow_cone.mtl', position=(c[0], 0.01, c[1])) for c in state.yellow_cones]
    [Cone(model='models/blue_cone.obj',texture='models/blue_cone.mtl', position=(c[0], 0.01, c[1])) for c in state.blue_cones]
    [Cone(model='models/orange_cone.obj',texture='models/orange_cone.mtl', position=(c[0], 0.01, c[1])) for c in state.orange_cones]
    [Cone(model='models/big_orange_cone.obj',texture='models/big_orange_cone.mtl', position=(c[0], 0.01, c[1])) for c in state.big_cones]

def render_car(state, formula, driver):
    heading_vec = angle_to_vector(state.heading)

    car_x, car_y = state.car_pos

    driver.position = Vec3(car_x, 0., car_y) - 1. * vec_to_3d(heading_vec) + Vec3(0., 0.7, 0.)
    driver.rotation= formula.offset_rot + Vec3(0., 0., state.heading)

    formula.position = Vec3(car_x, 0., car_y)
    formula.rotation = formula.offset_rot + Vec3(0., 0, state.heading)

    formula.fl_wheel.rotation = (0. , 0., state.steering_angle)
    formula.fr_wheel.rotation = (0. , 0., state.steering_angle)

if __name__ == '__main__':

    app = Ursina()

    # config
    camera.fov = 78
    window.title = "VirtualMilovice"
    window.size = (1280,720)
    window.borderless = False
    window.fps_counter.enabled = True
    app.AS = False

    if random.random() < 0.01:
        Audio(sound_file_name='models/terrymusic.mp3', autoplay=True, loop=True)

    ## 1. SETUP WORLD
    world_setup()

    # connect client
    if CONNECTION:
        ## DETECTIONS CONNECTION 
        app.conn_det = connect_client("localhost", 50000)
        det_msg_delta = 0.200 # s
        app.last_det_time = time.perf_counter()
        app.detections = None

        ## CAN1 CONNECTION
        app.conn_can1_out = connect_client("localhost", 50001)
        can1_msg_delta = 0.200 # s
        app.last_can1_time = time.perf_counter()
        app.can1_state = None

        app.conn_can1_in = connect_client("localhost", 50002)
        app.can1_in_state = None

        app.path_mem = shared_memory.ShareableList(name="path")
        app.path_entity = Entity(shader=lit_with_shadows_shader,color=color.red,model=Mesh(vertices=[[0., 0., 0.], [0., 0., 0.]], mode='line', thickness=50,colors=[color.red, color.red, color.red, color.red, color.red]))

    ## 2. SETUP STATE
    state = State("maps/circle_map.json")
    render_cones(state)
    formula = Formula()
    driver = Entity(model='sphere', scale=0.2)
    text_main = Text()
    Text.size = 0.05
    app.text_AS = Text(text="AS: OFF",origin=(3.5, -5.), color=color.red)
    app.cam_mode = CameraMode.WORLD

    def input(key):
        ## change camera mode
        if key == 'p':
            app.cam_mode = app.cam_mode.next()
            print(f"CAMERA MODE: {app.cam_mode}")

        if key == 'r':
            state.reset_state()

        if key == 'e':
            app.AS = True
            app.text_AS.color = color.lime
            app.text_AS.text = "AS: ON"

    def update():  
        if held_keys['w']:
            state.forward()
        elif held_keys['space']:
            state.brake()

        if held_keys['a']:
            state.steer_left()
        elif held_keys['d']:
            state.steer_right()
        else:
            state.steering_control = "NEUTRAL"

        state.update_state(time.dt)
        render_car(state, formula, driver)
        text = f"Speed: {state.speed:.2f}\nSteering angle: {state.steering_angle:.2f}\nHeading: {state.heading:.2f}\n"

        # obtain and send cone detections
        if CONNECTION and app.AS:
            path = render_path(app.path_mem, app.path_entity)
            app.path_entity.model.vertices = path
            app.path_entity.model = Mesh(vertices=path, mode='line', thickness=50,colors=[color.red, color.red, color.red, color.red, color.red])

            if time.perf_counter() - app.last_det_time >= det_msg_delta:
                app.detections = state.get_detections()
                app.conn_det.send(app.detections)
                app.last_det_time = time.perf_counter()
            # text += f"Detections:{app.detections}\n"

        # obtain and send can1 state
            if time.perf_counter() - app.last_can1_time >= can1_msg_delta:
                app.can1_state = state.get_can1_state()
                app.conn_can1_out.send(app.can1_state)
                app.last_can1_time = time.perf_counter()

            if app.conn_can1_in.poll():
                app.can1_in_state = list(app.conn_can1_in.recv())
                state.steering_angle = app.can1_in_state[0]
                state.forward()

            # text += f"Can1: {app.can1_state}\n"
            # text += f"Can1_in: {app.can1_in_state}\n"

        text_main.text = text

        # update camera
        if app.cam_mode == CameraMode.WORLD:
            camera.position = Vec3(0,15,-20)
            camera.look_at(formula)
        elif app.cam_mode == CameraMode.FIRST_PERSON:
            camera.position = driver.position
            camera.rotation = (0.,-state.heading,0.)
        elif app.cam_mode == CameraMode.THIRD_PERSON:
            rot_x, rot_y = rotate_around_point(-state.heading,(0,0),(10,0))
            camera.position = driver.position + Vec3(-rot_x,2,rot_y)
            camera.rotation = (0.,-state.heading,0.)

    app.run()

