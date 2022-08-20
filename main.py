from ursina import *
from ursina.shaders import lit_with_shadows_shader

import multiprocessing.connection as connection

from objects.formula import Formula
from objects.cone import Cone
from enum import Enum
import sys 
import numpy as np
import random
import socket
import pickle

from state import State
from math_helpers import angle_to_vector, vec_to_3d

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
        # texture='brick',
        texture='concrete.jpg',
        texture_scale = (8, 8),
        position=(0,0,0),
        scale = (200, 0, 500),
        collider = 'box',
        shader=lit_with_shadows_shader
    )

    sky = Sky(
        texture='sky_default',
        color=color.cyan
    )

    pivot = Entity()
    DirectionalLight(parent=pivot, position=(0.,10.,0.), shadows=True, rotation=(90.,0., 0.))

def connect_client():
    remote_address = "localhost", 50000

    listener = connection.Listener(remote_address)
    conn =  listener.accept()
    return conn

def render_cones(state):
    [Cone(color=color.yellow, position=(c[0], 0.01, c[1])) for c in state.yellow_cones]
    [Cone(color=color.blue, position=(c[0], 0.01, c[1])) for c in state.blue_cones]
    [Cone(color=color.orange, position=(c[0], 0.01, c[1])) for c in state.orange_cones]
    [Cone(color=color.red, position=(c[0], 0.01, c[1])) for c in state.big_cones]

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
    window.borderless = False
    window.fps_counter.enabled = True

    if random.random() < 0.01:
        Audio(sound_file_name='models/terrymusic.mp3', autoplay=True, loop=True)

    ## 1. SETUP WORLD
    world_setup()

    # connect client
    # conn = connect_client()
    # app.conn = conn

    ## 2. SETUP STATE
    state = State("maps/circle_map.json")
    render_cones(state)
    formula = Formula()
    driver = Entity(model='sphere', scale=0.2)
    text_main = Text()
    app.cam_mode = CameraMode.WORLD

    def input(key):
        ## change camera mode
        if key == 'p':
            app.cam_mode = app.cam_mode.next()
            print(f"CAMERA MODE: {app.cam_mode}")

        if key == 'r':
            state.reset_state()

    def update():  
        if held_keys['w']:
            state.forward()
        elif held_keys['space']:
            state.brake()

        if held_keys['a']:
            state.steer_left()
        elif held_keys['d']:
            state.steer_right()

        state.update_state(time.dt)
        render_car(state, formula, driver)

        text = f"Speed: {state.speed}\nSteering angle: {state.steering_angle:.2f}\nHeading: {state.heading}"
        text_main.text = text

        # speed_in_bytes = pickle.dumps(formula.speed)

        # get cones
        # cones_local = cone_track.get_cones_local(formula.position, formula.heading)

        # app.conn.send(formula.speed)

        # update camera
        if app.cam_mode == CameraMode.WORLD:
            camera.position = Vec3(0,15,-20)
            camera.look_at(formula)
        elif app.cam_mode == CameraMode.FIRST_PERSON:
            camera.position = driver.position
            camera.rotation = (0.,-state.heading,0.)
        elif app.cam_mode == CameraMode.THIRD_PERSON:
            app.cam_mode = app.cam_mode.next()

    app.run()


