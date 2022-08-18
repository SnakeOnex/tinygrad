from ursina import *
from ursina.shaders import lit_with_shadows_shader

from cone_track import ConeTrack
from objects.formula import Formula
from enum import Enum
import sys
import numpy as np

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

if __name__ == '__main__':

    app = Ursina()

    # config
    camera.fov = 78
    window.title = "VirtualMilovice"
    window.borderless = False
    window.fps_counter.enabled = True
    Audio(sound_file_name='models/terrymusic.mp3', autoplay=True, loop=True)


    # 1. SETUP WORLD
    world_setup()

    # 2. RENDER CONES
    # cone_track = ConeTrack('slam_hard_track.npy')
    cone_track = ConeTrack()
    # cone_track.load_from_npy('data/slam_hard_track.npy')
    cone_track.load_from_inner_outer(inner_path='data/slam_inner.npy', outer_path='data/slam_outer.npy')
    cone_track.render_cones()

    # 3. RENDER THE CAR
    formula = Formula()

    driver = Entity(
        model='sphere', 
        position = formula.driver_pos,
        rotation=formula.real_rot,
        scale=0.2
    )

    # 4. HANDLE CAMERA
    app.cam_mode = CameraMode.WORLD

    def input(key):
        ## change camera mode
        if key == 'p':
            app.cam_mode = app.cam_mode.next()
            print(f"CAMERA MODE: {app.cam_mode}")
            if app.cam_mode is not CameraMode.FIRST_PERSON:
                formula.cam = False

        if key == 'r':
            formula.position = np.array((0.,0.,0.))
            formula.reset_state()

    text_main = Text()

    def update():                  # update gets automatically called by the engine.
        if held_keys['w']:
            formula.forward()
        elif held_keys['space']:
            formula.brake()
        else:
            formula.neutral()

        if held_keys['a']:
            formula.left()

        if held_keys['d']:
            formula.right()

        # update camera
        if app.cam_mode == CameraMode.WORLD:
            driver.position = formula.driver_pos
            driver.rotation = formula.real_rot

            camera.position = Vec3(0,15,-20)
            camera.look_at(formula)
        elif app.cam_mode == CameraMode.FIRST_PERSON:
            formula.cam = True

            driver.position = formula.driver_pos
            driver.rotation = formula.real_rot
        elif app.cam_mode == CameraMode.THIRD_PERSON:
            app.cam_mode = app.cam_mode.next()

    app.run()
