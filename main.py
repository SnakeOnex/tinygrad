from ursina import *
from ursina.shaders import lit_with_shadows_shader

from cone_track import ConeTrack
from formula import Formula
from enum import Enum
import sys

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
        # texture='asfalt.jpg',
        position=(0,0,0),
        scale = (200, 0, 200),
        collider = 'box'
    )

    sky = Sky(
        texture='sky_default',
        color=color.cyan
    )

if __name__ == '__main__':

    app = Ursina()

    # 1. SETUP WORLD
    world_setup()

    # 2. RENDER CONES
    cone_track = ConeTrack('slam_hard_track.npy')
    cone_track.render_cones()

    # 3. RENDER THE CAR
    player = Formula()

    # driver_offset = Vec3(0., 1.2, -2.)
    driver_offset = Vec3(0.,0.,0.8)
    driver = Entity(
        model='sphere', 
        position = player.position + driver_offset,
        rotation=player.real_rot,
        scale=0.2
    )

    cone = Entity(
        model='models/cone_yellow.fbx', 
        color=color.yellow, 
        position=(0.,0.01, 0.), 
        scale=(0.1, 0.1, 0.1),
        shader=lit_with_shadows_shader
    )

    # Entity(model='models/whole_car.stl', color=color.red, position = (1, 0.2, 1), scale=(0.001, 0.001, 0.001), rotation=(270.,0.,0.))


    # 4. HANDLE CAMERA
    ## apparently all state has to be tied to the app object (not sure what to think)
    app.cam_mode = CameraMode.WORLD

    text = Text(text=f"{player.position}")

    pivot = Entity()
    DirectionalLight(parent=pivot, position=(0.,10.,0.), shadows=True, rotation=(90.,0., 0.))

    def input(key):
        if key == 'p':
            app.cam_mode = app.cam_mode.next()
            print(f"CAMERA MODE: {app.cam_mode}")

    def update():                  # update gets automatically called by the engine.
        if held_keys['w']:
            player.forward()

        if held_keys['a']:
            player.left()

        if held_keys['d']:
            player.right()

        text.text = f"Pos: {player.position}\n Rotation: {player.rotation} \nthrottle: {player.throttle}"

        # update camera
        if app.cam_mode == CameraMode.WORLD:
            driver.position = player.driver_pos
            driver.rotation = player.real_rot

            camera.position = Vec3(0,15,-20)
            camera.look_at(player)
        elif app.cam_mode == CameraMode.FIRST_PERSON:
            camera.position = player.driver_pos
            camera.rotation = player.real_rot

            driver.position = player.driver_pos
            driver.rotation = player.real_rot
        elif app.cam_mode == CameraMode.THIRD_PERSON:
            app.cam_mode = app.cam_mode.next()

    app.run()
