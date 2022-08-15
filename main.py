from ursina import *

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

    # 4. HANDLE CAMERA
    ## apparently all state has to be tied to the app object (not sure what to think)
    app.cam_mode = CameraMode.WORLD

    text = Text(text=f"{player.position}")

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
            camera.position = Vec3(0,15,-20)
            camera.look_at(player)
        elif app.cam_mode == CameraMode.FIRST_PERSON:
            # camera.position = player.position - player.get_heading_pos() * 3
            camera.position = player.position + Vec3(0., 0.5, 0.)
            camera.rotation = player.rotation
        elif app.cam_mode == CameraMode.THIRD_PERSON:
            camera.position = Vec3(0,15,-20)
            camera.look_at(player)
            app.cam_mode = app.cam_mode.next()

    app.run()
