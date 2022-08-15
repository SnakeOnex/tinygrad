from ursina import *

from cone_track import ConeTrack
from formula import Formula

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

def handle_camera(cam_mode, player):
    """
    setup camera based on the player position and camera mode
    """

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
    # cam_mode = "world"
    cam_mode = "first_person"

    if cam_mode == "world":
        camera.position = Vec3(0,15,-20)
    elif cam_mode == "first_person":
        camera.position = player.position - player.get_heading_pos() * 3

    text = Text(text=f"{player.position}")

    def update():                  # update gets automatically called by the engine.
        if held_keys['w']:
            player.forward()

        if held_keys['a']:
            player.left()

        if held_keys['d']:
            player.right()

        text.text = f"Pos: {player.position}\n Rotation: {player.rotation} \nthrottle: {player.throttle}"

        # update camera
        camera.position = player.position - player.get_heading_pos() * 3
        camera.look_at(player)

    app.run()
