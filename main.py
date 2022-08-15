from ursina import *

from cone_track import ConeTrack

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
    player = Entity(
        model = 'sphere',
        position=(1,1,1),
        texture = 'white_cube',
        color = color.orange,
        scale_y = 1
    )

    camera.position = Vec3(0,50,-50)
    camera.rotation = (35,0,0)


    print(camera.position)

    def update():                  # update gets automatically called by the engine.
        player.x += held_keys['d'] * .1
        player.x -= held_keys['a'] * .1
        player.z += held_keys['w'] * .1
        player.z -= held_keys['s'] * .1

        camera.rotation

    app.run()
