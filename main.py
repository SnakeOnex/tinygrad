from ursina import *

from cone_track import ConeTrack

app = Ursina()

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

player = Entity(
    model = 'cube',
    position=(1,1,1),
    texture = 'white_cube',
    color = color.orange,
    scale_y = 1
)

camera.position = Vec3(0,50,-50)
camera.rotation = (35,0,0)

# cone track
cone_track = ConeTrack('slam_hard_track.npy')

for yellow_cone in cone_track.yellow_cones:
    Entity(
        model = 'cube',
        color = color.yellow,
        texture = 'white_cube',
        position = (yellow_cone[0], 0, yellow_cone[1]),
        scale=(0.1,0.4,0.1)
    )

for blue_cone in cone_track.blue_cones:
    Entity(
        model = 'cube',
        color = color.blue,
        texture = 'white_cube',
        position = (blue_cone[0], 0, blue_cone[1]),
        scale=(0.1,0.4,0.1)
    )



# Sky(color = color.blue)

print(camera.position)

def update():                  # update gets automatically called by the engine.
    player.x += held_keys['d'] * .1
    player.x -= held_keys['a'] * .1
    player.z += held_keys['w'] * .1
    player.z -= held_keys['s'] * .1

    camera.rotation

app.run()
