import numpy as np
from ursina import *
from ursina.shaders import lit_with_shadows_shader

from objects.formula import Formula
from objects.cone import Cone

from sim.math_helpers import angle_to_vector, vec_to_3d, rotate_around_point, local_to_global
from sim.simulation import GUIValues, ControlsValues

def render_world():
    """
    Renders world environment like sky, ground plane and directional light.
    """
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
    dl = DirectionalLight(parent=pivot, position=(2., 10., 2.), shadows=True, rotation=(90., 0., 0.))
    dl.disable()

def render_cones(state):
    """
    Given the state, which contains attributes containing cone positions for each color, renders the cones onto the map
    """

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

def render_car(visual_state, formula, driver, car_rect):
    """
    Given visual_state list, provided by the simulation process, changes the car's position and orientation accordingly.
    args:
      visual_state - python list provided by simulation process containing information needed for visualization the simulation state
      formula - reference to the formula ursina object
      driver - reference to the driver ursina object
      car_rect - reference to rectangle spanning the car's hitbox ursina object 
    """

    car_x, car_y = visual_state[GUIValues.car_pos]
    heading = visual_state[GUIValues.car_heading]
    steering_angle = visual_state[GUIValues.steering_angle]

    heading_vec = angle_to_vector(heading)

    driver.position = Vec3(car_x, 0., car_y) - vec_to_3d(heading_vec) + Vec3(0., 0.7, 0.)
    driver.rotation = formula.offset_rot + Vec3(0., 0., heading)

    formula.position = Vec3(car_x, 0., car_y)
    formula.rotation = formula.offset_rot + Vec3(0., 0, heading)

    formula.fl_wheel.rotation = (0., 0., steering_angle)
    formula.fr_wheel.rotation = (0., 0., steering_angle)

    formula.steering_wheel.rotation = (steering_angle, 0., 0.)

    # draw rectangle around car (UNUSED CODE ATM, mainly useful for debugging of car's hitbox)
    car_rect.position = Vec3(car_x, 0., car_y) - 0.5 * vec_to_3d(heading_vec)
    car_rect.rotation = formula.offset_rot + Vec3(0., 0., heading)
    # car_rect = Entity(model='quad', color=color.black, position=Vec3(car_x, 0., car_y))


def cone_pos_to_mesh(cone_pos, width=1.0, height=2.0):
    """
    Function for computing vertices of a convex polyhedron that is bounded by six rectangular faces with eight vertices and twelve edges surrounding the cone.
    args:
      cone_pos - position of a given cone
      width - width of the shape surrounding the cone
      height - height of the shape surround the cone
    """

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
