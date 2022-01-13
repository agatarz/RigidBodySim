"""
 Początkowa pozycja fundamentu nr 2 to (650, 140). W funkcji update: dopóki pozycja y fundamenty nie będzie mniejsza niż
 150 m, czyli 10 metrow do gory od poczatkowego polozenia to cialo ma predkosc 5 niezaleznie od obiektow je otaczajacych.
 Po przekroczeniu 10 metrow cialo ma predkosc 0 - jest niewzruszone.

"""

import math
import pyglet
from pyglet.gl import *
import pymunk
import pymunk.pyglet_util
from pymunk import Vec2d

import numpy as np

velocity_UP = np.loadtxt('s_UP.txt')


global width, height, scale, friction_foundation, density, friction_column
global foundation, iter_dt


class Brick():
    def __init__(self, mass, points, position, friction, color, type='d', radius=0.00):
        self.mass = mass
        self.moment = pymunk.moment_for_poly(self.mass, points, (0, 0))
        if type == 's':
            self.body = pymunk.Body(self.mass, self.moment, body_type=pymunk.Body.STATIC)
        else:
            self.body = pymunk.Body(self.mass, self.moment, body_type=pymunk.Body.DYNAMIC)

        self.body.position = position
        self.body.angle = radius
        self.shape = pymunk.Poly(self.body, points)
        self.shape.friction = friction
        self.shape.color = color

    def add_space(self, space):
        space.add(self.body, self.shape)
        pass


def mirror_points(points_by_center):
    return [(-x, y) for (x, y) in points_by_center]


def mirror_position(position_by_center, os_odbicia):
    return [2 * os_odbicia - position_by_center[0], position_by_center[1]]


def scaled(number):
    """
    This function get the value (dimension, gravity) and return it multipled by scale factor number.
    By this, you don't have to thing about scaling during create_wors's body
    :param number - value to scale
    :return: scaled value
    """

    if type(number) in (int, float):
        return number * scale
    elif type(number) in (tuple, list):
        return tuple(_n * scale for _n in number)
    else:
        return TypeError


def make_points_from_dim(dimension):
    """
    gets dimension from rectangular shape as dimensions(length, height), and return top's points in local coordinates
     relative to the center of gravity this body
    :param dimension: tuple(length, height)
    :return: [(x0, y0),(x1, y1),(x2, y2),(x3, y3)]
    """
    return [(- dimension[0] / 2, - dimension[1] / 2),
            (- dimension[0] / 2, + dimension[1] / 2),
            (+ dimension[0] / 2, + dimension[1] / 2),
            (+ dimension[0] / 2, - dimension[1] / 2)]


def make_points_from_points(local_points, scale=False):
    """
    gets points from no rectangular shape as points in local coordinate system, based in point(0,0)
    :param local_points: list of tuples
    :return: list of points in local coordinate system based on center of gravity
    """
    if scale:
        local_points = [scaled(pair) for pair in local_points]
    poly_bad = pymunk.Poly(None, local_points)
    t = poly_bad.center_of_gravity
    mass = poly_bad.area * density

    return t, [(x - t[0], y - t[1]) for (x, y) in local_points], mass


def make_found(myc_system_s, space):
    points = [[(0, 0), (200, 0), (200, 50), (0, 50)],
              [(0, 0), (200, 0), (200, 100), (0, 100)],
              [(0, 0), (200, 0), (200, 50), (0, 50)]]

    for pillar in range(3):
        position_s = [myc_system_s[pillar][0],
                      myc_system_s[pillar][1] - scaled(points[pillar][2][1] / 2)]
        # print(position_s)
        trans_vec, center_points, mass = make_points_from_points(points[pillar], scale=True)
        found = Brick(mass, center_points, position_s, friction_foundation,
                      color=(160, 160, 160, 225), type='s', radius=0)
        if pillar == 1:
            found = Brick(mass, center_points, position_s, friction_foundation,
                          color=(160, 160, 160, 225), type='d', radius=0)

        found.add_space(space)
        global foundation

        if pillar == 1:
            foundation = space.shapes[1]


def make_columns(myc_system_s, space):
    full_brick_dim = [(0, 0), (100, 0), (100, 30), (0, 30)]
    half_brick_dim = [(0, 0), (50, 0), (50, 30), (0, 30)]

    hexagon_brick_dim = [(0, 0), (200, 0), (200, 23.88), (106.78, 60.14), (93.22, 60.14), (0, 23.88)]
    trapeze_brick_dim = [(8.22, 0.00), (21.78, 0.00), (30.00, 19.85), (0.00, 19.85)]

    half_brick_trans_vec_s, half_brick_center_points_s, half_brick_mass = make_points_from_points(half_brick_dim,
                                                                                                  scale=True)
    full_brick_trans_vec_s, full_brick_center_points_s, full_brick_mass = make_points_from_points(full_brick_dim,
                                                                                                  scale=True)
    hex_brick_trans_vec_s, hex_brick_center_points_s, hex_brick_mass = make_points_from_points(hexagon_brick_dim,
                                                                                               scale=True)
    trap_brick_trans_vec_s, trap_brick_center_points_s, trap_brick_mass = make_points_from_points(trapeze_brick_dim,
                                                                                                  scale=True)

    for pillar in range(3):
        for layer in range(10):

            if layer % 2:

                # 1 _.
                position_s = [myc_system_s[pillar][0] - 0.5 * scaled(half_brick_dim[2][0] + full_brick_dim[2][0]),
                              myc_system_s[pillar][1] + (0.5 + layer) * scaled(half_brick_dim[2][1])]
                brick = Brick(half_brick_mass, half_brick_center_points_s, position_s, friction_column,
                              color=(102, 0, 0, 255), type='d')
                brick.add_space(space)
                # print(position_s)
                # 2 _.__.
                position_s = [myc_system_s[pillar][0],
                              myc_system_s[pillar][1] + (0.5 + layer) * scaled(half_brick_dim[2][1])]
                brick = Brick(full_brick_mass, full_brick_center_points_s, position_s, friction_column,
                              color=(255, 159, 128, 0), type='d')
                brick.add_space(space)
                # print(position_s)
                # 3 _.__._
                position_s = [myc_system_s[pillar][0] + 0.5 * scaled(half_brick_dim[2][0] + full_brick_dim[2][0]),
                              myc_system_s[pillar][1] + (0.5 + layer) * scaled(half_brick_dim[2][1])]
                brick = Brick(half_brick_mass, half_brick_center_points_s, position_s, friction_column,
                              color=(102, 0, 0, 255), type='d')
                brick.add_space(space)
                # print(position_s)

            else:
                # 1 __.
                position_s = [myc_system_s[pillar][0] - 0.5 * scaled(full_brick_dim[2][0]),
                              myc_system_s[pillar][1] + (0.5 + layer) * scaled(full_brick_dim[2][1])]
                brick = Brick(full_brick_mass, full_brick_center_points_s, position_s, friction_column,
                              color=(255, 77, 77, 255), type='d')
                brick.add_space(space)
                # 2 __.__
                position_s = [myc_system_s[pillar][0] + 0.5 * scaled(full_brick_dim[2][0]),
                              myc_system_s[pillar][1] + (0.5 + layer) * scaled(full_brick_dim[2][1])]
                brick = Brick(full_brick_mass, full_brick_center_points_s, position_s, friction_column,
                              color=(255, 26, 26, 255), type='d')
                brick.add_space(space)

        # hexagon bricks
        hex_brick_position_s = [myc_system_s[pillar][0],
                                myc_system_s[pillar][1] + 10 * scaled(full_brick_dim[2][1]) +
                                hex_brick_trans_vec_s[1]]

        brick = Brick(hex_brick_mass, hex_brick_center_points_s, hex_brick_position_s, friction_column,
                      color=(0, 0, 77, 255), type='d')
        brick.add_space(space)

        # trapeze bricks
        trap_brick_position_s = [myc_system_s[pillar][0],
                                 myc_system_s[pillar][1] + 10 * scaled(full_brick_dim[2][1]) +
                                 scaled(hexagon_brick_dim[3][1]) + trap_brick_trans_vec_s[1]]
        # print(trap_brick_trans_vec_s, trap_brick_position_s)
        brick = Brick(trap_brick_mass, trap_brick_center_points_s, trap_brick_position_s, friction_column,
                      color=(153, 153, 255, 255), type='d')
        brick.add_space(space)

def make_arches(myc_system_s, center_arches, space):

    arches1_brick_dim = [(1.09, 0), (26.84, 0), (27.93, 50), (0, 50)]
    arches2_brick_dim = [(1.09, 0), (29.02, 0), (30.11, 50), (0, 50)]

    acrh1_trans_vec_s, arch1_center_points_s, arch1_mass = make_points_from_points(arches1_brick_dim, scale=True)
    acrh2_trans_vec_s, arch2_center_points_s, arch2_mass = make_points_from_points(arches2_brick_dim, scale=True)

    arch1_radius_vector_s = Vec2d(0, scaled(589.98) + acrh1_trans_vec_s[1])
    arch2_radius_vector_s = Vec2d(0, scaled(589.98 + 50) + acrh2_trans_vec_s[1])

    # TOP BRICK IN ARCH

    for i in range(2):
        arches1_position_s = (center_arches[i][0] + arch1_radius_vector_s[0],
                              center_arches[i][1] + arch1_radius_vector_s[1])
        arches2_position_s = (center_arches[i][0] + arch2_radius_vector_s[0],
                              center_arches[i][1] + arch2_radius_vector_s[1])

        brick = Brick(arch1_mass, arch1_center_points_s, arches1_position_s, friction_arches,
                      color=(102, 0, 51, 255), type='d')
        brick.add_space(space)
        brick = Brick(arch2_mass, arch2_center_points_s, arches2_position_s, friction_arches,
                      color=(0, 51, 102, 255), type='d')
        brick.add_space(space)


    for side in [-1, 1]:

        arch1_radius_vector_s = Vec2d(0, scaled(589.98) + acrh1_trans_vec_s[1])
        arch2_radius_vector_s = Vec2d(0, scaled(589.98 + 50) + acrh2_trans_vec_s[1])

        arch1_radius_vector_s = arch1_radius_vector_s.rotated_degrees(side * 2.5)
        arch2_radius_vector_s = arch2_radius_vector_s.rotated_degrees(side * 2.5)

        for br in range(1, 28):

            arch_radius = math.radians(side * 2.5 * br)

            for j in range(2):

                arches1_position_s = (center_arches[j][0] + arch1_radius_vector_s[0],
                                      center_arches[j][1] + arch1_radius_vector_s[1])
                arches2_position_s = (center_arches[j][0] + arch2_radius_vector_s[0],
                                      center_arches[j][1] + arch2_radius_vector_s[1])

                if br % 2:
                    arches2_color = (0, 77, 153, 255)
                    arches1_color = (153, 0, 77, 255)
                else:
                    arches2_color = (0, 51, 102, 255)
                    arches1_color = (102, 0, 51, 255)

                brick = Brick(arch1_mass, arch1_center_points_s, arches1_position_s, friction_arches,
                              arches1_color, type='d', radius=arch_radius)
                brick.add_space(space)

                brick = Brick(arch2_mass, arch2_center_points_s, arches2_position_s, friction_arches,
                              arches2_color, type='d', radius=arch_radius)
                brick.add_space(space)




            arch1_radius_vector_s = arch1_radius_vector_s.rotated_degrees(side * 2.5)
            arch2_radius_vector_s = arch2_radius_vector_s.rotated_degrees(side * 2.5)


def make_arches2(myc_system_s, center_arches, space):

    # arch dimension as [arch1, arch2]
    arch_brick_dim = [[(1.09, 0), (26.84, 0), (27.93, 50), (0, 50)], [(1.09, 0), (29.02, 0), (30.11, 50), (0, 50)]]

    arch_center_points_s = [0, 0]
    arch_mass = [0, 0]
    arch_trans_vec = [0, 0]

    for arch in range(2):
        arch_trans_vec[arch], arch_center_points_s[arch], arch_mass[arch] = make_points_from_points(arch_brick_dim[arch],
                                                                                                    scale=True)
    arch_radius_vector = [Vec2d(0, scaled(589.98) + arch_trans_vec[0][1]),
                          Vec2d(0, scaled(589.98 + 50) + arch_trans_vec[1][1])]

    # TOP BRICK IN ARCH as [arch1, arch2]
    arch_color = ((102, 0, 51, 255), (0, 51, 102, 255))

    for j in range(2):
        for arch in range(2):
            arch_position_s = (center_arches[j][0] + arch_radius_vector[arch][0], center_arches[j][1] + arch_radius_vector[arch][1])

            brick = Brick(arch_mass[arch], arch_center_points_s[arch], arch_position_s, friction_arches,
                          arch_color[arch], type='d', radius = 0.00)
            brick.add_space(space)

    # radius vector rotations to the right and to the left
    arch_radius_vector_l = arch_radius_vector
    arch_radius_vector_l = [arch_radius_vector_l[0].rotated_degrees(-2.5), arch_radius_vector_l[1].rotated_degrees(-2.5)]

    arch_radius_vector_r = arch_radius_vector
    arch_radius_vector_r = [arch_radius_vector_r[0].rotated_degrees(2.5), arch_radius_vector_r[1].rotated_degrees(2.5)]

    for br in range(1, 28):

        arch_radius_l = math.radians(-2.5 * br)
        arch_radius_r = math.radians(2.5 * br)

        for j in range(2):

            for arch in range(2):

                arch_position_s_l = (center_arches[j][0] + arch_radius_vector_l[arch][0],
                                     center_arches[j][1] + arch_radius_vector_l[arch][1])
                arch_position_s_r = (center_arches[j][0] + arch_radius_vector_r[arch][0],
                                     center_arches[j][1] + arch_radius_vector_r[arch][1])

                if br % 2:
                    arch_color = [(153, 0, 77, 255), (0, 77, 153, 255)]
                else:
                    arch_color = [(102,102,102,255), (0, 51, 102, 255)]

                brick = Brick(arch_mass[arch], arch_center_points_s[arch], arch_position_s_l, friction_arches,
                              arch_color[arch], type='d', radius=arch_radius_l)
                brick.add_space(space)

                brick = Brick(arch_mass[arch], arch_center_points_s[arch], arch_position_s_r, friction_arches,
                              arch_color[arch], type='d', radius=arch_radius_r)
                brick.add_space(space)

        arch_radius_vector_l = [arch_radius_vector_l[0].rotated_degrees(-2.5),
                                arch_radius_vector_l[1].rotated_degrees(-2.5)]

        arch_radius_vector_r = [arch_radius_vector_r[0].rotated_degrees(2.5),
                                arch_radius_vector_r[1].rotated_degrees(2.5)]


def make_funny_wall(myc_system_s, center_arches, space):

    fun_wall_dim = [[[(0.00, 0.00), (30.08, 1.31), (0.00, 1.31)],
                     [(0.00, 0.00), (30.00, 2.62), (30.00, 3.94),  (0.00, 3.94)],
                     [(0.00, 0.00), (29.85, 3.93), (29.85, 7.87),  (0.00, 7.87)],
                     [(0.00, 0.00), (29.65, 5.23), (29.65, 13.10), (0.00, 13.10)],
                     [(0.00, 0.00), (29.40, 6.52), (29.40, 19.61), (0.00, 19.61)],
                     [(0.00, 0.00), (29.09, 7.79), (29.09, 27.41), (0.00, 27.41)],
                     [(0.00, 0.00), (98.65, 0.00), (106.88, 2.59), (106.88, 30.00), (0.00, 30.00)]],
                    [[(0.00, 0.00), (20.49, 6.46), (0.00, 6.46)],
                     [(0.00, 0.00), (28.30, 10.30), (28.30, 16.76), (0.00, 16.76)],
                     [(0.00, 0.00), (27.82, 11.52), (27.82, 28.28), (0.00, 28.28)],
                     [(0.00, 0.00), (68.36, 0.00), (72.05, 1.72), (72.05, 30.00), (0.00, 30.00)]],
                    [[(0.00, 0.00), (23.61, 11.01), (0.00, 11.01)],
                     [(0.00, 0.00), (26.71, 13.90), (26.71, 24.91), (0.00, 24.91)],
                     [(0.00, 0.00), (59.24, 0.00), (68.05, 5.09), (68.05, 30.00), (0.00, 30.00)]],
                    [[(0.00, 0.00), (17.27, 9.97), (0.00, 9.97)],
                     [(0.00, 0.00), (25.40, 16.18), (25.40, 26.15), (0.00, 26.15)],
                     [(0.00, 0.00), (61.07, 0.00), (66.58, 3.85), (66.58, 30.00), (0.00, 30.00)]],
                    [[(0.00, 0.00), (19.16, 13.42), (0.00, 13.42)],
                     [(0.00, 0.00), (70.30, 0.00), (91.91, 16.58), (91.91, 30.00), (0.00, 30.00)]],
                    [[(0.00, 0.00), (2.28, 1.75), (0.00, 1.75)],
                     [(0.00, 0.00), (23.07, 19.36), (23.07, 21.10), (0.00, 21.10)],
                     [(0.00, 0.00), (85.25, 0.00), (94.95, 8.90), (94.95, 30.00), (0.00, 30.00)]],
                    [[(0.00, 0.00), (12.49, 11.45), (0.00, 11.45)],
                     [(0.00, 0.00), (4.20, 0.00), (22.75, 18.55), (22.75, 30.00), (0.00, 30.00)]],
                    [[(0.00, 0.00), (2.74, 2.74), (0.00, 2.74)],
                     [(0.00, 0.00), (20.34, 22.20), (20.34, 24.94), (0.00, 24.94)],
                     [(0.00, 0.00), (26.87, 0.00), (31.12, 5.06), (31.12, 30.00), (0.00, 30.00)]],
                    [[(0.00, 0.00), (15.11, 18.00), (0.00, 18.00)],
                     [(0.00, 0.00), (52.56, 0.00), (61.77, 12.0), (61.77, 30.00), (0.00, 30.00)]],
                    [[(0.00, 0.00), (9.13, 11.89), (0.00, 11.89)],
                     [(0.00, 0.00), (80.76, 0.00), (93.43, 18.11), (93.43, 30.00), (0.00, 30.00)]],
                    [[(0.00, 0.00), (4.59, 6.56), (0.00, 6.56)],
                     [(0.00, 0.00), (11.23, 0.00), (26.16, 23.44), (26.16, 30.00), (0.00, 30.00)]],
                    [[(0.00, 0.00), (1.24, 1.95), (0.00, 1.95)],
                     [(0.00, 0.00), (15.06, 26.08), (15.06, 28.03), (0.00, 28.03)]],
                    [[(0.00, 0.00), (12.88, 24.74), (0.00, 24.74)]],
                    [[(0.00, 0.00), (10.27, 22.03), (0.00, 22.03)]]
                    ]

    fun_wall_position = [[(45.14, 688.67), (75.13, 686.05), (104.99, 682.12), (134.64, 676.89),
                          (164.04, 670.37), (193.12, 662.58), (300.00, 659.98)],
                         [(221.84, 653.52), (250.14, 643.22), (277.96, 631.70), (350.00, 629.98)],
                         [(305.24, 618.98), (331.95, 605.07), (400.00, 599.98)],
                         [(358.03, 590.02), (383.43, 573.84), (450.00, 569.98)],
                         [(408.09, 556.57), (500.00, 539.98)],
                         [(431.98, 538.24), (455.05, 518.88), (550.00, 509.98)],
                         [(477.25, 498.54), (500.00, 479.98)],
                         [(498.54, 477.25), (518.88, 455.05), (550.00, 449.98)],
                         [(538.24, 431.98), (600.00, 419.98)],
                         [(556.57, 408.09), (650.00, 389.98)],
                         [(573.84, 383.43), (600.00, 359.98)],
                         [(590.02, 358.03), (605.07, 331.95)],
                         [(618.98, 305.24)],
                         [(631.70, 277.96)]
                        ]

    colors = [[(174, 152, 152, 255), (203, 197, 182, 255)],
              [(203, 197, 119, 255), (169, 153, 99, 255)]]

    # print(len(fun_wall_position[0]))
    for line in range(len(fun_wall_position)):
        for br in range(len(fun_wall_position[line])):

            fun_wall_trans_vector_s, fun_wall_dim_center_s, fun_brick_mass = \
                make_points_from_points(fun_wall_dim[line][br], scale=True)

            for side in range(2):

                for mirror in [-1, 1]:

                    _position_s = [center_arches[side][0] + mirror * scaled(fun_wall_position[line][br][0]),
                                   center_arches[side][1] + scaled(fun_wall_position[line][br][1])]
                    # print(center_arches[side],fun_wall_position[0][br], _position_s, fun_brick_mass)

                    _fun_wall_position_s = [_position_s[0] - mirror * fun_wall_trans_vector_s[0],
                                            _position_s[1] + fun_wall_trans_vector_s[1]]

                    # print(br, _position_s, _fun_wall_position_s, _fun_wall_position_s)
                    dimension = fun_wall_dim_center_s
                    if mirror == 1:
                        dimension = mirror_points(fun_wall_dim_center_s)

                    brick = Brick(fun_brick_mass, dimension, _fun_wall_position_s, friction_arches,
                                  colors[line%2][br%2], type='d', radius=0.00)
                    brick.add_space(space)

    fun_wall_dim = [[(3.30, 0.00), (33.30, 0.00), (36.60, 7.97), (36.60, 30.00), (0.00, 30.00), (0.00, 7.97)],
                    [(2.45, 0.00), (59.60, 0.00), (62.05, 5.25), (62.05, 30.00), (0.00, 30.00), (0.00, 5.26)],
                    [(1.03, 0.00), (88.83, 0.00), (89.86, 1.97), (89.86, 30.00), (0.00, 30.00), (0.00, 1.97)]]
    pillar = 1
    for layer in range(len(fun_wall_dim)):

        fun_wall_trans_vector_s, fun_wall_dim_center_s, fun_brick_mass = \
                make_points_from_points(fun_wall_dim[layer], scale=True)

        position_s = [myc_system_s[pillar][0],
                      myc_system_s[pillar][1] + scaled(379.98) + (0.5 + layer) * scaled(30)]
        brick = Brick(fun_brick_mass, fun_wall_dim_center_s, position_s, friction_column,
                          color=colors[layer % 2][1], type='d')
        brick.add_space(space)

    fun_wall_dim = [[(0.00, 0.00), (15.00, 0.00), (18.30, 7.97), (18.30, 30.00), (0.00, 30.00)],
                    [(0.00, 0.00), (78.57, 0.00), (81.03, 5.26), (81.03, 30.00), (0.00, 30.00)],
                    [(0.00, 0.00), (43.90, 0.00), (44.93, 1.97), (44.93, 30.00), (0.00, 30.00)]]
    pillar = 0
    for side in [-1,1]:
        for layer in range(len(fun_wall_dim)):
            fun_wall_trans_vector_s, fun_wall_dim_center_s, fun_brick_mass = \
                make_points_from_points(fun_wall_dim[layer], scale=True)

            if side == 1:
                fun_wall_dim_center_s = mirror_points(fun_wall_dim_center_s)

            if layer%2:
                position_s = [myc_system_s[pillar][0] + side*(scaled(50) - fun_wall_trans_vector_s[0]),
                              myc_system_s[pillar][1] + scaled(379.98) + (0.5 + layer) * scaled(30)]
            else:

                position_s = [myc_system_s[pillar][0] - side * fun_wall_trans_vector_s[0],
                              myc_system_s[pillar][1] + scaled(379.98) + (0.5 + layer) * scaled(30)]

            brick = Brick(fun_brick_mass, fun_wall_dim_center_s, position_s, friction_column,
                          color=colors[layer % 2][1], type='d')
            brick.add_space(space)
        pillar = 2

    fun_wall_dim = [[(0.00, 0.00), (67.12, 0.00), (93.22, 10.15), (85.00, 30.00), (0.00, 30.00)],
                    [(0.00, 0.00), (67.12, 26.10), (0.00, 26.10)]]

    pillar = 0
    for side in [-1, 1]:
        for layer in range(len(fun_wall_dim)):

            fun_wall_trans_vector_s, fun_wall_dim_center_s, fun_brick_mass = \
                make_points_from_points(fun_wall_dim[layer], scale=True)

            if side == 1:
                fun_wall_dim_center_s = mirror_points(fun_wall_dim_center_s)

            position_s = [myc_system_s[pillar][0] + side*(scaled(100) - fun_wall_trans_vector_s[0]),
                          myc_system_s[pillar][1] + scaled(379.98) - 0.5 * layer * scaled(30) -fun_wall_trans_vector_s[1]]

            brick = Brick(fun_brick_mass, fun_wall_dim_center_s, position_s, friction_column,
                          color=colors[layer % 2][1], type='d')
            brick.add_space(space)
        pillar = 2



def make_normal_wall(myc_system_s, space):

    brick_amount = [None, None, None, [0], [0], [0], [-0.5, 0.5], [-1, 0, 1], [-0.5, 0.5],
                    [-1, 0, 1], [-1.5,-0.5, 0.5, 1.5], [-2, -1, 0, 1, 2], [-2.5, -1.5, -0.5, 0.5, 1.5, 2.5],
                    list(range(-3,4)), [a-0.5 for a in list(range(-5,8))],list(range(-6,7)),
                   [a-0.5 for a in list(range(-5,8))]]

    full_brick_dim = [(0, 0), (100, 0), (100, 30), (0, 30)]
    half_brick_dim = [(0, 0), (50, 0), (50, 30), (0, 30)]

    half_brick_trans_vec_s, half_brick_center_points_s, half_brick_mass = make_points_from_points(half_brick_dim,
                                                                                                  scale=True)
    full_brick_trans_vec_s, full_brick_center_points_s, full_brick_mass = make_points_from_points(full_brick_dim,
                                                                                                     scale=True)


    for pillar in range(3):
        for layer in range(3, len(brick_amount)):

            key = brick_amount[layer]
            colors = [[(153, 102, 0, 255), (204, 153, 0, 255)],
                      [(153, 51, 51, 255), (153, 0, 51, 255)]]

            if pillar == 0:
                key = [i for i in key if i > -1]
                colors = [[(153, 102, 0, 255), (204, 153, 0, 255)],
                          [(153, 0, 51, 255), (153, 51, 51, 255)]]
            if pillar == 2:
                key = [i for i in key if i < 1]
                colors = [[(204, 153, 0, 255), (153, 102, 0, 255)],
                          [(153, 0, 51, 255), (153, 51, 51, 255)]]

            if layer == 4:
                continue

            else:
                for br in range(len(key)):

                    position_s = [myc_system_s[pillar][0] + key[br] * scaled(full_brick_dim[2][0]),
                                  myc_system_s[pillar][1] + scaled(379.98) + (0.5 + layer) * scaled(full_brick_dim[2][1])]
                    brick = Brick(full_brick_mass, full_brick_center_points_s, position_s, friction_column,
                              color=colors[layer % 2][br % 2], type='d')
                    brick.add_space(space)

    for pillar in [0,2]:
        for layer in list(range(1, 16, 2)):
            position_s = [myc_system_s[pillar][0] + (pillar%3 - 1)*0.75 * scaled(full_brick_dim[2][0]),
                          myc_system_s[pillar][1] + scaled(379.98) + (0.5 + layer) * scaled(full_brick_dim[2][1])]
            # print(layer, position_s)
            brick = Brick(half_brick_mass, half_brick_center_points_s, position_s, friction_column,
                          color=(102,102,102,255), type='d')
            brick.add_space(space)

        for layer in [0,2,4]:
            position_s = [myc_system_s[pillar][0] + (pillar % 3 - 1) * 0.5 * scaled(full_brick_dim[2][0]),
                          myc_system_s[pillar][1] + scaled(379.98) + (0.5 + layer) * scaled(full_brick_dim[2][1])]

            brick = Brick(full_brick_mass, full_brick_center_points_s, position_s, friction_column,
                          color=(153, 102, 0, 255), type='d')
            brick.add_space(space)


def make_buttresses(myc_system_s, space):
    buttresses_dim = [(0.00, 0.00), (200.00, 0.00), (200.00, 889.98), (50.00, 889.98)]

    butt_trans_vector_s, butt_dim_center_s, butt_mass = \
        make_points_from_points(buttresses_dim, scale=True)

    pillar = 0
    for side in [-1, 1]:

        position_s = [myc_system_s[pillar][0] + side * (scaled(100 + buttresses_dim[2][0]) - butt_trans_vector_s[0]),
                      myc_system_s[pillar][1] + butt_trans_vector_s[1]]

        brick = Brick(butt_mass, butt_dim_center_s, position_s, friction_column,
                      color=(220,220,220,255), type='s')

        brick.add_space(space)

        butt_dim_center_s = mirror_points(butt_dim_center_s)
        pillar = 2



class Main(pyglet.window.Window):
    def __init__(self, width, heigth):

        pyglet.window.Window.__init__(self, width, heigth, vsync=True)

        self.set_caption("Portal")
        pyglet.clock.schedule_interval(self.update, 0.005 / 60.0)
        self.fps_display = pyglet.window.FPSDisplay(self)
        self.create_world()
        self.draw_options = pymunk.pyglet_util.DrawOptions()
        self.draw_options.flags = self.draw_options.DRAW_SHAPES

    def create_world(self):

        self.space = pymunk.Space()
        self.space.gravity = Vec2d(0.0, scaled(-981) * 0.1)

        # The coordinate system 0 -> beginning at the first pillar
        # my_coord_0 = (x_start, y_start)

        pillar_2_position = scaled((1625, 400))
        # pillar_2_position = scaled((800, -670))

        # COORDINATE SYSTEM
        # Create local coordinates system for each pillar, _s - means just scaled dimensions
        # myc_system = [(x_0, y_0), (x_1, y_1), (x_2, y_2)

        myc_system_s = []
        for i in [-1, 0, 1]:
            myc_system_s.append((pillar_2_position[0] + i * scaled(1300),
                                 pillar_2_position[1]))
        # print(myc_system_s)

        center_arches_s = [[myc_system_s[1][0] - scaled(650), myc_system_s[1][1] + scaled(110)],
                           [myc_system_s[1][0] + scaled(650), myc_system_s[1][1] + scaled(110)]]

        # SETTING FOUNDATIONS
        # foundation's dimensions as  found_dim = [(length_0, height_0), (length_1, height_1), (length_2, height_2)]
        # where  _1 is number of pillar (or local coor dinate system)

        make_found(myc_system_s, self.space)

        # MAKE COLUMNS
        make_columns(myc_system_s, self.space)

        # MAKE ARCHES
        make_arches2(myc_system_s, center_arches_s, self.space)

        # MAKE FUNNY WALL
        make_funny_wall(myc_system_s, center_arches_s, self.space)

        # MAKE NORMAL WALL
        make_normal_wall(myc_system_s, self.space)

        # MAKE BUTTRESSES
        make_buttresses(myc_system_s, self.space)


    def update(self, dt):
        # Here we use a very basic way to keep a set space.step dt.
        # For a real game its probably best to do something more complicated.
        step_dt = 1 / 500.0
        x = 0

        global iter_dt

        while x < dt:
            x += step_dt
            self.space.step(step_dt)
            foundation.body.velocity=(0,-velocity_UP[iter_dt])
            iter_dt = iter_dt + 1
            print(iter_dt, -velocity_UP[iter_dt])

    def on_draw(self):
        self.clear()
        # self.text.draw()
        self.fps_display.draw()
        self.space.debug_draw(self.draw_options)


if __name__ == "__main__":
    friction_foundation = 0.5
    friction_column = 0.4
    friction_arches = 0.4
    number_of_pillars = 3
    scale = 0.7
    density = 2.75 / 10
    foundation = None
    iter_dt = 0

    scale = 0.4

    width = 1300
    height = 750

    main = Main(width, height)
    pyglet.app.run()
