import math
import numpy as np
import matplotlib.pyplot as plt
import pyglet
import pymunk
from pymunk.pyglet_util import DrawOptions

import analitically

"""
czas analityczny i z symualcji nie będzie się zgadzał 1:1 ponieważ ostatni czas zarejetsrowany przez program jest 
w chwili ostatniego kontaktu boxa i slope w danym przedziale czasu! dopiero znika w kolejnym przedziel czasu
"""

# create window to display  (width = 1200, height = 600)
window = pyglet.window.Window(1200, 600, "Pymunk Tester", resizable=False)
options = DrawOptions()

# ============== DATA ==================
# units:
#  - length - [m]
#  - mass - [kg]
#  - time - [s]
# gravity = 9.81 [m/s^2]

# dimensions

# ============ SETTINGS ===================
SlopeLength = 1000 #1000 cm = 10 m
SlopeHeight = 100 # 100 cm = 1 m
boxFriction = 0.05  # SlopeHeight/SlopeLength * 0.3  #* 0.4 #0.085
# angle = 20 #degrees
# SlopeHeight = SlopeLength * math.tan(math.degrees(angle))

# =========================================

BoxSize = 15 # 15 cm
BoxMass = 0.5
gravity = 981

# the angle of the slope
alfa = math.atan(SlopeHeight / SlopeLength)

PolyShapeHeight = SlopeHeight + BoxSize * math.sin(alfa)
PolyShapeLength = SlopeLength + BoxSize * math.cos(alfa)

# frictions
# - is calculated as multiplication of the friction coefficient of slope and the friction coefficient of box
# - default value (!!!) slope_friction = 1.0
slopeFriction = 1.0
# box_friction = 0.2  # SlopeHeight/SlopeLength * 0.3  #* 0.4 #0.085

# position of the slope on the screen (0,0)
slopePosition_x, slopePosition_y = 100 - BoxSize * math.cos(alfa), 100
space = pymunk.Space()

# gravity = 981 [cm/s^2]
space.gravity = 0, - gravity


def make_slope(slopeFriction=1, y_translation_position=0):
    """
    it creates the slope
    :param slopeFriction: set by default as 1.0 (don't change!!)
    :param y_translation_position: set only when is more than one slope displayed on the screen
    :return:
    """
    triangle_shape = pymunk.Poly(None, ((0, 0), (PolyShapeLength, 0), (0, PolyShapeHeight)))
    triangle_moment = pymunk.moment_for_poly(BoxMass, triangle_shape.get_vertices())
    triangle_body = pymunk.Body(BoxMass, triangle_moment, body_type=pymunk.Body.STATIC)
    triangle_body.position = slopePosition_x, slopePosition_y + y_translation_position
    triangle_shape.body = triangle_body
    triangle_shape.friction = slopeFriction
    space.add(triangle_body, triangle_shape)


def transformation(coordinatesByDefault):
    """
    By default center of box is set on point (slope_position_x, slope_position_y + slopeHeight), this function
    returns the coordinates of the position the center of the block so that it lies at the very beginning of the plane
    (without penetration of the block and the slope)
    :param coordinatesByDefault: (x, y) - the position of the center of the box
    :return: coordinates of the center of the box after transformation
    """
    x = coordinatesByDefault[0] - BoxSize / 2 * math.sqrt(2) * math.cos(math.radians(45) + alfa)
    y = coordinatesByDefault[1] + BoxSize / 2 * math.sqrt(2) * math.sin(math.radians(45) + alfa)
    return x, y


def make_box(boxFriction):
    """
    it creates box
    :param boxFriction: change here, not in make_slope
    :return:
    """
    poly_shape = pymunk.Poly.create_box(None, size=(BoxSize, BoxSize))
    poly_moment = pymunk.moment_for_poly(BoxMass, poly_shape.get_vertices())
    poly_body = pymunk.Body(BoxMass, poly_moment, body_type=pymunk.Body.DYNAMIC)
    poly_shape.body = poly_body
    poly_shape.body.angle = - math.atan(SlopeHeight / SlopeLength)
    poly_body.position = start_slope_global
    poly_shape.friction = boxFriction
    space.add(poly_body, poly_shape)


# DRAWING
@window.event
def on_draw():
    window.clear()
    space.debug_draw(options)
    label2.draw()
    label1.draw()


def update(dt):
#def benchmark(dt):
    global dataFromSimulation
    space.step(dt)  # 1/50 frames per second
    try:
        box = space.shapes[1]

        # When box is still on the slope, it's position and time interval is added to array
        # [actual_x_coordinate, actual_y_coordinate, x_velocity, y_velocity time_interval]
        # else box is out of the slope shape, body is deleted,
        # time_stop keeps the time when the box is off the slope for the first time

        if box.body.position.y >= end_slope_global[1] or box.body.position.x <= end_slope_global[0]:
            # slopePosition_x, slopePosition_y = 100 - BoxSize * math.cos(alfa), 100

            # punkt (0,0) drogi znajduje się w środku ciała po transformacji = transformation(100,200)
            dataFromSimulation = np.append(dataFromSimulation, np.array([
                [box.body.position.x - start_slope_global[0],
                 box.body.position.y - start_slope_global[1],
                 box.body.velocity.x,
                 box.body.velocity.y,
                 dt]]), axis=0)
        else:

            """dataFromSimulation = np.append(dataFromSimulation, np.array([
                [box.body.position.x - start_slope_global[0],
                 box.body.position.y - start_slope_global[1],
                 box.body.velocity.x,
                 box.body.velocity.y,
                 dt]]), axis=0)"""

            space.remove(box.body, box)
    except IndexError:
        pass







def make_data_to_graph(dataFromSimulation):
    """

    :param
    [actual_x_coordinate, actual_y_coordinate, x_velocity, y_velocity time_interval]
    :return: array with data: [distance, velocity, time]
    """

    # [0] sx_i, [1] sy_i, [2] t
    dataToGraph = np.empty((0, 5), float)
    dataToGraph = np.append(dataToGraph, np.array([[0, 0, 0, 0, 0]]), axis=0)

    for i in range(1, np.shape(dataFromSimulation)[0]):

        # total distance in i_time = sum(dt_i)
        sx_i = dataFromSimulation[i, 0]
        sy_i = dataFromSimulation[i, 1]
        s_i = math.sqrt(sx_i ** 2 + sy_i ** 2)

        """# distance increase in t_i time step
        dsx_i = dataFromSimulation[i, 0] - dataFromSimulation[i - 1, 0]
        dsy_i = dataFromSimulation[i, 1] - dataFromSimulation[i - 1, 1]
        ds_i = math.sqrt(dsx_i ** 2 + dsy_i ** 2)"""

        # time as sum time steps to i-moment
        t_i = dataToGraph[i - 1, 1] + dataFromSimulation[i, 4]

        # velocity at i-moment
        vx_i = dataFromSimulation[i, 2]
        vy_i = dataFromSimulation[i, 3]
        v_i = math.sqrt(vx_i ** 2 + vy_i ** 2)

        # velocity increase in t_i time step
        dvx_i = dataFromSimulation[i, 2] - dataFromSimulation[i - 1, 2]
        dvy_i = dataFromSimulation[i, 3] - dataFromSimulation[i - 1, 3]
        dv_i = math.sqrt(dvx_i ** 2 + dvy_i ** 2)

        # time increase in i-step - tick clock
        dt_i = dataFromSimulation[i, 4]

        if dt_i == 0:
            a_i = 0
        else:
            a_i = dv_i / dt_i

        # s_i - total distance to t_i time step
        # t_i - total time as sum(t_i) time step
        # v_i - velocity at t_i time step
        # a_i - acceleration at t_i time step
        # dt_i - value of every t_i time step
        dataToGraph = np.append(dataToGraph, np.array([[s_i, t_i, v_i, a_i, dt_i]]), axis=0)
    print('-------------------------------------------')
    print('Solution from simulation: ')
    print('t = \t', dataToGraph[-1, 1], '\t [s]')
    print('s = \t', dataToGraph[-1, 0], '\t [m]')
    print('a = \t', dataToGraph[-1, 3], '\t [m/s^2]')
    print('vk = \t', dataToGraph[-1, 2], '\t [m/s]')

    return dataToGraph


def calculate_analitically():
    # calculate time and acceleration using class SlopeBox methods
    analiza = analitically.SlopeBox(SlopeHeight, SlopeLength, boxFriction, gravity)
    totalTime = analiza.time_calc()
    acceleration = analiza.acceleration_calc()

    # create time from 0 to totalTime with 600 intervals
    time_steps = np.linspace(0, totalTime, 600)

    acceleration_steps = np.linspace(0, 0, 600) + acceleration

    # calculate a distance at all time_steps as s = 0.5 * a * t^2
    dist_func = acceleration * time_steps ** 2 / 2

    # calculate a velocity at all time_steps as v =  a * t
    velocity_func = acceleration * time_steps

    print('-------------------------------------------')
    print('Solution from calculation: ')
    print('t = \t', totalTime, '\t [s]')
    print('s = \t', dist_func[-1], '\t [m]')
    print('a = \t', acceleration, '\t [m/s^2]')
    print('vk = \t', velocity_func[-1], '\t [m/s]')
    print('-------------------------------------------')

    return np.array([dist_func, velocity_func, time_steps, acceleration_steps]).T


if __name__ == '__main__':

    # calculate the start position _local before transformation
    start_slope_local = (slopePosition_x + BoxSize * math.cos(alfa), slopePosition_y + SlopeHeight)
    start_slope_global = transformation(start_slope_local)

    # calculate the end position _local before transformation
    end_slope_local = (slopePosition_x + PolyShapeLength, slopePosition_y)
    end_slope_global = transformation(end_slope_local)

    # make slope an box and set theirs parameters
    make_slope()
    make_box(boxFriction)

    # create an array with shape (0 x 5)  to store data from simulation as:
    # [actual_x_coordinate, actual_y_coordinate, x_velocity, y_velocity time_interval]
    # and add first data of start point:
    # [0] actual_x_coordinate = set the box on the top of slope and make transformation of position of the box;
    # [1] actual_x_coordinate = set the box on the top of slope and make transformation of position of the box;
    # [2] x_velocity = 0;
    # [3] y_velocity = 0,
    # [4] time_interval (tick clock) = 0

    dataFromSimulation = np.empty((0, 5), float)
    dataFromSimulation = np.append(dataFromSimulation, np.array([[0, 0, 0, 0, 0]]), axis=0)

    # create a label to show actual coefficient of friction
    label2 = pyglet.text.Label('ni = ' + str(boxFriction),
                               font_name='Arial',
                               font_size=20,
                               x=600, y=500,
                               anchor_x='center', anchor_y='center')

    label1 = pyglet.text.Label('tg(alfa) = ' + str(math.tan(alfa)),
                               font_name='Arial',
                               font_size=20,
                               x=600, y=450,
                               anchor_x='center', anchor_y='center')

    # pyglet update method 0.005/ 60 times in second
    pyglet.clock.schedule_interval(update, 0.0005 / 60)
    #pyglet.clock.schedule(benchmark)

    pyglet.app.run()

    arrGraphData_simu = make_data_to_graph(dataFromSimulation)
    arrGraphData_anal = calculate_analitically()
    """
    print(['x', 'y', 't'])
    print(dataFromSimulation[:20, 0:2])
    print('.......................')
    print(['vx', 'vy', 'dt_i'])
    print(dataFromSimulation[:20, 2:])
    print('.......................')
    # print(arrGraphData_simu[:50, 1:])

    print(['s', 't'])
    print(arrGraphData_simu[:20, 0:2])
    print('.......................')
    print(['v', 'a', 'dt_i'])
    print(arrGraphData_simu[:20, 2:])
    print('.......................')
    # print(arrGraphData_simu[:50, 1:])
    """
    # (t, s) -> (1, 0)
    plt.plot(arrGraphData_simu[:, 1], arrGraphData_simu[:, 0], color="red", label='from simulation')
    # (t, s) -> (2, 0)
    plt.plot(arrGraphData_anal[:, 2], arrGraphData_anal[:, 0], 'b--', label='from calculation')
    plt.title('DISTANCE - TIME')
    # plt.xlabel('time [s]')
    plt.ylabel('distance [meters]')
    plt.legend()
    plt.xlim(0)
    plt.ylim(0)
    plt.show()

    # (t, v) -> (1, 2)
    plt.plot(arrGraphData_simu[:, 1], arrGraphData_simu[:, 2], color="red", label='from simulation')
    # (t, v) -> (2, 1)
    plt.plot(arrGraphData_anal[:, 2], arrGraphData_anal[:, 1], 'b--', label='from calculation')
    plt.title('VELOCITY - TIME')
    plt.xlabel('time [s]')
    plt.ylabel('velocity [m/s]')
    plt.legend()
    plt.xlim(0)
    plt.ylim(0)
    plt.show()

    # (t, a) -> (1, 3)
    plt.plot(arrGraphData_simu[:, 1], arrGraphData_simu[:, 3], color="red", label='from simulation')
    # (t, a) -> (2, 3)
    plt.plot(arrGraphData_anal[:, 2], arrGraphData_anal[:, 3], 'b--', label='from calculation')

    plt.title('ACCELERATION - TIME')
    plt.xlabel('time [s]')
    plt.ylabel('acceleration [m/s^2]')
    plt.legend()
    plt.xlim(0)
    plt.ylim(0)

    plt.show()




'''
    makeAnalytically = True
    if makeAnalytically:
        arrGraphData_anal = calculate_analitically()

        # arrGraphData_anal[[]]:
        # s -> 0
        # v -> 1
        # t -> 2
        # a -> 3

        # (t, s) -> (2, 0)
        plt.subplot(1, 3, 1)
        plt.plot(arrGraphData_anal[:, 2], arrGraphData_anal[:, 0], color='blue', label='from calculation')

        # (t, v) -> (2, 1)
        plt.subplot(1, 3, 2)
        plt.plot(arrGraphData_anal[:, 2], arrGraphData_anal[:, 1], color='blue', label='from calculation')

        # (t, a) -> (2, 3)
        plt.subplot(1, 3, 3)
        plt.plot(arrGraphData_anal[:, 2], arrGraphData_anal[:, 3], color='blue', label='from calculation')


    # arrGraphData_simu[[]]:
    # s -> 0
    # t -> 1
    # v -> 2
    # a -> 3
    # dt -> 4 - unused

    # (t, s) -> (1, 0)
    plt.subplot(1, 3, 1)
    plt.plot(arrGraphData_simu[:, 1], arrGraphData_simu[:, 0], color="red", label='from simulation')
    plt.title('DISTANCE - TIME')
    # plt.xlabel('time [s]')
    plt.ylabel('distance [meters]')
    plt.legend()
    plt.xlim(0)
    plt.ylim(0)

    # (t, v) -> (1, 2)
    plt.subplot(1, 3, 2)
    plt.plot(arrGraphData_simu[:, 1], arrGraphData_simu[:, 2], color="red", label='from simulation')
    plt.title('VELOCITY - TIME')
    plt.xlabel('time [s]')
    plt.ylabel('velocity [m/s]')
    plt.legend()
    plt.xlim(0)
    plt.ylim(0)

    # (t, a) -> (1, 3)
    plt.subplot(1, 3, 3)
    plt.plot(arrGraphData_simu[:, 1], arrGraphData_simu[:, 3], color="red", label='from simulation')
    plt.title('ACCELERATION - TIME')
    plt.xlabel('time [s]')
    plt.ylabel('acceleration [m/s^2]')
    plt.legend()
    plt.xlim(0)
    plt.ylim(0)

    plt.show()
'''