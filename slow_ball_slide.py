import math
from time import time
import numpy as np
import matplotlib.pyplot as plt
import pyglet
import pymunk
from pymunk.pyglet_util import DrawOptions
import analitically

"""
czas analityczny i z symualcji nie będzie się zgadzał 1:1 ponieważ ostatni czas zarejetsrowany przez program jest 
w chwili ostatniego kontaktu boxa i slope w danym przedziale czasu! jdopiero znika w kolejnym przedziel czasu
"""

# create window to display  (width = 1200, height = 600)
window = pyglet.window.Window(1200, 600, "Pymunk Tester", resizable=False)
options = DrawOptions()

# ============== DATA ==================
# units:
#  - length - [cm]
#  - mass - [kg]
#  - time - [s]
# gravity = 981 [cm/s^2]

# ============ SETTINGS ===================
SlopeLength = 1000
SlopeHeight = 100

frictionNoSlip = SlopeHeight / SlopeLength /3 + 0.1
frictionWithSlip = SlopeHeight / SlopeLength /3 - 0.01
rollerFriction = frictionWithSlip

#angle = 30 #degrees
#SlopeHeight = SlopeLength * math.tan(math.degrees(angle))

# =========================================

rollerRadius = 10
rollerMass = 0.3
gravity = 981

# the angle of the slope
alfa = math.atan(SlopeHeight / SlopeLength)

PolyShapeHeight = SlopeHeight + rollerRadius * math.tan(alfa)
PolyShapeLength = SlopeLength + rollerRadius

# frictions
# - is calculated as multiplication of the friction coefficient of slope and the friction coefficient of box
# - default value (!!!) slope_friction = 1.0
slopeFriction = 1.0



# position of the slope on the screen (0,0)
slopePosition_x, slopePosition_y = 100 - rollerRadius, 100
space = pymunk.Space()

# gravity = 981 [cm/s^2]
space.gravity = 0, - gravity

def make_slope(slopeFriction=1):
    """
    it creates the slope
    :param slopeFriction: set by default as 1.0 (don't change!!)
    :param y_translation_position: set only when is more than one slope displayed on the screen
    :return:
    """
    triangle_shape = pymunk.Poly(None, ((0, 0), (PolyShapeLength, 0), (0, PolyShapeHeight)))
    triangle_moment = pymunk.moment_for_poly(1, triangle_shape.get_vertices())
    triangle_body = pymunk.Body(1, triangle_moment, body_type=pymunk.Body.STATIC)
    triangle_body.position = slopePosition_x , slopePosition_y
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
    x = coordinatesByDefault[0] #- roller_radius * math.sin(alfa)
    y = coordinatesByDefault[1] + rollerRadius + rollerRadius * math.tan(alfa) * math.sin(alfa) * 0.5

    return x, y


def make_roller(rollerFriction):
    poly_shape = pymunk.Circle(None,rollerRadius,(0,0))
    poly_moment = pymunk.moment_for_circle(rollerMass, 0, rollerRadius)
    poly_body = pymunk.Body(rollerMass, poly_moment, body_type=pymunk.Body.DYNAMIC)
    poly_shape.body = poly_body
    poly_body.position = start_slope_global
    poly_shape.friction = rollerFriction
    space.add(poly_body, poly_shape)



# DRAWING
@window.event
def on_draw():
    window.clear()
    space.debug_draw(options)
    label2.draw()
    label1.draw()


#def update(dt):
def benchmark(dt):
    global dataFromSimulation
    space.step(dt)  # 1/50 frames per second

    try:
        roller = space.shapes[1]

        # When box is still on the slope, it's position and time interval is added to array
        # [actual_x_coordinate, actual_y_coordinate, x_velocity, y_velocity time_interval]
        # else box is out of the slope shape, body is deleted,
        # time_stop keeps the time when the box is off the slope for the first time
        if roller.body.position.x <= end_slope_global[0]:
        #if roller.body.position.y >= end_slope_global[1] or roller.body.position.x <= end_slope_global[0]:
            dataFromSimulation = np.append(dataFromSimulation, np.array([
                [roller.body.position.x - start_slope_global[0],
                 roller.body.position.y - start_slope_global[1],
                 roller.body.velocity.x,
                 roller.body.velocity.y,
                 roller.body.angular_velocity,
                 dt]]), axis=0)


        else:
            space.remove(roller.body, roller)
    except IndexError:
        pass



def make_data_to_graph(dataFromSimulation):
    """

    :param arrDataFromAnalyse: array with data as:
    [actual_x_coordinate, actual_y_coordinate, x_velocity, y_velocity time_interval]
    :return: array with data: [distance, velocity, time]
    """

    dataToGraph = np.empty((0, 7), float)
    dataToGraph = np.append(dataToGraph,np.array([[0, 0, 0, 0, 0, 0, 0]]), axis=0)

    for i in range (1, np.shape(dataFromSimulation)[0]):
        # total distance in i_time = sum(dt_i)
        sx_i = dataFromSimulation[i, 0] - dataFromSimulation[0, 0]
        sy_i = dataFromSimulation[i, 1] - dataFromSimulation[0, 1]
        s_i = math.sqrt(sx_i ** 2 + sy_i ** 2)

        # time as sum time steps to i-moment
        t_i = dataToGraph[i - 1, 1] + dataFromSimulation[i, 5]

        # velocity at i-moment
        vx_i = dataFromSimulation[i, 2]
        vy_i = dataFromSimulation[i, 3]
        v_i = math.sqrt(vx_i ** 2 + vy_i ** 2)

        # angular velocity at i-moment
        w_i = dataFromSimulation[i, 4]

        # angular velocity increase in t_i time step
        dw_i = dataFromSimulation[i, 4] - dataFromSimulation[i - 1, 4]

        # velocity increase in t_i time step
        dvx_i = dataFromSimulation[i, 2] - dataFromSimulation[i - 1, 2]
        dvy_i = dataFromSimulation[i, 3] - dataFromSimulation[i - 1, 3]
        dv_i = math.sqrt(dvx_i ** 2 + dvy_i ** 2)

        # time increase in i-step - tick clock
        dt_i = dataFromSimulation[i, 5]

        if dt_i == 0:
            a_i = 0
            e_i = 0
        else:
            a_i = dv_i / dt_i
            e_i = dw_i / dt_i

        dataToGraph = np.append(dataToGraph, np.array([[s_i, t_i, v_i, a_i, w_i, e_i, dt_i]]), axis=0)

    # s_i - total distance to t_i time step
    # t_i - total time as sum(t_i) time step
    # v_i - velocity at t_i time step
    # a_i - acceleration at t_i time step
    # w_i - angular velocity at t_i time step
    # e_i - angular acceleration at t_i time step
    # dt_i - value of every t_i time step

    print('-------------------------------------------')
    print('Solution from simulation: ')
    print('t = \t', dataToGraph[-1, 1], '\t [s]')
    print('s = \t', dataToGraph[-1, 0], '\t [m]')
    print('a = \t', dataToGraph[-1, 3], '\t [m/s^2]')
    print('vk = \t', dataToGraph[-1, 2], '\t [m/s]')
    print('e = \t', dataToGraph[-1, 5], '\t [1/s^2]')
    print('wk = \t', dataToGraph[-1, 4], '\t [1/s]')

    return dataToGraph

def calculate_analitically():

    # calculate time and acceleration using class SlopeBox methods
    analiza = analitically.SlopeRoller(SlopeHeight, SlopeLength, rollerFriction, gravity, rollerRadius, rollerMass)
    totalTime = analiza.time_calc()
    acceleration = analiza.acceleration_calc()
    angularAcceleration =  analiza.angular_acceleration_calc()

    # create time from 0 to totalTime with 600 intervals
    time_steps = np.linspace(0, totalTime, 1200)

    acceleration_steps = np.linspace(0, 0, 1200) + acceleration
    angularAcceleration_steps = np.linspace(0, 0, 1200) + angularAcceleration

    # calculate a distance at all time_steps as s = 0.5 * a * t^2
    dist_func = acceleration * time_steps ** 2 / 2

    # calculate a velocity at all time_steps as v =  a * t
    velocity_func = acceleration * time_steps
    angular_velocity_func = angularAcceleration * time_steps

    print('-------------------------------------------')
    print('Solution from calculation: ')
    print('t = \t', totalTime, '\t [s]')
    print('s = \t', dist_func[-1], '\t [m]')
    print('a = \t', acceleration, '\t [m/s^2]')
    print('vk = \t', velocity_func[-1], '\t [m/s]')
    print('e = \t', angularAcceleration, '\t [m/s^2]')
    print('wk = \t', angular_velocity_func[-1], '\t [m/s]')
    print('-------------------------------------------')


    return np.array([dist_func, time_steps,
                     velocity_func, angular_velocity_func,
                     acceleration_steps, angularAcceleration_steps]).T




if __name__ == '__main__':

    # calculate the start position _local before transformation
    start_slope_local = (slopePosition_x + rollerRadius, slopePosition_y + SlopeHeight)
    start_slope_global = transformation(start_slope_local)

    # calculate the end position _local before transformation
    end_slope_local = (slopePosition_x + PolyShapeLength, slopePosition_y)
    end_slope_global = transformation(end_slope_local)

    # make slope an box and set theirs parameters

    make_slope()
    make_roller(rollerFriction)

    # create an array with shape (0 x 5)  to store data from simulation as:
    # [actual_x_coordinate, actual_y_coordinate, x_velocity, y_velocity time_interval]
    # and add first data of start point:
    # [0] actual_x_coordinate = set the box on the top of slope and make transformation of position of the box;
    # [1] actual_x_coordinate = set the box on the top of slope and make transformation of position of the box;
    # [2] x_velocity = 0;
    # [3] y_velocity = 0,
    # [4] angular_velocity = 0,
    # [5] time_interval (tick clock) = 0

    dataFromSimulation = np.empty((0, 6), float)
    dataFromSimulation = np.append(dataFromSimulation, np.array([[0, 0, 0, 0, 0, 0]]), axis=0)

    # create a label to show actual coefficient of friction
    label2 = pyglet.text.Label('ni = ' + str(rollerFriction),
                               font_name='Arial',
                               font_size=20,
                               x=600, y=500,
                               anchor_x='center', anchor_y='center')

    label1 = pyglet.text.Label('tg(alfa) = ' + str(math.tan(alfa)),
                               font_name='Arial',
                               font_size=20,
                               x=600, y=450,
                               anchor_x='center', anchor_y='center')


    # let' start the simualtion
    pyglet.clock.schedule(benchmark)
    #pyglet.clock.schedule_interval(update, 0.0005 / 60)
    pyglet.app.run()

    # arrGraphData_simu[[]]:
    # s -> 0
    # t -> 1
    # v -> 2
    # a -> 3
    # w -> 4
    # e -> 5
    # dt -> 6 useless
    arrGraphData_simu = make_data_to_graph(dataFromSimulation)

    # arrGraphData_anal[[]]:
    # s -> 0
    # t -> 1
    # v -> 2
    # w -> 3
    # a -> 4
    # e -> 5
    arrGraphData_anal = calculate_analitically()


    # plotting

    # (t,s) -> (1, 0)
    plt.plot(arrGraphData_simu[:, 1], arrGraphData_simu[:, 0], color="blue", label='from simulation')
    # (t,s) -> (1, 0)
    plt.plot(arrGraphData_anal[:, 1], arrGraphData_anal[:, 0], 'r--', label = 'from calculation')
    plt.title('DISTANCE - TIME')
    plt.xlabel('time [s]')
    plt.ylabel('distance [m]')
    plt.legend()
    plt.xlim(0)
    plt.ylim(0)
    plt.show()

    # (t,v) -> (1, 2)
    plt.plot(arrGraphData_simu[:, 1], arrGraphData_simu[:, 2], color="blue", label='from simulation')
    # (t,v) -> (1, 2)
    plt.plot(arrGraphData_anal[:, 1], arrGraphData_anal[:, 2], 'r--' , label = 'from calculation')
    plt.title('VELOCITY - TIME')
    plt.xlabel('time [s]')
    plt.ylabel('velocity [m/s]')
    plt.legend()
    plt.xlim(0)
    plt.ylim(0)
    plt.show()

    # (t,w) -> (1, 4)
    plt.plot(arrGraphData_simu[:, 1], arrGraphData_simu[:, 4], color="blue", label='from simulation')
    # (t,w) -> (1, 3)
    plt.plot(arrGraphData_anal[:, 1], arrGraphData_anal[:, 3], 'r--', label='from calculation')
    plt.title('ANGULAR VELOCITY - TIME')
    plt.xlabel('time [s]')
    plt.ylabel('angular velocity [1/s]')
    plt.legend()
    plt.xlim(0)
    #plt.ylim(0)
    plt.show()

    # (t,a) -> (1, 3)
    plt.plot(arrGraphData_simu[:, 1], arrGraphData_simu[:, 3], color="blue", label='from simulation')
    # (t,a) -> (1, 4)
    plt.plot(arrGraphData_anal[:, 1], arrGraphData_anal[:, 4], 'r--', label='from calculation')
    plt.title('ACCELERATION - TIME')
    plt.xlabel('time [s]')
    plt.ylabel('acceleration [m/s^2]')
    plt.legend()
    plt.xlim(0)
    plt.ylim(0)
    plt.show()

    # (t,e) -> (1, 5)
    plt.plot(arrGraphData_simu[:, 1], arrGraphData_simu[:, 5], color="blue", label='from simulation')
    # (t,e) -> (1, 5)
    plt.plot(arrGraphData_anal[:, 1], arrGraphData_anal[:, 5], 'r--', label='from calculation')
    plt.title('ANGULAR ACCELERATION - TIME')
    plt.xlabel('time [s]')
    plt.ylabel('angular acceleration [1/s^2]')
    plt.legend()
    plt.xlim(0)
    #plt.ylim(0)
    plt.show()






