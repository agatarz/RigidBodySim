import numpy as np
import matplotlib.pyplot as plt
from numpy import linspace
import pyglet
from pyglet import shapes
import pymunk
from pymunk.pyglet_util import DrawOptions
import analitically

# create window to display  (width = 1200, height = 600)
window = pyglet.window.Window(1200, 300, "Pymunk Tester", resizable=False)
options = DrawOptions()

gravity = 981  # cm/s2
maxBounceCounter = 5
slopeFriction = 1
ballFriction = 1

slopeElasticity = 1  # 0* 0.99999
ballElasticity = 1   # 0* 0.99999

space = pymunk.Space()
space.gravity = 0, - gravity

ballRadius = 10  # 10 cm = 0.1 m
ballMass = 0.5   # kg

floorLevel = 50  # 50 cm
topLevel = 250   # 250 cm


def make_slope(slope_friction=1):
    """
    it creates the slope
    :param slope_friction: set by default as 1.0 (don't change!!)
    :return:
    """
    floor_shape = pymunk.Poly(None, ((500, 40), (700, 40), (700, floorLevel), (500, floorLevel)))
    floor_moment = pymunk.moment_for_poly(ballMass, floor_shape.get_vertices())
    floor_body = pymunk.Body(ballMass, floor_moment, body_type=pymunk.Body.STATIC)
    floor_body.position = 0, 0
    floor_shape.body = floor_body
    floor_shape.elasticity = slopeElasticity
    floor_shape.friction = slope_friction
    space.add(floor_body, floor_shape)


def transformation(coordinatesByDefault):

    x = coordinatesByDefault[0]
    y = coordinatesByDefault[1] + ballRadius
    return x, y


def make_ball(rollerFriction):
    ball_shape = pymunk.Circle(None, ballRadius, offset=(0, 0))
    ball_moment = pymunk.moment_for_circle(ballMass, 0, ballRadius)
    ball_body = pymunk.Body(ballMass, ball_moment, body_type=pymunk.Body.DYNAMIC)
    ball_shape.body = ball_body
    ball_body.position = start_bouncing
    ball_shape.friction = rollerFriction
    ball_shape.elasticity = ballElasticity
    space.add(ball_body, ball_shape)


# DRAWING
@window.event
def on_draw():
    window.clear()
    # draw the batch
    batch.draw()
    space.debug_draw(options)


def update(dt):

    step_dt = 1 / 500.0
    x = 0
    while x < dt:
        x += step_dt
        space.step(step_dt)

        global dataFromSimulation

        try:

            ball = space.shapes[1]
            """
            dataFromSimulation => [y_coord, v_velocity, time_step]
            y_coord -> 0' level is on the floor 'floorLevel'
            """
            dataFromSimulation = np.append(dataFromSimulation, np.array([
                [ball.body.position.y - floorLevel - ballRadius,
                 ball.body.velocity.y,
                 step_dt]]), axis=0)

        # if arbiter delete body and is no ball in the space stop updating
        except IndexError:
            pass


def make_data_to_graph(dataFromSimulation):
    # data = open('data.txt', 'w')

    dataToGraph = np.empty((0, 7), float)
    # dataToGraph = np.append(dataToGraph, np.array([[topLevel - floorLevel, 0, 0, 0, 0 ,0 ,0]]), axis=0)

    for i in range(0, np.shape(dataFromSimulation)[0]):

        # height relative to the floor level as thisHeight - floorLevel
        h_i = dataFromSimulation[i, 0]

        # time as sum time steps to i-moment
        t_i = sum(dataFromSimulation[:i, 2])

        # velocity at i-moment
        v_yi = dataFromSimulation[i, 1]

        # increase velocity at dt interval
        if i == 0:
            dv_yi = 0
        else:
            dv_yi = -((dataFromSimulation[i, 1]) - dataFromSimulation[i - 1, 1])

        # time increase in i-step - tick clock
        dt_i = dataFromSimulation[i, 2]

        # acceleration at i-dt step time
        if dt_i == 0:
            a_i = 0
        else:
            a_i = dv_yi / dt_i
        if abs(a_i) > 1.3 * gravity:
            a_i = dataToGraph[i - 1, 3]

        # potential energy as Ep = mgh at i-moment
        Ep_i = h_i * gravity * ballMass

        # kinetic energy es Ek=mvv/2 at i-moment
        Ek_i = ballMass * v_yi * v_yi / 2

        # mechanic energy as Em=Ep+Ek at i-moment
        Em_i = Ep_i + Ek_i

        """
        # write to file
        towrite = str(s_i) + '\t ' + str(v_yi) + '\t ' + str(t_i) + '\t ' + str(a_i) + '\t ' + str(Ep_i) + '\t ' + str(
            Ek_i) + '\t ' + str(Em_i) + '\n'
        data.write(towrite)
        """

        # h_i - height to t_i time step
        # v_yi - velocity at t_i step
        # t_i - total time as sum(t_i) time step
        # a_i - acceleration at t_i time step
        # Ep_i - potential energy to t_i time step
        # Ek_i - kinetic energy to t_i time step
        # Em_i - potential energy to t_i time step

        dataToGraph = np.append(dataToGraph, np.array([[h_i, v_yi, t_i, a_i, Ep_i, Ek_i, Em_i]]), axis=0)

    return dataToGraph


def find_top_and_floor(data):
    """
    this function search values of parameters when ball is in the highest position (top),
    and when its is touching the floor.
    When velocity value changes its sign from plus to minus - ball is on the top,
    When velocity value changes its sign from minus to plus - ball is on the floor,
    first row in return array is when ball is on the top position, next as floor-top-floor etc.
    :return: tops_anf_floors [h_avrg ,v_left, v_right, t_avrg, a_avrg, Ep_avrg, Ek_left, Ek_right,Em_left, Em_right]
    """
    tops_anf_floors = np.empty((0, 10), float)

    for i in range(0, np.shape(data)[0]):

        if i > 0 and ((data[i - 1, 1] <= 0 and data[i, 1] >= 0) or (
                data[i - 1, 1] > 0 and data[i, 1] < 0)):
            # if v(i-1) =<0 and v(i)>=0 h=0 & Ep = 0 - floor
            # if v(i-1) >=0 and v(i)<=0 h=max & Ek = 0 - top

            h_avrg = (data[i - 1, 0] + data[i, 0]) / 2
            v_left = data[i - 1, 1]
            v_right = data[i, 1]
            t_avrg = (data[i - 1, 2] + data[i, 2]) / 2
            a_avrg = (data[i - 1, 3] + data[i, 3]) / 2
            Ep_avrg = (data[i - 1, 4] + data[i, 4]) / 2
            Ek_left = data[i - 1, 5]
            Ek_right = data[i, 5]
            Em_left = data[i - 1, 6]
            Em_right = data[i, 6]

            tops_anf_floors = np.append(tops_anf_floors, np.array([[h_avrg, v_left, v_right, t_avrg, a_avrg,
                                                                    Ep_avrg, Ek_left, Ek_right, Em_left, Em_right
                                                                    ]]), axis=0)

        # when ball begins to wall for the first time (i==0) or for the last time (i=(np.shape(dataToGraph)[0] - 1))
        if i == 0:
            h_avrg = data[i, 0]
            v_left = data[i, 1]
            v_right = data[i, 1]
            t_avrg = data[i, 2]
            a_avrg = data[i, 3]
            Ep_avrg = data[i, 4]
            Ek_left = data[i, 5]
            Ek_right = data[i, 5]
            Em_left = data[i, 6]
            Em_right = data[i, 6]

            tops_anf_floors = np.append(tops_anf_floors, np.array([[h_avrg, v_left, v_right, t_avrg, a_avrg,
                                                                    Ep_avrg, Ek_left, Ek_right, Em_left, Em_right
                                                                    ]]), axis=0)

        if i == (np.shape(data)[0] - 1):
            h_avrg = data[i, 0]
            v_left = data[i, 1]
            v_right = data[i, 1]
            t_avrg = data[i, 2]
            a_avrg = data[i, 3]
            Ep_avrg = data[i, 4]
            Ek_left = data[i, 5]
            Ek_right = data[i, 5]
            Em_left = data[i, 6]
            Em_right = data[i, 6]

            tops_anf_floors = np.append(tops_anf_floors, np.array([[h_avrg, v_left, v_right, t_avrg, a_avrg,
                                                                    Ep_avrg, Ek_left, Ek_right, Em_left, Em_right
                                                                    ]]), axis=0)

    return tops_anf_floors


def coll_begin(arbiter, space, data):
    """
    When ball touch the floor bounceCounter + 1.
    When  bounceCounter == maxBounceCounter (when the ball bounces the set number of times) ball is removed
    from the space
    :param arbiter:
    :param space: space with all bodies and shapes
    :return:
    """
    global bounceCounter

    bounceCounter = bounceCounter + 1
    if bounceCounter == maxBounceCounter:
        ball = space.shapes[1]
        space.remove(ball.body, ball)
    return True


def calculate_analitically(acceleration=None):
    """
    This function create an array to graph with data received from analytical solution
    :return: array with [time_steps, kinetic_energy, potential_energy, mechanical_energy, actual_height, actual_velocity]
    """

    # pilka spada --------------------------------------------------
    to_fall = analitically.FallingBall(ballRadius, ballMass, topLevel - floorLevel, gravity)
    totalTime = to_fall.time_calc()

    time_steps = np.linspace(0, totalTime, 600)
    kinetic_energy = to_fall.kineticEnergy(time_steps)
    potential_energy = to_fall.potentialEnergy(time_steps)
    mechanical_energy = kinetic_energy + potential_energy
    actual_height = to_fall.actual_height(time_steps)
    actual_velocity = to_fall.actual_velocity(time_steps)

    for i in range(0, maxBounceCounter - 1):
        # pilka sie wznosi --------------------------------------------------
        to_raise = analitically.ThrowingBall(ballRadius, ballMass, actual_velocity[-1], gravity, ballElasticity)
        totalTime = to_raise.time_calc()

        # to calculate new time steps and another values to them
        next_time_steps = np.linspace(0, totalTime, 600)
        next_kinetic_energy = to_raise.kineticEnergy(next_time_steps)
        next_potential_energy = to_raise.potentialEnergy(next_time_steps)
        next_mechanical_energy = next_kinetic_energy + next_potential_energy
        next_actual_height = to_raise.actual_height(next_time_steps)
        next_actual_velocity = to_raise.actual_velocity(next_time_steps)
        next_time_steps = np.linspace(time_steps[-1], totalTime + time_steps[-1], 600)

        # add new values to previous
        time_steps = np.append(time_steps, next_time_steps)
        kinetic_energy = np.append(kinetic_energy, next_kinetic_energy)
        potential_energy = np.append(potential_energy, next_potential_energy)
        mechanical_energy = np.append(mechanical_energy, next_mechanical_energy)
        actual_height = np.append(actual_height, next_actual_height)
        actual_velocity = np.append(actual_velocity, next_actual_velocity)

        # pilka  spada --------------------------------------------------
        to_fall = analitically.FallingBall(ballRadius, ballMass, actual_height[-1], gravity)
        totalTime = to_fall.time_calc()

        next_time_steps = np.linspace(0, totalTime, 600)
        next_kinetic_energy = to_fall.kineticEnergy(next_time_steps)
        next_potential_energy = to_fall.potentialEnergy(next_time_steps)
        next_mechanical_energy = next_kinetic_energy + next_potential_energy
        next_actual_height = to_fall.actual_height(next_time_steps)
        next_actual_velocity = to_fall.actual_velocity(next_time_steps)
        next_time_steps = np.linspace(time_steps[-1], totalTime + time_steps[-1], 600)

        # add new values to previous
        time_steps = np.append(time_steps, next_time_steps)
        kinetic_energy = np.append(kinetic_energy, next_kinetic_energy)
        potential_energy = np.append(potential_energy, next_potential_energy)
        mechanical_energy = np.append(mechanical_energy, next_mechanical_energy)
        actual_height = np.append(actual_height, next_actual_height)
        actual_velocity = np.append(actual_velocity, next_actual_velocity)

    acceleration = time_steps * 0 + gravity

    return np.array([actual_height,
                     actual_velocity,
                     time_steps,
                     acceleration,
                     potential_energy,
                     kinetic_energy,
                     mechanical_energy]).T


if __name__ == '__main__':

    # setting arbiter collision begin and
    # set the number of bounces counter as 0:
    start_bouncing = transformation((600, topLevel))
    end_bouncing = transformation((600, floorLevel))
    bounceCounter = 0
    handler = space.add_default_collision_handler()
    handler.begin = coll_begin

    # creating a batch object - line - horizontal lines
    batch = pyglet.graphics.Batch()

    # levels of horizontal lines
    heights = linspace(50, 600, 12)
    lines = []

    for height in heights:
        # creating a line
        line = shapes.Line(200, height, 1000, height,
                           width=1, color=(255, 255, 255), batch=batch)

        # changing opacity of the line1
        # opacity is visibility (0 = invisible, 255 means visible)
        line.opacity = 100
        lines.append(line)

    # make numpy array with data from simulation: [y_coord, v_velocity,time_step]

    dataFromSimulation = np.empty((0, 3), float)
    dataFromSimulation = np.append(dataFromSimulation, np.array([[topLevel - floorLevel, 0, 0]]), axis=0)

    arrGraphData_anal = calculate_analitically()
    # arrGraphData_anal[[]] construction:
    # h_i  -> 0
    # v_yi -> 1
    # t_i  -> 2
    # a_i  -> 3
    # Ep_i -> 4
    # Ek_i -> 5
    # Em_i -> 6

    # make slope an box and set theirs parameters
    make_slope()
    make_ball(ballMass)

    pyglet.clock.schedule_interval(update, 0.0005 / 60)

    # run the pyglet application
    pyglet.app.run()

    # make data to graph from simulation's data
    arrGraphData_simu = make_data_to_graph(dataFromSimulation)
    # arrGraphData_sim[[]] construction:
    # h_i  -> 0
    # v_yi -> 1
    # t_i  -> 2
    # a_i  -> 3
    # Ep_i -> 4
    # Ek_i -> 5
    # Em_i -> 6

    max_min_points_simu = find_top_and_floor(arrGraphData_simu)
    max_min_points_anal = find_top_and_floor(arrGraphData_anal)

    # print(max_min_points_simu[:,:2])
    # print('----')
    # print(max_min_points_anal[:,:2])

    # (t, hi) -> (2, 0)
    plt.figure().add_subplot().plot(max_min_points_anal[:, 3], max_min_points_anal[:, 0], 'ko')
    plt.plot(arrGraphData_simu[:, 2], arrGraphData_simu[:, 0], color="red", label='from simulation')
    plt.plot(arrGraphData_anal[:, 2], arrGraphData_anal[:, 0], "b--", label='from calculation')

    plt.title('HEIGHT - TIME')
    plt.xlabel('time [s]')
    plt.ylabel('height [m]')
    plt.legend()
    plt.xlim(0)
    plt.grid()
    # plt.ylim(0)
    plt.show()

    # (t, v) -> (2, 1)
    plt.figure().add_subplot().plot(max_min_points_anal[:, 3], max_min_points_anal[:, 1:3], 'ko')
    plt.plot(arrGraphData_simu[:, 2], arrGraphData_simu[:, 1], color="red", label='from simulation')
    plt.plot(arrGraphData_anal[:, 2], arrGraphData_anal[:, 1], "b--", label='from calculation')

    plt.title('VELOCITY - TIME')
    plt.xlabel('time [s]')
    plt.ylabel('velocity [m/s]')
    plt.legend()
    plt.grid()
    plt.xlim(0)
    # plt.ylim(0)
    plt.show()

    # (t, a) -> (2, 3)
    plt.figure().add_subplot().plot(max_min_points_anal[:, 3], max_min_points_anal[:, 4], 'ko')
    plt.plot(arrGraphData_simu[:, 2], arrGraphData_simu[:, 3], color="red", label='from simulation')
    plt.plot(arrGraphData_anal[:, 2], arrGraphData_anal[:, 3], "b--", label='from calculation')
    plt.title('ACCELERATION - TIME')
    plt.xlabel('time [s]')
    plt.ylabel('velocity $[m/s^2]$')
    plt.legend()
    plt.grid()
    plt.xlim(0)
    # plt.ylim(0)
    plt.show()

    # PLOTS WITH POINTS

    # (t, Ep) -> (2, 4)
    # (t, Ek) -> (2, 5)
    # (t, Em) -> (2, 6)

    # from simulation
    plt.figure().add_subplot().plot(max_min_points_anal[:, 3], max_min_points_anal[:, 5:], 'ko')
    plt.plot(arrGraphData_simu[:, 2], arrGraphData_simu[:, 4], color="red", label='Ep (from simulation)')
    plt.plot(arrGraphData_simu[:, 2], arrGraphData_simu[:, 5], color="green", label='Ek (from simulation)')
    plt.plot(arrGraphData_simu[:, 2], arrGraphData_simu[:, 6], color="blue", label='Em (from simulation)')
    # from calculation
    plt.plot(arrGraphData_anal[:, 2], arrGraphData_anal[:, 4], "k--", label='Ep (from calculation)')
    plt.plot(arrGraphData_anal[:, 2], arrGraphData_anal[:, 5], "k--", label='Ek (from calculation)')
    plt.plot(arrGraphData_anal[:, 2], arrGraphData_anal[:, 6], "k--", label='Em (from calculation)')

    plt.title('ENERGY - TIME')
    plt.xlabel('time [s]')
    plt.ylabel('energy [J]')
    plt.grid()
    plt.legend()
    labels = []

    plt.xlim(0)
    # plt.ylim(-200)
    plt.show()

