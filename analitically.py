import math


class SlopeBox:
    """
    This class contains methods to resolve the problem of box sliding on the box in analytical way
    """

    def __init__(self, slope_height, slope_length, box_friction, gravity):
        """
        Define parameters to solve this problem
        gravity is set by default as 981
        :param slope_height: height of slope
        :param slope_length: length of slope
        :param box_friction: coefficient of friction
        """
        self.slopeHeight = slope_height
        self.slopeLength = slope_length
        self.boxFriction = box_friction
        self.gravity = gravity

    def angle_calc(self):
        """
        its calculate angular of slope as alfa = atan(height/length)
        :return: angular in radians
        """
        return math.atan(self.slopeHeight / self.slopeLength)

    def acceleration_calc(self):
        """
        its calculate acceleration of the box as a=g*(sin(alfa)-ni*cos(alfa)
        :return:
        """
        return self.gravity * (math.sin(self.angle_calc()) - math.cos(self.angle_calc()) * self.boxFriction)

    def distance_calc(self):
        """
        its calculate a distance of slope as s = sqrt(height^2 + length^2)
        :return:
        """
        return math.sqrt(self.slopeHeight ** 2 + self.slopeLength ** 2)

    def time_calc(self):
        """
        its calculate time of box sliding as t=sqrt(2*s/a)
        :return:
        """
        if self.acceleration_calc() <= 0:
            return 0
        return math.sqrt(2 * self.distance_calc() / self.acceleration_calc())


class SlopeRoller:
    """
    This class contains methods to resolve the problem of box sliding on the box in analytical way
    """

    def __init__(self, slope_height, slope_length, roller_friction, gravity, roller_radius, roller_mass):
        """
        Define parameters to solve this problem
        gravity is set by default as 981
        :param slope_height: height of slope
        :param slope_length: length of slope
        """
        self.slopeHeight = slope_height
        self.slopeLength = slope_length
        self.rollerFriction = roller_friction
        self.rollerRadius = roller_radius
        self.rollerMass = roller_mass
        self.gravity = gravity

    def moment_of_interia(self):
        """
        0.5*m*r*r
        :return:
        """
        return 0.5 * self.rollerRadius * self.rollerRadius * self.rollerMass

    def angle_calc(self):
        """
        its calculate angular of slope as alfa = atan(height/length)
        :return: angular in radians
        """
        return math.atan(self.slopeHeight / self.slopeLength)

    def acceleration_calc(self):
        """
        its calculate acceleration of the box as a=g*(sin(alfa)-ni*cos(alfa)
        :return:
        """
        if self.rollerFriction >= self.slopeHeight / self.slopeLength / 3:
            # brak poślizgu
            return 2 * self.gravity * math.sin(self.angle_calc()) / 3

        else:
            # poślizg
            return self.gravity * (math.sin(self.angle_calc()) - math.cos(self.angle_calc()) * self.rollerFriction)

    def angular_acceleration_calc(self):
        """
        its calculate acceleration of the box as a=g*(sin(alfa)-ni*cos(alfa)
        :return:
        """
        if self.rollerFriction >= self.slopeHeight / self.slopeLength / 3:
            # brak poślizgu
            return - 2 * self.gravity * math.sin(self.angle_calc()) / 3 / self.rollerRadius

        else:
            # poślizg
            return - 2 * self.rollerFriction * self.gravity * math.cos(self.angle_calc()) / self.rollerRadius

    def distance_calc(self):
        """
        its calculate a distance of slope as s = sqrt(height^2 + length^2)
        :return:
        """
        return math.sqrt(self.slopeHeight ** 2 + self.slopeLength ** 2)

    def time_calc(self):
        """
        its calculate time of box sliding as t=sqrt(2*s/a)
        :return:
        """
        if self.acceleration_calc() <= 0:
            return math.sqrt(2 * self.distance_calc() / abs(self.acceleration_calc()))
        return math.sqrt(2 * self.distance_calc() / self.acceleration_calc())

    def actual_height(self, actual_time):
        pass

    def kineticEnergy(self, actual_time):
        pass

    def potentialEnergy(self, actual_time):
        pass

    def mechanicalEnergy(self, actual_time):
        pass


class FallingBall:
    def __init__(self, ball_radius, ball_mass, height, gravity):
        self.ballRadius = ball_radius
        self.ballMass = ball_mass
        self.height = height
        self.gravity = gravity

    def time_calc(self):
        """
        t=sqrt(2*H/g)
        :return:
        """
        return math.sqrt(2 * self.height / self.gravity)

    def actual_velocity(self, actual_time):
        """
        v=gt
        :return:
        """
        return -self.gravity * actual_time

    def actual_height(self, actual_time):
        """
        h=gt^2/2
        :return:
        """
        return self.height - self.gravity * actual_time * actual_time / 2

    def kineticEnergy(self, actual_time):
        """
        Ek=mv^2/2
        :param actual_time:
        :return:
        """
        return self.ballMass * self.actual_velocity(actual_time) * self.actual_velocity(actual_time) / 2

    def potentialEnergy(self, actual_time):
        """
        Ep=mgh
        :param actual_time:
        :return:
        """
        return self.ballMass * self.gravity * self.actual_height(actual_time)

    def mechanicalEnergy(self, actual_time):
        """
        Ep=mgh
        :param actual_time:
        :return:
        """
        return self.kineticEnergy(actual_time) + self.potentialEnergy(actual_time)


class ThrowingBall:
    def __init__(self, ball_radius, ball_mass, velocity_start, gravity, elasticity):
        self.ballRadius = ball_radius
        self.ballMass = ball_mass
        self.velocity_start = -velocity_start * elasticity
        self.gravity = gravity

    def height_max(self):
        """
        hmax=v0^2/2/g
        :return:
        """
        return self.velocity_start * self.velocity_start / 2 / self.gravity

    def time_calc(self):
        """
        t=v0/g
        :return:
        """
        return self.velocity_start / self.gravity

    def actual_velocity(self, actual_time):
        """
        v=v0-gt
        :return:
        """
        return self.velocity_start - self.gravity * actual_time

    def actual_height(self, actual_time):
        """
        h=v0t-gt^2/2
        :return:
        """
        return self.velocity_start * actual_time - self.gravity * actual_time * actual_time / 2

    def kineticEnergy(self, actual_time):
        """
        Ek=mv^2/2
        :param actual_time:
        :return:
        """
        return self.ballMass * self.actual_velocity(actual_time) * self.actual_velocity(actual_time) / 2

    def potentialEnergy(self, actual_time):
        """
        Ep=mgh
        :param actual_time:
        :return:
        """
        return self.ballMass * self.gravity * self.actual_height(actual_time)

    def mechanicalEnergy(self, actual_time):
        """
        Ep=mgh
        :param actual_time:
        :return:
        """
        return self.kineticEnergy(actual_time) + self.potentialEnergy(actual_time)


if __name__ == '__main__':
    """
    analysis = SlopeBox(slopeHeight=100, slopeLength=1000, box_friction=0.05)
    totalTime = analysis.time_calc()
    acceleration = analysis.acceleration_calc()

    # create time periods (x-axis) from 0 to totalTime with 600 intervals
    time_steps = np.linspace(0, totalTime, 600)

    # calculate a distance (y-axis) as s = 0.5 * at^2
    dist_func = acceleration * time_steps ** 2 / 2
    plt.plot(time_steps, dist_func)
    plt.show()
    """

    gravity1 = 9.81
    slopeElasticity1 = 1  # * 0.99999999999999999999
    ballElasticity1 = 1  # *0.99999999999999999999
    ballRadius1 = 0.2
    ballMass1 = 0.5
    floorLevel1 = 0
    topLevel1 = 5

    analiza1 = FallingBall(ballRadius1, ballMass1, topLevel1 - floorLevel1, gravity1)
    totalTime1 = analiza1.time_calc()
    print(totalTime1)
    end_velocity1 = analiza1.actual_velocity(totalTime1)
    print(end_velocity1)

    print('-----')

    gravity1 = 981
    ballRadius1 = 0.2
    ballMass1 = 0.5
    floorLevel1 = 0
    topLevel1 = 500

    analiza1 = FallingBall(ballRadius1, ballMass1, topLevel1 - floorLevel1, gravity1)
    totalTime1 = analiza1.time_calc()
    print(totalTime1)
    end_velocity1 = analiza1.actual_velocity(totalTime1)
    print(end_velocity1)
