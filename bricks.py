import pymunk


class Foundation ():
    def __init__(self, points, position, friction):
        self.mass = 1.0
        self.moment = pymunk.moment_for_poly(self.mass, points, (0, 0))
        self.body = pymunk.Body(self.mass, self.moment, body_type=pymunk.Body.STATIC)
        self.body.position = position
        self.shape = pymunk.Poly(self.body, points)
        self.shape.friction = friction

    def add_space(self, space):
        space.add(self.body, self.shape)
        pass


class StandardBrick():
    def __init__(self, mass, points, position, friction, color, type = 'd'):
        self.mass = mass
        self.moment = pymunk.moment_for_poly(self.mass, points, (0, 0))
        if type == 's':
            self.body = pymunk.Body(self.mass, self.moment,  body_type=pymunk.Body.STATIC)
        else:
            self.body = pymunk.Body(self.mass, self.moment, body_type=pymunk.Body.DYNAMIC)
        self.body = pymunk.Body(self.mass, self.moment, body_type=pymunk.Body.DYNAMIC)
        self.body.position = position
        self.shape = pymunk.Poly(self.body, points)
        self.shape.friction = friction
        self.shape.color = color

    def add_space(self, space):
        space.add(self.body, self.shape)
        pass


class PolygonBrick():
    def __init__(self, mass, points, position, friction, color, radius=0):
        self.mass = mass
        self.moment = pymunk.moment_for_poly(self.mass, points, (0, 0))
        self.body = pymunk.Body(self.mass, self.moment)
        self.body.position = position
        self.body.angle = radius
        self.shape = pymunk.Poly(self.body, points)
        self.shape.friction = friction
        self.shape.color = color

    def add_space(self, space):
        space.add(self.body, self.shape)
        pass



class Brick1():
    def __init__(self, w, h, position, mass, friction, color):
        self.width = w-0.05
        self.height = h-0.05
        self.shape = pymunk.Poly(None, ((-w/2,-h/2), (w/2,-h/2), (w/2,h/2), (-w/2,h/2)))
        self.moment = pymunk.moment_for_poly(mass, self.shape.get_vertices(), (0,0))
        self.body = pymunk.Body(mass, self.moment)
        self.body.mass = 0.1
        self.shape.body = self.body
        self.shape.friction = 1
        self.body.position = position
        self.shape.elasticity = 0.5
        #self.body.position = position[0] + self.width/2, position[1] + self.height/2
        if color == 1:
            self.shape.color = (255, 0, 0, 255)
        else:
            self.shape.color = (149, 165, 166, 255)

    def add_space(self):
        space.add(self.body, self.shape)
        pass

    def center_of_grav(self):
        pass
        #print(self.shape.center_of_gravity)