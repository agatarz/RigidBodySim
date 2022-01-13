"""
Remake of the veritcal stack demo from the box2d testbed.
"""

import math

import pyglet
from pyglet.gl import *


import pymunk
import pymunk.pyglet_util
from pymunk import Vec2d


class Main(pyglet.window.Window):
    def __init__(self):

        pyglet.window.Window.__init__(self, 1200, 600, vsync=True)

        self.set_caption("Portal")
        pyglet.clock.schedule_interval(self.update, 0.005 / 60.0)
        self.fps_display = pyglet.window.FPSDisplay(self)
        self.create_world()
        self.draw_options = pymunk.pyglet_util.DrawOptions()
        self.draw_options.flags = self.draw_options.DRAW_SHAPES

    def create_world(self):
        self.space = pymunk.Space()
        self.space.gravity = Vec2d(0.0, -981*0.1)

        deviation = 0
        friction = 0.4

        size = 1
        floor_lvl = 25

        points = [(-590 * size, -1 * size),
                  (-590 * size, 1 * size),
                  (590 * size, 1 * size),
                  (590 * size,-1 * size)]
        mass = 1.0
        moment = pymunk.moment_for_poly(mass, points, (0, 0))
        body = pymunk.Body(mass, moment, body_type = pymunk.Body.STATIC)
        body.position = Vec2d(600, floor_lvl - size)
        shape = pymunk.Poly(body, points)
        shape.friction = friction
        self.space.add(body, shape)

        # COLUMNS
        for x in [-1, 0, 1]:
            for y in range(13):

                width = 100/2 - deviation
                heigth = 30/2 - deviation
                points = [(-width, -heigth),
                          (-width, heigth),
                          (width, heigth),
                          (width, -heigth)]
                mass = 720
                moment = pymunk.moment_for_poly(mass, points, (0, 0))
                body = pymunk.Body(mass, moment)
                body.position = Vec2d(600 + x * 391.71, floor_lvl + y * (30) + 15)
                shape = pymunk.Poly(body, points)
                if  y%2: shape.color = (149, 165, 166, 255)
                else: shape.color = (255, 0, 0, 255)
                shape.friction = friction
                self.space.add(body, shape)

        # TRAPEZE
        local_points = [(0, 0),
                        (100 - deviation, 0),
                        (100 - deviation, 16.79 - deviation),
                        (50 - deviation, 28.79 - deviation),
                        (0, 16.79 - deviation)]

        poly_bad = pymunk.Poly(None, local_points)
        t = (poly_bad.center_of_gravity)
        print(t)

        center_points = tuple((x - t[0], y - t[1]) for (x,y) in local_points)

        for x in [-1, 0, 1]:

            mass = 720
            moment = pymunk.moment_for_poly(mass, center_points, (0, 0))
            body = pymunk.Body(mass, moment)
            body.position = Vec2d(600 + x * 391.71, floor_lvl + 13 * (30) + t[1] + deviation )
            shape = pymunk.Poly(body, center_points)
            shape.friction = friction
            shape.color = (149, 165, 166, 255)
            self.space.add(body, shape)


        # ARCHES
        local_points = [(3.92 - deviation, 0),
                        (27.46 - deviation, 0),
                        (31.38 - deviation, 49.85 - deviation),
                        (0, 49.85 - deviation)]

        poly_bad = pymunk.Poly(None, local_points)
        t = (poly_bad.center_of_gravity)
        print(t)

        center_points = tuple((x - t[0], y - t[1]) for (x, y) in local_points)
        wektor = pymunk.Vec2d(0, 175.65)

        for x in (-1, 1):
            for y in range(9):

                mass = 360
                moment = pymunk.moment_for_poly(mass, center_points, (0, 0))
                # left
                if y==10:
                    body = pymunk.Body(mass, moment, body_type = pymunk.Body.STATIC)
                else:
                    body = pymunk.Body(mass, moment)

                body.angle = math.radians(9 * y)
                body.position = Vec2d(600 + 0.5 * x * 391.71 + wektor[0], 371.77 + wektor[1] + floor_lvl)
                shape = pymunk.Poly(body, center_points)
                if not y % 2:
                    shape.color = (255, 0, 0, 255)

                else:
                    shape.color = (149, 165, 166, 255)
                shape.friction = friction
                self.space.add(body, shape)

                if y > 0:
                # right
                    if y==10:
                        body = pymunk.Body(mass, moment, body_type = pymunk.Body.STATIC)
                    else:
                        body = pymunk.Body(mass, moment)

                    body.angle = math.radians(-9 * y)
                    body.position = Vec2d(600 + 0.5 * x * 391.71 - wektor[0], 371.77 + wektor[1] + floor_lvl)
                    shape = pymunk.Poly(body, center_points)
                    if not y % 2:
                        shape.color = (255, 0, 0, 255)
                    else:
                        shape.color = (149, 165, 166, 255)
                    shape.friction = friction
                    self.space.add(body, shape)

                wektor = wektor.rotated_degrees(9)
            wektor = pymunk.Vec2d(0, 175.65)

        # MAKE BC

        for x in [-1,1]:

            floor_lvl = 25

            points = [(-1, -10),
                      (-1, 10),
                      (1, 10),
                      (1, -10)]
            mass = 1.0
            moment = pymunk.moment_for_poly(mass, points, (0, 0))
            body = pymunk.Body(mass, moment, body_type=pymunk.Body.STATIC)
            body.position = Vec2d(600 + x * (391.71 + 51), floor_lvl + 13 * 30 + 10)
            shape = pymunk.Poly(body, points)
            shape.friction = friction
            self.space.add(shape, body)


    def update(self, dt):
        # Here we use a very basic way to keep a set space.step dt.
        # For a real game its probably best to do something more complicated.
        step_dt = 1 / 500.0
        x = 0
        while x < dt:
            x += step_dt
            self.space.step(step_dt)

    def on_draw(self):
        self.clear()
        #self.text.draw()
        self.fps_display.draw()
        self.space.debug_draw(self.draw_options)


if __name__ == "__main__":
    main = Main()
    pyglet.app.run()