from Box2D.b2 import*
from Box2D import b2PolygonShape

from pygame import gfxdraw

from numpy import array
import numpy as np

class Object:
    HALF_W = 0.075
    HALF_H = 0.075

    DENSITY = 2
    FRICTION = 1.0
    RESTITUTION = 0.0

    LINEAR_DAMPING = 0.8
    ANGULAR_DAMPING = 0.8

    """
        scale_real_to_sim: scale factor to go from real world to
            simulation coords (for numerical reasons)
        scale_real_to_vis: scale factor to go from real world to
            visualisation coords (meter to pixels)
    """
    def __init__(self, world, scale_real_to_sim, scale_real_to_vis, pos,
            shape):
        self.scale_sim_to_vis = (1.0 / scale_real_to_sim) * scale_real_to_vis
        self.scale_real_to_sim = scale_real_to_sim

        s = scale_real_to_sim
        self.body = world.CreateDynamicBody(
                position = scale_real_to_sim * vec2(pos[0], pos[1]),
                linearDamping = self.LINEAR_DAMPING,
                angularDamping = self.ANGULAR_DAMPING)

        if shape == 'quad':
            self.fixture = self.body.CreatePolygonFixture(
                    box = scale_real_to_sim * vec2(self.HALF_W, self.HALF_H),
                    density = self.DENSITY,
                    friction = self.FRICTION,
                    restitution = self.RESTITUTION)

        elif shape == 'circle':
            self.fixture = self.body.CreateCircleFixture(
                    radius = scale_real_to_sim * self.HALF_H,
                    density = self.DENSITY,
                    friction = self.FRICTION,
                    restitution = self.RESTITUTION)

        elif shape == 'l-form':
            v1 = [(0, -0.1), (0.05, -0.1),  (0.05, 0.05), (0, 0.05)]
            v1 = np.multiply(v1, scale_real_to_sim)
            v2 = [(0.05, -0.1), (0.1, -0.1), (0.1, -0.05), (0.05, -0.05)]
            v2 = np.multiply(v2, scale_real_to_sim)

            self.body.CreatePolygonFixture(
                shape=b2PolygonShape(vertices=v1.tolist()),
                density=self.DENSITY,
                friction=self.FRICTION,
                restitution=self.RESTITUTION)
            self.body.CreatePolygonFixture(
                shape=b2PolygonShape(vertices=v2.tolist()),
                density=self.DENSITY,
                friction=self.FRICTION,
                restitution=self.RESTITUTION)
            self.fixture = self.body.fixtures

        elif shape == 't-form':

            v1 = [(0, 0), (0.075, 0),  (0.075,0.05), (-0.075,0.05), (-0.075, 0)]
            v1 = np.multiply(v1, scale_real_to_sim)
            v2 = [(0.025, 0), (0.025, -0.1), (-0.025, -0.1), (-0.025, 0)]
            v2 = np.multiply(v2, scale_real_to_sim)

            self.body.CreatePolygonFixture(
                shape=b2PolygonShape(vertices=v1.tolist()),
                density=self.DENSITY,
                friction=self.FRICTION,
                restitution=self.RESTITUTION)
            self.body.CreatePolygonFixture(
                shape=b2PolygonShape(vertices=v2.tolist()),
                density=self.DENSITY,
                friction=self.FRICTION,
                restitution=self.RESTITUTION)

            self.fixture = self.body.fixtures
        else:
            print('invalid \'shape\' in Object constructor')

        self.shape = shape
        self.object_color = (127, 255, 127, 255)

    def getRealPosition(self):
        pos = self.body.position
        return array([pos[0], pos[1]]).reshape(1, 2) / self.scale_real_to_sim


    def getOrientation(self):
        orientation = self.body.angle
        return orientation

    def draw(self, screen):
        h = screen.get_height()
        s = self.scale_sim_to_vis

        if self.shape == 'quad':
            verts = self.fixture.shape.vertices
            verts = [self.body.transform * vert for vert in verts]
            verts = [(s * x, h - s * y) for (x, y) in verts]

            gfxdraw.aapolygon(screen, verts, self.object_color)
        elif self.shape == 'circle':
            x = self.body.position[0]
            y = self.body.position[1]
            gfxdraw.aacircle(screen, int(s * x), int(h - s * y), int(s *
                self.HALF_H * self.scale_real_to_sim), self.object_color)

        elif self.shape == 'l-form' or  self.shape == 't-form':

            for fixture in self.fixture:
                verts = fixture.shape.vertices
                verts = [self.body.transform * vert for vert in verts]
                verts = [(s * x, h - s * y) for (x, y) in verts]
                #for a nice anti-aliased object outline
                gfxdraw.aapolygon(screen, verts, self.object_color)
                gfxdraw.filled_polygon(screen, verts, self.object_color)
                gfxdraw.circle(screen, int(self.body.position[0] * s), int(h - self.body.position[1] * s), 10, (0, 255, 127, 255))