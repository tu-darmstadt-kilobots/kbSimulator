from Box2D.b2 import*
from pygame import gfxdraw

from numpy import array

class Object:
    HALF_W = 0.075
    HALF_H = 0.075

    DENSITY = 0.1
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
        else:
            print('invalid \'shape\' in Object constructor')

        self.shape = shape
        self.object_color = (127, 255, 127, 255)

    def getRealPosition(self):
        pos = self.body.position
        return array([pos[0], pos[1]]).reshape(1, 2) / self.scale_real_to_sim

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
