from Kilobot import Kilobot
from numpy import array
from numpy.linalg import norm
from Box2D.b2 import *


class Phototaxisbot(Kilobot):
    def __init__(self, world, scale_real_to_sim, scale_real_to_vis, pos, env):
        Kilobot.__init__(self, world, scale_real_to_sim, scale_real_to_vis, pos)

        self.scale_sim_to_real = 1.0 / scale_real_to_sim

        self.last_light = 0
        self.turn_cw = 1
        self.counter = 0

        self.env = env

    def step(self):
        light = self.env['light_pos']

        pos_real = self.scale_sim_to_real * \
                   self.body.GetWorldPoint((0.0, self.sim_radius))
        dist = (pos_real - vec2(light[0, 0], light[0, 1])).length

        current_light = 1.0 - dist

        if dist > 0.01:

            if current_light < self.last_light:
                self.counter = 0
                if self.turn_cw == 1:
                    self.turn_cw = 0
                else:
                    self.turn_cw = 1
            else:
                self.counter = self.counter + 1

            self.last_light = current_light

            if self.counter > 50:
                other = 30  # 150
            else:
                other = 0

            other = 0

            if self.turn_cw == 1:
                self.setMotor(255, other)
            else:
                self.setMotor(other, 255)
        else:
            self.setMotor(0, 0)
