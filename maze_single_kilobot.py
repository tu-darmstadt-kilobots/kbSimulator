#!/usr/bin/env python2

"""
    A single Kilobot uses Phototaxis to move through a maze.
    Allows communication with the learner to send samples.
    The light is moved based on a policy provided by the learner.

    NOTE: For now needs to be started before the learner to work correctly.
"""

import pygame
from pygame.locals import *
from pygame import draw, gfxdraw

import Box2D
from Box2D.b2 import*

from Labyrinth import Labyrinth
from Object import Object
from Phototaxisbot import Phototaxisbot

import random

from zmq import Context, PAIR
import pickle
import importlib

from numpy import *
import numpy as np
import math


class SingleKilobotMazeSimulator:
    WIDTH, HEIGHT = 1200, 600
    SCALE_REAL_TO_SIM = 10  # for numerical reasons
    SCALE_REAL_TO_VIS = HEIGHT  # 1m = HEIGHT pixels

    ZMQ_PORT = 2357

    def __init__(self):
        # pygame
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT),
                HWSURFACE | DOUBLEBUF, 32)
        pygame.display.set_caption('kbsim - 0.0s')
        self.clock = pygame.time.Clock()

        # pybox2d
        self.world = world(gravity=(0, 0), doSleep=True)

        self.labyrinth = Labyrinth(self.world, self.SCALE_REAL_TO_SIM,
                self.SCALE_REAL_TO_VIS)

        # zqm
        context = Context()
        self.socket = context.socket(PAIR)
        self.socket.connect('tcp://localhost:{}'.format(self.ZMQ_PORT))

    def run(self):
        while True:
            msg = pickle.loads(self.socket.recv())

            if msg['message'] == 'sentPolicyModules':
                for fileName, source in msg['modules']:
                    with open(fileName, 'w') as f:
                        f.write(source)
                policyModule = importlib.import_module(msg['policyModule'])
            elif msg['message'] == 'getSamples':
                policyDict = msg['policyDict']
                self.policy = policyModule.fromSerializableDict(policyDict)

                S, A, R, S_ = self._generateSamples()
                msg = {'message': 'sentSamples',
                       'samples': (S, A, R, S_)}
                self.socket.send(pickle.dumps(msg, protocol=2))
            else:
                print('got unexpected message')

    def _generateSamples(self):
        curr_time = 0
        time_step = 0.1
        max_time = 1000 # seconds

        S = []
        A = []
        R = []
        S_ = []

        x = random.random() * 2.0
        y = random.random() * 1.0

        env = {'light_pos': array([x, y]).reshape(1, 2)}
        kilobot = Phototaxisbot(self.world, self.SCALE_REAL_TO_SIM,
                self.SCALE_REAL_TO_VIS, (x, y), env)

        goal = array([0.25, 0.25]).reshape(1, 2)

        while curr_time < max_time:
            s = c_[kilobot.getRealPosition(), env['light_pos']]

            # select action
            a = self.policy.evaluate(s)
            r = linalg.norm(goal - s[0, 0:2])

            # apply action
            light_pos = env['light_pos'] + a
            light_pos[0, 0] = np.minimum(np.maximum(light_pos[0, 0], 0.1), 1.9)
            light_pos[0, 1] = np.minimum(np.maximum(light_pos[0, 1], 0.1), 0.9)
            env['light_pos'] = light_pos

            for i in range(200):
                self.screen.fill((0, 0, 0, 0))
                self.labyrinth.draw(self.screen)

                # draw light
                lx = int(self.SCALE_REAL_TO_VIS * light_pos[0, 0])
                ly = int(self.screen.get_height() -
                         self.SCALE_REAL_TO_VIS * light_pos[0, 1])
                gfxdraw.aacircle(self.screen, lx, ly, 5, (255, 255, 0, 255))

                # kilobots
                kilobot.step()
                kilobot.setVelocities()
                kilobot.draw(self.screen)

                self.world.Step(time_step, 10, 10) # 10 pos and vel update iterations

                curr_time = curr_time + time_step
                pygame.display.set_caption('kbsim - {:.2f}s - ts: {:.0f}ms'.
                        format(curr_time, time_step * 1000))

                pygame.display.flip()
                #clock.tick(60) # limit speed (wait to get 60 updates/sec)

            s_ = c_[kilobot.getRealPosition(), env['light_pos']]

            S += [s]
            A += [a]
            R += [r]
            S_ += [s_]

        return array(S).squeeze(), array(A).squeeze(), \
               array(R).squeeze(), array(S_).squeeze()


if __name__ == '__main__':
    sim = SingleKilobotMazeSimulator()
    sim.run()
