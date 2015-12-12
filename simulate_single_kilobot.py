#!/usr/bin/env python2

"""
    A single Kilobot moves directly to the light.
    Allows communication with the learner to send samples.
    The light is moved based on a policy provided by the learner.

    NOTE: For now needs to be started before the learner to work correctly.
"""

import pygame
from pygame.locals import *
from pygame import draw, gfxdraw
import pygame

import Box2D
from Box2D.b2 import*

from Labyrinth import Labyrinth
from Object import Object
from Kilobot import Kilobot

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
        self.kilobot = Kilobot(self.world, self.SCALE_REAL_TO_SIM,
                self.SCALE_REAL_TO_VIS, [0, 0])
        self.kilobot.fixture.friction = 20

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


                # read parameters
                self.numEpisodes = msg['numEpisodes']
                self.numStepsPerEpisode = msg['numStepsPerEpisode']

                self.stepsPerSec = msg['stepsPerSec']

                self.goalReward = msg['goalReward']
                self.wallPunishment = msg['wallPunishment']

                self.epsilon = msg['epsilon']
                self.useMean = msg['useMean']


                S, A, R, S_ = self._generateSamples()

                msg = {'message': 'sentSamples',
                       'samples': (S, A, R, S_)}
                self.socket.send(pickle.dumps(msg, protocol=2))
            else:
                print('got unexpected message')

    """
        epsilon: prob. to choose a random action
    """
    def _generateSamples(self):

        """ sample from random points """
        numSamples = self.numEpisodes * self.numStepsPerEpisode

        goal = np.matrix([0.25, 0.25])
        thresh = 0.1 # m

        # s: kilobot pos
        # a: kilobot movement (dx, dy)
        S = asmatrix(empty((numSamples, 2)))
        A = asmatrix(empty((numSamples, 2)))
        R = asmatrix(empty((numSamples, 1)))
        S_ = asmatrix(empty((numSamples, 2)))

        line_color = (0, 255, 0, 255)

        for ep in range(self.numEpisodes):
            path = []

            # random
            x = 0.1 + random.random() * 1.8 # in [0.1, 1.9]
            y = 0.1 + random.random() * 0.8 # in [0.1, 0.9]

            self.kilobot.body.position = vec2(self.SCALE_REAL_TO_SIM * x,
                    self.SCALE_REAL_TO_SIM * y)

            for step in range(self.numStepsPerEpisode):
                """ user interaction and drawing """
                # handle keys
                for event in pygame.event.get():
                    if event.type == KEYDOWN:
                        if event.key == K_PLUS:
                            self.stepsPerSec *= 2
                        elif event.key == K_MINUS:
                            self.stepsPerSec = np.max([1, self.stepsPerSec / 2])

                """ drawing """
                # draw labyrinth
                self.screen.fill((0, 0, 0, 0))
                self.labyrinth.draw(self.screen)

                # draw kilobot
                self.kilobot.draw(self.screen)

                # draw path
                for p in path:
                    pygame.draw.aaline(self.screen, line_color, p[0], p[1])

                # draw goal
                gx = int(self.SCALE_REAL_TO_VIS * goal[0, 0])
                gy = int(self.screen.get_height() - self.SCALE_REAL_TO_VIS *
                        goal[0, 1])
                gr = int(self.SCALE_REAL_TO_VIS * thresh)
                gfxdraw.aacircle(self.screen, gx, gy, gr, (0, 255, 0, 255))

                kbPos = self.kilobot.body.position / self.SCALE_REAL_TO_SIM
                pygame.display.set_caption(('ep: {} - step: {} - ' +
                    'stepsPerSec: {} - goalDist: {:.2f} cm')
                        .format(ep + 1, step + 1, self.stepsPerSec,
                            linalg.norm(goal - matrix([kbPos[0], kbPos[1]])) * 100))

                pygame.display.flip()
                self.clock.tick(self.stepsPerSec)

                """ simulation """
                # current state
                s = matrix([kbPos[0], kbPos[1]])

                # choose action
                if self.useMean:
                    a = self.policy.getMeanAction(s)
                else:
                    if random.random() <= self.epsilon:
                        a = self.policy.getRandomAction()
                    else:
                        a = self.policy.sampleActions(s)

                # take action TODO add noise?
                numSteps = 1
                stepTime = 1
                velocity = vec2(a[0, 0], a[0, 1]) / (numSteps * stepTime)
                self.kilobot.body.linearVelocity = velocity * self.SCALE_REAL_TO_SIM
                self.kilobot.body.linearDamping = 0.0
                for i in range(numSteps):
                    self.world.Step(stepTime, 10, 10)

                kbPos = self.kilobot.body.position / self.SCALE_REAL_TO_SIM
                s_ = matrix([kbPos[0], kbPos[1]])

                # binary reward + punishment for running into walls
                if linalg.norm(goal - s) <= thresh:
                    r = self.goalReward
                else:
                    a_real = s_ - s;
                    ratio = linalg.norm(a_real) / linalg.norm(a);
                    r = self.wallPunishment * (ratio - 1);
                    #distance = linalg.norm(s_ - (s + a)) / linalg.norm(a)
                    #r = -0.1*distance

                # add to path
                f = self.SCALE_REAL_TO_VIS
                path.append(((f * s[0, 0], self.HEIGHT - f * s[0, 1]),
                             (f * s_[0, 0], self.HEIGHT - f * s_[0, 1])))

                # record sample
                sampleIdx = ep * self.numStepsPerEpisode + step

                S[sampleIdx, :] = s
                A[sampleIdx, :] = a
                R[sampleIdx, :] = r
                S_[sampleIdx, :] = s_

        return S, A, R, S_

if __name__ == '__main__':
    sim = SingleKilobotMazeSimulator()
    sim.run()
