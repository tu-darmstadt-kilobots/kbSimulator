#!/usr/bin/env python2

"""
    Multiple Kilobots move directly to the light.
    They learn to push an object.
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


class KilobotsObjectMazeSimulator:
    WIDTH, HEIGHT = 1200, 600
    SCALE_REAL_TO_SIM = 10  # for numerical reasons
    SCALE_REAL_TO_VIS = HEIGHT  # 1m = HEIGHT pixels

    NUM_KILOBOTS = 10

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

        # create kilobots
        self.kilobots = []
        for i in range(self.NUM_KILOBOTS):
            kilobot = Kilobot(self.world, self.SCALE_REAL_TO_SIM,
                self.SCALE_REAL_TO_VIS, [0, 0])
            kilobot.fixture.friction = 20

            self.kilobots += [kilobot]

        # object
        self.pushObject = Object(self.world, self.SCALE_REAL_TO_SIM,
                self.SCALE_REAL_TO_VIS, [0, 0])

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

    def _generateSamples(self):
        """ sample from random points """
        numSamples = self.numEpisodes * self.numStepsPerEpisode

        goal = np.matrix([0.25, 0.25])
        thresh = 0.1 # m

        # s: obj.x obj.y obj.alpha light.x light.y kb.x1 kb.y1 ... kb.xn kb.yn
        # a: light movement (dx, dy)
        S = asmatrix(empty((numSamples, 3 + 2 + 2 * self.NUM_KILOBOTS)))
        A = asmatrix(empty((numSamples, 2)))
        R = asmatrix(empty((numSamples, 1)))
        S_ = asmatrix(empty((numSamples, 3 + 2 + 2 * self.NUM_KILOBOTS)))

        for ep in range(self.numEpisodes):
            objX = 0.1 + random.random() * 1.8
            objY = 0.1 + random.random() * 0.8

            self.pushObject.body.position = vec2(objX, objY) * self.SCALE_REAL_TO_SIM

            # kilobots start around the object
            for kilobot in self.kilobots:
                kbX = objX - 0.1 + random.random() * 0.2 # [x - 0.1, x + 0.1]
                kbY = objY - 0.1 + random.random() * 0.2 # [y - 0.1, y + 0.1]

                kilobot.body.position = vec2(kbX, kbY) * self.SCALE_REAL_TO_SIM

            lightPos = matrix([objX, objY])


            for step in range(self.numStepsPerEpisode):
                """ user interaction """
                # handle keys
                for event in pygame.event.get():
                    if event.type == KEYDOWN:
                        if event.key == K_PLUS:
                            self.stepsPerSec *= 2
                        elif event.key == K_MINUS:
                            self.stepsPerSec = np.max([1, self.stepsPerSec / 2])

                """ drawing """
                self.screen.fill((0, 0, 0, 0))
                self.labyrinth.draw(self.screen)

                self.pushObject.draw(self.screen)

                for kilobot in self.kilobots:
                    kilobot.draw(self.screen)

                # draw light
                lx = int(self.SCALE_REAL_TO_VIS * lightPos[0, 0])
                ly = int(self.screen.get_height() - self.SCALE_REAL_TO_VIS *
                        lightPos[0, 1])
                lr = int(self.SCALE_REAL_TO_VIS * 0.02)
                gfxdraw.aacircle(self.screen, lx, ly, lr, (255, 255, 0))

                # draw goal
                gx = int(self.SCALE_REAL_TO_VIS * goal[0, 0])
                gy = int(self.screen.get_height() - self.SCALE_REAL_TO_VIS *
                        goal[0, 1])
                gr = int(self.SCALE_REAL_TO_VIS * thresh)
                gfxdraw.aacircle(self.screen, gx, gy, gr, (0, 255, 0, 255))

                pygame.display.set_caption(('ep: {} - step: {} - ' +
                    'stepsPerSec: {}').format(ep + 1, step + 1, self.stepsPerSec))

                pygame.display.flip()
                self.clock.tick(self.stepsPerSec)

                """ simulation """
                # current state
                objPos = self.pushObject.getRealPosition()
                objAngle = self.pushObject.body.angle

                s = asmatrix(empty((1, S.shape[1])))
                s[0, 0] = objPos[0, 0]
                s[0, 1] = objPos[0, 1]
                s[0, 2] = objAngle
                s[0, 3] = lightPos[0, 0]
                s[0, 4] = lightPos[0, 1]

                for (i, kilobot) in zip(range(self.NUM_KILOBOTS), self.kilobots):
                    kbPos = kilobot.getRealPosition()
                    s[0, 5 + 2 * i + 0] = kbPos[0, 0]
                    s[0, 5 + 2 * i + 1] = kbPos[0, 1]

                # choose action
                if self.useMean:
                    a = self.policy.getMeanAction(s)
                else:
                    if random.random() <= self.epsilon:
                        a = self.policy.getRandomAction()
                    else:
                        a = self.policy.sampleActions(s)

                # take action / TODO add noise?
                lightPos += a
                lightPos[0, 0] = np.min([1.9, np.max([0.1, lightPos[0, 0]])])
                lightPos[0, 1] = np.min([0.9, np.max([0.1, lightPos[0, 1]])])

                # move directly toward the light
                for kilobot in self.kilobots:
                    kbPos = kilobot.getRealPosition()

                    v = (lightPos - kbPos) / 5.0
                    kilobot.body.linearVelocity = vec2(v[0, 0], v[0, 1]) * \
                            self.SCALE_REAL_TO_SIM
                    kilobot.body.linearDamping = 0.0

                self.world.Step(1, 10, 10)

                # next state
                objPos = self.pushObject.getRealPosition()
                objAngle = self.pushObject.body.angle

                s_ = asmatrix(empty((1, S.shape[1])))
                s_[0, 0] = objPos[0, 0]
                s_[0, 1] = objPos[0, 1]
                s_[0, 2] = objAngle
                s_[0, 3] = lightPos[0, 0]
                s_[0, 4] = lightPos[0, 1]

                for (i, kilobot) in zip(range(self.NUM_KILOBOTS), self.kilobots):
                    kbPos = kilobot.getRealPosition()
                    s_[0, 5 + 2 * i + 0] = kbPos[0, 0]
                    s_[0, 5 + 2 * i + 1] = kbPos[0, 1]

                # binary reward
                if linalg.norm(objPos - goal) <= thresh:
                    r = self.goalReward
                else:
                    r = 0
                    #a_real = s_ - s;
                    #ratio = linalg.norm(a_real) / linalg.norm(a);
                    #r = self.wallPunishment * (ratio - 1);

                # record sample
                sampleIdx = ep * self.numStepsPerEpisode + step

                S[sampleIdx, :] = s
                A[sampleIdx, :] = a
                R[sampleIdx, :] = r
                S_[sampleIdx, :] = s_

        return S, A, R, S_

if __name__ == '__main__':
    sim = KilobotsObjectMazeSimulator()
    sim.run()
