#!/usr/bin/env python2

"""
    Multiple Kilobots move directly to the light in order to solve a maze.
    The light is moved based on two policies.
    One policy moves the object only to the right (objPolicy) and the other
    policy gives the position to move to for solving the maze (mazePolicy).
"""

import pygame
from pygame.locals import *
from pygame import draw, gfxdraw
import pygame

import Box2D
from Box2D.b2 import*

from Object import Object
from Kilobot import Kilobot
from Labyrinth import Labyrinth

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

    ZMQ_PORT = 2358

    def __init__(self):
        # pygame
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT),
                HWSURFACE | DOUBLEBUF, 32)
        pygame.display.set_caption('kbsim')
        self.clock = pygame.time.Clock()

        # pybox2d
        self.world = world(gravity=(0, 0), doSleep=True)

        self.maze = Labyrinth(self.world, self.SCALE_REAL_TO_SIM, self.SCALE_REAL_TO_VIS)

        # zqm
        context = Context()
        self.socket = context.socket(PAIR)
        self.socket.connect('tcp://localhost:{}'.format(self.ZMQ_PORT))

    def run(self):
        sim_initialized = False
        while True:
            msg = pickle.loads(self.socket.recv())

            if msg['message'] == 'sentPolicyModules':
                for fileName, source in msg['modules']:
                    with open(fileName, 'w') as f:
                        f.write(source)
                objPolicyModule = importlib.import_module(msg['objPolicyModule'])
                mazePolicyModule = importlib.import_module(msg['mazePolicyModule'])
            elif msg['message'] == 'testMaze':

                # load the object policy
                objPolicyDict = msg['objPolicyDict']
                self.objPolicy = objPolicyModule.fromSerializableDict(objPolicyDict)

                if not sim_initialized:
                    print("init")

                    # load the maze policy
                    mazePolicyDict = msg['mazePolicyDict']
                    self.mazePolicy = mazePolicyModule.fromSerializableDict(mazePolicyDict)

                    # read parameters
                    self.objectShape = msg['objectShape']
                    self.numKilobots = msg['numKilobots']
                    self.stepsPerSec = msg['stepsPerSec']

                    self._initSimulation()

                    sim_initialized = True
                self._testMazePolicy()
            else:
                print('got unexpected message')

    def _initSimulation(self):
        # create kilobots
        self.kilobots = []
        for i in range(self.numKilobots):
            kilobot = Kilobot(self.world, self.SCALE_REAL_TO_SIM,
                              self.SCALE_REAL_TO_VIS, [0, 0])
            kilobot.fixture.friction = 20

            self.kilobots += [kilobot]

        self.pushObject = Object(self.world, self.SCALE_REAL_TO_SIM,
                                 self.SCALE_REAL_TO_VIS, [0, 0], self.objectShape)

        # fixed object start position
        objStartX = 1.25
        objStartY = 0.75

        r = self.kilobots[0].RADIUS
        kilobotOffsets = array([[-r, -r], [r, -r], [-r, r], [r, r]])

        self.pushObject.body.position = vec2(objStartX, objStartY) * \
                                        self.SCALE_REAL_TO_SIM
        self.pushObject.body.angle = 0

        # light starts over the object
        self.lightPos = matrix([objStartX, objStartY])

        HALF_W = self.pushObject.HALF_W

        # kilobots start left of the object
        for (i, kilobot) in zip(range(self.numKilobots), self.kilobots):
            x = objStartX - 2.0 * HALF_W + (1 + i / 4) * kilobotOffsets[i % 4, 0]
            y = objStartY + (1 + i / 4) * kilobotOffsets[i % 4, 1]
            kilobot.body.position = vec2(x, y) * self.SCALE_REAL_TO_SIM

        self.s = asmatrix(empty((1, 2 + 2 * self.numKilobots)))
        self.targetPos = matrix([objStartX, objStartY])


    def _testMazePolicy(self):

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

        self.maze.draw(self.screen)
        self.pushObject.draw(self.screen)

        for kilobot in self.kilobots:
            kilobot.draw(self.screen)

        # draw light
        lx = int(self.SCALE_REAL_TO_VIS * self.lightPos[0, 0])
        ly = int(self.screen.get_height() - self.SCALE_REAL_TO_VIS *
                self.lightPos[0, 1])
        lr = int(self.SCALE_REAL_TO_VIS * 0.02)
        gfxdraw.aacircle(self.screen, lx, ly, lr, (255, 255, 0))

        objPos = self.pushObject.getRealPosition()

        # draw line from object to target position
        ox = int(self.SCALE_REAL_TO_VIS * objPos[0, 0])
        oy = int(self.screen.get_height() - self.SCALE_REAL_TO_VIS *
                objPos[0, 1])
        tx = int(self.SCALE_REAL_TO_VIS * self.targetPos[0, 0])
        ty = int(self.screen.get_height() - self.SCALE_REAL_TO_VIS *
                self.targetPos[0, 1])

        pygame.draw.aaline(self.screen, (0, 0, 255), (ox, oy), (tx, ty))

        pygame.display.flip()
        self.clock.tick(self.stepsPerSec)

        """ simulation """
        # current state
        self.s[0, 0] = self.lightPos[0, 0] - objPos[0, 0]
        self.s[0, 1] = self.lightPos[0, 1] - objPos[0, 1]

        for (i, kilobot) in zip(range(self.numKilobots), self.kilobots):
            kbPos = kilobot.getRealPosition()
            self.s[0, 2 + 2 * i + 0] = kbPos[0, 0] - objPos[0, 0]
            self.s[0, 2 + 2 * i + 1] = kbPos[0, 1] - objPos[0, 1]

        # solve maze
        self.targetPos = self.mazePolicy.getTargetPosition(objPos)

        # rotate state
        direction = self.targetPos - objPos
        angle = -math.atan2(direction[0, 1], direction[0, 0])

        sx = self.s.flat[0::2] * math.cos(angle) - self.s.flat[1::2] * math.sin(angle)
        sy = self.s.flat[1::2] * math.cos(angle) + self.s.flat[0::2] * math.sin(angle)

        self.s.flat[0::2] = sx
        self.s.flat[1::2] = sy

        # choose action
        a = self.objPolicy.getMeanAction(self.s)

        # rotate action
        ax = a[0, 0] * math.cos(-angle) - a[0, 1] * math.sin(-angle)
        ay = a[0, 1] * math.cos(-angle) + a[0, 0] * math.sin(-angle)

        a[0, 0] = ax
        a[0, 1] = ay

        # take action
        n = linalg.norm(a)
        if n > 0.015:
            self.lightPos += (a * 0.015 / n)
        else:
            self.lightPos += a

        # move directly toward the light
        for kilobot in self.kilobots:
            kbPos = kilobot.getRealPosition()

            v = self.lightPos - kbPos

            # cap max velocity
            n = linalg.norm(v)
            if n > 0.01:
                v *= (0.01 / n)

            kilobot.body.linearVelocity = vec2(v[0, 0], v[0, 1]) * \
                    self.SCALE_REAL_TO_SIM
            kilobot.body.linearDamping = 0.0

        for i in range(10):
            self.world.Step(0.1, 10, 10)

if __name__ == '__main__':
    sim = KilobotsObjectMazeSimulator()
    sim.run()
