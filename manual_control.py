#!/usr/bin/env python2

"""
    This demonstrates how to use the simulator.
    It doesnt receive commands or send data and the light is controlled manually.
    The Labyrinth is 2m x 1m.
"""

# for visualization
import pygame
from pygame.locals import *
from pygame import draw, gfxdraw, mouse

# Box2D handles the physics
import Box2D
from Box2D.b2 import*

# Objects which can be placed in the physics world
from Labyrinth import Labyrinth
from Object import Object
from Phototaxisbot import Phototaxisbot

import random
import math
import time
from numpy import array


WIDTH, HEIGHT = 1200, 600
SCALE_REAL_TO_SIM = 10  # for numerical reasons
SCALE_REAL_TO_VIS = HEIGHT  # 1m = HEIGHT pixels

# initialize pygame window
screen = pygame.display.set_mode((WIDTH, HEIGHT), HWSURFACE | DOUBLEBUF, 32)
pygame.display.set_caption('kbsim - 0.0s')

clock = pygame.time.Clock()

# create Box2D physics world
world = world(gravity=(0, 0), doSleep=True)

# add the labyrinth and object to the world
labyrinth = Labyrinth(world, SCALE_REAL_TO_SIM, SCALE_REAL_TO_VIS)
push_object = Object(world, SCALE_REAL_TO_SIM, SCALE_REAL_TO_VIS, (1.25, 0.75),'quad')

# environment
env = {'light_pos': array([1.75, 0.75]).reshape(1, 2)}

# add some kilobots with Phototaxis behavior
kilobots = []
for i in range(25):
    x = random.random() * 0.5 + 1.0
    y = random.random() * 0.5 + 0.5
    kilobots += [Phototaxisbot(world, SCALE_REAL_TO_SIM, SCALE_REAL_TO_VIS,
                                (x, y), env)]


# main loop
running = True
paused = False
curr_time = 0
time_step = 1

while running:
    light_pos = env['light_pos']

    m = mouse.get_pos()
    mx = ((m[0])*1.0/SCALE_REAL_TO_VIS)
    my = ((SCALE_REAL_TO_VIS-m[1])*1.0/SCALE_REAL_TO_VIS)
    light_pos[0, 0] = mx
    light_pos[0, 1] = my
    # event queue
    for event in pygame.event.get():
        if event.type == QUIT or \
                (event.type == KEYDOWN and event.key == K_ESCAPE):
            running = False
        elif event.type == KEYDOWN:
            if event.key == K_UP:
                light_pos[0, 1] += 0.1
            elif event.key == K_RIGHT:
                light_pos[0, 0] += 0.1
            elif event.key == K_DOWN:
                light_pos[0, 1] -= 0.1
            elif event.key == K_LEFT:
                light_pos[0, 0] -= 0.1
            elif event.key == K_SPACE:
                paused = not paused
            elif event.key == K_PLUS:
                time_step = time_step * 1.1
            elif event.key == K_MINUS:
                time_step = time_step / 1.1

    env['light_pos'] = light_pos

    # draw labyrinth and object
    screen.fill((0, 0, 0, 0))
    labyrinth.draw(screen)
    push_object.draw(screen)

    # draw light
    light_pos = env['light_pos']
    lx = int(SCALE_REAL_TO_VIS * light_pos[0, 0])
    ly = int(screen.get_height() - SCALE_REAL_TO_VIS * light_pos[0, 1])
    gfxdraw.aacircle(screen, lx, ly, 5, (255, 255, 0, 255))

    # handle kilobot movement and drawing
    for kb in kilobots:
        kb.step()
        kb.setVelocities()
        kb.draw(screen)

    if not paused:
        # step physics objects using 10 pos and vel update iterations
        world.Step(time_step, 10, 10)

        curr_time = curr_time + time_step
        pygame.display.set_caption('kbsim - {:.2f}s - ts: {:.0f}ms'.
                format(curr_time, time_step * 1000))

    pygame.display.flip()

    # limit speed (wait to get 200 updates/sec)
    clock.tick(100)

pygame.quit()
