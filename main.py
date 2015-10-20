#!/usr/bin/env python2

import pygame
from pygame.locals import *

import Box2D
from Box2D.b2 import*

from Labyrinth import Labyrinth
from Object import Object
from Phototaxisbot import Phototaxisbot

import random
import math
import time

from pygame import draw, gfxdraw

# pygame
screen = pygame.display.set_mode((1200, 600), HWSURFACE|DOUBLEBUF, 32)
pygame.display.set_caption("kbsim - 0.0s")
clock = pygame.time.Clock()

# pybox2d
world = world(gravity=(0, 0), doSleep=True)

# labyrinth
labyrinth = Labyrinth(world, 10, 600)

# object
push_object = Object(world, 10, 600, (1.25, 0.75))

# environment
env = {'light_pos': [1.75, 0.75]}

# kilobots
Kilobots = []
for i in range(100):
    x = 1.0 + 0.5 * random.random()
    y = 0.5 + 0.5 * random.random()

    Kilobots += [Phototaxisbot(world, 10, 600, (x, y), env)]

# main loop
running = True
paused = False
curr_time = 0
time_step = 0.1

while running:
    light_pos = env['light_pos']

    # event queue
    for event in pygame.event.get():
        if event.type == QUIT or \
                (event.type == KEYDOWN and event.key == K_ESCAPE):
            running = False
        elif event.type == KEYDOWN:
            if event.key == K_UP:
                light_pos[1] = light_pos[1] + 0.1
            elif event.key == K_RIGHT:
                light_pos[0] = light_pos[0] + 0.1
            elif event.key == K_DOWN:
                light_pos[1] = light_pos[1] - 0.1
            elif event.key == K_LEFT:
                light_pos[0] = light_pos[0] - 0.1
            elif event.key == K_SPACE:
                paused = not paused
            elif event.key == K_PLUS:
                time_step = time_step + 0.01
            elif event.key == K_MINUS:
                time_step = time_step - 0.01

    env['light_pos'] = light_pos

    # drawing
    screen.fill((0, 0, 0, 0))
    labyrinth.draw(screen)
    push_object.draw(screen)

    # light
    light_pos = env['light_pos']
    lx = int(600 * light_pos[0])
    ly = int(screen.get_height() - 600 * light_pos[1])
    gfxdraw.aacircle(screen, lx, ly, 5, (255, 255, 0, 255))

    for kb in Kilobots:
        kb.step()
        kb.setVelocities()
        kb.draw(screen)

    if not paused:
        #start_time = time.time()
        world.Step(time_step, 10, 10)
        #print("step took {}ms".format((time.time() - start_time) * 1000))

        curr_time = curr_time + time_step
        pygame.display.set_caption("kbsim - {:.2f}s - ts: {:.0f}ms".
                format(curr_time, time_step * 1000))

    pygame.display.flip()
    #clock.tick(60)

pygame.quit()
