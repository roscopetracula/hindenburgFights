#!/usr/bin/python

import os
import sys
import time
import pygame, sys

from ctypes.util import find_library
from pygame.locals import *

from lib.controller import (
    load_controllers,
    has_xbox_controller,
    KeyboardController,
    XboxController,
)

btlib = find_library("bluetooth")
if not btlib:
    raise Exception(
        "Can't find required bluetooth libraries"
    )

WIDTH = 300
HEIGHT = 300
pygame.init()
pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
if has_xbox_controller():
    pygame.joystick.init()
controllers = load_controllers()

while True:
    events = pygame.event.get()
    for event in events:

        if (event.type == QUIT) or \
           (event.type == KEYDOWN and event.key == pygame.K_q):
            for controller in controllers:
                controller.cleanup()
            pygame.quit()
            sys.exit()

        if (event.type == KEYDOWN and event.key == pygame.K_r):
            for controller in controllers:
                controller.reconnect()

        if (event.type == KEYDOWN or event.type == KEYUP):
            for controller in controllers:
                if isinstance(controller, KeyboardController) and \
                   event.key in controller.handledKeys:
                    controller.handleEvt(event)

    for controller in controllers:
        if isinstance(controller, XboxController):
            controller.handleXbox()

    for controller in controllers:
        controller.retransmit()

    time.sleep(0.05)
