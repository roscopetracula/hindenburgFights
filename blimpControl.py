#!/usr/bin/python

import os
import sys
import time
import argparse
import pygame, sys

from ctypes.util import find_library
from pygame.locals import *

from pgu import gui
#GUI from pgu import html

from lib.controller import (
    load_controllers,
    has_xbox_controller,
    KeyboardController,
    XboxController,
)

def doQuit(value = None):
    for controller in controllers:
        controller.cleanup()
    pygame.quit()
    sys.exit()

def doReconnectAll(value = None):
    for controller in controllers:
        controller.reconnect()

parser = argparse.ArgumentParser(description='We be big blimpin.')
args = parser.parse_args()

UNIT_WIDTH = 320
UNIT_HEIGHT = 240
UNIT_COUNT = 2

#pygame.init()
#pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
if has_xbox_controller():
    pygame.joystick.init()
controllers = load_controllers()
numControllers = len(controllers)
controllersPerLine = 2 if numControllers < 5 else 3

# Initialize GUI.
guiApp = gui.Desktop()
#GUI guiAppContainer = gui.Container(width=UNIT_WIDTH*UNIT_COUNT, height=UNIT_HEIGHT)
guiAppTable = gui.Table()

guiAppTable.tr()
guiAppTable.td(gui.Label("Battle Blimps"), colspan=(2 if controllersPerLine == 2 else 4), style={'border':10});
guiAppButtonsTable = gui.Table()
guiReconnectButton=gui.Button("Reconnect All")
guiReconnectButton.connect(gui.CLICK, doReconnectAll, None);
guiQuitButton = gui.Button("Quit")
guiQuitButton.connect(gui.CLICK, doQuit, None);
guiAppButtonsTable.td(guiReconnectButton)
guiAppButtonsTable.td(guiQuitButton)
guiAppTable.td(guiAppButtonsTable)

for c in range(0, numControllers):
    controller = controllers[c]
    if (c % controllersPerLine == 0):
        guiAppTable.tr()
        guiAppTable.td(gui.Spacer(1,1))
        guiAppTable.tr()
    elif (c != 0):
        guiAppTable.td(gui.Spacer(1,1))
    guiAppTable.td(controller.bleBlimp.gui.outerFrame)
guiApp.init(guiAppTable)
    
while True:
    events = pygame.event.get()

    # Process all events in queue, including keybaord controller events.
    for event in events:

        if (event.type == QUIT) or \
           (event.type == KEYDOWN and event.key == pygame.K_q):
            doQuit()
            
        elif (event.type == KEYDOWN and event.key == pygame.K_r):
            doReconnectAll()
            
        elif (event.type == KEYDOWN or event.type == KEYUP):
            for controller in controllers:
                if isinstance(controller, KeyboardController) and \
                   event.key in controller.handledKeys:
                    controller.handleEvt(event)

        else:
            guiApp.event(event)
            
    # Check for any Xbox controller activity.
    for controller in controllers:
        if isinstance(controller, XboxController):
            controller.handleXbox()

    for controller in controllers:
        controller.retransmit()

    # time.sleep(0.01)

    # It's not clear if this is the exact set we need, but it appears
    # to work for now.
    guiApp.loop()
    guiApp.paint()
    pygame.display.flip()
