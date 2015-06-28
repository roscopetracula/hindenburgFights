#!/usr/bin/python

import os
import sys
import time
import argparse
import pygame, sys

DISPLAY_UPDATE_TIME = 0.0 # Don't update the display more than this
                          # often (s).

from ctypes.util import find_library
from pygame.locals import *

from pgu import gui

from lib.controller import (
    load_controllers,
    has_xbox_controller,
    KeyboardController,
    XboxController,
    bleBot,
)

def doQuit(value = None):
    for controller in controllers:
        controller.cleanup()
    pygame.quit()
    sys.exit()

def doDisableAll(value = None):
    for controller in controllers:
        controller.bleBlimp.disable()
    
def doEnableAll(value = None):
    for controller in controllers:
        controller.bleBlimp.enable()

def doResetAll(value = None):
    for controller in controllers:
        controller.bleBlimp.reset()

parser = argparse.ArgumentParser(description='We be big blimpin.')
group = parser.add_mutually_exclusive_group()
group.add_argument('--default-disabled', action='store_true', help='disable all blimps at startup')
group.add_argument('--default-enabled', action='store_true', help='enable all blimps at startup (default)')
args = parser.parse_args()

# If it was set on the command line, override the default blimp state.
if args.default_enabled:
    bleBot.DEFAULT_ENABLED = True
elif args.default_disabled:
    bleBot.DEFAULT_ENABLED = False

# Set up controllers and calculate gui controller layout.
if has_xbox_controller():
    pygame.joystick.init()
controllers = load_controllers()
numControllers = len(controllers)
controllersPerLine = 2 if numControllers < 5 else 3

# Initialize GUI.
guiApp = gui.Desktop()
guiAppTable = gui.Table()

guiAppTable.tr()
guiAppTable.td(gui.Label("Battle Blimps"), colspan=(2 if controllersPerLine <= 2 else 4), style={'border':10});
guiAppButtonsTable = gui.Table()
guiDisableAllButton=gui.Button("Disable All")
guiDisableAllButton.connect(gui.CLICK, doDisableAll, None);
guiEnableAllButton=gui.Button("Enable All")
guiEnableAllButton.connect(gui.CLICK, doEnableAll, None);
guiResetAllButton=gui.Button("Reset All")
guiResetAllButton.connect(gui.CLICK, doResetAll, None);
guiQuitButton = gui.Button("Quit")
guiQuitButton.connect(gui.CLICK, doQuit, None);
guiAppButtonsTable.td(guiDisableAllButton)
guiAppButtonsTable.td(guiEnableAllButton)
guiAppButtonsTable.td(guiResetAllButton)
guiAppButtonsTable.td(guiQuitButton)
if (numControllers == 1):
    guiAppTable.tr()
guiAppTable.td(guiAppButtonsTable, style={'border':3})

for c in range(0, numControllers):
    controller = controllers[c]
    if (c % controllersPerLine == 0):
        guiAppTable.tr()
        guiAppTable.td(gui.Spacer(1,1))
        guiAppTable.tr()
    elif (c != 0):
        guiAppTable.td(gui.Spacer(1,1))
    guiAppTable.td(controller.bleBlimp.gui.frame)
guiApp.init(guiAppTable)

lastDisplayUpdateTime = 0

while True:
    foundConnecting = False
    foundTimedOut = False

    # Check for and update any pending connection attempts.
    for controller in controllers:
        if controller.bleBlimp.connectionState == controller.bleBlimp.CONNECTING:
            # We are in the middle of an asynchronous connect, check the status.
            controller.bleBlimp.checkCompleteConnection()
            foundConnecting = True
        elif controller.bleBlimp.connectionState == controller.bleBlimp.FAILED:
            # We have failed a connection that hasn't timed out yet, retry.
            controller.bleBlimp.connect()
            foundConnecting = True
        elif controller.bleBlimp.connectionState == controller.bleBlimp.TIMED_OUT:
            # We have timed out; leave this controller alone.
            foundTimedOut = True
        elif controller.bleBlimp.connectionState == controller.bleBlimp.WAITING and not foundConnecting:
            # If we're waiting and another isn't already connecting, try to connect this one.
            controller.bleBlimp.connect()
            foundConnecting = True

    # If we didn't find any connecting but did find some timed out,
    # reset all of the timed out ones to waiting; we will retry the
    # connection next loop.
    if foundTimedOut and not foundConnecting:
        for controller in controllers:
            if controller.bleBlimp.connectionState == controller.bleBlimp.TIMED_OUT:
                controller.bleBlimp.updateConnectionState(controller.bleBlimp.WAITING)

    # Process all events in queue, including keybaord controller events.
    events = pygame.event.get()
    for event in events:

        if (event.type == QUIT) or \
           (event.type == KEYDOWN and event.key == pygame.K_q):
            doQuit()
            
        elif (event.type == KEYDOWN and event.key == pygame.K_e):
            doEnableAll()
            
        elif (event.type == KEYDOWN and event.key == pygame.K_d):
            doDisableAll()
            
        elif (event.type == KEYDOWN or event.type == KEYUP):
            for controller in controllers:
                if isinstance(controller, KeyboardController) and \
                   event.key in controller.handledKeys:
                    controller.handleEvt(event)
                    
        else:
            # Otherwise pass the event to the app.
            guiApp.event(event)

    # Check for any Xbox controller activity.
    for controller in controllers:
        if isinstance(controller, XboxController):
            controller.handleXbox()

    # Check if any controller activity needs to be transmitted.
    for controller in controllers:
        controller.bleBlimp.autoTxUpdate()
        
    # Update the gui display if sufficient time has passed.    
    loopTime = time.time()
    if loopTime - lastDisplayUpdateTime >= DISPLAY_UPDATE_TIME:
        lastDisplayUpdateTime = loopTime
        rects = guiApp.update()
        pygame.display.update(rects)


