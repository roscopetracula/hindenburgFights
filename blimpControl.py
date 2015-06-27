#!/usr/bin/python

import os
import sys
import time
import argparse
import pygame, sys

from ctypes.util import find_library
from pygame.locals import *

from pgu import gui

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

def doDisableAll(value = None):
    for controller in controllers:
        controller.bleBlimp.disable()
    

def doEnableAll(value = None):
    for controller in controllers:
        controller.bleBlimp.enable()

parser = argparse.ArgumentParser(description='We be big blimpin.')
args = parser.parse_args()

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
guiQuitButton = gui.Button("Quit")
guiQuitButton.connect(gui.CLICK, doQuit, None);
guiAppButtonsTable.td(guiDisableAllButton)
guiAppButtonsTable.td(guiEnableAllButton)
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
    guiAppTable.td(controller.bleBlimp.gui.outerFrame)
guiApp.init(guiAppTable)

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
            # We have failed a connection, retry.
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

        # Always pass the event to the app.
        guiApp.event(event)

    # Check for any Xbox controller activity.
    for controller in controllers:
        if isinstance(controller, XboxController):
            controller.handleXbox()

    # Check if any controller activity needs to be transmitted.
    for controller in controllers:
        controller.bleBlimp.autoTxUpdate()
        
    # Update the gui.    
    guiApp.loop()
