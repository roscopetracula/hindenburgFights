#!/usr/bin/python

import os
import sys
import time
import argparse
import pygame, sys
import subprocess
import pexpect
import string

DISPLAY_UPDATE_TIME = 0.0   # Don't update the display more than this
                            # often (s).
BLIMP_MISSING_TIME  = 1.0   # Time (s) without an update before a blimp is marked missing.
GAME_NAME = "Battle Blimps" # Game name for window title(s).
DEBUG_SCAN = False          # Debug blimp scanning.

from ctypes.util import find_library
from pygame.locals import *

from pgu import gui

from lib.controller import (
    load_controllers,
    create_controller, 
    has_xbox_controller,
    KeyboardController,
    XboxController,
    bleBot,
    Controller
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

def doRestart():
    os.execvp(sys.argv[0], sys.argv)

parser = argparse.ArgumentParser(description='We be big blimpin.')
parser.add_argument('--config', action='store', help='specificy configuration file (default config.py)', default='config.py')
parser.add_argument('--scan-devices', action='store', help='comma-separated list of bluetooth device(s) to use for background scanning (default is all devices); \"-\" is a ull device (effectively disabling scanning if alone), and \"+\" is all detected devices', default='+')
group = parser.add_mutually_exclusive_group()
group.add_argument('--default-disabled', action='store_true', help='disable all blimps at startup')
group.add_argument('--default-enabled', action='store_true', help='enable all blimps at startup (default)')
parser.add_argument('--minimum-blimps', action='store', help='make sure we have at least N blimps, creating dummies where needed', default='0', metavar="N", type=int)
args = parser.parse_args()
Controller.CONFIG_FILE = args.config

# Kill all bluepy helpers.  Note that this prevents running more than
# one blimpControl at once!
os.system("killall -q bluepy-helper")

# If it was set on the command line, override the default blimp state.
if args.default_enabled:
    bleBot.DEFAULT_ENABLED = True
elif args.default_disabled:
    bleBot.DEFAULT_ENABLED = False

# Discover bluetooth devices for scanning.
bleDeviceNames = []
hcitoolScanners = {}
# Parse any provided list of bluetooth devices and add those to the scanners list.
for dev in string.split(args.scan_devices,","):
    if dev == "+":
        # For "+", use hcitool to find all devices.
        try:
            hcitoolDevs = pexpect.spawn("hcitool dev")
        except:
            if DEBUG_SCAN:
                print "unable to obtain devs from hcitool"
        else:
            res = 0
            while res != 2:
                res = hcitoolDevs.expect(["Devices:", "(hci\d+)\s+([0-9A-F][0-9A-F]:[0-9A-F][0-9A-F]:[0-9A-F][0-9A-F]:[0-9A-F][0-9A-F]:[0-9A-F][0-9A-F]:[0-9A-F][0-9A-F])", pexpect.EOF])
                if res == 1:
                    newDev = hcitoolDevs.match.group(1)
                    bleDeviceNames.append(newDev)
                    if DEBUG_SCAN:
                        print "hcitool reported possible scanning device {:s}".format(newDev)
        finally:
            hcitoolDevs.close()

    elif dev == "-":
        # "-" means no device is added.
        if DEBUG_SCAN:
            print "null device added from command line"
        pass

    else:
        # Otherwise, assume this is a device name.
        bleDeviceNames.append(dev)
        if DEBUG_SCAN:
            print "added device {:s} from command line".format(dev)

if DEBUG_SCAN:
    print "devices to be scanned: {:s}".format(bleDeviceNames)

for bleDeviceName in bleDeviceNames:
    try:
        hcitoolScanners[bleDeviceName]
    except:
        try:
            newScanner = pexpect.spawn("hcitool -i {:s} lescan --passive --duplicate".format(bleDeviceName))
        except:
            if DEBUG_SCAN:
                print "hcitool scanner failed to start on device {:s}, not scanning".format(bleDeviceName)
            newScanner.close()
        else:
            hcitoolScanners[bleDeviceName] = newScanner
            if DEBUG_SCAN:
                print "hcitool opened device {:s}".format(bleDeviceName)
        
# Start scanning for blimps.
# Note that at start, all blimps are timed out.  This means that if
# you restart the GUI very quickly, there will be a short delay until
# the blimp realizes it is disconnected and starts advertising again.
# If this ever becomes an issue, we can default to non-missing until
# the first period expires.  This may, in turn, cause a delay while
# the first connect fails (if the blimp is missing), but then should
# return to normal.
lastScan = {}

# Set up controllers and calculate gui controller layout.
if has_xbox_controller():
    pygame.joystick.init()
controllers = load_controllers()
# Add any needed dummy controllers.
while (len(controllers) < args.minimum_blimps):
    controllers.append(create_controller(KeyboardController.DummyKeyboardController))       
numControllers = len(controllers)
controllersPerLine = 4

# Initialize GUI.
guiApp = gui.Desktop()
guiAppTable = gui.Table()

try:
    appIcon = pygame.image.load("lib/blimpControlIcon.png")
except pygame.error:
    print "failed to load icon: {:s}".format("lib/blimpControlIcon.png")
    appIcon = None
guiAppTable.tr()
#guiAppTable.td(gui.Image(pygame.transform.scale(appIcon, (48, 48))), colspan=1)
guiAppTable.td(gui.Label(GAME_NAME), colspan=2*(controllersPerLine-1), style={'border':10})
guiAppButtonsTable = gui.Table()
guiDisableAllButton=gui.Button("Disable All")
guiDisableAllButton.connect(gui.CLICK, doDisableAll, None)
guiEnableAllButton=gui.Button("Enable All")
guiEnableAllButton.connect(gui.CLICK, doEnableAll, None)
guiResetAllButton=gui.Button("Reset All")
guiResetAllButton.connect(gui.CLICK, doResetAll, None)
guiQuitButton = gui.Button("Quit")
guiQuitButton.connect(gui.CLICK, doQuit, None)
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
    else: 
        guiAppTable.td(gui.Spacer(1,1))
    guiAppTable.td(controller.bleBlimp.gui.frame)
if (appIcon):
    pygame.display.set_icon(appIcon)
pygame.display.set_caption(GAME_NAME)
guiApp.init(guiAppTable)

lastDisplayUpdateTime = 0

while True:
    foundConnecting = False
    foundTimedOut = False

    # Check for any blimp updates.
    for dev in hcitoolScanners.keys():
        hcitoolScanner = hcitoolScanners[dev]
        try:
            res = hcitoolScanner.expect(["LE Scan ...", "([0-9A-F][0-9A-F]:[0-9A-F][0-9A-F]:[0-9A-F][0-9A-F]:[0-9A-F][0-9A-F]:[0-9A-F][0-9A-F]:[0-9A-F][0-9A-F]) RFduino Blimp", pexpect.TIMEOUT], timeout=0)
            if res == 0:
                if DEBUG_SCAN:
                    print "hcitool reported scanning on device {:s}".format(dev)
            if res == 1:
                addr = hcitoolScanner.match.group(1).lower()
                lastScan[addr] = time.time()
                if DEBUG_SCAN:
                    print "hcitool reported blimp {:s} on device {:s}".format(addr, dev)
        except pexpect.EOF as e:
            if DEBUG_SCAN:
                print "hcitool scanner closed on device {:s}, ending scanning on that device; {:d} {:s}".format(dev, len(hcitoolScanners)-1, "device remains" if len(hcitoolScanners) == 2 else "devices remain")
            hcitoolScanner.close()
            del hcitoolScanners[dev]
            # If we have no more scanners, reset all missing blimps to
            # TIMED_OUT so they will try to connect next cycle.
            if len(hcitoolScanners) == 0:
                if DEBUG_SCAN:
                    print "no scanners remaining, returning MISSING blimps to active and disabling scanning"
                for controller in controllers:
                    if controller.bleBlimp.connectionState == controller.bleBlimp.MISSING:
                        controller.bleBlimp.updateConnectionState(controller.bleBlimp.TIMED_OUT)
                
    # Check for and update any pending connection attempts.
    loopTime = time.time()
    for controller in controllers:

        # Only do timing out of we have a functional hcitool scanner.
        if len(hcitoolScanners) > 0:
            # Pretend we've seen the dummy.
            lastScan["dummy"] = loopTime

            # If we're not connected, disabled, or missing, and don't have
            # a recent update, we should be missing.
            if not controller.bleBlimp.connectionState in (controller.bleBlimp.CONNECTED, controller.bleBlimp.DISABLED, controller.bleBlimp.MISSING):
                lastUpdate = lastScan.get(controller.bleBlimp.ble_adr.lower(), 0)
                if (loopTime - lastUpdate > BLIMP_MISSING_TIME):
                    if DEBUG_SCAN:
                        print controller.bleBlimp.ble_adr, "is MISSING, marking it so"
                    controller.bleBlimp.disconnect()
                    controller.bleBlimp.updateConnectionState(controller.bleBlimp.MISSING)
            if controller.bleBlimp.connectionState == controller.bleBlimp.MISSING:
                # See if we have found the missing blimp.  If so,
                # transition to TIMED_OUT so it doesn't do any weird queue
                # jumping.
                lastUpdate = lastScan.get(controller.bleBlimp.ble_adr.lower(), 0)
                if (loopTime - lastUpdate <= BLIMP_MISSING_TIME):
                    if DEBUG_SCAN:
                        print controller.bleBlimp.ble_adr, "is FOUND"
                    controller.bleBlimp.updateConnectionState(controller.bleBlimp.TIMED_OUT)
            if controller.bleBlimp.connectionState == controller.bleBlimp.CONNECTED:
                # Update the last time we saw the blimp.  If we want this
                # to be more accurate, we can do it on receive, but this
                # setup encourages a quick reconnect even if the
                # disconnect happens towards the end of an update
                # period. (Note that the blimp is not advertising when
                # connected.)
                lastScan[controller.bleBlimp.ble_adr.lower()] = loopTime
                # Also, for real blimps, poll the status, which
                # appears to cause pending messages to be received.
                if (controller.bleBlimp.ble_adr.lower() != "dummy"):
                    controller.bleBlimp.btlePeripheral.status()
                
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
            
        elif (event.type == KEYDOWN and event.key == pygame.K_r):
            doResetAll()
            
        elif (event.type == KEYDOWN and event.key == pygame.K_x):
            doRestart()
            
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


