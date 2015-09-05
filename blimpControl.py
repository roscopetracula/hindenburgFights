#!/usr/bin/python

import os
import sys
import time
import argparse
import pygame, sys
import subprocess
import pexpect
import string
from ctypes.util import find_library
from pygame.locals import *
from lib.constants import *
from lib.blimpTracker import *

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

def shutdownBlimps():
    blimpTracker.logGlobalEvent("stop")
    for controller in controllers:
        controller.cleanup()
    blimpTracker.close()
    while len(hcitoolScanners) > 0:
        scanner = hcitoolScanners.keys()[0]
        hcitoolScanners[scanner].close()
        del hcitoolScanners[scanner]
    pygame.quit()
    os.system("killall -q bluepy-helper")
    
def doQuit(value = None):
    shutdownBlimps()
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
    shutdownBlimps()
    os.execvp(sys.argv[0], sys.argv)

def doModeChange(guiModeGroup):
    blimpTracker.setPlayMode(guiModeGroup.value)
    if guiModeGroup.value == "Run":
        for controller in controllers:
            controller.bleBlimp.setLocks(False, False)
    elif guiModeGroup.value == "No Fire":
        for controller in controllers:
            controller.bleBlimp.setLocks(True, False)
    elif guiModeGroup.value == "Lockdown":
        for controller in controllers:
            controller.bleBlimp.setLocks(True, True)
    else:
        raise Exception("unknown mode change \"{:s}\"".format(guiModeGroup.value))
    if DEBUG_MODE_CHANGE:
        print "game mode change: {:s}".format(guiModeGroup.value)
    
def doAppGuiCallback(cmd, bleBlimp):
    if cmd == "getControllerNumber":
        return controllers.index(bleBlimp.controller)
    elif cmd == "grab":
        c = bleBlimp.controller
        i = controllers.index(c)
        if DEBUG_CONTROLLERS:
            print "grab on controller #{:d}".format(i)
        controllers[i] = c.otherController
        if c.isAdmin:
            # This was the admin controller.
            if DEBUG_CONTROLLERS:
                print "{:s} swapping out admin controller".format(bleBlimp.ble_adr)
            bleBlimp.gui.grabButton.value.set_text("g")
        else:
            # This was the original controller.
            if DEBUG_CONTROLLERS:
                print "{:s} swapping in admin controller".format(bleBlimp.ble_adr)
            bleBlimp.gui.grabButton.value.set_text("G")

        bleBlimp.controller = controllers[i]
        bleBlimp.gui.updateControllerDisplay()
        bleBlimp.immediateUpdate = True
        
    elif cmd == "template":
        curController = bleBlimp.controller
        curController.curTemplate = (curController.curTemplate + 1) % len(CONTROLLER_TEMPLATES)
        newTemplate = CONTROLLER_TEMPLATES[curController.curTemplate]
        if DEBUG_CONTROLLERS:
            print "template on current controller for blimp {:s} switched to {:s}/{:s} ({:d})".format(bleBlimp.name, newTemplate["name"], newTemplate["char_name"], curController.curTemplate)
        curController.axisMap = newTemplate["orientation"]
        curController.leftTriggerAxis = newTemplate["leftTriggerAxis"]
        curController.rightTriggerAxis = newTemplate["rightTriggerAxis"]
        curController.igniterButton = newTemplate["igniterButton"]
        
        bleBlimp.gui.updateControllerDisplay()
        bleBlimp.immediateUpdate = True

    elif cmd == "left" or cmd == "right":
        curController = bleBlimp.controller
        i = controllers.index(curController)
        if DEBUG_CONTROLLERS:
            print "left on controller #{:d}".format(i)
        if (cmd == "left" and i == 0) or (cmd == "right" and i == len(controllers) - 1):
            if DEBUG_CONTROLLERS:
                print "controller {:d} out of range, not moving".format(i)
            return
        if cmd == "left":
            dir = -1
        else:
            dir = +1;

        # For convenience, the original and replacement
        # blimp/controller pairs.
        curBlimp = curController.bleBlimp
        otherController = controllers[i+dir]
        otherBlimp = otherController.bleBlimp

        # Swap the controllers in the array.
        controllers[i] = otherController
        controllers[i+dir] = curController

        # Update the blimp/controller matchings.
        curController.bleBlimp = otherBlimp
        curController.otherController.bleBlimp = otherBlimp
        otherBlimp.controller = curController
        otherController.bleBlimp = curBlimp
        otherController.otherController.bleBlimp = curBlimp
        curBlimp.controller = otherController

        # Update the GUI.
        curBlimp.gui.updateControllerDisplay()
        curBlimp.immediateUpdate = True
        otherBlimp.gui.updateControllerDisplay()
        otherBlimp.immediateUpdate = True
        
        if DEBUG_CONTROLLERS:
            print "swapped controllers {:d} and {:d}".format(i, i+dir)
    else:
        print "unknown app callback command {:s}".format(cmd)
        
parser = argparse.ArgumentParser(description='We be big blimpin.', formatter_class=argparse.RawDescriptionHelpFormatter, epilog="global keys:\n  q\t\tquit\n  e\t\tenable all blimps\n  d\t\tdisable all blimps\n  r\t\treset all blimps\n  x\t\texit and restart blimpControl\n  enter\t\trun mode\n  space\t\tlockdown mode")
parser.add_argument('--config', action='store', help='specificy configuration file (default config.py)', default='config.py')
parser.add_argument('--scan-devices', action='store', help='comma-separated list of bluetooth device(s) to use for background scanning (default is all devices); \"-\" is a null device (effectively disabling scanning if alone), and \"+\" is all detected devices', default='+')
group = parser.add_mutually_exclusive_group()
group.add_argument('--default-disabled', action='store_true', help='disable all blimp connections at startup')
group.add_argument('--default-enabled', action='store_true', help='enable all blimp connections at startup (default)')
group = parser.add_mutually_exclusive_group()
group.add_argument('--default-mode-lockdown', action='store_true', help='disable all motors and igniters at startup')
group.add_argument('--default-mode-run', action='store_true', help='enable all motors and igniters at startup')
group.add_argument('--default-mode-nofire', action='store_true', help='disable igniters only at startup (default)')
parser.add_argument('--minimum-blimps', action='store', help='make sure we have at least N blimps, creating dummies where needed', default='0', metavar="N", type=int)
args = parser.parse_args()
Controller.CONFIG_FILE = args.config

# Start logging.
blimpTracker.logGlobalEvent("start")
    
# Kill all bluepy helpers.  Note that this prevents running more than
# one blimpControl at once!
os.system("killall -q bluepy-helper")

# If it was set on the command line, override the default blimp state.
if args.default_enabled:
    bleBot.DEFAULT_ENABLED = True
elif args.default_disabled:
    bleBot.DEFAULT_ENABLED = False

# Get the last stored mode if present.
if (not args.default_mode_run and
    not args.default_mode_lockdown and
    not args.default_mode_nofire):
    storedMode = blimpTracker.getPlayMode()
    if storedMode == "Run":
        args.default_mode_run = True
    elif storedMode == "Lockdown":
        args.default_mode_lockdown = True
    else:
        args.default_mode_nofire = True

# Set the mode defaults.
if args.default_mode_run:
    bleBot.DEFAULT_IGNITER_LOCK = False
    bleBot.DEFAULT_MOTORS_LOCK = False
    blimpTracker.setPlayMode("Run")
elif args.default_mode_lockdown:
    bleBot.DEFAULT_IGNITER_LOCK = True
    bleBot.DEFAULT_MOTORS_LOCK = True
    blimpTracker.setPlayMode("Lockdown")
elif args.default_mode_nofire:
    bleBot.DEFAULT_IGNITER_LOCK = True
    bleBot.DEFAULT_MOTORS_LOCK = False
    blimpTracker.setPlayMode("No Fire")
else:
    bleBot.DEFAULT_IGNITER_LOCK = True
    blimpTracker.setPlayMode("No Fire")
    bleBot.DEFAULT_MOTORS_LOCK = False    
    
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
controllers = load_controllers(doAppGuiCallback)

# Add any needed dummy controllers.
while (len(controllers) < args.minimum_blimps):
    controllers.append(create_controller(KeyboardController.DummyKeyboardController))       
numControllers = len(controllers)
if numControllers < 2:
    controllersPerLine = 2
elif numControllers == 3:
    controllersPerLine = 3
else:
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
guiAppTable.td(gui.Label(GAME_NAME), colspan=2*(controllersPerLine-1), style={'border':10}, rowspan=2)
guiDisableAllButton=gui.Button("Disable")
guiDisableAllButton.connect(gui.CLICK, doDisableAll, None)
guiEnableAllButton=gui.Button("Enable")
guiEnableAllButton.connect(gui.CLICK, doEnableAll, None)
guiResetAllButton=gui.Button("Reset")
guiResetAllButton.connect(gui.CLICK, doResetAll, None)
guiQuitButton = gui.Button("Quit")
guiQuitButton.connect(gui.CLICK, doQuit, None)
global guiModeGroup
if args.default_mode_run:
    guiModeGroup = gui.Group(name="mode",value="Run")
elif args.default_mode_lockdown:
    guiModeGroup = gui.Group(name="mode",value="Lockdown")
else:
    guiModeGroup = gui.Group(name="mode",value="No Fire")
guiModeGroup.connect(gui.CHANGE, doModeChange, guiModeGroup)

# Build the global control table.
guiGlobalControlTable = gui.Table()
guiGlobalControlTable.tr()
guiGlobalControlTable.td(gui.Radio(guiModeGroup, "Run"), align=1, colspan=1)
guiGlobalControlTable.td(gui.Label("  Run"), align=-1, colspan=2)
guiGlobalControlTable.td(gui.Label(), colspan=1, width=COL_WIDTH)
guiGlobalControlTable.td(guiDisableAllButton, colspan=2, align=-1)
guiGlobalControlTable.tr()
guiGlobalControlTable.td(gui.Radio(guiModeGroup, "No Fire"), align=1, colspan=1)
guiGlobalControlTable.td(gui.Label("  No Fire"), align=-1, colspan=2)
guiGlobalControlTable.td(gui.Label(), colspan=1)
guiGlobalControlTable.td(guiEnableAllButton, colspan=2, align=-1)
guiGlobalControlTable.tr()
guiGlobalControlTable.td(gui.Radio(guiModeGroup, "Lockdown"), align=1, colspan=1)
guiGlobalControlTable.td(gui.Label("  Lockdown"), align=-1, colspan=2)
guiGlobalControlTable.td(gui.Label(), colspan=1)
guiGlobalControlTable.td(guiResetAllButton, colspan=2, align=-1)
guiGlobalControlTable.tr()
guiGlobalControlTable.td(gui.Label(), colspan=4)
guiGlobalControlTable.td(guiQuitButton, colspan=2, align=-1)
if (numControllers == 1):
    guiAppTable.tr()
guiAppTable.td(guiGlobalControlTable, style={'border':3})

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

print "Battle Blimps ready to go!"

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
            
        elif (event.type == KEYDOWN and event.key == pygame.K_RETURN ):
            guiModeGroup.value = "Run"
            # Manually call the mode change callback in case we had
            # manually switched the state of a motor/ingiter lock.
            doModeChange(guiModeGroup)

        elif (event.type == KEYDOWN and event.key == pygame.K_BACKSLASH ):
            guiModeGroup.value = "No Fire"
            # Manually call the mode change callback in case we had
            # manually switched the state of a motor/ingiter lock.
            doModeChange(guiModeGroup)

        elif (event.type == KEYDOWN and event.key == pygame.K_SPACE):
            guiModeGroup.value = "Lockdown"
            # Manually call the mode change callback in case we had
            # manually switched the state of a motor/ingiter lock.
            doModeChange(guiModeGroup)
            
        elif (event.type == KEYDOWN and event.key == pygame.K_BACKSPACE):
            for controller in controllers:
                if not controller.forcedIgniter:
                    controller.bleBlimp.immmediateUpdate = True
                    controller.forcedIgniter = True
                
        elif (event.type == KEYUP and event.key == pygame.K_BACKSPACE):
            for controller in controllers:
                if controller.forcedIgniter:
                    controller.bleBlimp.immmediateUpdate = True
                    controller.forcedIgniter = False

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

        # Update uptimes.
        for controller in controllers:
            controller.bleBlimp.gui.updateTime()

        # Update display.
        rects = guiApp.update()
        pygame.display.update(rects)

        # Update any logs.
        blimpTracker.sync()


