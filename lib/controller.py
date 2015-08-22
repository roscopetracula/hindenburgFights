from lib.constants import *
from lib.bleBot import bleBot
import pygame
import imp
from pygame.locals import *

CONTROLLERS = None

def load_config():
    global CONTROLLERS, ADMIN_CONTROLLER
    if not CONTROLLERS:
        try:
            # Import the configuration by hand to allow for changing it.
            i = imp.load_source("CONTROLLERS", Controller.CONFIG_FILE)
            CONTROLLERS = i.CONTROLLERS
            try:
                ADMIN_CONTROLLER = i.ADMIN_CONTROLLER
            except:
                ADMIN_CONTROLLER = None
        except:
            print "unable to load config file \"{:s}\"".format(Controller.CONFIG_FILE)
            raise
    
def load_controllers(doAppGuiCallback):
    load_config()
    return [
        create_controller(controller_config, doAppGuiCallback)
        for controller_config in CONTROLLERS
    ]

def has_xbox_controller():
    load_config()

    for controller_config in CONTROLLERS:
        if controller_config['type'] == XBOX:
            return True
    
    if ADMIN_CONTROLLER and ADMIN_CONTROLLER['type'] == XBOX:
        return True

    return False

def create_controller(cfg, doAppGuiCallback):
    ble_control = bleBot(cfg['name'], cfg['uuid'], doAppGuiCallback)
    
    if cfg['type'] == KEYBOARD:
        controller = KeyboardController(
            ble_control,
            cfg['orientation'],
            cfg['keys']
        )
    else:
        try:
            joystick = pygame.joystick.Joystick(cfg['joystick'])
        except pygame.error as e:
            if (str(e) == "Invalid joystick device number"):
                print "xbox controller missing, we should create a fake joystick here"
                joystick = None
                raise
            else:
                raise

        controller = XboxController(
            ble_control,
            cfg['orientation'],
            cfg['leftTriggerAxis'],
            cfg['rightTriggerAxis'],
            cfg['igniterButton'],
            joystick
        )

    if ADMIN_CONTROLLER:
        if ADMIN_CONTROLLER['type'] == KEYBOARD:
            controller.otherController = KeyboardController(
                ble_control,
                ADMIN_CONTROLLER['orientation'],
                ADMIN_CONTROLLER['keys']
            )
            controller.otherController.isAdmin = True
        else:
            try:
                joystick = pygame.joystick.Joystick(ADMIN_CONTROLLER['joystick'])
            except pygame.error as e:
                if (str(e) == "Invalid joystick device number"):
                    print "xbox controller missing, we should create a fake joystick here"
                    joystick = None
                    raise
                else:
                    raise
                
            controller.otherController = XboxController(
                ble_control,
                ADMIN_CONTROLLER['orientation'],
                ADMIN_CONTROLLER['leftTriggerAxis'],
                ADMIN_CONTROLLER['rightTriggerAxis'],
                ADMIN_CONTROLLER['igniterButton'],
                joystick
            )
            controller.otherController.isAdmin = True

        # Loop the admin controller back to this one.
        controller.otherController.otherController = controller
    
    ble_control.controller = controller
    ble_control.gui.updateControllerDisplay()
    return controller

class Controller(object):
    CONFIG_FILE = "config.py"

    def __init__(self, bleBlimp):
        self.bleBlimp = bleBlimp
        self.isAdmin = False
        self.curTemplate = -1
        
    def cleanup(self):
        self.bleBlimp.cleanup()

    def reconnect(self):
        self.bleBlimp.reconnect()

class KeyboardController(Controller):

    DummyKeyboardController = {
        'type': KEYBOARD,
        'uuid': "dummy",
        'orientation': {
            "f_b": (AXIS_2, INVERTED),
            "r_l": (AXIS_1, NORMAL),
            "u_d": (AXIS_0, INVERTED),
        },
        'keys': {
            "f": pygame.K_UP,
            "b": pygame.K_DOWN,
            "r": pygame.K_RIGHT,
            "l": pygame.K_LEFT,
            "u": pygame.K_o,
            "d": pygame.K_l,
	    "i": pygame.K_SPACE,
            "green": pygame.K_z,
            "red": pygame.K_x,
        }
    }

    def __init__(self,bleBlimp,axisToMotorMap,keyMap):
        Controller.__init__(self,bleBlimp)
        # super(self, Controller).__init__(bleBlimp)
        self.type = XBOX
        self.axisToMotorMap = axisToMotorMap
        self.keyMap = keyMap
        self.handledKeys = self.keyMap.values()
        self.isAdmin = False
        
    def handleEvt(self,evt):
        keyAction = [km[0] for km in self.keyMap.items() if km[1]==evt.key][0]

        blimpFlags = self.bleBlimp.getBlimpFlags()

        if keyAction == "i":
            if evt.type==KEYUP:
                blimpFlags = blimpFlags & (0xff ^ FLAGS_IGNITER_BIT)
            else:
                blimpFlags = blimpFlags | FLAGS_IGNITER_BIT
        elif keyAction == "red":
            if evt.type==KEYUP:
                blimpFlags = blimpFlags & (0xff ^ FLAGS_RIGHT_TRIGGER_BIT)
            else:
                blimpFlags = blimpFlags | FLAGS_RIGHT_TRIGGER_BIT
        elif keyAction == "green":
            if evt.type==KEYUP:
                blimpFlags = blimpFlags & (0xff ^ FLAGS_LEFT_TRIGGER_BIT)
            else:
                blimpFlags = blimpFlags | FLAGS_LEFT_TRIGGER_BIT

        else:
            if keyAction == "f":
                motorIndex = self.axisToMotorMap["f_b"][0]
                motorPolarity = self.axisToMotorMap["f_b"][1]
            elif keyAction == "b":
                motorIndex = self.axisToMotorMap["f_b"][0]
                motorPolarity = not self.axisToMotorMap["f_b"][1]
            elif keyAction == "l":
                motorIndex = self.axisToMotorMap["r_l"][0]
                motorPolarity = self.axisToMotorMap["r_l"][1]
            elif keyAction == "r":
                motorIndex = self.axisToMotorMap["r_l"][0]
                motorPolarity = not self.axisToMotorMap["r_l"][1]
            elif keyAction == "u":
                motorIndex = self.axisToMotorMap["u_d"][0]
                motorPolarity = self.axisToMotorMap["u_d"][1]
            elif keyAction == "d":
                motorIndex = self.axisToMotorMap["u_d"][0]
                motorPolarity = not self.axisToMotorMap["u_d"][1]

            if evt.type==KEYUP:
                self.bleBlimp.setMotorState(motorIndex, "00", "00")
            else:
                if motorPolarity:
                    motorDirection = "01"
                else:
                    motorDirection = "02"
                self.bleBlimp.setMotorState(motorIndex, motorDirection, numToMotorCode(1))

	self.bleBlimp.setBlimpFlags(blimpFlags)
        self.bleBlimp.autoTxUpdate()


class XboxController(Controller):
    def __init__(self,bleBlimp,axisMap,leftTriggerAxis,rightTriggerAxis,igniterButton,joystick,deadzone=0.3):
        Controller.__init__(self,bleBlimp)
        self.type = XBOX
        self.axisMap = axisMap
        self.joystick = joystick
        self.joystick.init()

        #used to track changes in state; send only upon state change.
        self.axisState = []
        self.deadzone = deadzone
        self.leftTriggerAxis = leftTriggerAxis
        self.rightTriggerAxis = rightTriggerAxis
        self.igniterButton = igniterButton

    def undeadzone(self,x):
        return max(0,min((abs(x)-self.deadzone)/(1-self.deadzone),1))

    def handleXbox(self):
        nowAxisState = ["00","00","00","00"]
        for axis in self.axisMap.items():
            # axis[1][0]  --motorIndex
            # axis[1][1]  --xbox axis
            # axis[1][2]  --axis direction multiplier

            # Hack to deal with triggers
            if axis[1][1] == 5:
                axisVal = (self.joystick.get_axis(5) - self.joystick.get_axis(2)) * axis[1][2] / 2.0
                #print "{:f} {:f} {:f}".format(self.joystick.get_axis(5), self.joystick.get_axis(2), axisVal)
            else:
                axisVal = self.joystick.get_axis( axis[1][1] ) * axis[1][2]
            if axisVal > self.deadzone:
                motorDirection = "01"
                motorSpeed = numToMotorCode(self.undeadzone(axisVal))
            elif axisVal < -self.deadzone:
                motorDirection = "02"
                motorSpeed = numToMotorCode(self.undeadzone(axisVal))
            else:
                motorDirection = "00"
                motorSpeed = "00"
            nowAxisState[axis[1][0]]=motorSpeed
            self.bleBlimp.setMotorState(axis[1][0], motorDirection, motorSpeed)

        # Start with current flags with the joystick bits turned off.
        blimpFlags = self.bleBlimp.getBlimpFlags() & (0xff ^ (FLAGS_IGNITER_BIT | FLAGS_LEFT_TRIGGER_BIT | FLAGS_RIGHT_TRIGGER_BIT))

        if self.joystick.get_button( self.igniterButton):
            blimpFlags = blimpFlags | FLAGS_IGNITER_BIT

	if self.joystick.get_axis( self.leftTriggerAxis) > 0:
            blimpFlags = blimpFlags | FLAGS_LEFT_TRIGGER_BIT

	if self.joystick.get_axis( self.rightTriggerAxis) > 0:
            blimpFlags = blimpFlags | FLAGS_RIGHT_TRIGGER_BIT

 	hexStr = hex(blimpFlags)[-2:]
        hexStr = "0"+hexStr[1] if hexStr[0]=="x" else hexStr
        nowAxisState[3] = hexStr

        self.bleBlimp.setBlimpFlags(blimpFlags)

        if nowAxisState != self.axisState:
            self.axisState = nowAxisState
            self.bleBlimp.autoTxUpdate()
            # print nowAxisState


def numToMotorCode(x):
    # input: x, a number in [0, 1]
    # output: a valid DRV8830 hex code [0x06, 0x3F]

    # retain the last two chars for transmission, and pad with a 0 as needed
    hexStr = hex(int(round(0x06 + x*(0x3f-0x06))))[-2:]
    hexStr = "0"+hexStr[1] if hexStr[0]=="x" else hexStr
    return hexStr


