from lib.constants import *
from lib.bleBot import bleBot
import pygame
import imp
from pygame.locals import *

CONTROLLERS = None

def load_config():
    global CONTROLLERS
    if not CONTROLLERS:
        try:
            # Import the configuration by hand to allow for changing it.
            i = imp.load_source("CONTROLLERS", Controller.CONFIG_FILE)
            CONTROLLERS = i.CONTROLLERS
        except:
            print "unable to load config file \"{:s}\"".format(Controller.CONFIG_FILE)
            raise
    
def load_controllers():
    load_config()
    return [
        create_controller(controller_config)
        for controller_config in CONTROLLERS
    ]

def has_xbox_controller():
    load_config()
    for controller_config in CONTROLLERS:
        if controller_config['type'] == XBOX:
            return True
    return False

def create_controller(cfg):
    ble_control = bleBot(cfg['uuid'], cfg['type'])

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
            cfg['igniterAxis'],
            joystick
        )

    ble_control.controller = controller
        
    return controller

class Controller(object):
    CONFIG_FILE = "config.py"

    def __init__(self, bleBlimp):
        self.bleBlimp = bleBlimp

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
        }
    }

    def __init__(self,bleBlimp,axisToMotorMap,keyMap):
        Controller.__init__(self,bleBlimp)
        # super(self, Controller).__init__(bleBlimp)
        self.axisToMotorMap = axisToMotorMap
        self.keyMap = keyMap
        self.handledKeys = self.keyMap.values()

    def handleEvt(self,evt):
        keyAction = [km[0] for km in self.keyMap.items() if km[1]==evt.key][0]

        if keyAction == 'i':
            if evt.type==KEYUP:
                self.bleBlimp.setIgniterState("00")
            else:
                self.bleBlimp.setIgniterState("01")
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

        self.bleBlimp.autoTxUpdate()


class XboxController(Controller):
    def __init__(self,bleBlimp,axisMap,igniterAxis,joystick,deadzone=0.3):
        Controller.__init__(self,bleBlimp)
        self.axisMap = axisMap
        self.joystick = joystick
        #used to track changes in state; send only upon state change.
        self.axisState = []
        self.deadzone = deadzone
        self.igniterAxis = igniterAxis

    def undeadzone(self,x):
        return max(0,min((abs(x)-self.deadzone)/(1-self.deadzone),1))

    def handleXbox(self):
        nowAxisState = ["00","00","00","00"]
        self.joystick.init()
        for axis in self.axisMap.items():
            # axis[1][0]  --motorIndex
            # axis[1][1]  --xbox axis
            # axis[1][2]  --axis direction multiplier
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
        
        if self.joystick.get_axis( self.igniterAxis)>0:
            self.bleBlimp.setIgniterState("01")
            nowAxisState[3] = "01"
        else:
            self.bleBlimp.setIgniterState("00")
            nowAxisState[3] = "00"

        if nowAxisState != self.axisState:
            self.bleBlimp.autoTxUpdate()
            self.axisState = nowAxisState
            # print nowAxisState


def numToMotorCode(x):
    # input: x, a number in [0, 1]
    # output: a valid DRV8830 hex code [0x06, 0x3F]

    # retain the last two chars for transmission, and pad with a 0 as needed
    hexStr = hex(int(round(0x06 + x*(0x3f-0x06))))[-2:]
    hexStr = "0"+hexStr[1] if hexStr[0]=="x" else hexStr
    return hexStr


