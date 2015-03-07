from config import CONTROLLERS
from lib.constants import XBOX, KEYBOARD
from lib.bleBot import bleBot
import pygame
from pygame.locals import *

def load_controllers():
    return [
        create_controller(controller_config)
        for controller_config in CONTROLLERS
    ]

def has_xbox_controller():
    for controller_config in CONTROLLERS:
        if controller_config['type'] == XBOX:
            return True
    return False

def create_controller(cfg):
    ble_control = bleBot(cfg['uuid'])
    ble_control.connect()

    if cfg['type'] == KEYBOARD:
        controller = KeyboardController(
            ble_control,
            cfg['orientation'],
            cfg['keys']
        )
    else:
        controller = XboxController(
            ble_control,
            cfg['orientation'],
            pygame.joystick.Joystick(
                cfg['joystick']
            )
        )

    return controller

class Controller(object):
    def __init__(self, bleBlimp):
        self.bleBlimp = bleBlimp

    def cleanup(self):
        self.bleBlimp.cleanup()

    def reconnect(self):
        self.bleBlimp.reconnect()

    def retransmit(self):
        self.bleBlimp.reTxState()

class KeyboardController(Controller):
    def __init__(self,bleBlimp,axisToMotorMap,keyMap):
        Controller.__init__(self,bleBlimp)
        # super(self, Controller).__init__(bleBlimp)
        self.axisToMotorMap = axisToMotorMap
        self.keyMap = keyMap
        self.handledKeys = self.keyMap.values()

    def handleEvt(self,evt):
        keyAction = [km[0] for km in self.keyMap.items() if km[1]==evt.key][0]
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
            motorDirection = "00"
        elif motorPolarity:
            motorDirection = "01"
        else:
            motorDirection = "02"

        self.bleBlimp.setMotorState(motorIndex, motorDirection, numToMotorCode(1))
        self.bleBlimp.transmitState()

class XboxController(Controller):
    def __init__(self,bleBlimp,axisMap,joystick,deadzone=0.3):
        Controller.__init__(self,bleBlimp)
        self.axisMap = axisMap
        self.joystick = joystick
        #used to track changes in state; send only upon state change.
        self.axisState = ["00","00","00"]
        self.deadzone = deadzone

    def undeadzone(self,x):
        return max(0,min((abs(x)-self.deadzone)/(1-self.deadzone),1))

    def handleXbox(self):
        thisAxisState = ["00","00","00"]
        self.joystick.init()
        for axis in self.axisMap.items():
            # axis[1][0]  --motorIndex
            # axis[1][1]  --xbox axis
            # axis[1][2]  --axis direction multiplier
            axisVal = self.joystick.get_axis( axis[1][1] ) * axis[1][2]
            if axisVal > self.deadzone:
                motorDirection = "01"
                motorSpeed = numToMotorCode(self.undeadzone(axisVal))
                # print "a",thisAxisState,self.deadzone
            elif axisVal < -self.deadzone:
                motorDirection = "02"
                motorSpeed = numToMotorCode(self.undeadzone(axisVal))
                # print "b",thisAxisState,self.deadzone
            else:
                motorDirection = "00"
                motorSpeed = "00"
                # print "c",thisAxisState,self.deadzone

            thisAxisState[axis[1][0]]=motorSpeed
            # self.bleBlimp.setMotorState(motorCode,)
            self.bleBlimp.setMotorState(axis[1][0], motorDirection, motorSpeed)

        if thisAxisState != self.axisState:
            print thisAxisState, self.bleBlimp.motorState,axisVal
            self.bleBlimp.transmitState()
            self.axisState = thisAxisState

def numToMotorCode(x):
    # input: x, a number in [0, 1]
    # output: a valid DRV8830 hex code [0x06, 0x3F]

    # retain the last two chars for transmission, and pad with a 0 as needed
    hexStr = hex(int(round(0x06 + x*(0x3f-0x06))))[-2:]
    hexStr = "0"+hexStr[1] if hexStr[0]=="x" else hexStr
    return hexStr

