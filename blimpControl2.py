import pygame, sys
from pygame.locals import *

import os, sys
from ctypes.util import find_library
import pexpect, traceback, threading, Queue, time, socket, select
import blescan


if not os.geteuid() == 0:
    sys.exit("script only works as root")

btlib = find_library("bluetooth")
if not btlib:
    raise Exception(
        "Can't find required bluetooth libraries"
    )


class bleBot:
    def __init__( self, ble_adr ):
        self.ble_adr = ble_adr
        self.con = pexpect.spawn('gatttool -b ' + self.ble_adr + ' -I -t random')
        self.con.delaybeforesend = 0 #THIS LINE IS SUPER IMPORTANT
        self.con.expect('\[LE\]', timeout=1)

        # for IMUduino: char-write-cmd 0x000b 41424344; hande-> 'b'
        # for RFduino:  char-write-cmd 0x0011 41424344; handle-> '11'
        self.handle = '0011' #!! this is the TX service on the nRF8001 adafruit breakout with callbackEcho sketch

        self.motorState = [("00","00"), ("00","00"), ("00","00")]
        self.lastTx = 0


    def connect( self ):
        print "Preparing to connect. Address: " + self.ble_adr
        self.con.sendline('connect')
        try:
            self.con.read_nonblocking(2048,0) #flush the read pipe!! SUPER IMPORTANT
        except:
            pass
        i = self.con.expect(['Attempting', 'Error'], timeout=1)
        if i == 0:
            print 'Attempting to connect'
            j = self.con.expect(['Connection successful', 'No route', 'busy', pexpect.TIMEOUT], timeout = 1)
            if j == 0:
                print self.ble_adr, ': connected!'
            if j == 1:
                print self.ble_adr, ': No route to host, is USB dongle plugged in?'
                self.cleanup()
            if j == 2:
                print self.ble_adr, ': Device busy, is something else already connected to it?'
                c = True
                while c:
                    inp = raw_input('Try hitting reset. Type "y" to continue or "n" to quit.')
                    if inp.lower().startswith('y'):
                        self.con.sendline('connect')
                        try:
                            self.con.read_nonblocking(2048,0) #flush the read pipe!! SUPER IMPORTANT
                        except:
                            pass
                        k = self.con.expect(['Connection successful', pexpect.TIMEOUT], timeout = 1)
                        print 'k: ', k
                        if k == 0:
                            print self.ble_adr, ': connected!'
                        if k == 1:
                            print self.ble_adr, ': Could not connect'
                            self.cleanup()
                        break
                    elif inp.lower().startswith('n'):
                        self.cleanup()
                        break
                    else:
                        print 'Did not understand command. Try again.'
            if j == 3:
                print 'Attempting to connect, is device on and in range? '
                #foostr = raw_input('Type anything to continue, or enter to cancel')
                self.con.sendline('connect')
                try:
                    self.con.read_nonblocking(2048,0) #flush the read pipe!! SUPER IMPORTANT
                except:
                    pass
                k = self.con.expect(['Connection successful', pexpect.TIMEOUT], timeout = 3)
                print 'k: ', k
                if k == 0:
                    print self.ble_adr, ': connected!'
                if k == 1:
                    print self.ble_adr, ': Could not connect'
                    self.cleanup()
        if i == 1:
            print 'Is USB dongle plugged in?'
            self.cleanup()
        return self
         

    def char_write_cmd( self, value ):
        # in BLE gatttool,
        # for IMUduino: char-write-cmd 0x000b 41424344
        # for RFduino: char-write-cmd 0x0011 41424344555555
        cmd = 'char-write-cmd 0x%s %s' % (self.handle, value)
        print self.ble_adr, cmd
        self.con.sendline( cmd )
        try:
            rnb = self.con.read_nonblocking(2048,0) #flush the read pipe!! SUPER IMPORTANT
            # print "----RNB----\n", rnb, "\n-----END RNB-----"
            if "Command Failed:" in rnb:
                print "attempt reconnect..."
                # FIXME this is where the reconnect should happen
                # should it just be: self.connect() ??
                self.connect()
        except:
            print "could not 'flush read pipe'"
        # print 'After sending command, before: ', self.con.before, 'after :', self.con.after
        return

    def cleanup( self ):
        print self.ble_adr, ': attempting to disconnect'
        try:
            self.con.sendline('disconnect')
            self.con.sendline('exit')
            try:
                self.con.read_nonblocking(2048,0) #flush the read pipe!! SUPER IMPORTANT
            except:
                pass
            isalive = self.con.terminate(force=True)
            print self.ble_adr, ': has been terminated? ', isalive
            self.con.close(force=True)
            #print self.ble_adr, 'is alive: ', self.con.isalive()
        except OSError:
            print self.ble_adr, ': OSError'
            pass
        return

    def reconnect( self ):
        print self.ble_adr, 'attempting to RECONNECT'
        self.con.sendline('disconnect')
        try:
            self.con.read_nonblocking(2048,0)
        except:
            pass
        self.connect()

    def transmitState(self):
        self.char_write_cmd("".join(["".join(tup) for tup in self.motorState]))
        self.lastTx = time.time()

    def reTxState(self):
        if time.time() - self.lastTx > reTxTimeout and any([byte[0]!="00" for byte in self.motorState]):
            #...send the motorState if it hasn't been sent in a while and it's non-zero.
            self.transmitState()


    def setMotorState(self, motorIndex, motorDirection, motorSpeed):
        self.motorState[motorIndex] = (motorDirection,motorSpeed)



def numToMotorCode(x):
    # x is a number in [0,1] that we map to the valid range of DRV8830 hex codes: 0x06 to 0x3F
    # we only retain the last two characters for transmission, and pad with a 0 if need be
    hexStr = hex(int(round(0x06 + x*(0x3f-0x06))))[-2:]
    hexStr = "0"+hexStr[1] if hexStr[0]=="x" else hexStr
    return hexStr



class Controller(object):
    def __init__(self,bleBlimp):
        self.bleBlimp = bleBlimp
    def quit(self):
        pass
        # self.bleBlimp.cleanup()
        # pygame.quit()
        # sys.exit()
    def reconnect(self):
        self.bleBlimp.reconnect()


class KbdCtrl(Controller):
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
            motorPolarity = "00"
        elif motorPolarity:
            motorDirection = "01"
        else:
            motorDirection = "02"

        self.bleBlimp.setMotorState(motorIndex, motorDirection, numToMotorCode(1))
        self.bleBlimp.transmitState()



class XboxCtrl(Controller):
    def __init__(self,bleBlimp,axisMap,joystick,deadzone=0.3):
        Controller.__init__(self,bleBlimp)
        self.axisMap = axisMap
        self.joystick = joystick
        self.axisState = ["00","00","00"] #used to track changes in state; send only upon state change.
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
            # print "xbox tx"





pygame.init()
BLACK = (0,0,0)
WIDTH = 300
HEIGHT = 300
windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)

windowSurface.fill(BLACK)


imuDuino = "FD:B3:BC:98:29:EF"
rfduino = "C9:44:30:80:0D:1A"
rfduinoBlimpMini1 = "D2:9F:90:14:98:4C"
rfduinoBlimpTiny1 = "FC:E5:E2:09:9C:0E"

# b1 = bleBot(rfduinoBlimpTiny1)
# b2 = bleBot(rfduino)
b2 = bleBot(rfduinoBlimpMini1)

# b1.connect()
b2.connect()

blimps=[b2]

# the tuples for the axisToMotorMap give the axis and the direction of the motor.
# the second entry in the tuple is a boolean flag saying whether the motor defaults to cw or ccw.
keymap1 = {"f":pygame.K_UP, "b": pygame.K_DOWN, "r" :pygame.K_RIGHT, "l":pygame.K_LEFT, "u":pygame.K_o, "d": pygame.K_l}
# kbd1 = KbdCtrl(b1, {"f_b":(2,False), "r_l":(1,True), "u_d":(0,False)}, keymap1)
# kbds = [kbd1]
kbds=[]

# keymap2 = {"f":pygame.K_d, "b": pygame.K_c, "r" :pygame.K_v, "l":pygame.K_x, "u":pygame.K_a, "d": pygame.K_z, "quit":pygame.K_q, "reconnect":pygame.K_r}
# kbd2 = KbdCtrl(b2, {"f_b":(0,True), "r_l":(1,True), "u_d":(2,False)}, keymap2)
# kbds = [kbd1,kbd2]





pygame.joystick.init()
# the axisMap tuples are: (motorNumber, joystickAxNumber, joystickAxisDirection)
# where joystickAxisDirection is a multiplier on returned axis value to set the mapping of joystick direction to motor direction
xbox1 = XboxCtrl(b2, {"f_b":(0,4,-1.0), "r_l":(1,3,-1.0), "u_d":(2,1,1.0)}, pygame.joystick.Joystick(0))

xboxen = [xbox1]
# motorBytes = ["00","00","00"]

lastTx = time.time()
reTxTimeout=.9 #retransmit every x sec




while True:
    events = pygame.event.get()
    for event in events:
        # print event
        if (event.type == QUIT) or (event.type == KEYDOWN and event.key == pygame.K_q):
            for blimp in blimps:
                blimp.cleanup()
            # b1.cleanup()
            pygame.quit()
            sys.exit()
        if (event.type == KEYDOWN and event.key == pygame.K_r):
            for blimp in blimps:
                blimp.reconnect()

        if (event.type == KEYDOWN or event.type == KEYUP) and kbds:
            for kbd in kbds:
                if event.key in kbd.handledKeys:
                    kbd.handleEvt(event)
    for xbox in xboxen:
        xbox.handleXbox()
    for blimp in blimps:
        blimp.reTxState()
    time.sleep(0.0001)



