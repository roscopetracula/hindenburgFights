import Queue
import select
import socket
import threading
import time
import traceback
import string
import struct
from bluepy.bluepy import btle
from pgu import gui
from lib.constants import XBOX, KEYBOARD

# Flags
DEBUG_TX = False        # Print debug messages for transmissions.
DEBUG_RX = True         # Print debug messages for generic received
                        # text.
DEBUG_UPDATE = False    # Print debug messages for received updates.
DEBUG_CONNECT = False   # Print debug messages for connections and
                        # disconnections.

# Constants
TRANSMISSION_TIMEOUT  = 0.75 # Send an update if we haven't transmitted
                             # in this period (s).
MIN_TRANSMIT_INTERVAL = 0.1 # Don't transmit more than once in this
                            # period (s).
MAX_CONNECT_TIME = 0.5      # Maximum time (s) to spend on a
                            # connection attempt before timing out.
                        
# Colors
RED=(255,0,0)
GREEN=(0,255,0)
BLUE=(0,0,255)
CYAN=(0,255,255)
PURPLE=(255,0,255)
YELLOW=(255,255,0)        
WHITE=(255,255,255)
BLACK=(0,0,0)

BLIMP_OUTER_BORDER=3
BLIMP_INNER_BORDER=1
BLIMP_AXIS_WIDTH=120
BLIMP_AXIS_SLIDER_WIDTH=BLIMP_AXIS_WIDTH*0.9
BLIMP_AXIS_SLIDER_HEIGHT=100

class bleBotGui():
    axisNoMap = {0:0, 1:2, 2:1}
    axisDirMap = {0:"01", 1:"01", 2:"01"}

    def __init__( self, ble_adr, ble_bot, type ):
        # Set up the frame (table) representing this bot's controls.
        self.bot = ble_bot
        self.frame = gui.Table()

        # Build the table from the gui parts.
        #####################################

        # Add the heading.
        self.frame.tr()
        if (type == XBOX):
            typename = "xbox"
        else:
            typename = "kbd"
        self.frame.td(gui.Label("{:s} ({:s})".format(ble_adr, typename)), colspan=6, style={'border_left':BLIMP_OUTER_BORDER, 'border_right':BLIMP_OUTER_BORDER, 'border_top':BLIMP_OUTER_BORDER, 'border_bottom':BLIMP_INNER_BORDER})
    
        # Build the table of axes, including sliders and faults.
        self.axisLabels = [gui.Label("Throttle"), gui.Label("Pitch"), gui.Label("Yaw"), gui.Label("Igniter")]
        self.axisSliders = [gui.VSlider(value=0, min=-63, max=63, size=1, height=BLIMP_AXIS_SLIDER_HEIGHT), gui.VSlider(value=0, min=-63, max=63, size=1, height=BLIMP_AXIS_SLIDER_HEIGHT), gui.HSlider(value=0, min=-63, max=63, size=1, width=BLIMP_AXIS_SLIDER_WIDTH), gui.Label("Temp")]
        self.axisBorders = [{'border_right':BLIMP_INNER_BORDER, 'border_left':BLIMP_OUTER_BORDER}, {'border_right':BLIMP_INNER_BORDER}, {'border_right':BLIMP_OUTER_BORDER}, {}]
        self.axisFaultLabel = [gui.Label("No Fault", background=GREEN), gui.Label("No Fault", background=GREEN), gui.Label("No Fault", background=GREEN), gui.Label("No Fault", background=GREEN)]
        # Note that we're currently ignoring the igniter box.
        self.frame.tr()
        for i in range(0, 3):         
            self.frame.td(self.axisLabels[i], style=self.axisBorders[i], colspan=2, width=BLIMP_AXIS_WIDTH)
        self.frame.tr()
        for i in range(0, 3): 
            self.frame.td(self.axisSliders[i], style=self.axisBorders[i], colspan=2)
        self.frame.tr()
        for i in range(0, 3): 
            self.frame.td(self.axisFaultLabel[i], style=self.axisBorders[i], colspan=2)

        # Build the status section.
        self.rssiTempLabel = gui.Label("? / ?\xb0F")
        self.voltageLabel = gui.Label("?v", background=GREEN)
        self.trgLabel = gui.Label("trg", background=GREEN);
        self.ignLabel = gui.Label("ign", background=GREEN);
        self.frame.tr()
        self.frame.td(self.rssiTempLabel, style={'border_left':BLIMP_OUTER_BORDER, 'border_right':BLIMP_INNER_BORDER, 'border_top':BLIMP_INNER_BORDER, 'border_bottom':BLIMP_INNER_BORDER}, colspan=2)
        self.frame.td(self.voltageLabel, style={'border_top':BLIMP_INNER_BORDER, 'border_bottom':BLIMP_INNER_BORDER, 'border_right':BLIMP_INNER_BORDER}, colspan=2)
        self.frame.td(self.trgLabel, style={'border_top':BLIMP_INNER_BORDER, 'border_bottom':BLIMP_INNER_BORDER, 'border_right':BLIMP_INNER_BORDER}, colspan=1)
        self.frame.td(self.ignLabel, style={'border_top':BLIMP_INNER_BORDER, 'border_bottom':BLIMP_INNER_BORDER, 'border_right':BLIMP_OUTER_BORDER}, colspan=1)
        # Build the connection info and enable/disable button.
        if bleBot.DEFAULT_ENABLED:
            self.stateLabel = gui.Label("Waiting...", background=YELLOW)
            self.disableButtonLabel = gui.Label("Disable")
        else:
            self.stateLabel = gui.Label("DISABLED", background=RED)
            self.disableButtonLabel = gui.Label("Enable")
        self.disableButton = gui.Button(self.disableButtonLabel)
        self.disableButton.connect(gui.CLICK, self.doDisableOrEnable, None)
        self.resetButtonLabel = gui.Label("Reset")
        self.resetButton = gui.Button(self.resetButtonLabel)
        self.resetButton.disabled = True
        self.resetButton.connect(gui.CLICK, self.doReset, None)
        self.frame.tr()
        self.frame.td(self.stateLabel, style={'border_bottom':BLIMP_OUTER_BORDER, 'border_left':BLIMP_OUTER_BORDER, 'border_right':BLIMP_INNER_BORDER}, colspan=2)
        self.frame.td(self.disableButton, style={'border_bottom':BLIMP_OUTER_BORDER}, colspan=2)
        self.frame.td(self.resetButton, style={'border_bottom':BLIMP_OUTER_BORDER, 'border_right':BLIMP_OUTER_BORDER}, colspan=2)

        # Done!
        return
    
    def updateConnectionState(self):
        if self.bot.connectionState == bleBot.CONNECTED:
            self.stateLabel.set_text("Connected")
            self.stateLabel.style.background = GREEN
            self.resetButton.disabled = False
        elif self.bot.connectionState == bleBot.CONNECTING:
            self.stateLabel.set_text("Connecting...")
            self.stateLabel.style.background = YELLOW
            self.resetButton.disabled = True
        elif self.bot.connectionState == bleBot.FAILED:
            self.stateLabel.set_text("Retrying...")
            self.stateLabel.style.background = YELLOW
            self.resetButton.disabled = True
        elif self.bot.connectionState == bleBot.WAITING or self.bot.connectionState == bleBot.TIMED_OUT:
            self.stateLabel.set_text("Waiting...")
            self.stateLabel.style.background = YELLOW
            self.resetButton.disabled = True
        elif self.bot.connectionState == bleBot.DISABLED:
            self.stateLabel.set_text("DISABLED")
            self.stateLabel.style.background = RED
            self.resetButton.disabled = True
        elif self.bot.connectionState == bleBot.MISSING:
            self.stateLabel.set_text("MISSING")
            self.stateLabel.style.background = RED
            self.resetButton.disabled = True
        else:
            self.stateLabel.set_text("UNKNOWN {:d}".format(self.bot.connectionState))
            self.stateLabel.style.background = PURPLE            
            self.resetButton.disabled = True
        return

    def doReset(self, value):
        self.bot.reset()
        
    def doDisableOrEnable(self, value):
        if self.bot.connectionState != bleBot.DISABLED:
            self.bot.disable()
        else:
            self.bot.enable()

    def setAxis(self, axisNo, newDirection, newValue):
        newValue = int(newValue, 16)
        axisNo = self.axisNoMap[axisNo]
        self.axisSliders[axisNo].value =  newValue if newDirection == self.axisDirMap[axisNo] else -newValue

    def updateFaults(self):
        # We start all faults as green/No Fault.  When faults are
        # active, it turns red.  When they are no longer active, it
        # turns yellow but the fault information remains.
        for i in range(0, 3):
            if (self.bot.lastFault[i] == 0):
                if (self.axisFaultLabel[i].style.background == RED):
                    self.axisFaultLabel[i].style.background = YELLOW
            else:
                self.axisFaultLabel[i].set_text(self.bot.decodeFaultsShort(self.bot.lastFault[i]))
                self.axisFaultLabel[i].style.background = RED
            
class bleBot():
    # Constants
    RETURN_MSG_STRING = 0x00
    RETURN_MSG_UPDATE = 0x01
    RETURN_MSG_FAULT = 0x02   
    MISSING=-2
    DISABLED=-1
    CONNECTED=0
    TIMED_OUT=1
    WAITING=2
    CONNECTING=3
    FAILED=4
    DEFAULT_ENABLED = True  # Enable blimps when the program is started.

    def __init__( self, ble_adr, type ):

        # Set up connection-related data.
        self.protocolversion = "01"
        self.controller = None
        self.ble_adr = ble_adr
        self.btlePeripheral = btle.Peripheral()
        self.btleDebug = False
        if bleBot.DEFAULT_ENABLED:
            self.connectionState = self.WAITING
        else:
            self.connectionState = self.DISABLED
        self.curRSSI = "?"
        self.curTemp = "?"
        self.motorState = [("00","00"), ("00","00"), ("00","00")]
        self.lastTxState = [("00","00"), ("00","00"), ("00","00")]
        self.igniterState = ("00","00") # send ["01",xx] for on, anything else for off
        self.lastTxIgniterState = ("00","00") # send ["01",xx] for on, anything else for off
        self.ignState = 0
        self.trgState = 0
        self.lastTxTime = 0
        self.counter = 0xff
        self.curReceiveString = ""
        self.lastFault = [0, 0, 0]
        self.lastFaultTime = [0, 0, 0]
        self.lastConnectStart = 0
        self.lastUpdate = time.time()
        self.gui = bleBotGui(ble_adr, self, type)
        return

    def reset(self):
        if self.connectionState == self.CONNECTED:
            self.sendMessage("040000")
            
    def disable(self):
        if self.connectionState != self.DISABLED:
            self.disconnect()
            self.updateConnectionState(bleBot.DISABLED)
            self.gui.disableButtonLabel.set_text("Enable")
        return
    
    def enable(self):
        if self.connectionState == self.DISABLED:
            self.updateConnectionState(bleBot.WAITING)
            self.gui.disableButtonLabel.set_text("Disable")
        return
    
    def updateConnectionState(self, newConnectionState):
        self.connectionState = newConnectionState
        self.gui.updateConnectionState()
        return

    def decodeFaults(self, faultValue):
        faults = [];
        if (faultValue & 0x01):
            faults.append("FAULT")
        if (faultValue & 0x02):
            faults.append("OCP")
        if (faultValue & 0x04):
            faults.append("UVLO")
        if (faultValue & 0x08):
            faults.append("OTS")
        if (faultValue & 0x10):
            faults.append("ILIMIT")
        return faults

    def decodeFaultsShort(self, faultValue):
        return "{:s}|{:s}|{:s}|{:s}|{:s}".format("FA" if faultValue & 0x01 else "fa", "O" if faultValue & 0x02 else "oc", "UV" if faultValue & 0x04 else "uv", "OT" if faultValue & 0x08 else "ot", "IL" if faultValue & 0x10 else "il") 
    
    def handleNotification(self, cHandle, data): 
        if (cHandle <> 14):
            print "MESSAGE RECEIVED ON UNEXPECTED HANDLE [", cHandle, "]: ", data
            return

        # First byte is the command; the rest is the actual payload.
        rcvCmd = ord(data[0])
        data = data[1:]
        
        if rcvCmd == self.RETURN_MSG_STRING:
            # For strings, print the complete string if there's a null termination,
            self.curReceiveString = self.curReceiveString + data
            if (string.find(data, "\x00") != -1):
                if DEBUG_RX:
                    print "{:s} {:f} rcv> {:s}".format(self.ble_adr, time.time(), self.curReceiveString)
                self.curReceiveString = ""
        elif rcvCmd == self.RETURN_MSG_UPDATE:
            # The update message starts with a 4 byte RSSI integer and then a 4 byte temperature float.
            self.curRSSI = struct.unpack("<i", "".join(data[0:4]))[0]
            self.curTemp = struct.unpack("f", "".join(data[4:8]))[0]

            # next 3 bytes: fault data
            for i in range(0, 3):
                j = self.gui.axisNoMap[i]
                self.lastFault[j] = ord(data[8+i])
                if (self.lastFault[j] != 0):
                    self.lastFaultTime[j] = time.time()
            self.gui.updateFaults()

            # two bytes: igniter, trigger
            # Update the igniter and trigger displays if they have changed.
            if (self.ignState != ord(data[8+3])):
                self.ignState = ord(data[8+3])
                if self.ignState:
                    self.gui.ignLabel.set_text("IGN")
                    self.gui.ignLabel.style.background = RED
                else:
                    self.gui.ignLabel.set_text("ign")
                    self.gui.ignLabel.style.background = GREEN
            if (self.trgState != ord(data[8+4])):
                self.trgState = ord(data[8+4])
                if self.trgState:
                    self.gui.trgLabel.set_text("TRG")
                    self.gui.trgLabel.style.background = RED
                else:
                    self.gui.trgLabel.set_text("trg")
                    self.gui.trgLabel.style.background = GREEN

            self.returnStatus = ord(data[13])
            self.batteryVoltage = struct.unpack("<H", "".join(data[14:16]))[0] * 0.01;
            self.gui.rssiTempLabel.set_text("{:d} / {:.1f}\xb0F".format(self.curRSSI,self.curTemp))
            self.gui.voltageLabel.set_text("{:.2f}v ".format(self.batteryVoltage))

            # Match the voltage reading color to the report of low voltage.
            if (self.returnStatus & 0x01):
                if (self.gui.voltageLabel.style.background != RED):
                    self.gui.voltageLabel.style.background = RED;
            else:    
                if (self.gui.voltageLabel.style.background != GREEN):
                    self.gui.voltageLabel.style.background = GREEN;
                
            # Give us debug info.
            if DEBUG_UPDATE:
                curTime = time.time()
                print "{:s} update {:.03f}> rssi {:d}, temp {:.1f}, batt (:.2f), ign: {:d}, trg: {:d}, faults: {:02x}/{:02x}/{:02x} {:s}/{:s}/{:s}".format(self.ble_adr, curTime - self.lastUpdate, self.curRSSI, self.curTemp, self.batteryVoltage, self.ignState, self.trgState, self.lastFault[0], self.lastFault[1], self.lastFault[2], self.decodeFaults(self.lastFault[0]), self.decodeFaults(self.lastFault[1]), self.decodeFaults(self.lastFault[2]))
                self.lastUpdate = curTime
                

        elif rcvCmd == self.RETURN_MSG_FAULT:
            # Decode the raw fault data.
            faultMotor = ord(data[0])
            faultValue = ord(data[1])
            self.lastFault[faultMotor] = faultValue
            self.lastFaultTime[faultMotor] = time.time()
            self.gui.updateFaults()
            faults = self.decodeFaults(faultValue)            
            print "motor {:d} fault: {:s}".format(faultMotor, faults)
            
        else:
            print "received unknown command:", rcvCmd
            
    def connect( self ):

        if (self.ble_adr == "dummy"):
            print "dummy connection established"
            self.updateConnectionState(self.CONNECTED)
            return self

        if self.connectionState == self.WAITING:
            if DEBUG_CONNECT:
                print self.ble_adr, 'attempting to CONNECT'
            self.lastConnectStart = time.time()
            self.updateConnectionState(self.CONNECTING)
        elif self.connectionState == self.FAILED:
            if time.time() - self.lastConnectStart > MAX_CONNECT_TIME:
                if DEBUG_CONNECT:
                    print self.ble_adr, 'connection TIMED OUT'
                self.updateConnectionState(self.TIMED_OUT)
                return
            else:
                if DEBUG_CONNECT:
                    print self.ble_adr, 'attempting to CONNECT again'
                self.updateConnectionState(self.CONNECTING)
        else:
            print "ERROR, connect() from unexpected connectionState {:d}".format(self.connectionState)
            
        try:
            self.btlePeripheral.connect_async(self.ble_adr, btle.ADDR_TYPE_RANDOM)
        except OSError as e:
            print "-------------------------------------------------------------------"
            print "CONNECTION FAILED"
            print e
            print "OSError raised.  bluepy-helper is likely missing or not executable."
            print "To compile, run \"make clean; make\" from bluepy/bluepy directory."
            print "-------------------------------------------------------------------"
            print e
            self.updateConnectionState(self.FAILED)
            return self
        return self

    def checkCompleteConnection( self ):
        if time.time() - self.lastConnectStart > MAX_CONNECT_TIME:
            if DEBUG_CONNECT:
                print self.ble_adr, 'connection TIMED OUT'
            self.disconnect()
            self.updateConnectionState(self.TIMED_OUT)
            return

        try: 
            curStat = self.btlePeripheral.connect_stat()
        except btle.BTLEException as e:
            if (e.code == btle.BTLEException.DISCONNECTED):
                print "----------------------------------------------------------------------"
                print "CONNECTION FAILED"
                print e
                print "BTLEException raised.  bluepy-helper may or may not be a zombie."
                print "Try running \"killall bluepy-helper\" before running blimpControl again."
                print "----------------------------------------------------------------------"
                self.updateConnectionState(self.FAILED)
                return
            else:
                raise

        if (curStat == "tryconn"):
            return
        elif (curStat != "conn"):
            print "{:s} UNKNOWN CONNECTION STATUS {:s}; ASSUMING CONNECTION FAILURE".format(self.ble_adr, curStat)
            return

        self.updateConnectionState(self.CONNECTED)
        self.btlePeripheral.setDelegate(self)
        if DEBUG_CONNECT:
            print self.ble_adr, "connection successful"
        
        # If debugging, print info about connection.  We don't
        # currently show descriptors, but we could.
        if (self.btleDebug):
            self.btleServices = self.btlePeripheral.getServices()
            for svc in self.btleServices:
                print "------ SERVICE: ",svc
                chars = svc.getCharacteristics()
                for char in chars:
                    print "CHARACTERISTIC ",char.getHandle(),": ",char," ",char.propertiesToString()

        self.btlePeripheral.writeCharacteristic(0x000f, "0300".decode("hex"), False)
        return
    
    def char_write_cmd( self, value ):
        # in BLE gatttool,
        # for IMUduino: char-write-cmd 0x000b 41424344
        # for RFduino: char-write-cmd 0x0011 41424344555555
        if DEBUG_TX:
            print "{:s} {:s}> {:s}".format(self.ble_adr, "snd" if self.connectionState == self.CONNECTED else "noconn", value)

        if (self.connectionState != self.CONNECTED):
            return
        
        # Fake sending if we are using a dummy blimp.
        if (self.ble_adr == "dummy"):
            return
        
        # Write to the output characteristic.
        try:
            self.btlePeripheral.writeCharacteristic(0x0011, value.decode("hex"), False)
        except btle.BTLEException as e:
            if (e.code == btle.BTLEException.DISCONNECTED):
                if DEBUG_CONNECT:
                    print self.ble_adr, "DISCONNECTED"
                self.updateConnectionState(self.WAITING)
                pass
            else:
                raise
        return

    def cleanup( self ):
        # Just disconnect.
        self.disconnect()
        return

    def disconnect( self ):

        if DEBUG_CONNECT:
            print self.ble_adr, 'attempting to DISCONNECT'

        self.updateConnectionState(self.WAITING)
        if (self.ble_adr != "dummy"):
            # Disconnect from bluetooth.
            self.btlePeripheral.disconnect()

        if DEBUG_CONNECT:
            print self.ble_adr, "connection closed"

    def reconnect( self ):
        if DEBUG_CONNECT:
            print self.ble_adr, 'attempting to RECONNECT...'

        # bluepy is currently unstable if another connection is
        # attempted while one is pending.  If we somehow need this
        # functionality, we can add it later.
        if self.connectionState == self.CONNECTING:
            if DEBUG_CONNECT:
                print self.ble_adr, "already connecting, ignoring reconnect attempt."
            return
        
        # Disconnect and reconnect.
        self.disconnect()
        self.connect()

    # Sends a message, prepending any protocol data.
    def sendMessage(self, data):
        self.char_write_cmd(self.protocolversion + format(self.counter, '02x') + data)
        # The counter starts at 0xFF to indicate a new connection and
        # then always rolls over after 0xFE.  This bizarre way of
        # calculating the modulus is used because it starts at 0xFF so
        # incrementing would roll it over, but only on the very first
        # try.  When someone has 5 minutes they might improve this.
        # Another option would be to do something along the lines of
        # ((counter + 1) % 256) % 255.
        self.counter = self.counter + 1 if self.counter < 254 else 0

    # Send a full state update.
    def txState(self):
        tmpMsg = ""
        for i in range(3):
            self.lastTxState[i] = self.motorState[i] 
            tmpMsg += "0"+str(i)+"".join(self.motorState[i])
        self.lastTxIgniterState = self.igniterState
        tmpMsg += "03"+"".join(self.igniterState)
        self.lastTxTime = time.time()
        self.sendMessage(tmpMsg)

    # Send a full state update when necessary (time out, new data, etc).
    def autoTxUpdate(self):
        curTime = time.time()
        
        # Don't send anything if not enough time has passed.
        if (curTime < self.lastTxTime + MIN_TRANSMIT_INTERVAL):
            return

        # If we've timed out, always retransmit.
        if (curTime > self.lastTxTime + TRANSMISSION_TIMEOUT):
            self.txState()
            
        # Transmit everything if anything has changed.
        changeFound = False
        for i in range(3):
            if self.lastTxState[i] != self.motorState[i]:
                self.lastTxState[i] = self.motorState[i]
                changeFound = True
        if self.igniterState != self.lastTxIgniterState:
            # Note that interim igniter states can be lost. If we ever
            # return to manual triggering, we should make sure to keep
            # track of interim triggers in a separte variable.
            self.lastTxIgniterState = self.igniterState
            changeFound = True

        # If anything changed, retransmit.
        if changeFound: 
            self.txState()
            
    def setMotorState(self, motorIndex, motorDirection, motorSpeed):
        self.motorState[motorIndex] = (motorDirection,motorSpeed)
        self.gui.setAxis(motorIndex, motorDirection, motorSpeed)
        
    def setIgniterState(self, onOrOff):
        self.igniterState = (onOrOff,"00")
