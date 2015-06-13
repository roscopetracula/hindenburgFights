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

TRANSMISSION_TIMEOUT = .9

class bleBotGui():
    WIDTH=320
    HEIGHT=240
    AXIS_WIDTH=WIDTH/3
    BLIMP_OUTER_BORDER=3
    axisNoMap = {0:1, 1:2, 2:0}
    axisDirMap = {0:"01", 1:"01", 2:"02"}

    def __init__( self, ble_adr, ble_bot, type ):
        # State variables.
        self.isConnected = False
        
        # Set up the individual controls.
        self.bot = ble_bot
        self.outerFrame = gui.Table()
        self.outerFrame.tr()
        self.outerFrame.td(gui.Label(ble_adr + (" (xbox)" if type == XBOX else " (kbd)")), colspan=1024, style={'border_left':self.BLIMP_OUTER_BORDER, 'border_right':self.BLIMP_OUTER_BORDER, 'border_top':self.BLIMP_OUTER_BORDER})
        self.frame = gui.Table(width=self.WIDTH, style={'border_left':self.BLIMP_OUTER_BORDER, 'border_right':self.BLIMP_OUTER_BORDER, 'border_bottom':self.BLIMP_OUTER_BORDER})
        self.outerFrame.tr()
        self.outerFrame.td(self.frame)
    
        # Build the table from the gui parts.
        #####################################

        # Build the table of axes.
        self.axisOuterTable = gui.Table()
        self.frame.tr()
        self.frame.td(self.axisOuterTable, style={'border_top':1, 'border_bottom':1})
        self.axisLabels = [gui.Label("Throttle"), gui.Label("Pitch"), gui.Label("Yaw"), gui.Label("Igniter")]
        self.axisSliders = [gui.VSlider(value=0, min=-63, max=63, size=1, height=100), gui.VSlider(value=0, min=-63, max=63, size=1, height=100), gui.HSlider(value=0, min=-63, max=63, size=1, width=100), gui.Label("Temp")]
        self.axisBorders = [{'border_right':1}, {'border_right':1}, {}, {}]
        self.axisFaultLabel = [gui.Label("No Fault"), gui.Label("No Fault"), gui.Label("No Fault"), gui.Label("No Fault")]
        # Note that we're currently ignoring the igniter box.
        self.axisOuterTable.tr()
        for i in range(0, 3):         
            self.axisOuterTable.td(self.axisLabels[i], width=self.AXIS_WIDTH, style=self.axisBorders[i])
        self.axisOuterTable.tr()
        for i in range(0, 3): 
            self.axisOuterTable.td(self.axisSliders[i], style=self.axisBorders[i])
        self.axisOuterTable.tr()
        for i in range(0, 3): 
            self.axisOuterTable.td(self.axisFaultLabel[i], style=self.axisBorders[i])

        # Build the status section.
        self.statusTable = gui.Table()
        self.rssiLabel = gui.Label("RSSI: ?")
        self.tempLabel = gui.Label("Temp: ?")
        self.statusTable.tr()
        self.statusTable.td(self.rssiLabel, style={'border_right':1}, width=self.WIDTH/2)
        self.statusTable.td(self.tempLabel, width=self.WIDTH/2)
        self.frame.tr()
        self.frame.td(self.statusTable, style={'border_bottom':1})
                              
        # Build the table with the connect/reconnect info and buttons.
        self.connectionTable = gui.Table()
        self.frame.tr()
        self.frame.td(self.connectionTable)
        self.stateLabel = gui.Label("DISCONNECTED")
        self.connectButtonLabel = gui.Label("Connect")
        self.connectButton = gui.Button(self.connectButtonLabel)
        self.connectButton.connect(gui.CLICK,self.connectOrDisconnect,None)
        self.reconnectButton = gui.Button("Reconnect")
        self.reconnectButton.connect(gui.CLICK,self.reconnect,None)
        self.reconnectButton.disabled = True
        self.connectionTable.tr()
        self.connectionTable.td(self.stateLabel)
        self.connectionTable.td(self.reconnectButton)
        self.connectionTable.td(self.connectButton)

    def setConnected(self, isConnected):
        self.isConnected = isConnected
        if isConnected:
            self.connectButtonLabel.set_text("Disconnect")
            self.stateLabel.set_text("Connected")
            self.reconnectButton.disabled = False
        else:
            self.connectButtonLabel.set_text("Connect")
            self.stateLabel.set_text("DISCONNECTED")
            self.reconnectButton.disabled = True

    def reconnect(self, value):
        self.bot.reconnect()

    def connectOrDisconnect(self, value):
        if self.isConnected:
            self.bot.disconnect()
        else:
            self.bot.connect()

    def setAxis(self, axisNo, newDirection, newValue):
        newValue = int(newValue, 16)
        axisNo = self.axisNoMap[axisNo]
        self.axisSliders[axisNo].value =  newValue if newDirection == self.axisDirMap[axisNo] else -newValue

    def updateFaults(self):
        for i in range(0, 3):
            if (self.bot.lastFault[i] == 0):
                self.axisFaultLabel[i].set_text("No Fault")
            else:
                self.axisFaultLabel[i].set_text("Fault: {:02X}".format(self.bot.lastFault[i]))
            
class bleBot():
    RETURN_MSG_STRING = 0x00
    RETURN_MSG_UPDATE = 0x01
    RETURN_MSG_FAULT = 0x02   

    def __init__( self, ble_adr, type ):
        # Set up connection-related data.
        self.ble_adr = ble_adr
        self.btlePeripheral = btle.Peripheral()
        self.btleDebug = False
        self.isConnected = False # We may ultimately need more than two states here.
        self.curRSSI = "?"
        self.curTemp = "?"
        self.motorState = [("00","00"), ("00","00"), ("00","00")]
        self.lastTxState = [("00","00"), ("00","00"), ("00","00")]
        self.igniterState = ("00","00") # send ["01",xx] for on, anything else for off
        self.lastTxIgniterState = ("00","00") # send ["01",xx] for on, anything else for off
        self.lastTxTime = 0
        self.counter = 0xff
        self.curReceiveString = ""
        self.lastFault = [0, 0, 0]
        self.lastFaultTime = [0, 0, 0]
        self.gui = bleBotGui(ble_adr, self, type)
        
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
                print "rcv>", self.curReceiveString
                self.curReceiveString = ""
        elif rcvCmd == self.RETURN_MSG_UPDATE:
            # The update message is just a 4 byte RSSI integer and then a 4 byte temperature float.
            self.curRSSI = struct.unpack("<i", "".join(data[0:4]))[0]
            self.curTemp = struct.unpack("f", "".join(data[4:8]))[0]
            self.gui.rssiLabel.set_text("RSSI: {:d}".format(self.curRSSI))
            self.gui.tempLabel.set_text("Temp: {:.1f}\xb0F".format(self.curTemp))
            print "update> {:s} rssi {:d}, temp {:.1f}".format(self.ble_adr, self.curRSSI, self.curTemp)
        elif rcvCmd == self.RETURN_MSG_FAULT:
            # Decode the raw fault data.
            faultMotor = ord(data[0])
            faultValue = ord(data[1])
            self.lastFault[faultMotor] = faultValue
            self.lastFaultTime[faultMotor] = time.time()
            self.gui.updateFaults()
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
            
            print "motor {:d} fault: {:s}".format(faultMotor, faults)
            
        else:
            print "received unknown command:", rcvCmd
            
    def connect( self ):

        self.gui.stateLabel.set_text("Connecting...")
        if (self.ble_adr == "dummy"):
            print "dummy connection established"
            self.isConnected = True
            self.gui.setConnected(self.isConnected)
            return self

        self.isConnected = False
        print self.ble_adr, 'attempting to CONNECT'
        try:
            self.btlePeripheral.connect(self.ble_adr, btle.ADDR_TYPE_RANDOM)
        except OSError as e:
            print "-------------------------------------------------------------------"
            print "CONNECTION FAILED"
            print e
            print "OSError raised.  bluepy-helper is likely missing or not executable."
            print "To compile, run \"make clean; make\" from bluepy/bluepy directory."
            print "-------------------------------------------------------------------"
            print e
            self.gui.setConnected(self.isConnected)
            return self
        except btle.BTLEException as e:
            if (e.code == btle.BTLEException.DISCONNECTED):
                print "----------------------------------------------------------------------"
                print "CONNECTION FAILED"
                print e
                print "BTLEException raised.  bluepy-helper may be a zombie."
                print "Try running \"killall bluepy-helper\" before running blimpControl again."
                print "----------------------------------------------------------------------"
                self.gui.setConnected(self.isConnected)
                return self
            else:
                raise

        self.isConnected = True
        self.gui.setConnected(self.isConnected)
        self.btlePeripheral.setDelegate(self)
        print "connection attempt complete, status:", self.btlePeripheral.status()
        
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
        return self


    def char_write_cmd( self, value ):
        # in BLE gatttool,
        # for IMUduino: char-write-cmd 0x000b 41424344
        # for RFduino: char-write-cmd 0x0011 41424344555555
        print "{:s}> {:s} {:s}".format("snd" if self.isConnected else "noconn", self.ble_adr, value)

        if (not self.isConnected):
            return
        
        # Fake sending if we are using a dummy blimp.
        if (self.ble_adr == "dummy"):
            return
        
        # Write to the output characteristic.
        try:
            self.btlePeripheral.writeCharacteristic(0x0011, value.decode("hex"), False)
        except btle.BTLEException as e:
            if (e.code == btle.BTLEException.DISCONNECTED):
                print self.ble_adr, "DISCONNECTED"
                self.isConnected = False
                pass
            else:
                raise
            
        return

    def cleanup( self ):

        # Just disconnect.
        self.disconnect()

        return

    def disconnect( self ):

        self.gui.stateLabel.set_text("Disconnecting...")
        print self.ble_adr, 'attempting to DISCONNECT'
        self.isConnected = False
        self.gui.setConnected(self.isConnected)

        if (self.ble_adr == "dummy"):
            self.isConnected = False
        else: 
            # Disconnect from bluetooth.
            self.btlePeripheral.disconnect()

        print self.ble_adr, "connection closed"

    def reconnect( self ):
        print self.ble_adr, 'attempting to RECONNECT...'
        
        # Disconnect and reconnect.
        self.disconnect()
        self.connect()

    # Sends a message, prepending any protocol data.
    def sendMessage(self, data):
        self.char_write_cmd("00" + format(self.counter, '02x') + data)
        # The counter starts at 0xFF to indicate a new connection and
        # then always rolls over after 0xFE.  This bizarre way of
        # calculating the modulus is used because it starts at 0xFF so
        # incrementing would roll it over, but only on the very first
        # try.  When someone has 5 minutes they might improve this.
        # Another option would be to do something along the lines of
        # ((counter + 1) % 256) % 255.
        self.counter = self.counter + 1 if self.counter < 254 else 0
        self.lastTxTime = time.time()

    # Transmits a channel/data pair.
    def sendToChannel(self, channel, data):
        self.sendMessage(channel+data)

    def txStateChanges(self):
        tmpMsg = ""
        for i in range(3):
            if self.lastTxState[i] != self.motorState[i]:
                tmpMsg += "0"+str(i)+"".join(self.motorState[i])
                self.lastTxState[i] = self.motorState[i]
        if self.igniterState != self.lastTxIgniterState:
            tmpMsg += "03"+"".join(self.igniterState)
            self.lastTxIgniterState = self.igniterState
        if tmpMsg != "":
            # self.sendMessage(tmpMsg) # Use this to just send differences.
            self.reTxState(True)         # Use this to send all changes.
            
    def reTxState(self, force = False):
        if force or time.time() - self.lastTxTime > TRANSMISSION_TIMEOUT:
            #retransmit any states that haven't been sent in a while
            tmpMsg = ""
            for i in range(3):
                tmpMsg += "0"+str(i)+"".join(self.motorState[i])
            tmpMsg += "03"+"".join(self.igniterState)
            self.sendMessage(tmpMsg)

    def setMotorState(self, motorIndex, motorDirection, motorSpeed):
        self.motorState[motorIndex] = (motorDirection,motorSpeed)
        self.gui.setAxis(motorIndex, motorDirection, motorSpeed)
        
    def setIgniterState(self, onOrOff):
        self.igniterState = (onOrOff,"00")
