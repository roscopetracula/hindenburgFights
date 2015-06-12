import Queue
import select
import socket
import threading
import time
import traceback
import string
import struct
from bluepy.bluepy import btle

TRANSMISSION_TIMEOUT = .9

class bleBot():
    RETURN_MSG_STRING = 0x00
    RETURN_MSG_UPDATE = 0x01
    RETURN_MSG_FAULT = 0x02   

    def __init__( self, ble_adr ):
        # Set up connection-related data.
        self.ble_adr = ble_adr
        self.btlePeripheral = btle.Peripheral()
        self.btleDebug = False
        self.isConnected = False
        self.curRSSI = 0
        self.curTemp = 0.0
        self.motorState = [("00","00"), ("00","00"), ("00","00")]
        self.lastTxState = [("00","00"), ("00","00"), ("00","00")]
        self.igniterState = ("00","00") # send ["01",xx] for on, anything else for off
        self.lastTxIgniterState = ("00","00") # send ["01",xx] for on, anything else for off
        self.lastTxTime = 0
        self.counter = 0xff
        self.curReceiveString = ""
        
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
            print "update: rssi {:d}, temp {:.1f}".format(self.curRSSI, self.curTemp)
        elif rcvCmd == self.RETURN_MSG_FAULT:
            # Decode the raw fault data.
            faultMotor = ord(data[0])
            faultValue = ord(data[1])
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

        if (self.ble_adr == "dummy"):
            print "Dummy connection established."
            self.isConnected = True
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
            return self
        except btle.BTLEException as e:
            if (e.code == btle.BTLEException.DISCONNECTED):
                print "----------------------------------------------------------------------"
                print "CONNECTION FAILED"
                print e
                print "BTLEException raised.  bluepy-helper may be a zombie."
                print "Try running \"killall bluepy-helper\" before running blimpControl again."
                print "----------------------------------------------------------------------"
                return self
            else:
                raise

        # We should handle bluepy.bluepy.btle.BTLEException here.
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
        self.isConnected = True
        return self


    def char_write_cmd( self, value ):
        # in BLE gatttool,
        # for IMUduino: char-write-cmd 0x000b 41424344
        # for RFduino: char-write-cmd 0x0011 41424344555555
        print "{:s}> {:s} {:s}".format(self.isConnected and "snd" or "noconn", self.ble_adr, value)

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

        print self.ble_adr, 'attempting to DISCONNECT'
        self.isConnected = False
        
        if (self.ble_adr == "dummy"):
            print "Dummy connection closed."
            self.isConnected = False
            return self

        # Disconnect from bluetooth.
        self.btlePeripheral.disconnect()

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

    def setIgniterState(self, onOrOff):
        self.igniterState = (onOrOff,"00")
