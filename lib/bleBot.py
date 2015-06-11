import Queue
import string
import select
import socket
import threading
import time
import traceback
from bluepy.bluepy import btle

TRANSMISSION_TIMEOUT = .9

class bleBot():
    def __init__( self, ble_adr ):
        # Connect to the remote BLE device.
        self.ble_adr = ble_adr
        self.btlePeripheral = btle.Peripheral()
        self.btleDebug = False
        self.motorState = [("00","00"), ("00","00"), ("00","00")]
        self.lastTxState = [("00","00"), ("00","00"), ("00","00")]
        self.igniterState = ("00","00") # send ["01",xx] for on, anything else for off
        self.lastTxIgniterState = ("00","00") # send ["01",xx] for on, anything else for off
        self.lastTxTime = 0
        self.counter = 0xff
        self.curReceiveString = ""

    def handleNotification(self, cHandle, data):
        if (cHandle == 14):
            self.curReceiveString = self.curReceiveString + data
            if (string.find(data, "\x00") != -1):
                print "rcv>", self.curReceiveString
                self.curReceiveString = ""
        else:
            print "MESSAGE RECEIVED ON UNEXPECTED HANDLE [", cHandle, "]: ", data

    def connect( self ):

        if (self.ble_adr == "dummy"):
            print "Dummy connection established."
            return self

        print "Preparing to connect. Address: " + self.ble_adr
        try:
            self.btlePeripheral.connect(self.ble_adr, btle.ADDR_TYPE_RANDOM)
        except OSError:
            print "------------------------------------------------------"
            print "OSError raised.  bluepy-helper is likely missing."
            print "To compile, run \"make\" from bluepy/bluepy directory."
            print "------------------------------------------------------"
            raise
        except btle.BTLEException:
            print "----------------------------------------------------------------"
            print "BTLEException raised.  bluepy-helper may be a zombie."
            print "Try running \"killall bluepy-helper\" before running this again."
            print "----------------------------------------------------------------"
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
        return self


    def char_write_cmd( self, value ):
        # in BLE gatttool,
        # for IMUduino: char-write-cmd 0x000b 41424344
        # for RFduino: char-write-cmd 0x0011 41424344555555
        print self.ble_adr, value

        # Fake sending if we are ussing a dummy blimp.
        if (self.ble_adr == "dummy"):
            return
        
        # Write to the output characteristic.
        self.btlePeripheral.writeCharacteristic(0x0011, value.decode("hex"), False)
        return

    def cleanup( self ):
        # If we're using a dummy, just let us know and return.
        if (self.ble_adr == "dummy"):
            print "Dummy connection closed."
            return

        print self.ble_adr, ': attempting to disconnect'

        self.btlePeripheral.disconnect()
        return

    def reconnect( self ):
        print self.ble_adr, 'attempting to RECONNECT'

        # Simply return if a dummy device is reconnected.
        if (self.ble_adr == "dummy"):
            return
        
        # Otherwise, send a disconnect commend and re-call connect().
        self.btlePeripheral.disconnect()
        self.btlePeripheral.connect(self.ble_adr, btle.ADDR_TYPE_RANDOM)


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
