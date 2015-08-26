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
from lib.constants import *
from lib.blimpTracker import *

# Build a border dictionary quickly.
def makeBorder( l, r, t, b ):
    borders = {}
    for n in (l, "border_left"), (r, "border_right"), (t, "border_top"), (b, "border_bottom"):
        if n[0] != 0:
            borders[n[1]] = n[0]
    return borders
    
class bleBotGui():
    axisNoMap = {0:0, 1:2, 2:1}
    axisDirMap = {0:"01", 1:"01", 2:"01"}

    def doMotorLockSwitchChanged(self, value):
        self.bot.immediateUpdate = True
        if self.motorLockSwitch.value:
            self.bot.blimpFlags = self.bot.blimpFlags | FLAGS_LOCK_MOTORS_BIT
        else:
            self.bot.blimpFlags = self.bot.blimpFlags & (0xff ^ FLAGS_LOCK_MOTORS_BIT)
    
    def doIgniterLockSwitchChanged(self, value):
        self.bot.immediateUpdate = True
        if self.igniterLockSwitch.value:
            self.bot.blimpFlags = self.bot.blimpFlags | FLAGS_LOCK_IGNITER_BIT
        else:
            self.bot.blimpFlags = self.bot.blimpFlags & (0xff ^ FLAGS_LOCK_IGNITER_BIT)
    
    def updateControllerDisplay( self ):
        if not self.bot.controller:
            self.controllerLabel.set_text("DETACHED")
            return
        
        if self.bot.controller.type == XBOX:
            typename = "xbox #{:d}".format(self.bot.controller.joystick.get_id())
        elif self.bot.controller.type == KEYBOARD:
            typename = "kbd"
        else:
            typename = "UNKNOWN"

        if self.bot.controller.curTemplate == -1:
            self.controllerTemplateButton.value = "*"
        else:
            self.controllerTemplateButton.value = CONTROLLER_TEMPLATES[self.bot.controller.curTemplate]["char_name"]
            
        self.controllerLabel.set_text("{:s}{:s}".format(typename, " (G)" if self.bot.controller.isAdmin else ""))
    
    def __init__( self, ble_bot ):
        # Set up the frame (table) representing this bot's controls.
        self.bot = ble_bot
        self.frame = gui.Table()

        # Build the table from the gui parts.
        #####################################

        # Add the heading.
        self.frame.tr()
        self.frame.td(gui.Label("{:s}".format(self.bot.name)), colspan=6, style=makeBorder(O, O, O, 0))
        #self.frame.tr()
        #self.frame.td(gui.Label("{:s}".format(self.bot.ble_adr)), colspan=6, style=makeBorder(O, O, 0, 0))
        self.frame.tr()
        self.controllerLabel = gui.Label()
        self.frame.td(self.controllerLabel, colspan=2, style=makeBorder(O, 0, 0, I))
        self.grabButton = gui.Button("g")
        self.controllerRemapLeftButton = gui.Button("<")
        self.controllerRemapRightButton = gui.Button(">")
        self.controllerTemplateButton = gui.Button("*")
        self.frame.td(self.controllerTemplateButton, style=makeBorder(0, 0, 0, I), colspan=1, width=COL_WIDTH)
        self.frame.td(self.controllerRemapLeftButton, style=makeBorder(0, 0, 0, I), colspan=1, width=COL_WIDTH)
        self.frame.td(self.grabButton, style=makeBorder(0, 0, 0, I), colspan=1, width=COL_WIDTH)
        self.frame.td(self.controllerRemapRightButton, style=makeBorder(0, O, 0, I), colspan=1, width=COL_WIDTH)
        self.grabButton.connect(gui.CLICK, self.doGrab, None)
        self.controllerRemapLeftButton.connect(gui.CLICK, self.doControllerLeft, None)
        self.controllerRemapRightButton.connect(gui.CLICK, self.doControllerRight, None)
        self.controllerTemplateButton.connect(gui.CLICK, self.doTemplateRotate, None)
    
        # Build the table of axes, including sliders and faults.
        self.axisLabels = [gui.Label("Throttle"), gui.Label("Pitch"), gui.Label("Yaw")]
        self.axisSliders = [gui.VSlider(value=0, min=-63, max=63, size=1, height=BLIMP_AXIS_SLIDER_HEIGHT), gui.VSlider(value=0, min=-63, max=63, size=1, height=BLIMP_AXIS_SLIDER_HEIGHT), gui.HSlider(value=0, min=-63, max=63, size=1, width=BLIMP_AXIS_SLIDER_WIDTH)]
        self.axisBorders = [makeBorder(O, I, 0, 0), makeBorder(0, O, 0, 0)]
        self.axisFaultLabel = [gui.Label("No Fault", background=GREEN), gui.Label("No Fault", background=GREEN), gui.Label("No Fault", background=GREEN)]

        # Note that we're currently ignoring the igniter box.
        self.frame.tr()
        for i in range(0, 2):         
            self.frame.td(self.axisLabels[i], style=self.axisBorders[i], colspan=3, width=BLIMP_AXIS_WIDTH)
        self.frame.tr()
        for i in range(0, 2): 
            self.frame.td(self.axisSliders[i], style=self.axisBorders[i], colspan=3)
        self.frame.tr()
        for i in range(0, 2): 
            self.frame.td(self.axisFaultLabel[i], style=self.axisBorders[i], colspan=3)
        self.frame.tr()

        # Add the yaw frame under the throttle/pitch frames.
        self.frame.td(self.axisLabels[2], style=makeBorder(O, O, I, 0), colspan=6)
        self.frame.tr()
        self.frame.td(self.axisSliders[2], style=makeBorder(O, O, 0, 0), colspan=6)
        self.frame.tr()
        self.frame.td(self.axisFaultLabel[2], style=makeBorder(O, O, 0, 0), colspan=6)
            
        # Build the status section.
        self.rssiTempLabel = gui.Label("? / ?\xb0F")
        self.voltageLabel = gui.Label("?v", background=GREEN)
        self.voltageOverrideButton = gui.Button(gui.Label("O"));
        self.voltageOverrideButton.connect(gui.CLICK, self.doVoltageOverride, None)
        self.voltageOverrideButton.disabled = True
        self.trgLabel = gui.Label("trg", background=GREEN);
        self.ignLabel = gui.Label("ign", background=GREEN);

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
        self.frame.td(self.rssiTempLabel, style=makeBorder(O, 0, I, 0), colspan=4)
        self.frame.td(self.trgLabel, style=makeBorder(I, 0, I, 0), colspan=1)
        self.frame.td(self.ignLabel, style=makeBorder(I, O, I, 0), colspan=1)
        self.frame.tr()
        self.frame.td(self.stateLabel, style=makeBorder(O, 0, I, 0), colspan=3)
        self.frame.td(self.voltageLabel, style=makeBorder(I, 0, I, 0), colspan=2)
        self.frame.td(self.voltageOverrideButton, style=makeBorder(0, O, I, 0), colspan=1)
        self.frame.tr()
        #self.frame.td(gui.Label("Lock: "), style=makeBorder(O, 0, I, 0), colspan=2)
        self.frame.td(gui.Label("Motor Lock"), style=makeBorder(O, 0, I, 0), colspan=2)
        self.motorLockSwitch = gui.Switch(value=self.bot.DEFAULT_MOTORS_LOCK, name='motor_lock')
        self.motorLockSwitch.connect(gui.CHANGE, self.doMotorLockSwitchChanged, None)
        self.frame.td(self.motorLockSwitch, style=makeBorder(0, I, I, 0), colspan=1)
        self.igniterLockSwitch = gui.Switch(value=self.bot.DEFAULT_IGNITER_LOCK, name='igniter_lock')
        self.igniterLockSwitch.connect(gui.CHANGE, self.doIgniterLockSwitchChanged, None)
        self.frame.td(self.igniterLockSwitch, style=makeBorder(0, 0, I, 0), colspan=1)
        self.frame.td(gui.Label("Igniter Lock"), style=makeBorder(0, O, I, 0), colspan=2)
        self.frame.tr()
        self.frame.td(self.disableButton, style=makeBorder(O, 0, I, O), colspan=3)
        self.frame.td(self.resetButton, style=makeBorder(0, O, I, O), colspan=3)

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

    def doGrab(self, value):
        self.bot.doAppGuiCallback("grab", self.bot)

    def doControllerLeft(self, value):
        self.bot.doAppGuiCallback("left", self.bot)
        
    def doControllerRight(self, value):
        self.bot.doAppGuiCallback("right", self.bot)

    def doTemplateRotate(self, value):
        self.bot.doAppGuiCallback("template", self.bot)
    
    def doVoltageOverride(self, value):
        # Note that this currently is a one-way override; you need to reset the device to
        # undo the override.
        if self.bot.connectionState == bleBot.CONNECTED:
            self.bot.sendMessage("050100");
        pass

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
    DEFAULT_ENABLED = True      # Enable blimps when the program is started.
    DEFAULT_MOTORS_LOCK = False 
    DEFAULT_IGNITER_LOCK = True
    
    def __init__( self, name, ble_adr, doAppGuiCallback ):

        # Set up connection-related data.
        self.protocolVersion = PROTOCOL_VERSION
        self.doAppGuiCallback = doAppGuiCallback
        self.controller = None
        self.ble_adr = ble_adr
        self.name = name
        self.btlePeripheral = btle.Peripheral()
        self.btleDebug = False

        self.blimpFlags = 0   # Flags for triggers and igniter lock.
        if bleBot.DEFAULT_IGNITER_LOCK:
            self.blimpFlags = self.blimpFlags | FLAGS_LOCK_IGNITER_BIT
        if bleBot.DEFAULT_MOTORS_LOCK:
            self.blimpFlags = self.blimpFlags | FLAGS_LOCK_MOTORS_BIT
        if bleBot.DEFAULT_ENABLED:
            self.connectionState = self.WAITING
        else:
            self.connectionState = self.DISABLED

        self.immediateUpdate = True # Update blimp soon as possible.
        self.igniterLocked = True   # Default to igniter locked.
        self.curRSSI = "?"
        self.curTemp = "?"
        self.motorState = [("00","00"), ("00","00"), ("00","00")]
        self.lastTxState = [("00","00"), ("00","00"), ("00","00")]
        self.lastTxBlimpFlags = self.blimpFlags   # 
        self.batterySessionStart = time.time()
        self.ignState = 0
        self.trgState = 0
        self.lastTxTime = 0
        self.counter = 0xff
        self.curReceiveString = ""
        self.lastFault = [0, 0, 0]
        self.lastFaultTime = [0, 0, 0]
        self.lastConnectStart = 0
        self.lastUpdate = time.time()
        self.gui = bleBotGui(self)
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
        return "{:s}|{:s}|{:s}|{:s}|{:s}".format("FA" if faultValue & 0x01 else "fa", "OC" if faultValue & 0x02 else "oc", "UV" if faultValue & 0x04 else "uv", "OT" if faultValue & 0x08 else "ot", "IL" if faultValue & 0x10 else "il") 
    
    def handleNotification(self, cHandle, data): 
        if (cHandle <> 14):
            print "MESSAGE RECEIVED ON UNEXPECTED HANDLE [", cHandle, "]: ", data
            return

        # Get the current time.
        curTime = time.time()

        # Log it.
        blimpTracker.logBlimpString("receive", self, ''.join(x[0].encode("hex") for x in data))
        
        # First byte is the command; the rest is the actual payload.
        rcvCmd = ord(data[0])
        data = data[1:]
        
        if rcvCmd == self.RETURN_MSG_STRING:
            # For strings, print the complete string if there's a null termination,
            self.curReceiveString = self.curReceiveString + data
            if (string.find(data, "\x00") != -1):
                if DEBUG_RX:
                    print "{:s} {:f} rcv> {:s}".format(self.ble_adr, curTime, self.curReceiveString)
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
                    self.lastFaultTime[j] = curTime
            self.gui.updateFaults()

            # two bytes: igniter, trigger
            # Update the igniter and trigger displays if they have changed.
            if (self.ignState != ord(data[8+3])):
                self.ignState = ord(data[8+3])
                if self.ignState:
                    self.gui.ignLabel.set_text("IGN")
                    self.gui.ignLabel.style.background = RED
                    if DEBUG_IGNITER:
                        print "{:s} IGNITER ON".format(self.ble_adr)
                else:
                    self.gui.ignLabel.set_text("ign")
                    self.gui.ignLabel.style.background = GREEN
                    if DEBUG_IGNITER:
                        print "{:s} igniter off".format(self.ble_adr)
            if (self.trgState != ord(data[8+4])):
                self.trgState = ord(data[8+4])
                if self.trgState:
                    self.gui.trgLabel.set_text("TRG")
                    self.gui.trgLabel.style.background = RED
                    if DEBUG_TRIGGER:
                        print "{:s} TRIGGER ON".format(self.ble_adr)
                else:
                    self.gui.trgLabel.set_text("trg")
                    self.gui.trgLabel.style.background = GREEN
                    if DEBUG_TRIGGER:
                        print "{:s} trigger off".format(self.ble_adr)

            self.returnStatus = ord(data[13])
            self.batteryVoltage = struct.unpack("<H", "".join(data[14:16]))[0] * 0.01;
            self.gui.rssiTempLabel.set_text("{:d} / {:.1f}\xb0F".format(self.curRSSI,self.curTemp))
            self.gui.voltageLabel.set_text("{:.2f}v ".format(self.batteryVoltage))

            # Match the voltage reading color to the report of low voltage.
            if (self.returnStatus & 0x01):
                if (self.gui.voltageLabel.style.background != RED):
                    if DEBUG_VOLTAGE:
                        sessionLen = int(curTime - self.batterySessionStart)
                        print "{:s} transitioned to LOW VOLTAGE mode ({:f}) - {:02d}m:{:02d}s session".format(self.ble_adr, self.batteryVoltage, int(sessionLen / 60), sessionLen % 60)
                    self.gui.voltageOverrideButton.disabled = False
                    self.gui.voltageLabel.style.background = RED;
                        
            elif (self.batteryVoltage < 3.0):
                if (self.gui.voltageLabel.style.background != YELLOW):
                    if DEBUG_VOLTAGE:
                        if self.gui.voltageLabel.style.background == RED:
                            self.batterySessionStart = curTime
                        print "{:s} voltage below 3.0 but low voltage mode not yet triggered ({:f})".format(self.ble_adr, self.batteryVoltage)
                    self.gui.voltageOverrideButton.disabled = True
                    self.gui.voltageLabel.style.background = YELLOW;

            else:
                if (self.gui.voltageLabel.style.background != GREEN):
                    if DEBUG_VOLTAGE:
                        if self.gui.voltageLabel.style.background == RED:
                            self.batterySessionStart = curTime
                        print "{:s} voltage mode returned to NORMAL ({:f})".format(self.ble_adr, self.batteryVoltage)
                    self.gui.voltageOverrideButton.disabled = True
                    self.gui.voltageLabel.style.background = GREEN;
                
            # Give us debug info.
            if DEBUG_UPDATE:
                curTime = time.time()
                print "{:s} update {:.03f}> rssi {:d}, temp {:.1f}, batt {:.2f}, ign: {:d}, trg: {:d}, faults: {:02x}/{:02x}/{:02x} {:s}/{:s}/{:s}".format(self.ble_adr, curTime - self.lastUpdate, self.curRSSI, self.curTemp, self.batteryVoltage, self.ignState, self.trgState, self.lastFault[0], self.lastFault[1], self.lastFault[2], self.decodeFaults(self.lastFault[0]), self.decodeFaults(self.lastFault[1]), self.decodeFaults(self.lastFault[2]))
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

        # Always immediately update after connect.
        self.immediateUpdate = True

        if (self.ble_adr == "dummy"):
            if DEBUG_CONNECT:
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
                print "BTLEException raised.  This may mean:"
                print "1) You don't have any Bluetooth 4.0 adapters attached to your system."
                print "   Install at least one (and preferably two)."
                print "2) bluepy-helper may be a zombie.  although this should be automatic,"
                print "   try running \"killall bluepy-helper\" then blimpControl again."
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

        # Send a configuration message.
        configMessage = "06"
        for clientConfig in BLIMP_REMOTE_CONFIGS:
            if clientConfig == None:
                configMessage += "FFFF"
            else:
                configMessage += "{:04x}".format(clientConfig)
        self.sendMessage(configMessage)
        return
    
    def char_write_cmd( self, value ):
        # in BLE gatttool,
        # for IMUduino: char-write-cmd 0x000b 41424344
        # for RFduino: char-write-cmd 0x0011 41424344555555
        if DEBUG_TX:
            print "{:s} {:s}> {:s}".format(self.ble_adr, "snd" if self.connectionState == self.CONNECTED else "noconn", value)
            
        if (self.connectionState != self.CONNECTED):
            return

        # Log it. 
        blimpTracker.logBlimpString("send", self, value)

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
        self.char_write_cmd(format(self.protocolVersion, '02x') + format(self.counter, '02x') + data)
        
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
        tmpMsg = "0300{:02x}".format(self.blimpFlags)
        for i in range(3):
            self.lastTxState[i] = self.motorState[i] 
            tmpMsg += "0"+str(i)+"".join(self.motorState[i])

        self.lastTxBlimpFlags = self.blimpFlags

        self.lastTxTime = time.time()
        self.immediateUpdate = False
        self.sendMessage(tmpMsg)

    # Send a full state update when necessary (time out, new data, etc).
    def autoTxUpdate(self):
        # Don't send anything if  we are not connected.
        if self.connectionState != self.CONNECTED:
            return

        curTime = time.time()
        
        # Don't send anything if not enough time has passed.
        if ((not self.immediateUpdate) and (curTime < self.lastTxTime + MIN_TRANSMIT_INTERVAL)):
            return

        # If we've timed out or are forcing an update, always retransmit.
        if (self.immediateUpdate or (curTime > self.lastTxTime + TRANSMISSION_TIMEOUT)):
            self.txState()
            return
        
        # Transmit everything if anything has changed.
        changeFound = False
        for i in range(3):
            if self.lastTxState[i] != self.motorState[i]:
                self.lastTxState[i] = self.motorState[i]
                changeFound = True
        if self.blimpFlags != self.lastTxBlimpFlags:
            # Note that interim igniter states can be lost. If we ever
            # return to manual triggering, we should make sure to keep
            # track of interim triggers in a separte variable.
            self.lastTxBlimpFlags = self.blimpFlags
            changeFound = True

        # If anything changed, retransmit.
        if changeFound: 
            self.txState()

    def setLocks(self, newIgniterLockState, newMotorLockState = False):
        if newIgniterLockState:
            self.gui.igniterLockSwitch.value = True
        else:
            self.gui.igniterLockSwitch.value = False

        if newMotorLockState:
            self.gui.motorLockSwitch.value = True
        else:
            self.gui.motorLockSwitch.value = False

    def setMotorState(self, motorIndex, motorDirection, motorSpeed):
        self.motorState[motorIndex] = (motorDirection,motorSpeed)
        self.gui.setAxis(motorIndex, motorDirection, motorSpeed)
        
    def setBlimpFlags(self, newBlimpFlags):
        self.blimpFlags = newBlimpFlags

    def getBlimpFlags(self):
        return self.blimpFlags

