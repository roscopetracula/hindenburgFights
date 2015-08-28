import shelve
import time
from lib.constants import *

class blimpState:
    allStates = None
    
    def __init__(self, blimpAddr):
        self.lastGoodVoltage = time.time()
        self.lastBadVoltage = None
        self.lastVoltageWasGood = True
        self.curConnectionState = None
        self.blimpAddr = blimpAddr
        self.lastUptime = 0.0
        
    @staticmethod
    def init():
        if not blimpState.allStates:
            blimpState.allStates = shelve.open(PERSIST_FILE, writeback=True)

    @staticmethod
    def close():
        if blimpState.allStates:
            blimpState.allStates.close()
        
    @staticmethod
    def sync():
        if blimpState.allStates:
            blimpState.allStates.sync()
        
    @staticmethod
    def getState(blimpAddr):
        try:
            return blimpState.allStates[blimpAddr]
        except:
            newState = blimpState(blimpAddr)
            blimpState.allStates[blimpAddr] = newState
            return newState
    
class blimpTracker:
    loggerFile = None
    logOpenAttempted = False
    
    @staticmethod
    def init():
        if not blimpTracker.logOpenAttempted:
            # Open our shelf.
            blimpState.init()

            # Open the log file.
            if DEBUG_LOG_FILE:
                blimpTracker.loggerFile = open(DEBUG_LOG_FILE, "a")
            blimpTracker.logOpenAttempted = True

    @staticmethod
    def logGlobalEvent(logEvent):
        blimpTracker.init()
        if blimpTracker.loggerFile:
            blimpTracker.loggerFile.write("{:f} {:s}\n".format(time.time(), logEvent))
        
    @staticmethod
    def logGlobalString(logEvent, logString):
        blimpTracker.init()
        if blimpTracker.loggerFile:
            blimpTracker.loggerFile.write("{:f} {:s} {:s}\n".format(time.time(), logEvent, logString))
        
    @staticmethod
    def logBlimpString(logEvent, logBlimp, logString):
        blimpTracker.init()
        if blimpTracker.loggerFile:
            blimpTracker.loggerFile.write("{:f} {:s} {:s} {:s}\n".format(time.time(), logEvent, logBlimp.ble_adr, logString))        

    @staticmethod
    def logBlimpEvent(logEvent, logBlimp):
        blimpTracker.init()
        if blimpTracker.loggerFile:
            blimpTracker.loggerFile.write("{:f} {:s} {:s}\n".format(time.time(), logEvent, logBlimp.ble_adr))        

    @staticmethod
    def logBlimpStateChange(logBlimp):
        blimpTracker.init()
        if blimpTracker.loggerFile:
            blimpTracker.loggerFile.write("{:f} connection {:s} {:s}\n".format(time.time(), logBlimp.ble_adr, logBlimp.stateNames[logBlimp.connectionState]))        
        logState = blimpState.getState(logBlimp.ble_adr)
        # print "{:s} changed {:d} to {:d}".format(logState.blimpAddr, logState.curConnectionState, logBlimp.connectionState)
        logState.curConnectionState = logBlimp.connectionState

    @staticmethod
    def getBlimpUptime(logBlimp):
        blimpTracker.init()
        logState = blimpState.getState(logBlimp.ble_adr)

        # Only update if we're connected.  Note the hard-coded 0.
        # This will take some rearranging to fix, and we're out of
        # time.
        
        if logBlimp.connectionState == 0:
            if logState.lastVoltageWasGood:
                logState.lastUptime = time.time() - logState.lastGoodVoltage
            else:
                logState.lastUptime = logState.lastBadVoltage - logState.lastGoodVoltage
        return logState.lastUptime
    
    @staticmethod
    def logBlimpVoltage(logBlimp):
        # Ignore this state if voltage is overridden.
        if logBlimp.returnStatus & 0x02:
            return

        blimpTracker.init()
        logState = blimpState.getState(logBlimp.ble_adr)

        if logBlimp.returnStatus & 0x01 and logState.lastVoltageWasGood:
            # Transition to bad.
            logState.lastVoltageWasGood = False
            logState.lastBadVoltage = time.time()
            uptime = blimpTracker.getBlimpUptime(logBlimp)
            blimpTracker.logBlimpString("battery_low", logBlimp, "{:f}".format(uptime))
            if DEBUG_VOLTAGE:
                curMins = int(uptime/60)
                curSecs = int(uptime) - curMins * 60
                print "blimp \"{:s}\" has run out of batteries after {:02d}:{:02d}".format(logBlimp.name, curMins, curSecs)
            
        elif logBlimp.returnStatus & 0x01 == 0 and not logState.lastVoltageWasGood:

            # Transition to good.
            blimpTracker.logBlimpEvent("battery_high", logBlimp)
            if DEBUG_VOLTAGE:
                if logState.lastBadVoltage and logState.lastGoodVoltage:
                    print "blimp \"{:s}\" has been recharged".format(logBlimp.name)
            logState.lastVoltageWasGood = True
            logState.lastGoodVoltage = time.time()
            
    @staticmethod
    def close():
        if blimpTracker.loggerFile:
            blimpTracker.loggerFile.close()
        blimpState.close()

    @staticmethod
    def sync():
        if blimpTracker.loggerFile:
            blimpTracker.loggerFile.flush()
        blimpState.sync()
        
