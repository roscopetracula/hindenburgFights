import shelve
import time
from lib.constants import *

class blimpState:
    allStates = None
    
    def __init__(self, blimpAddr):
        self.lastVoltageGood = None
        self.lastVoltageBad = None
        self.curConnectionState = None
        self.blimpAddr = blimpAddr

    @staticmethod
    def init():
        if not blimpState.allStates:
            blimpState.allStates = shelve.open(PERSIST_FILE, writeback=True)

    @staticmethod
    def close():
        if blimpState.allStates:
            blimpState.allStates.close()
        
    @staticmethod
    def getState(blimpAddr):
        try:
            return blimpState.allStates[blimpAddr]
        except:
            print "new state {:s}".format(blimpAddr)
            newState = blimpState(blimpAddr)
            blimpState.allStates[blimpAddr] = newState
            return newState
    
class blimpTracker:
    loggerFile = None
    logOpenAttempted = False
    
    @staticmethod
    def init():
        if not blimpTracker.logOpenAttempted:
            blimpState.init()
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
    def logBlimpStateChange(logBlimp):
        blimpTracker.init()
        if blimpTracker.loggerFile:
            blimpTracker.loggerFile.write("{:f} connection {:s} {:s}\n".format(time.time(), logBlimp.ble_adr, logBlimp.stateNames[logBlimp.connectionState]))        
        logState = blimpState.getState(logBlimp.ble_adr)
        logState.curConnectionState = logBlimp.connectionState
        
    @staticmethod
    def close():
        if blimpTracker.loggerFile:
            blimpTracker.loggerFile.close()
        blimpState.close()
