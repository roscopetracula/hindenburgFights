import pexpect
import Queue
import select
import socket
import threading
import time
import traceback

TRANSMISSION_TIMEOUT = .9

class bleBot:
    def __init__( self, ble_adr ):
        self.ble_adr = ble_adr
        
        # Only actually try to connect for real devices.
        if (ble_adr != "dummy"):
            self.con = pexpect.spawn(
                'gatttool -b %s -I -t random' % self.ble_adr
            )
            self.con.delaybeforesend = 0 #THIS LINE IS SUPER IMPORTANT
            self.con.expect('\[LE\]', timeout=1)
        
        # for IMUduino: char-write-cmd 0x000b 41424344; hande-> 'b'
        # for RFduino:  char-write-cmd 0x0011 41424344; handle-> '11'
        self.handle = '0011' #!! this is the TX service on the nRF8001 adafruit breakout with callbackEcho sketch

        self.motorState = [("00","00"), ("00","00"), ("00","00")]
        self.lastTxState = [("00","00"), ("00","00"), ("00","00")]
        self.igniterState = ("00","00") # send ["00",xx] for on, anything else for off
        self.lastTxIgniterState = ("00","00") # send ["00",xx] for on, anything else for off
        self.lastTxTime = 0

    def connect( self ):
        
        if (self.ble_adr == "dummy"):
            print "Dummy connection established."
            return self

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

        # Fake sending if we are ussing a dummy blimp.
        if (self.ble_adr == "dummy"):
            return

        self.con.sendline( cmd )
        try:
            rnb = self.con.read_nonblocking(2048,0) #flush the read pipe!! SUPER IMPORTANT
            if "Command Failed:" in rnb:
                print "attempt reconnect..."
                self.connect()
        except:
            print "could not 'flush read pipe'"
        return

    def cleanup( self ):
        # If we're using a dummy, just let us know and return.
        if (self.ble_adr == "dummy"):
            print "Dummy connection closed."
            return

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

        # Not sure why this should happen, but simply return if a
        # dummy device is reconnected.
        if (self.ble_adr == "dummy"):
            return
        
        # Otherwise, send a disconnect commend and re-call connect().
        self.con.sendline('disconnect')
        try:
            self.con.read_nonblocking(2048,0)
        except:
            pass
        self.connect()

    def sendToChannel(self, channel, data):
        self.char_write_cmd(channel+data)
        self.lastTxTime = time.time()

    def txStateChanges(self):
        for i in range(3):
            if self.lastTxState[i] != self.motorState[i]:
                self.sendToChannel("0"+str(i), "".join(self.motorState[i]))
                self.lastTxState[i] = self.motorState[i]
        if self.igniterState != self.lastTxIgniterState:
            self.sendToChannel("03", "".join(self.igniterState))
            self.lastTxIgniterState = self.igniterState

    def reTxState(self):
        if time.time() - self.lastTxTime > TRANSMISSION_TIMEOUT:
            #retransmit any states that haven't been sent in a while
            for i in range(3):
                self.sendToChannel("0"+str(i), "".join(self.motorState[i]))
            self.sendToChannel("03", "".join(self.igniterState))

    def setMotorState(self, motorIndex, motorDirection, motorSpeed):
        self.motorState[motorIndex] = (motorDirection,motorSpeed)

    def setIgniterState(self, onOrOff):
        self.igniterState = (onOrOff,"00")
