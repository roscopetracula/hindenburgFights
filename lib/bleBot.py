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
        self.igniterState = ("00","00") # send ["01",xx] for on, anything else for off
        self.lastTxIgniterState = ("00","00") # send ["01",xx] for on, anything else for off
        self.lastTxTime = 0
        self.counter = 0xff

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
        i = self.con.expect(['Connecting', 'Attempting', 'Error'], timeout=1)
        if i == 0 or i == 1:
            print 'Attempting to connect'
            j = self.con.expect(['Connection successful', '[CON]', 'No route', 'busy', pexpect.TIMEOUT], timeout = 3)
#            print "j: ", j
            if j == 0 or j == 1:
                print self.ble_adr, ': connected!'
            if j == 2:
                print self.ble_adr, ': No route to host, is USB dongle plugged in?'
                self.cleanup()
            if j == 3:
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
            if j == 4:
                print 'Attempting to connect, is device on and in range? '
                #foostr = raw_input('Type anything to continue, or enter to cancel')
                self.con.sendline('connect')
                try:
                    self.con.read_nonblocking(2048,0) #flush the read pipe!! SUPER IMPORTANT
                except:
                    pass
                k = self.con.expect(['Connection successful', pexpect.TIMEOUT], timeout = 3)
#                print 'k: ', k
                if k == 0:
                    print self.ble_adr, ': connected!'
                if k == 1:
                    print self.ble_adr, ': Could not connect'
                    self.cleanup()
        if i == 2:
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
