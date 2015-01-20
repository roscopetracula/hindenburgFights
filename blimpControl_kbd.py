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
        #print self.ble_adr, cmd
        self.con.sendline( cmd )
        try:
            rnb = self.con.read_nonblocking(2048,0) #flush the read pipe!! SUPER IMPORTANT
            print "----RNB----\n", rnb, "\n-----END RNB-----"
            if "Command Failed: Disconnected" in rnb:
                print "attempt reconnect..."
                # FIXME this is where the reconnect should happen
                # should it just be: self.connect() ??
                self.connect()
        except:
            print "could not 'flush read pipe'"
        print 'After sending command, before: ', self.con.before, 'after :', self.con.after
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
        self.con.read_nonblocking(2048,0)
        self.connect()



pygame.init()
BLACK = (0,0,0)
WIDTH = 300
HEIGHT = 300
windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)

windowSurface.fill(BLACK)


imuDuino="FD:B3:BC:98:29:EF"
rfduino= "C9:44:30:80:0D:1A"
rfduino2="D2:9F:90:14:98:4C"

b=bleBot(rfduino2)
conn = b.connect()
# b.char_write_cmd("414345")
# time.sleep(.5)
# b.char_write_cmd("30")
# time.sleep(.5)z


motorCodes = [{"f":47,"r":48},{"f":43,"r":44},{"f":41,"r":42},{"f":45,"r":46}]
keysDown = set()


motorBytes = ["00","00","00"]

lastTx = time.time()
reTxTimeout=.9 #retransmit every x sec

f_b=2
r_l=1
u_d=0

handledKeys = [pygame.K_RIGHT,pygame.K_LEFT,pygame.K_UP,pygame.K_DOWN,pygame.K_a,pygame.K_z,pygame.K_q,pygame.K_r]

while True:
    events = pygame.event.get()
    for event in events:
        # print event
        if event.type == QUIT:
            conn.cleanup()
            pygame.quit()
            sys.exit()

        if (event.type == KEYDOWN or event.type == KEYUP) and event.key in handledKeys:
            if event.type == KEYDOWN:
                # motorBytes = ["00","00","00"]
                # codes: '01':forward, "02": reverse, all others: brake
                if event.key == pygame.K_RIGHT:
                    motorBytes[r_l]="02"
                if event.key == pygame.K_LEFT:
                    motorBytes[r_l]="01"
                if event.key == pygame.K_UP:
                    motorBytes[f_b]="02"
                if event.key == pygame.K_DOWN:
                    motorBytes[f_b]="01"
                if event.key == pygame.K_a:
                    motorBytes[u_d]="02"
                if event.key == pygame.K_z:
                    motorBytes[u_d]="01"

                if event.key == pygame.K_q:
                    conn.cleanup()
                    pygame.quit()
                    sys.exit()
                if event.key == pygame.K_r:
                    print "RECONNECT"
                    b.reconnect()

            if event.type == KEYUP:
                if event.key == pygame.K_RIGHT:
                    motorBytes[r_l]="00"
                if event.key == pygame.K_LEFT:
                    motorBytes[r_l]="00"
                if event.key == pygame.K_UP:
                    motorBytes[f_b]="00"
                if event.key == pygame.K_DOWN:
                    motorBytes[f_b]="00"
                if event.key == pygame.K_a:
                    motorBytes[u_d]="00"
                if event.key == pygame.K_z:
                    motorBytes[u_d]="00"
            b.char_write_cmd("".join(motorBytes))

    if time.time() - lastTx >reTxTimeout and any([byte!="00" for byte in motorBytes]):
        b.char_write_cmd("".join(motorBytes))
        lastTx = time.time()

    # do your non-rendering game loop computation here
    # to reduce CPU usage, call this guy:
    time.sleep(0.001)     



