import pygame

import os, sys
from ctypes.util import find_library
import pexpect, traceback, threading, Queue, time, socket, select
import blescan

from contextlib import contextmanager
import sys, os



@contextmanager
def suppress_stdout():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        sys.stdout = devnull
        try:  
            yield
        finally:
            sys.stdout = old_stdout


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



# pygame.init()
# BLACK = (0,0,0)
# WIDTH = 300
# HEIGHT = 300
# windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)

# windowSurface.fill(BLACK)


imuDuino="FD:B3:BC:98:29:EF"
rfduino= "C9:44:30:80:0D:1A"
rfduino2="D2:9F:90:14:98:4C"

b=bleBot(rfduino2)
conn = b.connect()


# motorBytes = ["00","00","00"]

lastTx = time.time()
reTxTimeout=.9 #retransmit every x sec

f_b=0
r_l=2
u_d=1

handledKeys = [pygame.K_RIGHT,pygame.K_LEFT,pygame.K_UP,pygame.K_DOWN,pygame.K_a,pygame.K_z,pygame.K_q,pygame.K_r]



BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)

# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputing the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def write(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height
        
    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15
        
    def indent(self):
        self.x += 10
        
    def unindent(self):
        self.x -= 10
    

pygame.init()
 
# Set the width and height of the screen [width,height]
size = [500, 700]
screen = pygame.display.set_mode(size)

pygame.display.set_caption("My Game")

#Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Initialize the joysticks
pygame.joystick.init()
    
# Get ready to print
textPrint = TextPrint()



class motorMsg:
    def __init__(self, fbAxis=1,lrAxis=2,udAxis=3):
        self.fbAxis=fbAxis-1
        self.lrAxis=lrAxis-1
        self.udAxis=udAxis-1
        self.codes = ["00","00","00"]
        self.CHANGED = False
    def fb(self, val):
        self.CHANGED = True
        if val==1:
            self.codes[self.fbAxis]="01"
        elif val==-1:
            self.codes[self.fbAxis]="02"
        else:
            self.codes[self.fbAxis]="00"
            return self
    def lr(self, val):
        self.CHANGED = True
        if val==1:
            self.codes[self.lrAxis]="01"
        elif val==-1:
            self.codes[self.lrAxis]="02"
        else:
            self.codes[self.lrAxis]="00"
            return self
    def ud(self, val):
        self.CHANGED = True
        if val==1:
            self.codes[self.udAxis]="01"
        elif val==-1:
            self.codes[self.udAxis]="02"
        else:
            self.codes[self.udAxis]="00"
            return self
    def __repr__(self):
        return "".join(self.codes)
    def any(self):
        return any([byte!="00" for byte in self.codes])
    def changed(self):
        return self.CHANGED
    def sent(self):
        self.CHANGED = False






motorBytes = motorMsg()



# -------- Main Program Loop -----------
while done==False:
    # EVENT PROCESSING STEP
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
        
        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")
            
    # DRAWING STEP
    # First, clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
    screen.fill(WHITE)
    textPrint.reset()

    # Get count of joysticks
    joystick_count = pygame.joystick.get_count()

    textPrint.write(screen, "Number of joysticks: {}".format(joystick_count) )
    textPrint.indent()
    
    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
    
        textPrint.write(screen, "Joystick {}".format(i) )
        textPrint.indent()
    
        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()
        textPrint.write(screen, "Joystick name: {}".format(name) )
        
        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        # axes = joystick.get_numaxes()
        # textPrint.write(screen, "Number of axes: {}".format(axes) )
        # textPrint.indent()
        
        # for i in range( axes ):
        #     axis = joystick.get_axis( i )
        #     textPrint.write(screen, "Axis {} value: {:>6.3f}".format(i, axis) )
        # textPrint.unindent()
            
        buttons = joystick.get_numbuttons()
        textPrint.write(screen, "Number of buttons: {}".format(buttons) )
        textPrint.indent()
        
        thisUD = False
        for i in range( buttons ):
            # with suppress_stdout():
            sys.stdout = os.devnull
            sys.stderr = os.devnull
            button = joystick.get_button( i )
            sys.stdout = sys.__stdout__
            sys.stderr = sys.__stderr__
            textPrint.write(screen, "Button {:>2} value: {}".format(i,button) )
            if button:
                if i == 7:
                    b.reconnect()
                if i ==6:
                    conn.cleanup()
                    pygame.quit()
                    sys.exit()
                if i ==3:
                    motorBytes.ud(1)
                    thisUD = True
                if i ==0:
                    motorBytes.ud(-1)
                    thisUD = True
        if not thisUD:
            motorBytes.ud(0)

        textPrint.unindent()
            
        # Hat switch. All or nothing for direction, not like joysticks.
        # Value comes back in an array.
        # hats = joystick.get_numhats()
        # textPrint.write(screen, "Number of hats: {}".format(hats) )
        # textPrint.indent()

        # for i in range( hats ):
        #     hat = joystick.get_hat( i )
        #     textPrint.write(screen, "Hat {} value: {}".format(i, str(hat)) )
        # textPrint.unindent()
        
        textPrint.unindent()

    if (time.time() - lastTx >reTxTimeout and motorBytes.any()) or motorBytes.changed():
        b.char_write_cmd(str(motorBytes))
        motorBytes.sent()
        lastTx = time.time()
    

    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
    
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

    # Limit to 20 frames per second
    clock.tick(100)
    
# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit ()






