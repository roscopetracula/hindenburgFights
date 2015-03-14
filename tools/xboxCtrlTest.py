#!/usr/bin/python

import pygame

# Define some colors
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
        axes = joystick.get_numaxes()
        textPrint.write(screen, "Number of axes: {}".format(axes) )
        textPrint.indent()
        
        for i in range( axes ):
            axis = joystick.get_axis( i )
            textPrint.write(screen, "Axis {} value: {:>6.3f}".format(i, axis) )
        textPrint.unindent()
            
        buttons = joystick.get_numbuttons()
        textPrint.write(screen, "Number of buttons: {}".format(buttons) )
        textPrint.indent()

        for i in range( buttons ):
            button = joystick.get_button( i )
            textPrint.write(screen, "Button {:>2} value: {}".format(i,button) )
        textPrint.unindent()
            
        # Hat switch. All or nothing for direction, not like joysticks.
        # Value comes back in an array.
        hats = joystick.get_numhats()
        textPrint.write(screen, "Number of hats: {}".format(hats) )
        textPrint.indent()

        for i in range( hats ):
            hat = joystick.get_hat( i )
            textPrint.write(screen, "Hat {} value: {}".format(i, str(hat)) )
        textPrint.unindent()
        
        textPrint.unindent()

    
    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
    
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

    # Limit to 20 frames per second
    clock.tick(20)
    
# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit ()


# import pygame, sys
# from pygame.locals import *

# import os, sys
# from ctypes.util import find_library
# import pexpect, traceback, threading, Queue, time, socket, select



# class TextPrint:
#     def __init__(self):
#         self.reset()
#         self.font = pygame.font.Font(None, 20)

#     def write(self, screen, textString):
#         textBitmap = self.font.render(textString, True, BLACK)
#         screen.blit(textBitmap, [self.x, self.y])
#         self.y += self.line_height
        
#     def reset(self):
#         self.x = 10
#         self.y = 10
#         self.line_height = 15
        
#     def indent(self):
#         self.x += 10
        
#     def unindent(self):
#         self.x -= 10
    

# pygame.init()
# BLACK = (0,0,0)
# WIDTH = 600
# HEIGHT = 600
# screen = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)

# pygame.display.set_caption("Input listener")

# screen.fill(BLACK)

# pygame.joystick.init()
# joystick_count = pygame.joystick.get_count()
# print "joystick_count: ",joystick_count

# textPrint = TextPrint()

# textPrint.write(screen, "Number of joysticks: {}".format(joystick_count) )

# motorBytes = ["00","00","00"]
# f_b=0
# r_l=2
# u_d=1

# while True:
#     events = pygame.event.get()
#     for event in events:
#         print event
#         if event.type == QUIT:
#             conn.cleanup()
#             pygame.quit()
#             sys.exit()
#         if event.type == KEYDOWN:
#             # motorBytes = ["00","00","00"]
#             # codes: '01':forward, "02": reverse, all others: brake
#             if event.key == pygame.K_RIGHT:
#                 motorBytes[r_l]="02"
#             if event.key == pygame.K_LEFT:
#                 motorBytes[r_l]="01"
#             if event.key == pygame.K_UP:
#                 motorBytes[f_b]="02"
#             if event.key == pygame.K_DOWN:
#                 motorBytes[f_b]="01"
#             if event.key == pygame.K_a:
#                 motorBytes[u_d]="02"
#             if event.key == pygame.K_z:
#                 motorBytes[u_d]="01"
#             if event.key == pygame.K_q:
#                 # conn.cleanup()
#                 pygame.quit()
#                 sys.exit()

#         if event.type == KEYUP:
#             if event.key == pygame.K_RIGHT:
#                 motorBytes[r_l]="00"
#             if event.key == pygame.K_LEFT:
#                 motorBytes[r_l]="00"
#             if event.key == pygame.K_UP:
#                 motorBytes[f_b]="00"
#             if event.key == pygame.K_DOWN:
#                 motorBytes[f_b]="00"
#             if event.key == pygame.K_a:
#                 motorBytes[u_d]="00"
#             if event.key == pygame.K_z:
#                 motorBytes[u_d]="00"
#         if event.type == pygame.JOYBUTTONDOWN:
#             print "Joystick button pressed."
#         if event.type == pygame.JOYBUTTONUP:
#             print "Joystick button released."


#     for i in range(joystick_count):
#         joystick = pygame.joystick.Joystick(i)
#         joystick.init()
    
#         textPrint.write(screen, "Joystick {}".format(i) )
#         textPrint.indent()
    
#         # Get the name from the OS for the controller/joystick
#         name = joystick.get_name()
#         textPrint.write(screen, "Joystick name: {}".format(name) )
        
#         # Usually axis run in pairs, up/down for one, and left/right for
#         # the other.
#         axes = joystick.get_numaxes()
#         textPrint.write(screen, "Number of axes: {}".format(axes) )
#         textPrint.indent()
        
#         for i in range( axes ):
#             axis = joystick.get_axis( i )
#             textPrint.write(screen, "Axis {} value: {:>6.3f}".format(i, axis) )
#         textPrint.unindent()
            
#         buttons = joystick.get_numbuttons()
#         textPrint.write(screen, "Number of buttons: {}".format(buttons) )
#         textPrint.indent()

#         for i in range( buttons ):
#             button = joystick.get_button( i )
#             textPrint.write(screen, "Button {:>2} value: {}".format(i,button) )
#         textPrint.unindent()
            
#         # Hat switch. All or nothing for direction, not like joysticks.
#         # Value comes back in an array.
#         hats = joystick.get_numhats()
#         textPrint.write(screen, "Number of hats: {}".format(hats) )
#         textPrint.indent()

#         for i in range( hats ):
#             hat = joystick.get_hat( i )
#             textPrint.write(screen, "Hat {} value: {}".format(i, str(hat)) )
#         textPrint.unindent()
        
#         textPrint.unindent()

    
#     # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
    
#     # Go ahead and update the screen with what we've drawn.
#     pygame.display.flip()

#     # Limit to 20 frames per second
#     clock.tick(20)
    




