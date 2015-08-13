# Connection Constants
TRANSMISSION_TIMEOUT  = 0.75 # Send an update if we haven't transmitted
                             # in this period (s).
MIN_TRANSMIT_INTERVAL = 0.1 # Don't transmit more than once in this
                            # period (s).
MAX_CONNECT_TIME = 0.5      # Maximum time (s) to spend on a
                            # connection attempt before timing out.

# Controller Constants
XBOX = 1
KEYBOARD = 2

AXIS_0 = 0
AXIS_1 = 1
AXIS_2 = 2

NORMAL = True
INVERTED = False


# Color Constants
RED = (255,0,0)
GREEN = (0,255,0)
BLUE = (0,0,255)
CYAN = (0,255,255)
PURPLE = (255,0,255)
YELLOW = (255,255,0)        
WHITE = (255,255,255)
BLACK = (0,0,0)


# Flags bits
FLAGS_IGNITER_BIT = 0x08
FLAGS_LEFT_TRIGGER_BIT = 0x04
FLAGS_RIGHT_TRIGGER_BIT = 0x20
FLAGS_LOCK_IGNITER_BIT = 0x01

