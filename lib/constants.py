# Debug Flags
DEBUG_SCAN = False        # Debug blimp scanning.
DEBUG_TX = False          # Print debug messages for transmissions.
DEBUG_RX = True           # Print debug messages for generic received
                          # text.
DEBUG_UPDATE = False      # Print debug messages for received updates.
DEBUG_CONNECT = False     # Print debug messages for connections and
                          # disconnections.
DEBUG_VOLTAGE = True      # Print debug messages for blimp voltage
                          # status.
DEBUG_TRIGGER = False     # Print debug messages of trigger on/off.
DEBUG_IGNITER = False     # Print debug messages of igniter on/off.
DEBUG_CONTROLLERS = True  # Debug controller configurations.


# Connection Constants
TRANSMISSION_TIMEOUT  = 0.75 # Send an update if we haven't transmitted
                             # in this period (s).
MIN_TRANSMIT_INTERVAL = 0.1  # Don't transmit more than once in this
                             # period (s).
MAX_CONNECT_TIME = 2.5       # Maximum time (s) to spend on a
                             # connection attempt before timing out.
BLIMP_MISSING_TIME  = 1.0    # Time (s) without an update before a blimp is marked missing.


# Controller Constants
XBOX = 1
KEYBOARD = 2

AXIS_0 = 0
AXIS_1 = 1
AXIS_2 = 2

NORMAL = True
INVERTED = False


# GUI Constants
DISPLAY_UPDATE_TIME = 0.0   # Don't update the display more than this
                            # often (s).
GAME_NAME = "Battle Blimps" # Game name for window title(s).
BLIMP_OUTER_BORDER=O=3
BLIMP_INNER_BORDER=I=1
COL_WIDTH=16
BLIMP_AXIS_COL_WIDTH=COL_WIDTH*3
BLIMP_AXIS_WIDTH=BLIMP_AXIS_COL_WIDTH*3
BLIMP_AXIS_SLIDER_WIDTH=BLIMP_AXIS_WIDTH*1.5
BLIMP_AXIS_SLIDER_HEIGHT=100


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

