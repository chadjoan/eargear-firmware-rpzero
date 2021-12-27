# List of all the board related files.
BOARDSRC = $(CHIBIOS)/boards/RP_ZERO/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/boards/RP_ZERO

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
