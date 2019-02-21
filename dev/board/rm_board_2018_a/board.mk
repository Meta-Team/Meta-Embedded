# List of all the board related files.
BOARDSRC = $(CHIBIOS)/dev/board/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/dev/board

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
