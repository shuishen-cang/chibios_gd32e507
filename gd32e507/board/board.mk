# List of all the board related files.
BOARDSRC = $(CHIBIOS)/gd32e507/board/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/gd32e507/board


ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
