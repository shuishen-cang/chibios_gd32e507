# List of all the board related files.
GDLIBSRC = 	$(CHIBIOS)/GD32E50x_standard_peripheral/system_gd32e50x.c 		\
			$(CHIBIOS)/GD32E50x_standard_peripheral/Source/gd32e50x_misc.c	\
			$(CHIBIOS)/GD32E50x_standard_peripheral/Source/gd32e50x_rcu.c	\
			$(CHIBIOS)/GD32E50x_standard_peripheral/Source/gd32e50x_timer.c	\
			$(CHIBIOS)/GD32E50x_standard_peripheral/Source/gd32e50x_usart.c	\
			$(CHIBIOS)/GD32E50x_standard_peripheral/Source/gd32e50x_gpio.c	

# Required include directories
GDLIBINC = $(CHIBIOS)/GD32E50x_standard_peripheral/Include

# Shared variables
ALLCSRC += $(GDLIBSRC)
ALLINC  += $(GDLIBINC)
