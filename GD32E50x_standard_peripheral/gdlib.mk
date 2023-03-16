# List of all the board related files.
GDLIBSRC = 	$(CHIBIOS)/GD32E50x_standard_peripheral/system_gd32e50x.c 		\
			$(CHIBIOS)/GD32E50x_standard_peripheral/Source/gd32e50x_misc.c	\
			$(CHIBIOS)/GD32E50x_standard_peripheral/Source/gd32e50x_rcu.c	\
			$(CHIBIOS)/GD32E50x_standard_peripheral/Source/gd32e50x_timer.c	\
			$(CHIBIOS)/GD32E50x_standard_peripheral/Source/gd32e50x_usart.c	\
			$(CHIBIOS)/GD32E50x_standard_peripheral/Source/gd32e50x_gpio.c	\
			$(CHIBIOS)/GD32E50x_standard_peripheral/Source/gd32e50x_pmu.c	\
			$(CHIBIOS)/GD32E50x_standard_peripheral/usb_driver/Source/drv_usb_core.c	\
			$(CHIBIOS)/GD32E50x_standard_peripheral/usb_driver/Source/drv_usb_dev.c		\
			$(CHIBIOS)/GD32E50x_standard_peripheral/usb_driver/Source/drv_usbd_int.c 	\
			$(CHIBIOS)/GD32E50x_standard_peripheral/usb_driver/Source/usbd_core.c		\
			$(CHIBIOS)/GD32E50x_standard_peripheral/usb_driver/Source/usbd_enum.c		\
			$(CHIBIOS)/GD32E50x_standard_peripheral/usb_driver/Source/usbd_transc.c		
			


# Required include directories
GDLIBINC = 	$(CHIBIOS)/GD32E50x_standard_peripheral/Include \
			$(CHIBIOS)/GD32E50x_standard_peripheral/usb_driver/Include 

# Shared variables
ALLCSRC += $(GDLIBSRC)
ALLINC  += $(GDLIBINC)

