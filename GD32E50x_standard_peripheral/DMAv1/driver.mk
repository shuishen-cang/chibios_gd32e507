GD32_DMASRC += $(CHIBIOS)/GD32E50x_standard_peripheral/DMAv1/gd32_dma.c
GD32_DMAINC += $(CHIBIOS)/GD32E50x_standard_peripheral/DMAv1

# Shared variables
ALLCSRC += $(GD32_DMASRC)
ALLINC  += $(GD32_DMAINC)
