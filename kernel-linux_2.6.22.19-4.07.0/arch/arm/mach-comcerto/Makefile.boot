ifeq ($(CONFIG_EVM_C300V2), y)

# 512M of SDRAM is available on C300_V2, we need to map it into extended memory
ifeq ($(CONFIG_MSP_CARRIER), y)
  zreladdr-y     := 0x81008000
  params_phys-y  := 0x81000100
else
  zreladdr-y     := 0x80008000
  params_phys-y  := 0x80000100
endif

else

ifeq ($(CONFIG_MSP_CARRIER), y)
  zreladdr-y     := 0x01008000
  params_phys-y  := 0x01000100
else
  zreladdr-y     := 0x00008000
  params_phys-y  := 0x00000100
endif

endif
