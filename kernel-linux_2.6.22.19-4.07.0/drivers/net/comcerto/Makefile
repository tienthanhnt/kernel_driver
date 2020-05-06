obj-y += eth.o

ifeq ($(CONFIG_COMCERTO_ETH), y)
obj-$(CONFIG_ARCH_M823XX) += gemac.o gemac-mdio.o
obj-$(CONFIG_ARCH_M825XX2) += emac.o emac-mdio.o
endif

obj-$(CONFIG_COMCERTO_VED) += smi.o ved.o
