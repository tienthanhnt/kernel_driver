#
# Makefile for the linux kernel.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).


# Object file lists.

obj-y := irq.o gpio.o
obj-m :=
obj-n :=
obj- :=

obj-$(CONFIG_PCI) += pci.o

obj-$(CONFIG_ARCH_M823XX) += comcerto-300.o time.o
obj-$(CONFIG_ARCH_M825XX2) += comcerto-515.o time.o

obj-$(CONFIG_MSP_CARRIER) += msp.o

obj-$(CONFIG_EVM_C300V1) += board-c300evm.o
obj-$(CONFIG_EVM_C300V2) += board-c300v2evm.o
obj-$(CONFIG_EVM_MEGAMOMBASA515) += board-megamombasa515.o
