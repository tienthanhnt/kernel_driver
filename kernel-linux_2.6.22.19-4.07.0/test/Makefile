PWD := $(shell pwd)
obj-m += hello.o
GCC := /home/macom/OpenWrtToolChain/staging_dir/toolchain-arm_gcc-4.3.4_uClibc-0.9.30.1/usr/bin/arm-openwrt-linux-
KERNEL := /home/macom/linux_kernel/macom/working/kernel_driver/kernel-linux_2.6.22.19-4.07.0/
LD_DEBUG=all make
all:
	make ARCH=arm CROSS_COMPILE=$(GCC) -C $(KERNEL) M=$(PWD) 
	$(GCC)gcc  hello.c -o hello
clean:
	make -C $(KERNEL) M=$(PWD) clean
