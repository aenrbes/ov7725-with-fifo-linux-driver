
obj-m += ov7725_fifo.o

KERNEL_DIR ?= /your/linux/source_tree/

all:
	make -C $(KERNEL_DIR) \
		ARCH=riscv CROSS_COMPILE=/your/toolchain/bin/riscv32-buildroot-linux-gnu- \
		SUBDIRS=$(PWD) modules
clean:
	make -C $(KERNEL_DIR) \
		ARCH=riscv CROSS_COMPILE=/your/toolchain/bin/riscv32-buildroot-linux-gnu- \
		SUBDIRS=$(PWD) clean
