KERNELDIR := /home/alientek/sdk/kernel
CURRENT_PATH := $(shell pwd)

# obj-m := v4l2.o
# obj-m := piris.o
obj-m := ms41908.o
# obj-m := ms41908_piris.o

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
	