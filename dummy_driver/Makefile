## Makefile for the dummy_driver kernel module.

obj-m := dummy_driver.o
KDIR := /lib/modules/`uname -r`

default:
	make -C $(KDIR)/build M=`pwd` modules
install:
	sudo cp dummy_driver.ko $(KDIR)
	sudo depmod -a
clean:
	make -C $(KDIR)/build M=`pwd` clean
	sudo rm $(KDIR)/dummy_driver.ko



