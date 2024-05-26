TARGET		:= $(shell uname -r)
HOME=$(shell pwd)
KERNEL_MODULES	:= /lib/modules/$(TARGET)
KERNEL_BUILD	:= /usr/src/linux-headers-$(TARGET)

SYSTEM_MAP	:= /boot/System.map-$(TARGET)

DRIVER := spd5118

# Directory below /lib/modules/$(TARGET)/kernel into which to install
# the module:
MOD_SUBDIR = drivers/hwmon

obj-m	:= $(DRIVER).o

MAKEFLAGS += --no-print-directory

.PHONY: all install modules modules_install clean

all: modules

modules:
	@$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) $@

clean:
	@$(MAKE) -C $(KERNEL_BUILD) M=$(PWD) $@

install: modules_install

modules_install:
	cp $(DRIVER).ko $(KERNEL_MODULES)/kernel/$(MOD_SUBDIR)
	depmod -a -F $(SYSTEM_MAP) $(TARGET)
