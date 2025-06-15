MOD=argo_radio_sail_module
OVERLAY=argo_radio_servo_overlay
SYS=/sys/kernel/argo_radio_sail
obj-m := $(MOD).o		

obj-m := argo_radio_sail_module # gpio_pulse_measurement_module.o # pwm_capture_module.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

overlay:
	 cpp -nostdinc -undef -x assembler-with-cpp -I ${KDIR}/include $(OVERLAY).dts $(OVERLAY).dts.cpp \
  	 && dtc -I dts -O dtb -o $(OVERLAY).dtbo $(OVERLAY).dts.cpp

install_overlay:
	cp $(OVERLAY).dtbo /boot/dtb/allwinner/overlay/sun50i-h616-$(OVERLAY).dtbo


insmod:
	# insmod pwm_capture_module.ko
	insmod $(MOD).ko

rmmod:
	rmmod $(MOD)

watch:
	watch -n .2 "cat $(SYS)/radio_sail;cat $(SYS)/radio_rudder"
