obj-m := gpio_pulse_measurement_module.o # pwm_capture_module.o

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

overlay:
	 cpp -nostdinc -undef -x assembler-with-cpp -I ${KDIR}/include pwm_capture_module_gpio_overlay.dts pwm_capture_module_gpio_overlay.dts.cpp \
  	 && dtc -I dts -O dtb -o pwm_capture_module_gpio_overlay.dtbo pwm_capture_module_gpio_overlay.dts.cpp

install_overlay:
	cp pwm_capture_module_gpio_overlay.dtbo /boot/dtb/allwinner/overlay/sun50i-h616-pwm_capture_module_overlay.dtbo

gpio_pulse_measurement_module_overlay:
	 cpp -nostdinc -undef -x assembler-with-cpp -I ${KDIR}/include gpio_pulse_measurement_module.dts gpio_pulse_measurement_module.dts.cpp \
  	 && dtc -I dts -O dtb -o gpio_pulse_measurement_module.dtbo gpio_pulse_measurement_module.dts.cpp

install_gpio_pulse_measurement_module_overlay:
	cp gpio_pulse_measurement_module.dtbo /boot/dtb/allwinner/overlay/sun50i-h616-gpio_pulse_measurement_module.dtbo
	echo 'edit /boot/orangepiEnv.tx to include the overlay as gpio_pulse_measurement_module'


insmod:
	# insmod pwm_capture_module.ko
	insmod gpio_pulse_measurement_module.ko

rmmod:
	rmmod gpio_pulse_measurement_module

watch:
	watch -n .2 cat /sys/kernel/allwinner_h618_gpio_pulse_time/pulse_time
