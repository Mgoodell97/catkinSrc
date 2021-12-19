# mps_driver
Driver for interfacing with NN MPS sensor through I2C.

## Prerequisite ##
Please install the wiringPi library from [github](https://github.com/hardkernel/wiringPi) first.

Edit ```/etc/modules``` and add ```i2c-dev```, ```i2c-gpio```, ```aml_i2c```.

```sudo apt-get update && sudo apt-get install python-smbus i2c-tools```

```sudo adduser odroid i2c```

## Use GPIO for Power Control ##
To enable the GPIO from wiringPi, follow the instruction on [Rootless GPIO access](http://odroid.com/dokuwiki/doku.php?id=en:gpiomem).

> **If no .ko file is provided.**

> - Download [available kernel](https://github.com/hardkernel/linux/tree/odroidc2-3.14.y)
> - ```cd linux-odroidc2-3.14.y```
> - Change Makefile SUBLEVEL = 65-61, which can be read from ```uname -r```
> - ```make defconfig```
> - ```make drivers/char/meson-gpiomem.ko```
> - ```sudo cp drivers/char/meson-gpiomem.ko /lib/modules/3.14.65-61/kernel/drivers/char/meson-gpiomem.ko```
> - ```sudo modinfo meson-gpiomem```
> - ```sudo modprobe meson-gpiomem```

> **If .ko file is provided.**

> - ```sudo cp /PATH_TO_KO/meson-gpiomem.ko /lib/modules/3.14.65-61/kernel/drivers/char/meson-gpiomem.ko```
> - ```sudo modinfo meson-gpiomem```
> - ```sudo modprobe meson-gpiomem```
