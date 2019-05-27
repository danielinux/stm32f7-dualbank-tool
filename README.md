# STM32F7 dual-bank utility

This small utility consists of two programs running on STM32F7[6|7]x used to switch between single mode and dual-mode flash.

## Pre-requisites

- openocd
- arm-none-eabi toolchain

## how to build

Type `make`. 

## how to use

The `make` command will generate two binaries:
   - single-bank.bin
   - dualbank.bin

Switching mode is done by executing either firmware from the target RAM, using openocd.

Automatic openocd scripts can be executed using

`make single`

or

`make dualbank`

to set the flash configuration to single or dual-bank respectively.

## checking current configuration

Type `make check`. The output will show either 

```
Info : Single Bank 2048 kiB STM32F76x/77x found
```

or

```
Info : Dual Bank 2048 kiB STM32F76x/77x found
```

## More information

See STMicroelectronics "AN4826: STM32F7 Series Flash memory dual bank mode"
