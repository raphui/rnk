rnk
===

rnk is a RTOS targeting ARM architecture.

For now, only the Cortex-M4 is supported.

Features
--------

  * Premptive scheduling
  * Dynamic user application loading using custom file format - RFLAT
  * Isolation using MPU
  * Device tree utilisation
  * POSIX API
  * SEGGER SystemView support

Dependencies
------------
  * arm-none-eabi toolchain (https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
  * kconfig-frontends (http://ymorin.is-a-geek.org/projects/kconfig-frontends)
  * python3 with pyfdt module

Building
--------
  * export ARCH=arm
  * export your cross compiler bin/ folder in PATH
  * export CROSS_COMPILE=arm-none-eabi-
or
  * edit the cross compiler path in "setup.sh" and use it like: $. setup.sh
  
then
  * make the boards config you wanted (take a look at: arch/arm/configs), for example: make stm32f429_disco_defconfig
  * and then: make clean && make
  * flash the kernel on the board & enjoy

Current known bugs
------------------
