rnk
===

rnk is a RTOS targeting ARM architecture.

Features
--------

  * Targeting Cortex-M
  * Premptive scheduling
  * Support static and dynamic application (using custom file format - RFLAT)
  * Isolation using MPU and privileged/unprivileged modes
  * Device tree support
  * POSIX API
  * SEGGER SystemView support
  
Cortex-M MCU supported
----------------------

  * STM32F4 family (actually developed and tested on a STM32F401)
  * STM32L4 family (actually developed and tested on a STM32L443)

Dependencies
------------
  * arm-none-eabi toolchain (https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
  * kconfiglib (https://github.com/ulfalizer/Kconfiglib)

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
  
Configuration
-------------

Build configurations are powered by Kconfig, and can be used by typing: make menuconfig (or nconfig)

Current known bugs
------------------
