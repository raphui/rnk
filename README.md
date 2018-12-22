rnk
===

rnk is a RTOS targeting ARM architecture.

For now, only the Cortex-M4 is supported.


Building
--------

  * get a cross compiler for baremetal arm, here: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
  * install kconfig-frontends: http://ymorin.is-a-geek.org/projects/kconfig-frontends
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
