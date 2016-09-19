rnk
===

rnk is a RTOS targeting ARM architecture.

For now, only the Cortex-M4 is supported. ARM7TDMI is no longer supported.


Building
--------

  * get a cross compiler for baremetal arm, here: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
  * export ARCH=arm
  * export your cross compiler bin/ folder in PATH
  * export CROSS_COMPILE=arm-none-eabi-
  
or
  * edit the cross compiler path in "setup.sh" and use it like: $. setup.sh
  
then
  * make the boards config you wanted (take a look at: arch/arm/configs), for example: make stm32f429_disco_defconfig
  * and then: make symbols-clean && make clean && make all && make symbols-make && make clean && make all (yes it is painful but it is needed for include symbol lookup table)
  * flash the kernel on the board & enjoy

Current known bugs
------------------

  * Building the kernel with symbols lookup table for using apps/ and quite painful since you need to build 2 times the kernel for : compile without the symbols / generate the symbols lookup array / compile with the symbols lookup array.
  * Scheduling algo needs to be improved on real applications.
  * Writing data in flash on stm32 boards are buggy.
