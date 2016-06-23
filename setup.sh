#!/bin/bash

export PATH=$PATH:/opt/cross/x-tools/gcc-arm-none-eabi-4_9-2014q4/bin
export CROSS_COMPILE=arm-none-eabi-
export KERNEL_BASE=$(pwd)
export ARCH=arm
