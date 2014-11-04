#!/bin/bash

export PATH=$PATH:/opt/cross/x-tools/arm-unknown-eabi/bin
export CROSS_COMPILE=arm-unknown-eabi-
export TOOLCHAIN_DIR=/opt/cross/x-tools/arm-unknown-eabi/
export KERNEL_BASE=$(pwd)
