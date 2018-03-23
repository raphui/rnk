#!/bin/bash

export CROSS_COMPILE=arm-none-eabi-
export KERNEL_BASE=$(pwd)
export APPS_BASE=$KERNEL_BASE/apps
export ARCH=arm
export KCONFIG=/usr/local/bin
