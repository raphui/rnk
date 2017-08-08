# Copyright (C) 2015 Raphaël Poggi. All rights reserved.
# Author: Raphaël Poggi <poggi.raph@gmail.com>
#
# Provided under the three clause BSD license found in the LICENSE file.

subdirs-y += kernel

obj-y := arch.o
obj-y += aeabifuncs.o
obj-y += div0.o
obj-y += exceptions.o
obj-$(CONFIG_MACH_HAVE_GENERIC_TIMER) += timer.o
