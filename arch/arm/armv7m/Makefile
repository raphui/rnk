# Copyright (C) 2015 Raphaël Poggi. All rights reserved.
# Author: Raphaël Poggi <poggi.raph@gmail.com>
#
# Provided under the three clause BSD license found in the LICENSE file.

subdirs-y += kernel

obj-y := arch.o
obj-y += nvic.o
obj-y += mpu.o
obj-$(CONFIG_SWO_DEBUG) += swo.o
obj-y += systick.o
obj-y += handlers.o
obj-y += vector.o
