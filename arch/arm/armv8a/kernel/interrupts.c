/*
 * interrupts.c - Interrupt Support Routines
 *
 * Copyright (c) 2007 Sascha Hauer <s.hauer@pengutronix.de>, Pengutronix
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/**
 * @file
 * @brief Interrupt Support Routines
 */

#include <stdio.h>

static void do_exception(void)
{
	error_printk("do_exception\n");
}

/**
 * The CPU runs into an undefined instruction. That really should not happen!
 * @param[in]  Register set content when the accident happens
 */
void do_undefined_instruction(void)
{
	error_printk ("undefined instruction\n");
	do_exception();
}

/**
 * The CPU catches a software interrupt
 * @param[in]  Register set content when the interrupt happens
 *
 * There is no function behind this feature. So what to do else than
 * a reset?
 */
void do_software_interrupt(void)
{
	error_printk ("software interrupt\n");
	do_exception();
}

/**
 * The CPU catches a prefetch abort. That really should not happen!
 * @param[in]  Register set content when the accident happens
 *
 * instruction fetch from an unmapped area
 */
void do_prefetch_abort(void)
{
	error_printk ("prefetch abort\n");
	do_exception();
}

/**
 * The CPU catches a data abort. That really should not happen!
 * @param[in]  Register set content when the accident happens
 *
 * data fetch from an unmapped area
 */
void do_data_abort(void)
{

	do_exception();
}

/**
 * The CPU catches a fast interrupt request.
 * @param[in]  Register set content when the interrupt happens
 *
 * We never enable FIQs, so this should not happen
 */
void do_fiq(void)
{
	error_printk ("fast interrupt request\n");
	do_exception();
}

/**
 * The CPU catches a regular interrupt.
 * @param[in]  Register set content when the interrupt happens
 *
 * We never enable interrupts, so this should not happen
 */
void do_irq(void)
{
	error_printk ("interrupt request\n");
	do_exception();
}

void do_bad_sync(void)
{
	error_printk("bad sync\n");
	do_exception();
}

void do_bad_irq(void)
{
	error_printk("bad irq\n");
	do_exception();
}

void do_bad_fiq(void)
{
	error_printk("bad fiq\n");
	do_exception();
}

void do_bad_error(void)
{
	error_printk("bad error\n");
	do_exception();
}

void do_sync(void)
{
	error_printk("sync exception\n");
	do_exception();
}


void do_error(void)
{
	error_printk("error exception\n");
	do_exception();
}

extern volatile int arm_ignore_data_abort;
extern volatile int arm_data_abort_occurred;

void data_abort_mask(void)
{
	arm_data_abort_occurred = 0;
	arm_ignore_data_abort = 1;
}

int data_abort_unmask(void)
{
	arm_ignore_data_abort = 0;

	return arm_data_abort_occurred != 0;
}
