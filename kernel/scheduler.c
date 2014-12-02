#include <scheduler.h>
#include <task.h>
#include <io.h>
#include <pit.h>
#include <board.h>

int task_switching = 0;

void start_schedule(void)
{
	int i;

	pit_init(PIT_PERIOD, BOARD_MCK / 1000000);
	pit_enable_it();
	pit_enable();

	i = find_next_task();
	first_switch_task(i);
}


void schedule(void)
{
	int i;
	int task_count = get_task_count();

	pit_read_pivr();

	if (task_count == 1)
		printk(".");
	else if (task_count == 2)
		printk(":");
	else
		printk(";");

	printk("choosing/");

	i = find_next_task();
	switch_task(i);
	task_switching = 1;
}

/* Since tasks cannot end, if we jump into this functions it's mean that the context switch is buggy */
void end_task(void)
{
	void (*func)(void) = (void (*)())0xfffffd39;

	func();
}
