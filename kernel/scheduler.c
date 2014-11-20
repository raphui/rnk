#include <scheduler.h>
#include <task.h>
#include <io.h>
#include <pit.h>
#include <board.h>


void start_schedule(void)
{
	int i;
	int task_count = get_task_count();

	/* Initialise and start PIT */
	pit_init(PIT_PERIOD, BOARD_MCK / 1000000);
	pit_enable_it();
	pit_enable();

	for (i = 0; i < task_count; i++) {
		if (task[i].state == TASK_STOPPED) {
			first_switch_task(task[i]);
		}
	}
}


/* At the moment it's a round-robin schedule */
/* TODO: the timer to schedule task can be based on TC interrupt */
void schedule(void)
{
	int i;
	int task_count = get_task_count();

	if (task_count)
		printk(".\r\n");
	else
		printk(";\r\n");

	for (i = 0; i < task_count; i++) {
		if (task[i].state == TASK_STOPPED) {
			printk("switching to : %x\r\n", i);
			switch_task(task[i]);
		}
	}
}

void end_task(void)
{

}
