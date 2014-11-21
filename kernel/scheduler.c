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
			first_switch_task(i);
		}
	}
}


/* At the moment it's a round-robin schedule */
/* TODO: the timer to schedule task can be based on TC interrupt */
void schedule(void)
{
	int i;
	int task_count = get_task_count();

	if (task_count == 1)
		printk(".");
	else if (task_count == 2)
		printk(":");
	else
		printk(";");

	for (i = 0; i < task_count; i++) {
		if (task[i].state == TASK_STOPPED) {
			if (i == 0)
				printk("idx0\r\n");
			else if (i == 1)
				printk("idx1\r\n");
			switch_task(i);
		}
	}
}

void end_task(void)
{

}
