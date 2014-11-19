#include <scheduler.h>
#include <task.h>
#include <io.h>


/* At the moment it's a round-robin schedule */
/* TODO: the timer to schedule task can be based on TC interrupt */
void schedule(void)
{
	int i;
	printk(".");

	for (i = 0; i < task_count; i++) {
		if (task[i].state == TASK_STOPPED) {
			switch_task(task[i]);
		}
	}
}

void end_task(void)
{

}
