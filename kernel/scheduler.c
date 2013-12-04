#include <stdint.h>
#include <scheduler.h>


/* At the moment it's a round-robin schedule */
void schedule( void )
{
	int i;
	long quantum = 150;	/* in ms */

	while( 1 )
	{
		i = NR_TASK;

		current_task = &task[i];
		
		while( i-- )
		{
			if( (*current_task )->state == TASK_RUNNING )
			{

			}
		}

	}

}
