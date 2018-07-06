#ifndef __OS_H__

#include <kernel/ksem.h>
#include <kernel/ktime.h>


typedef struct semaphore SEMAPHORE_TYPE ;

#define SEM_POST(a)				ksem_post(&a)
#define SEM_POST_ISR(a)			ksem_post(&a)
#define SEM_WAIT(a)				ksem_wait(&a)
#define SEM_INIT(sem,cnt,lim)	ksem_init(&sem,lim)

#define DELAY(ms)				uDelay(ms * 1000)
#define GET_TIME()				//nanoGetTime()
#define SET_PRIORITY( a, b )	// nvic_setpriority( (a), b) )

#endif // __OS_H__
