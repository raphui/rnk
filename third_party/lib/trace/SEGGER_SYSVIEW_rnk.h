/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*       (c) 2015 - 2017  SEGGER Microcontroller GmbH & Co. KG        *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER SystemView * Real-time application analysis           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* SEGGER strongly recommends to not make any changes                 *
* to or modify the source code of this software in order to stay     *
* compatible with the RTT protocol and J-Link.                       *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* conditions are met:                                                *
*                                                                    *
* o Redistributions of source code must retain the above copyright   *
*   notice, this list of conditions and the following disclaimer.    *
*                                                                    *
* o Redistributions in binary form must reproduce the above          *
*   copyright notice, this list of conditions and the following      *
*   disclaimer in the documentation and/or other materials provided  *
*   with the distribution.                                           *
*                                                                    *
* o Neither the name of SEGGER Microcontroller GmbH & Co. KG         *
*   nor the names of its contributors may be used to endorse or      *
*   promote products derived from this software without specific     *
*   prior written permission.                                        *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************
*                                                                    *
*       SystemView version: V2.52d                                    *
*                                                                    *
**********************************************************************
-------------------------- END-OF-HEADER -----------------------------

File    : SEGGER_SYSVIEW_FreeRTOS.h
Purpose : Interface between FreeRTOS and SystemView.
Revision: $Rev: 7745 $

Notes:
  (1) Include this file at the end of FreeRTOSConfig.h
*/

#ifndef SYSVIEW_RNK_H
#define SYSVIEW_RNK_H

#include "SEGGER_SYSVIEW.h"

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/

/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/
#define apiID_OFFSET                              (32)

#define apiID_THREAD_CREATE		(1)
#define apiID_THREAD_STOP		(2)
#define apiID_THREAD_JOIN		(3)
#define apiID_THREAD_SWITCH		(4)
#define apiID_THREAD_MAKE_RUNNABLE	(5)
#define apiID_THREAD_MAKE_BLOCKED	(6)
#define apiID_SCHED_THREAD		(7)
#define apiID_SCHED_YIELD		(8)
#define apiID_MUTEX_CREATE		(9)
#define apiID_MUTEX_LOCK		(10)
#define apiID_MUTEX_UNLOCK		(11)
#define apiID_SEM_CREATE		(12)
#define apiID_SEM_WAIT			(13)
#define apiID_SEM_POST			(14)
#define apiID_QUEUE_CREATE		(15)
#define apiID_QUEUE_SEND		(16)
#define apiID_QUEUE_RECEIVE		(17)
#define apiID_QUEUE_CLEAR		(18)
#define apiID_QUEUE_DESTROY		(19)
#define apiID_QUEUE_UPDATE		(20)
#define apiID_TIME_SLEEP		(21)
#define apiID_MEM_ALLOC			(22)
#define apiID_MEM_FREE			(23)


#ifdef CONFIG_TRACE
//#define trace_thread_create(thread)			SEGGER_SYSVIEW_RecordU32(apiID_OFFSET + apiID_THREAD_CREATE, thread)
#define trace_thread_join(thread)			SEGGER_SYSVIEW_RecordU32(apiID_OFFSET + apiID_THREAD_JOIN, SEGGER_SYSVIEW_ShrinkId((U32)thread))
//#define trace_thread_runnable(thread)			SEGGER_SYSVIEW_RecordU32(apiID_OFFSET + apiID_THREAD_MAKE_RUNNABLE, thread)
//#define trace_thread_blocked(thread)			SEGGER_SYSVIEW_RecordU32(apiID_OFFSET + apiID_THREAD_MAKE_BLOCKED, thread)
//#define trace_sched_thread(thread)			SEGGER_SYSVIEW_RecordU32x2(apiID_OFFSET + apiID_SCHED_THREAD, thread, get_current_thread())
#define trace_sched_yield()				SEGGER_SYSVIEW_RecordVoid(apiID_OFFSET + apiID_SCHED_YIELD)
#define trace_mutex_create(mutex)			SEGGER_SYSVIEW_RecordU32(apiID_OFFSET + apiID_MUTEX_CREATE, SEGGER_SYSVIEW_ShrinkId((U32)mutex)) 
#define trace_mutex_lock(mutex)				SEGGER_SYSVIEW_RecordU32(apiID_OFFSET + apiID_MUTEX_LOCK, SEGGER_SYSVIEW_ShrinkId((U32)mutex)) 
#define trace_mutex_unlock(mutex)			SEGGER_SYSVIEW_RecordU32(apiID_OFFSET + apiID_MUTEX_UNLOCK, SEGGER_SYSVIEW_ShrinkId((U32)mutex))
#define trace_sem_create(sem)				SEGGER_SYSVIEW_RecordU32(apiID_OFFSET + apiID_SEM_CREATE, SEGGER_SYSVIEW_ShrinkId((U32)sem))
#define trace_sem_wait(sem)				SEGGER_SYSVIEW_RecordU32x2(apiID_OFFSET + apiID_SEM_WAIT, SEGGER_SYSVIEW_ShrinkId((U32)sem), sem->count)
#define trace_sem_post(sem)				SEGGER_SYSVIEW_RecordU32x2(apiID_OFFSET + apiID_SEM_POST, SEGGER_SYSVIEW_ShrinkId((U32)sem), sem->count)
#define trace_queue_create(queue, item_size, size)	SEGGER_SYSVIEW_RecordU32x3(apiID_OFFSET + apiID_QUEUE_CREATE, SEGGER_SYSVIEW_ShrinkId((U32)queue), item_size, size)
#define trace_queue_send(queue, item)			SEGGER_SYSVIEW_RecordU32x2(apiID_OFFSET + apiID_QUEUE_SEND, SEGGER_SYSVIEW_ShrinkId((U32)queue), (U32)item)
#define trace_queue_receive(queue, timeout)		SEGGER_SYSVIEW_RecordU32x2(apiID_OFFSET + apiID_QUEUE_RECEIVE, SEGGER_SYSVIEW_ShrinkId((U32)queue), timeout)
#define trace_queue_clear(queue)			SEGGER_SYSVIEW_RecordU32(apiID_OFFSET + apiID_QUEUE_CLEAR, SEGGER_SYSVIEW_ShrinkId((U32)queue))
#define trace_queue_destroy(queue)			SEGGER_SYSVIEW_RecordU32(apiID_OFFSET + apiID_QUEUE_DESTROY, SEGGER_SYSVIEW_ShrinkId((U32)queue))
#define trace_queue_update(queuei, item_size, size)	SEGGER_SYSVIEW_RecordU32x3(apiID_OFFSET + apiID_QUEUE_UPDATE, SEGGER_SYSVIEW_ShrinkId((U32)queue), item_size, size)
#define trace_mem_alloc(size)				SEGGER_SYSVIEW_RecordU32(apiID_OFFSET + apiID_MEM_ALLOC, size)
#define trace_mem_free(size)				SEGGER_SYSVIEW_RecordU32(apiID_OFFSET + apiID_MEM_FREE, size)



#define trace_thread_stop(thread)                   {                                                                                                   \
                                                      SEGGER_SYSVIEW_RecordU32(apiID_OFFSET + apiID_THREAD_STOP, SEGGER_SYSVIEW_ShrinkId((U32)thread));  \
                                                      SYSVIEW_DeleteTask((U32)thread);                                                                   \
                                                    }


#define trace_thread_create(thread)                  if (thread != NULL) {						\
                                                      SEGGER_SYSVIEW_OnTaskCreate((U32)thread);				\
                                                      SYSVIEW_AddTask((U32)thread,					\
                                                                      thread->name,					\
                                                                      thread->start_stack,				\
								      CONFIG_THREAD_STACK_SIZE				\
                                                                      );                                                \
                                                    }

#define trace_sched_thread(thread)                   {                                                                   \
                                                      if (thread->pid != 0) {           \
                                                        SEGGER_SYSVIEW_OnTaskStartExec((U32)thread);              \
                                                      } else {                                                          \
                                                        SEGGER_SYSVIEW_OnIdle();                                        \
                                                      }                                                                 \
                                                    }

#define trace_thread_runnable(thread)		{								\
                                                      if (thread->pid != 0) {					\
							SEGGER_SYSVIEW_OnTaskStartReady((U32)thread);		\
						      }								\
						}								\

#define trace_time_usleep(thread)		SEGGER_SYSVIEW_OnTaskStopReady((U32)thread,  1)
#define trace_thread_blocked(thread)		SEGGER_SYSVIEW_OnTaskStopReady((U32)thread, 4)


//#define traceISR_EXIT_TO_SCHEDULER()                SEGGER_SYSVIEW_RecordExitISRToScheduler()
#define trace_exit_isr()                            SEGGER_SYSVIEW_RecordExitISR()
#define trace_enter_isr()                           SEGGER_SYSVIEW_RecordEnterISR()
#else

//#define trace_thread_create(thread)
#define trace_thread_join(thread)
//#define trace_thread_runnable(thread)
//#define trace_thread_blocked(thread)
//#define trace_sched_thread(thread)
#define trace_sched_yield()
#define trace_mutex_create(mutex)
#define trace_mutex_lock(mutex)
#define trace_mutex_unlock(mutex)
#define trace_sem_create(sem)
#define trace_sem_wait(sem)
#define trace_sem_post(sem)
#define trace_queue_create(queue, item_size, size)
#define trace_queue_send(queue, item)
#define trace_queue_receive(queue, timeout)
#define trace_queue_clear(queue)
#define trace_queue_destroy(queue)
#define trace_queue_update(queuei, item_size, size)
#define trace_mem_alloc(size)
#define trace_mem_free(size)
#define trace_thread_stop(thread)
#define trace_thread_create(thread)
#define trace_sched_thread(thread)
#define trace_thread_runnable(thread)
#define trace_time_usleep(thread)
#define trace_thread_blocked(thread)


//#define traceISR_EXIT_TO_SCHEDULER()                SEGGER_SYSVIEW_RecordExitISRToScheduler()
#define trace_exit_isr()
#define trace_enter_isr()
#endif

/*********************************************************************
*
*       API functions
*
**********************************************************************
*/
#ifdef __cplusplus
extern "C" {
#endif
void SYSVIEW_AddTask      (unsigned int pid, char *name, unsigned int stack, unsigned int top_stack);
void SYSVIEW_UpdateTask   (unsigned int pid, char *name, unsigned int stack, unsigned int top_stack);
void SYSVIEW_DeleteTask   (unsigned int pid);
void SYSVIEW_SendTaskInfo (unsigned int pid, char *name, unsigned int stack, unsigned int top_stack);

#ifdef __cplusplus
}
#endif

#endif

/*************************** End of file ****************************/
