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

File    : SEGGER_SYSVIEW_FreeRTOS.c
Purpose : Interface between FreeRTOS and SystemView.
Revision: $Rev: 7947 $
*/
#include <kernel/thread.h>
#include <arch/thread.h>
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_SYSVIEW_rnk.h"
#include <string.h> // Required for memset



typedef struct SYSVIEW_RNK_TASK_STATUS SYSVIEW_RNK_TASK_STATUS;

struct SYSVIEW_RNK_TASK_STATUS {
	unsigned int pid;
	char *name;
	unsigned int stack;
	unsigned int top_stack;
};

static SYSVIEW_RNK_TASK_STATUS _aTasks[10];
static unsigned _NumTasks;

extern unsigned int system_tick;

/*********************************************************************
 *
 *       _cbSendTaskList()
 *
 *  Function description
 *    This function is part of the link between FreeRTOS and SYSVIEW.
 *    Called from SystemView when asked by the host, it uses SYSVIEW
 *    functions to send the entire task list to the host.
 */
static void _cbSendTaskList(void) {
	unsigned n;

	for (n = 0; n < _NumTasks; n++) {
#if INCLUDE_uxTaskGetStackHighWaterMark // Report Task Stack High Watermark
		_aTasks[n].uStackHighWaterMark = uxTaskGetStackHighWaterMark((TaskHandle_t)_aTasks[n].xHandle);
#endif
		SYSVIEW_SendTaskInfo(_aTasks[n].pid, _aTasks[n].name, _aTasks[n].stack, _aTasks[n].top_stack);
	}
}

/*********************************************************************
 *
 *       _cbGetTime()
 *
 *  Function description
 *    This function is part of the link between FreeRTOS and SYSVIEW.
 *    Called from SystemView when asked by the host, returns the
 *    current system time in micro seconds.
 */
static U64 _cbGetTime(void) {
	U64 Time;

	Time = system_tick;

	return Time;
}

/*********************************************************************
 *
 *       Global functions
 *
 **********************************************************************
 */
/*********************************************************************
 *
 *       SYSVIEW_AddTask()
 *
 *  Function description
 *    Add a task to the internal list and record its information.
 */
void SYSVIEW_AddTask(unsigned int pid, char *name, unsigned int stack, unsigned int top_stack)
{ 
	if (pid == 0) {
		return;
	}

	if (_NumTasks >= 10) {
		SEGGER_SYSVIEW_Warn("SYSTEMVIEW: Could not record task information. Maximum number of tasks reached.");
		return;
	}

	_aTasks[_NumTasks].pid = pid;
	_aTasks[_NumTasks].name = name;
	_aTasks[_NumTasks].stack = stack;
	_aTasks[_NumTasks].top_stack = top_stack;

	_NumTasks++;

	SYSVIEW_SendTaskInfo(pid, name, stack, top_stack);

}

/*********************************************************************
 *
 *       SYSVIEW_UpdateTask()
 *
 *  Function description
 *    Update a task in the internal list and record its information.
 */
void SYSVIEW_UpdateTask(unsigned int pid, char *name, unsigned int stack, unsigned int top_stack)
{
	unsigned n;

	if (pid == 0) {
		return;
	}


	for (n = 0; n < _NumTasks; n++) {
		if (_aTasks[n].pid == pid) {
			break;
		}
	}
	if (n < _NumTasks) {
		_aTasks[_NumTasks].pid = pid;
		_aTasks[_NumTasks].name = name;
		_aTasks[_NumTasks].stack = stack;
		_aTasks[_NumTasks].top_stack = top_stack;


		SYSVIEW_SendTaskInfo(pid, name, stack, top_stack);
	} else {
		SYSVIEW_SendTaskInfo(pid, name, stack, top_stack);
	}
}

/*********************************************************************
 *
 *       SYSVIEW_DeleteTask()
 *
 *  Function description
 *    Delete a task from the internal list.
 */
void SYSVIEW_DeleteTask(unsigned int pid)
{
	unsigned n;

	if (_NumTasks == 0) {
		return; // Early out
	}  
	for (n = 0; n < _NumTasks; n++) {
		if (_aTasks[n].pid == pid) {
			break;
		}
	}
	if (n == (_NumTasks - 1)) {  
		//
		// Task is last item in list.
		// Simply zero the item and decrement number of tasks.
		//
		memset(&_aTasks[n], 0, sizeof(_aTasks[n]));
		_NumTasks--;
	} else if (n < _NumTasks) {
		//
		// Task is in the middle of the list.
		// Move last item to current position and decrement number of tasks.
		// Order of tasks does not really matter, so no need to move all following items.
		//
		_aTasks[n].pid             = _aTasks[_NumTasks - 1].pid;
		_aTasks[n].name   = _aTasks[_NumTasks - 1].name;
		_aTasks[n].stack             = _aTasks[_NumTasks - 1].stack;
		_aTasks[n].top_stack = _aTasks[_NumTasks - 1].top_stack;
		memset(&_aTasks[_NumTasks - 1], 0, sizeof(_aTasks[_NumTasks - 1]));
		_NumTasks--;
	}
}

/*********************************************************************
 *
 *       SYSVIEW_SendTaskInfo()
 *
 *  Function description
 *    Record task information.
 */
void SYSVIEW_SendTaskInfo (unsigned int pid, char *name, unsigned int stack, unsigned int top_stack)
{
	SEGGER_SYSVIEW_TASKINFO TaskInfo;

	memset(&TaskInfo, 0, sizeof(TaskInfo)); // Fill all elements with 0 to allow extending the structure in future version without breaking the code
	TaskInfo.TaskID     = pid;
	TaskInfo.sName       = name;
	TaskInfo.StackBase  = stack;
	TaskInfo.StackSize  = CONFIG_THREAD_STACK_SIZE;
	SEGGER_SYSVIEW_SendTaskInfo(&TaskInfo);
}

/*********************************************************************
 *
 *       Public API structures
 *
 **********************************************************************
 */
// Callbacks provided to SYSTEMVIEW by FreeRTOS
const SEGGER_SYSVIEW_OS_API SYSVIEW_X_OS_TraceAPI = {
	_cbGetTime,
	_cbSendTaskList,
};

/*************************** End of file ****************************/
