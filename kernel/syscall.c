#include <stdarg.h>
#include <errno.h>
#include <arch/system.h>
#include <kernel/syscall.h>
#include <kernel/scheduler.h>
#include <kernel/kmutex.h>
#include <kernel/kqueue.h>
#include <kernel/ksem.h>
#include <kernel/ktime.h>
#include <fs/io_ops.h>
#include <elf/elfloader.h>
#include <mm/mm.h>
#include <drv/pio.h>

struct syscall syscall_table[] = {
	{SYSCALL_THREAD_SWITCH,	(unsigned int *)&schedule_yield,	SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_THREAD_CREATE, (unsigned int *)&thread_create,		SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_THREAD_JOIN,	(unsigned int *)&thread_join,		SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_THREAD_STOP,	(unsigned int *)&schedule_thread_stop,	SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_THREAD_SUSPEND,(unsigned int *)&thread_suspend,	SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_THREAD_RESUME,	(unsigned int *)&thread_resume,		SYSCALL_PRIVILEGE_OPERATION},
#ifdef CONFIG_MUTEX
	{SYSCALL_MUTEX_CREATE,	(unsigned int *)&kmutex_init,		SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_MUTEX_ACQUIRE, (unsigned int *)&kmutex_lock,		SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_MUTEX_RELEASE, (unsigned int *)&kmutex_unlock,		SYSCALL_PRIVILEGE_ELEVATION},
#endif /* CONFIG_MUTEX */
#ifdef CONFIG_SEMAPHORE
	{SYSCALL_SEM_CREATE,	(unsigned int *)&ksem_init,		SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_SEM_WAIT,	(unsigned int *)&ksem_wait,		SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_SEM_POST,	(unsigned int *)&ksem_post,		SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_SEM_TIMEDWAIT,	(unsigned int *)&ksem_timedwait,	SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_SEM_GET_COUNT,	(unsigned int *)&ksem_get_count,	SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_SEM_RESET,	(unsigned int *)&ksem_reset,		SYSCALL_PRIVILEGE_OPERATION},
#endif /* CONFIG_SEMAPHORE */
	{SYSCALL_TIME_USLEEP,	(unsigned int *)&ktime_usleep,		SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_TIME_ONESHOT,	(unsigned int *)&ktime_oneshot,		SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_TIME_ONESHOT_CANCEL, (unsigned int *)&ktime_oneshot_cancel, SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_TIME_GET_TICKS, (unsigned int *)&ktime_get_ticks,	SYSCALL_PRIVILEGE_OPERATION},
#ifdef CONFIG_QUEUE
	{SYSCALL_QUEUE_CREATE,	(unsigned int *)&kqueue_init,		SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_QUEUE_POST,	(unsigned int *)&kqueue_post,		SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_QUEUE_RECEIVE, (unsigned int *)&kqueue_receive,	SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_QUEUE_UPDATE,	(unsigned int *)&kqueue_update,		SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_QUEUE_DESTROY, (unsigned int *)&kqueue_destroy,	SYSCALL_PRIVILEGE_OPERATION},
#endif /* CONFIG_QUEUE */
	{SYSCALL_FD_OPEN,	(unsigned int *)&svc_open,		SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_FD_CLOSE,	(unsigned int *)&svc_close,		SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_FD_WRITE,	(unsigned int *)&svc_write,		SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_FD_READ,	(unsigned int *)&svc_read,		SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_FD_LSEEK,	(unsigned int *)&svc_lseek,		SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_FD_IOCTL,	(unsigned int *)&svc_ioctl,		SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_ALLOC,		(unsigned int *)umalloc,		SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_FREE,		(unsigned int *)ufree,			SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_PIO_EXPORT,	(unsigned int *)pio_export,		SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_PIO_SET_STATE, (unsigned int *)pio_set_state,		SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_PIO_GET_STATE, (unsigned int *)pio_get_state,		SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_PIO_REQUEST_IRQ, (unsigned int *)pio_request_interrupt, SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_PIO_ENABLE_IRQ, (unsigned int *)pio_enable_interrupt, SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_PIO_DISABLE_IRQ, (unsigned int *)pio_disable_interrupt, SYSCALL_PRIVILEGE_OPERATION},
	{SYSCALL_PRINT,		(unsigned int *)__printk,		SYSCALL_PRIVILEGE_ELEVATION},
	{SYSCALL_REBOOT,	(unsigned int *)arch_sys_reboot,	SYSCALL_PRIVILEGE_OPERATION},
};
