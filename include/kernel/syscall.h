#ifndef SYSCALL_H
#define SYSCALL_H

#define SYSCALL_PRIVILEGE_OPERATION	0
#define SYSCALL_PRIVILEGE_ELEVATION	1

enum service_calls {
	SYSCALL_THREAD_SWITCH,
	SYSCALL_THREAD_CREATE,
	SYSCALL_THREAD_JOIN,
	SYSCALL_THREAD_STOP,
	SYSCALL_THREAD_SUSPEND,
	SYSCALL_THREAD_RESUME,
#ifdef CONFIG_MUTEX
	SYSCALL_MUTEX_CREATE,
	SYSCALL_MUTEX_ACQUIRE,
	SYSCALL_MUTEX_RELEASE,
#endif
#ifdef CONFIG_SEMAPHORE
	SYSCALL_SEM_CREATE,
	SYSCALL_SEM_WAIT,
	SYSCALL_SEM_POST,
	SYSCALL_SEM_TIMEDWAIT,
#endif
	SYSCALL_TIME_USLEEP,
	SYSCALL_TIME_ONESHOT,
#ifdef CONFIG_QUEUE
	SYSCALL_QUEUE_CREATE,
	SYSCALL_QUEUE_POST,
	SYSCALL_QUEUE_RECEIVE,
	SYSCALL_QUEUE_UPDATE,
	SYSCALL_QUEUE_DESTROY,
#endif
	SYSCALL_FD_OPEN,
	SYSCALL_FD_CLOSE,
	SYSCALL_FD_WRITE,
	SYSCALL_FD_READ,
	SYSCALL_FD_LSEEK,
	SYSCALL_ALLOC,
	SYSCALL_FREE,
	SYSCALL_PIO_EXPORT,
	SYSCALL_PIO_SET_STATE,
	SYSCALL_PRINT,
	SYSCALL_END,
};

struct syscall {
	int number;
	unsigned int *handler;
	unsigned char type;
};

extern struct syscall syscall_table[];

#endif /* SYSCALL_H */
