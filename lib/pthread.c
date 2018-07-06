#include <pthread.h>
#include <kernel/syscall.h>
#include <unistd.h>
#include <export.h>

int pthread_create(void (*start_routine)(void *), void *arg, unsigned int priority)
{
	return syscall(SYSCALL_THREAD_CREATE, start_routine, arg, priority);
}
EXPORT_SYMBOL(pthread_create);

int pthread_mutex_init(pthread_mutex_t *mutex)
{
	return syscall(SYSCALL_MUTEX_CREATE, &mutex->kmutex);
}
EXPORT_SYMBOL(pthread_mutex_init);

int pthread_mutex_lock(pthread_mutex_t *mutex)
{
	return syscall(SYSCALL_MUTEX_ACQUIRE, &mutex->kmutex);
}
EXPORT_SYMBOL(pthread_mutex_lock);

int pthread_mutex_unlock(pthread_mutex_t *mutex)
{
	return syscall(SYSCALL_MUTEX_RELEASE, &mutex->kmutex);
}
EXPORT_SYMBOL(pthread_mutex_unlock);
