#include <semaphore.h>
#include <kernel/syscall.h>
#include <unistd.h>
#include <export.h>

int sem_init(sem_t *sem, unsigned int value)
{
	return syscall(SYSCALL_SEM_CREATE, &sem->ksem, value);
}
EXPORT_SYMBOL(sem_init);

int sem_wait(sem_t *sem)
{
	return syscall(SYSCALL_SEM_WAIT, &sem->ksem);
}
EXPORT_SYMBOL(sem_wait);


int sem_timedwait(sem_t *sem, int timeout)
{
	return syscall(SYSCALL_SEM_TIMEDWAIT, &sem->ksem, timeout);
}
EXPORT_SYMBOL(sem_timedwait);

int sem_post(sem_t *sem)
{
	return syscall(SYSCALL_SEM_POST, &sem->ksem);
}
EXPORT_SYMBOL(sem_post);

int sem_get_count(sem_t *sem)
{
	return syscall(SYSCALL_SEM_GET_COUNT, &sem->ksem);
}
EXPORT_SYMBOL(sem_get_count);

int sem_reset(sem_t *sem)
{
	return syscall(SYSCALL_SEM_RESET, &sem->ksem);
}
EXPORT_SYMBOL(sem_reset);
