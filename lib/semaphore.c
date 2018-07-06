#include <semaphore.h>
#include <kernel/syscall.h>
#include <unistd.h>
#include <export.h>

void sem_init(sem_t *sem, unsigned int value)
{
	syscall(SYSCALL_SEM_CREATE, &sem->ksem, value);
}
EXPORT_SYMBOL(sem_init);

void sem_wait(sem_t *sem)
{
	syscall(SYSCALL_SEM_WAIT, &sem->ksem);
}
EXPORT_SYMBOL(sem_wait);

void sem_post(sem_t *sem)
{
	syscall(SYSCALL_SEM_POST, &sem->ksem);
}
EXPORT_SYMBOL(sem_post);
