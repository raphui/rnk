#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>

static sem_t sem;

void thread_a(void *arg)
{
	int times_wakeup = 0;

	printf("starting thread A\n");

	while (1) {
		printf("[A] waiting on sem\n");
		sem_wait(&sem);

		if (!(++times_wakeup % 2)) {
			printf("[A] waking up thread B\n");
			sem_post(&sem);

			printf("[A] should wakup after waiting again...\n");
		}
	}
}

void thread_b(void *arg)
{
	printf("starting thread B\n");

	while (1) {
		printf("[B] waking up thread A\n");
		sem_post(&sem);

		printf("[B] waiting on sem\n");
		sem_wait(&sem);
	}
}

void thread_c(void *arg)
{
	printf("starting thread C\n");

	while (1) {
		printf("[C] waking up thread A\n");
		sem_post(&sem);
	}
}

int main(void)
{
	printf("Starting mutex tests\n");

	sem_init(&sem, 1);

	printf("- adding thread A (%x)\n", &thread_a);
	pthread_create(&thread_a, NULL, 4);

	printf("- adding thread B(%x)\n", &thread_b);
	pthread_create(&thread_b, NULL, 3);

	printf("- adding thread C(%x)\n", &thread_c);
	pthread_create(&thread_c, NULL, 2);
}
