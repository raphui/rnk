#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>

static sem_t sem;

void thread_a(void *arg)
{
	printf("starting thread A\n");

	while (1) {
		printf("[A] waiting sem\n");
		sem_wait(&sem);
		printf("[A] posting sem\n");
		sem_post(&sem);
	}
}

void thread_b(void *arg)
{
	printf("starting thread B\n");

	while (1) {
		printf("[B] waiting sem\n");
		sem_wait(&sem);
		printf("[B] posting sem\n");
		sem_post(&sem);
	}
}

void thread_c(void *arg)
{
	printf("starting thread C\n");

	while (1) {
		printf("[C] waiting sem\n");
		sem_wait(&sem);
		printf("[C] posting sem\n");
		sem_post(&sem);
	}
}

int main(void)
{
	printf("Starting mutex tests\n");

	sem_init(&sem, 2);

	printf("- adding thread A (%x)\n", &thread_a);
	pthread_create(&thread_a, NULL, 4);

	printf("- adding thread B(%x)\n", &thread_b);
	pthread_create(&thread_b, NULL, 3);

	printf("- adding thread C(%x)\n", &thread_c);
	pthread_create(&thread_c, NULL, 2);
}
