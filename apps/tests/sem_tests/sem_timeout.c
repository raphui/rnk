#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>

#define SEM_TIMEOUT		10000
#define THREAD_B_TIMEOUT	11000

static pthread_t thr_a;
static pthread_t thr_b;
static sem_t sem;

void thread_a(void *arg)
{
	int ret;
	printf("starting thread A\n");

	while (1) {
		printf("[A] waiting on sem\n");
		ret = sem_timedwait(&sem, SEM_TIMEOUT);
		if (ret)
			printf("[A] timeout !\n");
		else
			printf("[A] wake up !\n");
	}
}

void thread_b(void *arg)
{
	int i = 0;
	printf("starting thread B\n");

	while (1) {
		if (i++ % 2) {
			printf("[B] NOT waking up thread A\n");
			time_usleep(THREAD_B_TIMEOUT);
		} else {
			printf("[B] waking up thread A\n");
			sem_post(&sem);
		}
	}
}

int main(void)
{
	printf("Starting mutex tests\n");

	sem_init(&sem, 1);

	printf("- adding thread A (%x)\n", &thread_a);
	pthread_create(&thr_a, &thread_a, NULL, 4);

	printf("- adding thread B(%x)\n", &thread_b);
	pthread_create(&thr_b, &thread_b, NULL, 3);

	pthread_resume(&thr_a);
	pthread_resume(&thr_b);
}
