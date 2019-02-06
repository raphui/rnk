#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>

static pthread_t thr_a;
static pthread_t thr_b;
static pthread_t thr_c;

static sem_t sem;

static int is_suspend;

void thread_a(void *arg)
{
	printf("starting thread A\n");

	while (1) {
		printf("A\n");

		printf("wait on sem\n");
		sem_wait(&sem);

		if (is_suspend == 1)
			printf("TEST FAIL ! Thread A was supposed to be suspend !\n");

		is_suspend = 0;
	}
}

void thread_b(void *arg)
{
	printf("starting thread B\n");

	while (1) {
		printf("B\n");

		printf("suspend thread A\n");
		pthread_suspend(&thr_a);
		is_suspend = 1;

		time_usleep(3000000);
	}
}

void thread_c(void *arg)
{
	printf("starting thread C\n");

	while (1) {
		printf("C\n");

		printf("signal thread A\n");

		sem_post(&sem);

		if (is_suspend == 1)
			printf("TEST SUCCESS ! Thread A has not been wakeup !\n");

		is_suspend = 0;
		pthread_resume(&thr_a);
		time_usleep(4000000);
	}
}

int main(void)
{
	printf("Starting thread tests\n");

	printf("- adding thread A (%x)\n", &thread_a);
	pthread_create(&thr_a, &thread_a, NULL, 4);

	printf("- adding thread B(%x)\n", &thread_b);
	pthread_create(&thr_b, &thread_b, NULL, 3);

	printf("- adding thread C(%x)\n", &thread_c);
	pthread_create(&thr_c, &thread_c, NULL, 2);

	sem_init(&sem, 1);

	return 0;
}
