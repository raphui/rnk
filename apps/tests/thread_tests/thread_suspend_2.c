#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>

static pthread_t thr_a;
static pthread_t thr_b;
static pthread_t thr_c;

void thread_a(void *arg)
{
	printf("starting thread A\n");

	while (1) {
		printf("A\n");

		pthread_suspend(&thr_a);
	}
}

void thread_b(void *arg)
{
	printf("starting thread B\n");

	while (1) {
		printf("B\n");

		pthread_suspend(&thr_b);
	}
}

void thread_c(void *arg)
{
	printf("starting thread C\n");

	while (1) {
		printf("C\n");

		pthread_resume(&thr_a);
		pthread_resume(&thr_b);
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

	return 0;
}
