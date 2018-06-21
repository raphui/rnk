#include <stdio.h>
#include <pthread.h>
#include <time.h>

void thread_a(void *arg)
{
	printf("starting thread A\n");

	while (1) {
		printf("A\n");
		time_usleep(30000);
	}
}

void thread_b(void *arg)
{
	printf("starting thread B\n");

	while (1) {
		printf("B\n");
		time_usleep(10000);
	}
}

void thread_c(void *arg)
{
	printf("starting thread C\n");

	while (1) {
		printf("C\n");
	}
}

int main(void)
{
	printf("Starting thread tests\n");

	printf("- adding thread A (%x)\n", &thread_a);
	pthread_create(&thread_a, NULL, 4);

	printf("- adding thread B(%x)\n", &thread_b);
	pthread_create(&thread_b, NULL, 3);

	printf("- adding thread C(%x)\n", &thread_c);
	pthread_create(&thread_c, NULL, 2);

	return 0;
}
