#include <stdio.h>
#include <pthread.h>

static pthread_mutex_t mutex;

void thread_a(void *arg)
{
	printf("starting thread A\n");

	while (1) {
		printf("[A] locking mutex\n");
		pthread_mutex_lock(&mutex);
		printf("[A] unlocking mutex\n");
		pthread_mutex_unlock(&mutex);
	}
}

void thread_b(void *arg)
{
	printf("starting thread B\n");

	while (1) {
		printf("[B] locking mutex\n");
		pthread_mutex_lock(&mutex);
		printf("[B] unlocking mutex\n");
		pthread_mutex_unlock(&mutex);
	}
}

int main(void)
{
	printf("Starting mutex tests\n");

	pthread_mutex_init(&mutex);

	printf("- adding thread A (%x)\n", &thread_a);
	pthread_create(&thread_a, NULL, 10);

	printf("- adding thread B(%x)\n", &thread_b);
	pthread_create(&thread_b, NULL, 2);
}
