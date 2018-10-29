#include <stdio.h>
#include <pthread.h>
#include <time.h>

#define THREAD_A_SLEEP	30000
#define THREAD_B_SLEEP	100000

static pthread_t thr_a;
static pthread_t thr_b;
static pthread_mutex_t mutex;
static int last_thread;
static int thread_a_locked;

void thread_a(void *arg)
{
	int busy;

	printf("starting thread A\n");

	while (1) {
		printf("[A] sleeping %d us...\n", THREAD_A_SLEEP);

		time_usleep(THREAD_A_SLEEP);

		thread_a_locked = 1;

		printf("[A] locking mutex, should fail... (thread C hold it)\n");
		pthread_mutex_lock(&mutex);

		last_thread = 1;
		thread_a_locked = 0;

		busy = 0xFF;
		while (busy--)
			printf("A");

		printf("[A] unlocking mutex\n");
		pthread_mutex_unlock(&mutex);
	}
}

void thread_b(void *arg)
{
	printf("starting thread B\n");

	while (1) {
		printf("[B] sleeping %d us...\n", THREAD_B_SLEEP);

		time_usleep(THREAD_B_SLEEP);

		if (last_thread == 2 && thread_a_locked)
			printf("[B] priority inversion ! Thread C has been preempted while thread A still locked on mutex\n");
		else
			printf("[B] no priority inversion\n");

	}
}

void thread_c(void *arg)
{
	int busy;

	printf("starting thread C\n");

	while (1) {
		printf("[C] locking mutex\n");
		pthread_mutex_lock(&mutex);

		last_thread = 2;

		busy = 0xFF;
		while (busy--)
			printf("C");

		printf("[C] unlocking mutex\n");
		pthread_mutex_unlock(&mutex);
	}
}

int main(void)
{
	printf("Starting mutex tests\n");

	pthread_mutex_init(&mutex);

	printf("- adding thread A (%x)\n", &thread_a);
	pthread_create(&thr_a, &thread_a, NULL, 10);

	printf("- adding thread B(%x)\n", &thread_b);
	pthread_create(&thr_b, &thread_b, NULL, 5);

	printf("- adding thread C(%x)\n", &thread_c);
	pthread_create(&thr_b, &thread_c, NULL, 2);
}
