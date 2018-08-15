#include <stdio.h>
#include <pthread.h>
#include <time.h>

static pthread_t thr_a;
static pthread_t thr_b;
static pthread_t thr_c;

static int token = 0;

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

	printf("B\n");
	time_usleep(10000);
	token = 1;
}

void thread_c(void *arg)
{
	int last_token;
	int i = 0;
	int ret = 0;
	printf("starting thread C\n");

	while (1) {
		printf("C\n");
		time_usleep(3000);

		last_token = token;

		printf("joining on thread B, should: %s\n", i ? "FAIL" : "SUCCEEDED");

		ret = pthread_join(&thr_b, NULL);
		if ((ret == 0) && (last_token != token))
			printf("pthread_join SUCCEEDED, %d-%d\n", last_token, token);
		else
			printf("pthread_join FAIL, error code: %d\n", ret);

		i++;
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
