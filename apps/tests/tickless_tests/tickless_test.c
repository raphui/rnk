#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>

#define msleep(ms) time_usleep((ms)*1000)

static int now_ms(void)
{
	return time_get_ticks();
}

void high_prio_task(void* arg)
{
	printf("[HP] Task started at %d ms\n", now_ms());
	printf("[HP] Sleeping for 500ms...\n");
	msleep(500);
	printf("[HP] Woke up at %d ms\n", now_ms());
}

void mid_prio_task(void* arg)
{
	printf("[MP] Task running at %d ms\n", now_ms());
	for (int i = 0; i < 10; i++) {
		printf("[MP] Iteration %d at %d ms\n", i, now_ms());
		msleep(100);
	}
	printf("[MP] Done at %d ms\n", now_ms());
}

void low_prio_idle(void* arg)
{
	printf("[LP] Idle thread running (should sleep often)\n");
	while (1) {
		printf("[LP] Sleeping at %d ms\n", now_ms());
		msleep(100);
	}
}

int main(void)
{
	pthread_t hp, mp, lp;

	// Create low-priority idle task
	pthread_create(&lp, low_prio_idle, NULL, 10);

	msleep(100); // Let LP settle

	// Create mid-priority task
	pthread_create(&mp, mid_prio_task, NULL, 11);

	msleep(100); // Let MP start

	// Create high-priority task delayed
	pthread_create(&hp, high_prio_task, NULL, 12);

	pthread_join(&hp, NULL);
	pthread_join(&mp, NULL);
	// LP runs forever

	return 0;
}

