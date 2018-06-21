#include <stdio.h>
#include <pthread.h>
#include <mqueue.h>
#include <unistd.h>

static mqd_t mqueue_rw;

static struct mq_attr attr = {
	.mq_maxmsg = 2,
	.mq_msgsize = 4,
};

static struct mq_attr attr_alt = {
	.mq_maxmsg = 5,
	.mq_msgsize = 4,
};

void thread_a(void *arg)
{
	int a;

	printf("starting thread A\n");

	while (1) {
		printf("[A] receiving from queue: ");
		mq_receive(mqueue_rw, (char *)&a, sizeof(int), 0);
		printf("%d\n", a);
	}
}

void thread_b(void *arg)
{
	int b = 1;

	printf("starting thread B\n");

	while (1) {
		printf("[B] posting from queue: ");
		mq_send(mqueue_rw, (const char *)&b, sizeof(int), 0);
		printf("%d\n", b);
		b++;
	}
}

int main(void)
{
	int ret = 0;
	struct mq_attr attr_ret;

	printf("Starting queue tests\n");

	mqueue_rw = mq_open("rw", O_RDWR, 0, &attr);
	printf("# mqueue creation: %s\n", (mqueue_rw < 0) ? "KO" : "OK");
	if (mqueue_rw < 0)
		goto out;

	ret = mq_getattr(mqueue_rw, &attr_ret);
	printf("# mqueue getattr: %s\n", (ret < 0) ? "KO" : "OK");
	if (ret < 0 || attr_ret.mq_maxmsg != attr.mq_maxmsg)
		goto out;


	ret = mq_setattr(mqueue_rw, &attr_alt, &attr_ret);
	printf("# mqueue setattr: %s\n", (ret < 0) ? "KO" : "OK");
	if (ret < 0 || attr_ret.mq_maxmsg != attr.mq_maxmsg) {
		printf("\t\t%d, %d, %d\n", ret, attr_ret.mq_maxmsg, attr.mq_maxmsg);
		goto out;
	}

	ret = mq_getattr(mqueue_rw, &attr_ret);
	printf("# mqueue getattr (alt): %s\n", (ret < 0) ? "KO" : "OK");
	if (ret < 0 || attr_ret.mq_maxmsg != attr_alt.mq_maxmsg)
		goto out;

	printf("- adding thread A (%x)\n", &thread_a);
	pthread_create(&thread_a, NULL, 4);

	printf("- adding thread B(%x)\n", &thread_b);
	pthread_create(&thread_b, NULL, 3);

out:
	return 0;
}
