#ifndef PM_H
#define PM_H

#include <list.h>

struct pm_notifier {
	int (*state_entry)(int state, void *pdata);
	int (*state_exit)(int state, void *pdata);
	void *pdata;
	struct list_node node;
};

enum {
	POWER_STATE_IDLE,
	POWER_STATE_SLEEP,
	POWER_STATE_DEEPSLEEP,
};

void pm_notifier_register(struct pm_notifier *notifier);
int pm_enter_state(int state);
int pm_exit_state(int state);

#endif /* PM_H */
