#include <pm/pm.h>
#include <init.h>

static struct list_node pm_notifiers;

void pm_notifier_register(struct pm_notifier *notifier)
{
	list_add_tail(&pm_notifiers, &notifier->node);
}

int pm_enter_state(int state)
{
	int ret = 0;
	struct pm_notifier *notifier = NULL;
	int (*callback)(int, void *);

	list_for_every_entry(&pm_notifiers, notifier, struct pm_notifier, node) {
		callback = notifier->state_entry;

		if (callback) {
			ret = callback(state, notifier->pdata);
			if (ret < 0)
				break;
		}
	}

	return ret;
}

int pm_exit_state(int state)
{
	int ret = 0;
	struct pm_notifier *notifier = NULL;
	int (*callback)(int, void *);


	list_for_every_entry(&pm_notifiers, notifier, struct pm_notifier, node) {
		callback = notifier->state_exit;

		if (callback) {
			ret = callback(state, notifier->pdata);
			if (ret < 0)
				break;
		}
	}

	return ret;
}

int pm_init(void)
{
	list_initialize(&pm_notifiers);

	return 0;
}
pure_initcall(pm_init);
