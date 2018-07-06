#include <armv7m/thread.h>
#include <kernel/syscall.h>

int arch_syscall_is_elevation(unsigned int svc_number)
{
	int ret = 0;
	unsigned char type;

	type = syscall_table[svc_number].type;

	if (type == SYSCALL_PRIVILEGE_ELEVATION) {
		arch_thread_switch_priv();
		ret = 1;
	}

	return ret;
}
