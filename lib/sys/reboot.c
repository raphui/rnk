#include <unistd.h>
#include <sys/reboot.h>
#include <kernel/syscall.h>

void __attribute__((noreturn)) sys_reboot(void)
{
	syscall(SYSCALL_REBOOT);

	while (1)
		;
}
