#include <arch/system.h>
#include <trace.h>
#include <utils.h>
#include <SEGGER_SYSVIEW.h>

void trace_enable(void)
{
	int v;

	v = readl(SCS_DEMCR);
	v |= SCS_DEMCR_TRCENA;

	writel(SCS_DEMCR, v);

	if (!(DWT_CTRL & DWT_CTRL_NOCYCCNT)) {
		writel(DWT_CYCCNT, 0);

		v = readl(DWT_CTRL);
		v |= DWT_CTRL_CYCCNTENA;
		writel(DWT_CTRL, v);
	}

	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();
}
