#ifndef BACKTRACE_H
#define BACKTRACE_H

extern void unwind_backtrace(unsigned int fp, unsigned int sp, unsigned int lr, unsigned pc);
extern int unwind_init(void);

#endif /* BACKTRACE_H */
