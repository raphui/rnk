#ifndef SVC_H
#define SVC_H

void svc_create_context(unsigned int sp, unsigned int func, unsigned int end, unsigned int fp);
void svc_activate_context(unsigned int sp);
void svc_switch_context(unsigned int sp);

#endif /* SVC_H */
