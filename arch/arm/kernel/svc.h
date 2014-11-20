#ifndef SVC_H
#define SVC_H

void svc_create_context(unsigned int sp, unsigned int func, unsigned int end, unsigned int fp);
void svc_activate_context(struct registers *_reg);
void svc_switch_context(struct registers *_curr_reg, struct registers *_reg);

#endif /* SVC_H */
