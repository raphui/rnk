#ifndef SVC_H
#define SVC_H

void svc_create_context(struct registers *_reg, unsigned int sp, unsigned int func, unsigned int end);
void svc_activate_context(struct registers *_reg);
void svc_switch_context(struct registers *_curr_reg, struct registers *_reg);
void svc_save_user_context(void);
void svc_get_user_context(void);

#endif /* SVC_H */
