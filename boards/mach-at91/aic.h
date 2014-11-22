#ifndef AIC_H
#define AIC_H

#include <sam7s-reg.h>
#include <utils.h>

void aic_register_handler(unsigned int source, unsigned int mode, void (*handler)(void));
void aic_enable_it(unsigned int source);
void aic_disable_it(unsigned int source);
void aic_enable_debug(void);

#endif /* AIC_H */
