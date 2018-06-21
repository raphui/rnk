#ifndef VECTOR_H
#define VECTOR_H

struct isr_entry {
	void *arg;
	void (*isr)(void *);
};

unsigned int vector_current_irq(void);
void vector_set_isr_wrapper(void *wrapper);
int vector_set_isr_entry(struct isr_entry *entry, int irq);
struct isr_entry *vector_get_isr_entry(int irq);

#endif /* VECTOR_H */
