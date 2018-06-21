#ifndef TIME_H
#define TIME_H

void time_usleep(unsigned int usec);
void time_oneshot(int delay, void (*handler)(void *), void *arg);

#endif /* TIME_H */
