#ifndef MPU_H
#define MPU_H

int mpu_init(void);
int mpu_map_from_low(void *base, int size, int attr);
int mpu_map_from_high(void *base, int size, int attr);
int mpu_unmap(void *base, int size);
int mpu_unmap_prio(int prio);

#endif /* MPU_H */
