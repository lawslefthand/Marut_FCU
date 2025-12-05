#ifndef ACRO_H_
#define ACRO_H_

#include <stdint.h>

extern float Gx;
extern float Gy;
extern float Gz;
void mpu_gyro_read(void);

void acro_init(void);
void acro_run(float dt);        /* dt in seconds */

void acro_enable(void);
void acro_disable(void);
uint8_t acro_is_enabled(void);

void acro_reset_integrators(void);


void acro_set_gains(float kp_r, float ki_r, float kd_r,
                    float kp_p, float ki_p, float kd_p);


//void acro_task(void *argument);

#endif 
/* ACRO_H_ */
