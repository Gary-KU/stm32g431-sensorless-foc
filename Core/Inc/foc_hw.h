#ifndef FOC_HW_H
#define FOC_HW_H

#include <stdint.h>

/* Hardware-accelerator layer for the sensorless FOC.
 *
 * On target the trig helpers run on the STM32G4 CORDIC peripheral; on a PC
 * build (host unit tests) they fall back to libm so the FOC math stays
 * testable without hardware. */

void FOC_Hw_Init(void);

/* angle is in radians; results are sin/cos of that angle. */
float FOC_Sin(float angle_rad);
float FOC_Cos(float angle_rad);
void  FOC_SinCos(float angle_rad, float *sin_out, float *cos_out);

/* atan2(y, x) in radians, range (-pi, pi]. */
float FOC_Atan2(float y, float x);

/* Diagnostics: FOC_Hw_SelfTest() fills this array so the CORDIC path can be
 * checked on real silicon through the debugger. */
extern volatile float g_foc_hw_selftest[8];
void FOC_Hw_SelfTest(void);

#endif /* FOC_HW_H */
