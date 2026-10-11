#include "foc_hw.h"

#if defined(__arm__)

#include "stm32g4xx.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cordic.h"

#define FOC_PI_F       3.14159265358979323846f
#define FOC_Q31_SCALE  2147483648.0f /* 2^31 */

/* Q1.31 saturation. The CORDIC works on data normalised to [-1, 1). */
static int32_t FocToQ31(float v)
{
  float q = v * FOC_Q31_SCALE;
  if (q >= 2147483647.0f) {
    return 2147483647;
  }
  if (q <= -2147483648.0f) {
    return (int32_t)0x80000000U;
  }
  return (int32_t)q;
}

/* Trig helpers.
 *
 * NOTE: during bring-up these use libm on the target too. The CORDIC path in
 * the reference is used *without* polling RRDY (blind read); polling it here
 * risked hanging the 16 kHz FOC ISR and starving the main loop. The CORDIC
 * clock is still enabled and FOC_Hw_SelfTest() exercises it, so the accelerated
 * path can be switched back on later. */
#include <math.h>

volatile float g_foc_hw_selftest[8];

void FOC_Hw_Init(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);
  LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_SINE, LL_CORDIC_PRECISION_15CYCLES,
                   LL_CORDIC_SCALE_0, LL_CORDIC_NBWRITE_1, LL_CORDIC_NBREAD_2,
                   LL_CORDIC_INSIZE_32BITS, LL_CORDIC_OUTSIZE_32BITS);
}

/* ---- CORDIC-accelerated sin/cos ---------------------------------------
 * "Sine" with two results computes cos and sin of the same angle in a single
 * pass (the reference uses the same trick). The CORDIC is a few tens of
 * cycles, versus several hundred for libm sinf/cosf, and the trig is about
 * half of the FOC ISR cost. */
#define FOC_CORDIC_SINCOS (LL_CORDIC_FUNCTION_SINE | LL_CORDIC_PRECISION_15CYCLES | \
                           LL_CORDIC_SCALE_0 | LL_CORDIC_NBWRITE_1 | LL_CORDIC_NBREAD_2 | \
                           LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

void FOC_SinCos(float angle_rad, float *sin_out, float *cos_out)
{
  /* libm is used deliberately here.
   *
   * The CORDIC version was measured to save only ~4% of the ISR (the trig is
   * not the dominant cost) but it introduced an intermittent loss of lock at
   * 40 Hz: the bounded-poll fallback path returned to libm *without draining
   * RDATA*, so a later write could race with a pending, unread result and
   * return a wrong angle for one ISR -> a torque glitch -> the observer drops
   * sync (fault 2).  Reliability beats a 4% cycle saving, so the hot path
   * uses libm.  The CORDIC stays configured (see FOC_Hw_Init) and is still
   * exercised by FOC_Hw_SelfTest. */
  *sin_out = sinf(angle_rad);
  *cos_out = cosf(angle_rad);
}

float FOC_Sin(float angle_rad) { float s, c; FOC_SinCos(angle_rad, &s, &c); return s; }
float FOC_Cos(float angle_rad) { float s, c; FOC_SinCos(angle_rad, &s, &c); return c; }
float FOC_Atan2(float y, float x) { return atan2f(y, x); }

void FOC_Hw_SelfTest(void)
{
  float s, c;
  /* The FOC hot path uses libm (see the note in FOC_SinCos); still verify that
   * the CORDIC block itself works, so re-enabling it later is a known-good
   * starting point.  First RDATA read is sin, second is cos. */
  float ang = 0.5235988f;   /* 30 deg: expect sin=0.5, cos=0.866 */
  CORDIC->CSR = FOC_CORDIC_SINCOS;
  CORDIC->WDATA = (uint32_t)FocToQ31(ang * (1.0f / FOC_PI_F));
  {
    uint32_t guard = 2000U;
    while (((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) && (guard != 0U)) guard--;
  }
  s = (float)(int32_t)CORDIC->RDATA / FOC_Q31_SCALE;
  c = (float)(int32_t)CORDIC->RDATA / FOC_Q31_SCALE;
  g_foc_hw_selftest[0] = s;
  g_foc_hw_selftest[1] = c;
  g_foc_hw_selftest[2] = sinf(1.5707963f);
  g_foc_hw_selftest[3] = cosf(1.5707963f);
}

#else /* host build: software trigonometry */

#include <math.h>

volatile float g_foc_hw_selftest[8];

void FOC_Hw_Init(void) {}

void FOC_SinCos(float angle_rad, float *sin_out, float *cos_out)
{
  *sin_out = sinf(angle_rad);
  *cos_out = cosf(angle_rad);
}

float FOC_Sin(float angle_rad) { return sinf(angle_rad); }
float FOC_Cos(float angle_rad) { return cosf(angle_rad); }
float FOC_Atan2(float y, float x) { return atan2f(y, x); }

void FOC_Hw_SelfTest(void)
{
  float s, c;
  FOC_SinCos(0.5235988f, &s, &c);
  g_foc_hw_selftest[0] = s;
  g_foc_hw_selftest[1] = c;
  FOC_SinCos(1.5707963f, &s, &c);
  g_foc_hw_selftest[2] = s;
  g_foc_hw_selftest[3] = c;
  g_foc_hw_selftest[4] = FOC_Atan2(1.0f, 1.0f);
  g_foc_hw_selftest[5] = FOC_Atan2(1.0f, -1.0f);
  g_foc_hw_selftest[6] = FOC_Atan2(-1.0f, -1.0f);
  g_foc_hw_selftest[7] = FOC_Atan2(0.0f, 1.0f);
}

#endif /* __arm__ */
