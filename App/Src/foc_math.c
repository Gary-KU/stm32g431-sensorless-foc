#include "foc_math.h"
#include "foc_hw.h"
#include <math.h>

#define SQRT3_OVER_2 0.8660254037844386f
#define ONE_OVER_SQRT3 0.5773502691896258f

void FOC_Clarke(float ia, float ib, FOC_AlBe *out)
{
  out->alpha = ia;
  out->beta = (ia + 2.0f * ib) * ONE_OVER_SQRT3;
}

void FOC_Park(const FOC_AlBe *in, float angle_rad, FOC_DQ *out)
{
  float s, c;
  FOC_SinCos(angle_rad, &s, &c);
  out->d = c * in->alpha + s * in->beta;
  out->q = -s * in->alpha + c * in->beta;
}

void FOC_InvPark(const FOC_DQ *in, float angle_rad, FOC_AlBe *out)
{
  float s, c;
  FOC_SinCos(angle_rad, &s, &c);
  out->alpha = c * in->d - s * in->q;
  out->beta = s * in->d + c * in->q;
}

void FOC_SVPWM(const FOC_AlBe *v, float vbus, float *du, float *dv, float *dw)
{
  float va, vb, vc, highest, lowest, common;
  if (!(vbus > 0.0f) || !isfinite(v->alpha) || !isfinite(v->beta)) {
    *du = *dv = *dw = 0.5f;
    return;
  }
  va = v->alpha;
  vb = -0.5f * v->alpha + SQRT3_OVER_2 * v->beta;
  vc = -0.5f * v->alpha - SQRT3_OVER_2 * v->beta;
  highest = fmaxf(va, fmaxf(vb, vc));
  lowest = fminf(va, fminf(vb, vc));
  common = -0.5f * (highest + lowest);
  *du = fminf(FOC_DUTY_MAX, fmaxf(FOC_DUTY_MIN, 0.5f + (va + common) / vbus));
  *dv = fminf(FOC_DUTY_MAX, fmaxf(FOC_DUTY_MIN, 0.5f + (vb + common) / vbus));
  *dw = fminf(FOC_DUTY_MAX, fmaxf(FOC_DUTY_MIN, 0.5f + (vc + common) / vbus));
}
