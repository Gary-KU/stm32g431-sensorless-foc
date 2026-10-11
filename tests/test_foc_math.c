#include "foc_math.h"
#include "foc_hw.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846f

static void test_clarke_park_roundtrip(void)
{
  int step;
  for (step = -720; step < 720; ++step) {
    float angle = (float)step * PI / 360.0f;
    float ia, ib, ic, alpha_ref, beta_ref;
    FOC_AlBe ab;
    FOC_DQ dq;
    FOC_AlBe back;
    /* A balanced three-phase set with amplitude 1. */
    ia = cosf(angle);
    ib = cosf(angle - 2.0f * PI / 3.0f);
    ic = cosf(angle + 2.0f * PI / 3.0f);
    assert(fabsf(ia + ib + ic) < 1e-5f);
    /* Two-shunt Clarke must match the full Clarke. */
    FOC_Clarke(ia, ib, &ab);
    alpha_ref = ia;
    beta_ref = (ia + 2.0f * ib) / sqrtf(3.0f);
    assert(fabsf(ab.alpha - alpha_ref) < 1e-5f);
    assert(fabsf(ab.beta - beta_ref) < 1e-5f);
    /* With the rotor aligned to the field, Park must give (d=1, q=0). */
    FOC_Park(&ab, angle, &dq);
    assert(fabsf(dq.d - 1.0f) < 1e-4f);
    assert(fabsf(dq.q) < 1e-4f);
    /* Inverse Park must undo the rotation exactly. */
    FOC_InvPark(&dq, angle, &back);
    assert(fabsf(back.alpha - ab.alpha) < 1e-5f);
    assert(fabsf(back.beta - ab.beta) < 1e-5f);
  }
}

static void test_svpwm(void)
{
  int step;
  const float bus = 16.0f;
  for (step = -360; step < 360; ++step) {
    float angle = (float)step * PI / 180.0f;
    float u, v, w, vu, vv, vw;
    FOC_AlBe ref;
    ref.alpha = -0.8f * sinf(angle);
    ref.beta = 0.8f * cosf(angle);
    FOC_SVPWM(&ref, bus, &u, &v, &w);
    assert(u >= 0.0f && u <= 1.0f && v >= 0.0f && v <= 1.0f && w >= 0.0f && w <= 1.0f);
    /* Reconstructed phase voltages must match the reference (common mode
     * cancels in the line-to-line differences). */
    vu = (u - 0.5f) * bus;
    vv = (v - 0.5f) * bus;
    vw = (w - 0.5f) * bus;
    assert(fabsf((vu - vv) - (ref.alpha - (-0.5f * ref.alpha + 0.8660254f * ref.beta))) < 1e-3f);
    assert(fabsf((vv - vw) - ((-0.5f * ref.alpha + 0.8660254f * ref.beta) -
                              (-0.5f * ref.alpha - 0.8660254f * ref.beta))) < 1e-3f);
  }
}

static void test_atan2(void)
{
  assert(fabsf(FOC_Atan2(0.0f, 1.0f) - 0.0f) < 1e-4f);
  assert(fabsf(FOC_Atan2(1.0f, 1.0f) - PI / 4.0f) < 1e-4f);
  assert(fabsf(FOC_Atan2(1.0f, 0.0f) - PI / 2.0f) < 1e-4f);
  assert(fabsf(FOC_Atan2(1.0f, -1.0f) - 3.0f * PI / 4.0f) < 1e-4f);
  assert(fabsf(FOC_Atan2(-1.0f, -1.0f) + 3.0f * PI / 4.0f) < 1e-4f);
}

int main(void)
{
  test_clarke_park_roundtrip();
  test_svpwm();
  test_atan2();
  puts("PASS: Clarke/Park/invPark roundtrip, SVPWM line voltages, atan2");
  return 0;
}
