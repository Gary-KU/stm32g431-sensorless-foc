#ifndef FOC_MATH_H
#define FOC_MATH_H

/* Coordinate transforms for the sensorless FOC. Amplitude-invariant Clarke /
 * Park convention. All angles are electrical radians.
 *
 * The Park / inverse-Park rotations run on the CORDIC accelerator through
 * foc_hw. */

typedef struct {
  float alpha;
  float beta;
} FOC_AlBe;

typedef struct {
  float d;
  float q;
} FOC_DQ;

/* Two-shunt Clarke. ia and ib are measured; ic = -(ia + ib) is implied. */
void FOC_Clarke(float ia, float ib, FOC_AlBe *out);

/* Park / inverse Park rotation by angle_rad. */
void FOC_Park(const FOC_AlBe *in, float angle_rad, FOC_DQ *out);
void FOC_InvPark(const FOC_DQ *in, float angle_rad, FOC_AlBe *out);

/* SVPWM with common-mode injection. v is the (alpha, beta) reference and
 * vbus the measured DC link. Outputs are normalised duties clamped to
 * [FOC_DUTY_MIN, FOC_DUTY_MAX]. */
#define FOC_DUTY_MIN 0.0f
#define FOC_DUTY_MAX 1.0f
void FOC_SVPWM(const FOC_AlBe *v, float vbus, float *du, float *dv, float *dw);

#endif /* FOC_MATH_H */
