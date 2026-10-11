#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include <stdint.h>

/* Sensorless FOC application layer for the 嘉立创 STM32G431 board.
 * Algorithm modules (user_vars.c / user_motor.c) are ported from the ALIENTEK
 * X-STAR-S eSMO reference; the hardware layer below is specific to this board:
 *   - TIM1 center aligned, 16 kHz, CH1/2/3 single ended (EG2133 makes the
 *     complementary pair + dead time), CH4 = ADC injected trigger.
 *   - ADC1 injected CH5 (U) + CH14 (V): 2-shunt reconstruction. W is left out
 *     because its shunt amplifier is dead on this board.
 *   - ADC2 CH15 (PB15) = DC bus, polled every 1 ms. */

#define FOC_ISR_HZ 16000U

extern volatile uint32_t g_foc_arm;         /* 1 = one start attempt armed   */
extern volatile int32_t  g_foc_command;     /* 1 fwd, -1 rev, 2 stop/clear   */
extern volatile float    g_foc_speed_ref_hz;
extern volatile float    g_foc_vbus_v;
extern volatile uint32_t g_foc_offset_ready;
extern volatile uint32_t g_foc_isr_count;
extern volatile uint32_t g_foc_main_alive;    /* bumped by the main loop */
extern volatile uint32_t g_foc_main_timeout;  /* main-loop watchdog trips */
extern volatile uint32_t g_foc_state;       /* MOTOR_Status_e                */
extern volatile uint16_t g_foc_ishunt_adc[3];
extern volatile float    g_foc_ishunt_a[3];
extern volatile float    g_foc_theta_rad;
extern volatile float    g_foc_speed_hz;
extern volatile float    g_foc_id_a;
extern volatile float    g_foc_iq_a;
extern volatile float    g_foc_vd_v;
extern volatile float    g_foc_vq_v;
extern volatile float    g_foc_duty[3];
extern volatile float    g_foc_align_deg;

/* ---- Field weakening ----------------------------------------------------
 * When the commanded voltage approaches VsMax, push id negative to reduce the
 * flux linkage and allow a higher speed (Morimoto 1990). Uses the previous
 * ISR's |V| as the feedback. g_foc_fw_gain is A/(V*ISR); g_foc_fw_max_a caps
 * |id|. Set g_foc_fw_enable=0 to disable. */
extern volatile uint32_t g_foc_fw_enable;
extern volatile float    g_foc_fw_gain;
extern volatile float    g_foc_fw_max_a;
extern volatile float    g_foc_fw_id_a;   /* debug: current id offset */

/* ---- Observer selection -------------------------------------------------
 * 0 = eSMO (default), 1 = stator-flux observer + PLL.
 * The flux observer integrates (v - R*i), so its signal does NOT vanish at
 * low speed the way the back-EMF does, which should extend the sensorless
 * range below the current ~8 Hz I/F hand-over point. */
extern volatile uint32_t g_foc_obs_mode;
extern volatile float    g_foc_flux_wc;    /* integrator LPF pole, rad/s */
extern volatile float    g_foc_flux_kp;    /* PLL gain */
extern volatile float    g_foc_flux_ki;
extern volatile float    g_foc_flux_psi;   /* rotor flux linkage, Wb */

/* ---- MTPA (maximum torque per ampere) -----------------------------------
 * For an IPM (Lq > Ld) a small negative id adds reluctance torque:
 *   id = A - sqrt(A^2 + iq^2),  A = psi_r / (2*(Lq-Ld))
 * For this motor psi_r is large and (Lq-Ld) small, so the benefit is tiny
 * (id ~ -12 mA at iq = 0.5 A) but the implementation is cheap. */
extern volatile uint32_t g_foc_mtpa_enable;
extern volatile float    g_foc_ld_h;
extern volatile float    g_foc_lq_h;
extern volatile float    g_foc_mtpa_id_a;  /* debug: current MTPA id */

/* ---- dead-time compensation ---------------------------------------------
 * The EG2133 inserts a dead time Td; while neither switch conducts the phase
 * current freewheels through a diode, so the applied phase voltage lags the
 * command by -sign(i)*(Td/Ts)*Vbus.  We add the opposite in the alpha-beta
 * frame.  td_us = 0 disables.  Tune by minimising g_foc_iq_pp at low speed. */
extern volatile float    g_foc_dtcomp_td_us;   /* dead time, us (0=off) */
extern volatile float    g_foc_dtcomp_ith_a;   /* sign threshold, A */
extern volatile float    g_foc_iq_pp;          /* Iq peak-to-peak, last 200ms */

/* Close-loop hand-over speed override. >0 replaces the compiled-in
 * MOTOR_vars.speedForce_Hz, which USER_setMotor1Params() re-writes on every
 * start (so a runtime poke at the struct field does not stick). */
extern volatile float    g_foc_speed_force_hz;

/* Open-loop (I/F) start overrides.  >0 replaces the compiled-in defaults;
 * these are re-written from USER_setMotor1Params() on every start, so a runtime
 * poke at the struct fields does not stick. */
extern volatile float    g_foc_accel_start_hzps;  /* I/F ramp rate, Hz/s */
extern volatile float    g_foc_start_current_a;   /* I/F forcing current, A */

/* Speed-loop feedback source.  0 = raw PLL speed (no added lag, better dynamic
 * response), 1 = low-pass filtered estimate (less noise, but the ~1 ms filter
 * adds phase lag and can make the loop under-damped). */
extern volatile uint32_t g_foc_spd_fb_filter;

/* Hand-over gate: the I/F -> closed-loop switch also requires the eSMO to have
 * converged, i.e. |thetaErr| (the observer's normalised angle error) below this.
 * A speed threshold alone switched the loop even while the rotor lagged the I/F
 * field; the speed loop then saw a huge error, saturated and tripped the stall
 * protection (measured 0/10 successful starts, then 4/4 and 1/3 -- inherently
 * flaky).  0 disables the gate. */
extern volatile float    g_foc_handover_errmax;

/* Run-time override for the commanded speed-change rate (Hz/s).  0 = use the
 * compiled default (USER_MOTOR1_ACCEL_MAX_Hzps).  Applies to both accelerating
 * and decelerating towards a new speed reference. */
extern volatile float    g_foc_accel_max_hzps;

/* ISR cycle budget measurement (DWT cycle counter). */
extern volatile uint32_t g_foc_isr_cycles;      /* last ISR */
extern volatile uint32_t g_foc_isr_cycles_max;  /* worst seen */
extern volatile uint32_t g_foc_isr_cycles_sum;  /* cumulative (for load) */

/* Overcurrent protection. Checks the peak phase current every ISR: an
 * instantaneous trip at g_foc_oc_inst_a, or a debounced trip at
 * g_foc_oc_trip_a sustained for FOC_OC_DEBOUNCE samples. Latched in
 * g_foc_fault (1 = overcurrent) until the next start / a stop command. */
extern volatile float    g_foc_oc_trip_a;
extern volatile float    g_foc_oc_inst_a;
extern volatile float    g_foc_imax_a;      /* debug: last peak phase current */
extern volatile uint32_t g_foc_fault;       /* 0 none, 1 overcurrent, 2 stall, 3 main-loop stall */
extern volatile int32_t  g_foc_stall_ms;    /* stall trip time, ms (0 = off) */
extern volatile float    g_foc_stop_decel_hzps; /* controlled-stop ramp, Hz/s */

/* ---- High-frequency injection (HFI) --------------------------------------
 * Pulsating injection on the estimated d-axis for zero/low-speed sensorless
 * operation. The q-axis HF current is demodulated to sin(2*dtheta) and a
 * tracking loop drives it to zero. Set g_foc_hfi_enable=1 before arming: the
 * FOC aligns the rotor, then switches to HFI tracking instead of I/F start.
 * g_foc_hfi_stage: 0 idle, 1 tracking. */
extern volatile uint32_t g_foc_hfi_enable;
extern volatile uint32_t g_foc_hfi_stage;
extern volatile float    g_foc_hfi_v;      /* injection amplitude, V */
extern volatile float    g_foc_hfi_hold_v; /* DC hold on axis 0 (diagnostic), V */
extern volatile float    g_foc_hfi_kp;     /* tracking gain */
extern volatile float    g_foc_hfi_kp_sign;/* +1/-1, set by polarity test */
extern volatile uint32_t g_foc_hfi_freeze; /* 1 = hold theta at theta0 (diagnostic) */
extern volatile float    g_foc_hfi_theta;  /* estimated electrical angle, rad */
extern volatile float    g_foc_hfi_theta0; /* initial angle (for convergence test) */
extern volatile float    g_foc_hfi_err;    /* demodulated error (debug) */
extern volatile float    g_foc_hfi_id;
extern volatile float    g_foc_hfi_iq;

/* Bench bring-up helpers: auto-stop after g_foc_run_ms and log the run so it
 * can be read back over SWD once the PWM has stopped (SWD is unusable while
 * the power stage switches). */
extern volatile int32_t  g_foc_run_ms;   /* 0 = run forever */
extern volatile uint32_t g_foc_log_idx;
extern volatile int16_t  g_foc_log[256][4]; /* state, speed*100, Iq*1000, Vs*10 */

/* ---- HF parameter identification (bench) ---------------------------------
 * Set g_foc_id_enable = 1 before arming. The FOC aligns the rotor at theta=0,
 * then injects a 1 kHz voltage along the d axis (stage 1) and the q axis
 * (stage 2) and records the current response, so Ld/Lq (and the saliency
 * ratio) can be computed offline. g_foc_id_stage: 0 idle, 1 injecting d,
 * 2 injecting q, 3 done. */
extern volatile uint32_t g_foc_id_enable;
extern volatile uint32_t g_foc_id_stage;
extern volatile uint32_t g_foc_id_cnt;
extern volatile float    g_foc_id_v;        /* injection amplitude, V */
extern volatile float    g_foc_id_hold_v;   /* DC holding voltage on the align axis */
extern volatile int32_t  g_foc_id_angle_deg;/* angle currently being measured */
extern volatile int16_t  g_foc_id_ampd[16]; /* |Id| HF amplitude*1000 per angle */
extern volatile int16_t  g_foc_id_ampq[16]; /* |Iq| HF amplitude*1000 per angle */
#define FOC_ID_NANGLES 6

void MotorFOC_Init(void);
void MotorFOC_Loop1ms(void);      /* housekeeping: bus read, command, LED */
void MotorFOC_TimerISR(void);     /* call from TIM1_UP_TIM16_IRQHandler     */
void MotorFOC_Brake(void);

#endif /* MOTOR_FOC_H */
