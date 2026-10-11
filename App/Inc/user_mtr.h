
#ifndef USER_MTR_H
#define USER_MTR_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif
	
/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ                      150000000uL
#define TIM_CLOCK_DIVIDER                1
#define ADV_TIM_CLK_MHz                  150 /* Actual TIM clk including Timer clock divider*/
#define ADC_CLK_MHz                      42
#define HALL_TIM_CLK                     170000000uL
#define APB1TIM_FREQ                     170000000uL

/*************** Timer for PWM generation & currenst sensing parameters  ******/
/* PWM generation and current reading */
#define PWM_FREQUENCY                       16000
#define PWM_FREQ_SCALING                    1
//#define LOW_SIDE_SIGNALS_ENABLING           LS_PWM_TIMER
#define SW_DEADTIME_NS                      1000 /*!< Dead-time to be inserted by FW, only if low side signals are enabled */

#define PWM_PERIOD_CYCLES                   (uint16_t)(((uint32_t)ADV_TIM_CLK_MHz * (uint32_t)1000000u\
                                            / ((uint32_t)(PWM_FREQUENCY))) & (uint16_t)0xFFFE)

#define Half_PWMPeriod                      (uint16_t)(PWM_PERIOD_CYCLES/2u)

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE           1 /*!< FOC execution rate in number of PWM cycles */
#define ISR_FREQUENCY_HZ                    (PWM_FREQUENCY/REGULATION_EXECUTION_RATE) /*!< @brief FOC execution rate in Hz */

#define REP_COUNTER                         (uint16_t)((REGULATION_EXECUTION_RATE * 2u) - 1u)
#define SYS_TICK_FREQUENCY                  (uint16_t)2000

#define HTMIN 1 /* Required for main.c compilation only, CCR4 is overwritten at runtime */

#define DEADTIME_NS                         SW_DEADTIME_NS
#define DEAD_TIME_ADV_TIM_CLK_MHz           (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1                  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS / 1000uL)
#if (DEAD_TIME_COUNTS_1 <= 255)
#define DEAD_TIME_COUNTS                    (uint16_t)DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
#define DEAD_TIME_COUNTS                    (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
#define DEAD_TIME_COUNTS                    (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
#define DEAD_TIME_COUNTS                    (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS 510
#endif

#define LED_BLINK_FREQ_Hz           (0.5f)       // 1Hz

//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (16.0f)

// the number of pole pairs of the motor
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)

// the stator resistance value of the motor, in Ohm
#define USER_MOTOR1_Rs_Ohm                 (1.25f)

// the stator inductance value of the motor in the direct direction, in H
#define USER_MOTOR1_Ls_d_H                 (0.00156f)   /* HF scan, rotor held */

// the stator inductance value of the motor in the quadrature direction, in H
#define USER_MOTOR1_Ls_q_H                 (0.00239f)   /* HF scan, rotor held */

//! \brief Defines the controller execution period, usec
//!
#define USER_M1_CTRL_PERIOD_usec            (1000000.0f /(float32_t)PWM_FREQUENCY)

//! \brief Defines the controller execution period, sec
//!
#define USER_M1_CTRL_PERIOD_sec             ((float32_t)USER_M1_CTRL_PERIOD_usec / 1000000.0f)

//! \brief Defines the maximum Vs magnitude in per units allowed
//! \brief This value sets the maximum magnitude for the output of the Id and
//! \brief Iq PI current controllers. The Id and Iq current controller outputs
//! \brief are Vd and Vq. The relationship between Vs, Vd, and Vq is:
//! \brief Vs = sqrt(Vd^2 + Vq^2).  In this FOC controller, the Vd value is set
//! \brief equal to USER_MAX_VS_MAG*USER_VD_MAG_FACTOR.
//! \brief so the Vq value is set equal to sqrt(USER_MAX_VS_MAG^2 - Vd^2).
//!
//! \brief Set USER_MAX_VS_MAG = 0.5 for a pure sinewave with a peak at
//! \brief SQRT(3)/2 = 86.6% duty cycle.  No current reconstruction
//! \brief is needed for this scenario.
//!
//! \brief Set USER_MAX_VS_MAG = 1/SQRT(3) = 0.5774 for a pure sinewave
//! \brief with a peak at 100% duty cycle.  Current reconstruction
//! \brief will be needed for this scenario (Lab08).
//!
//! \brief Set USER_MAX_VS_MAG = 2/3 = 0.6666 to create a trapezoidal
//! \brief voltage waveform.  Current reconstruction will be needed
//! \brief for this scenario (Lab08).
//!
//! \brief For space vector over-modulation, see lab08 for details on
//! \brief system requirements that will allow the SVM generator to
//! \brief go all the way to trapezoidal.
//!
//#define USER_M1_MAX_VS_MAG_PU             (0.66f)
//#define USER_M1_MAX_VS_MAG_PU             (0.65f)
//#define USER_M1_MAX_VS_MAG_PU             (0.576f)
//#define USER_M1_MAX_VS_MAG_PU               (0.565f)
#define USER_M1_MAX_VS_MAG_PU             	(0.55f)

//! \brief Defines the direct voltage (Vd) scale factor
//!
#define USER_M1_VD_SF               (0.95f)

//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (36.3f)  //R37=100k/R45=10k x11 * 3.3V
//! \brief Defines the maximum current at the AD converter
#define USER_M1_ADC_FULL_SCALE_CURRENT_A      	 (20.0f)  //10mOhm*16.5 -> 3.3V/0.165=20A

//! \brief Defines the voltage scale factor for the system
//!
#define USER_M1_VOLTAGE_SF          (USER_M1_ADC_FULL_SCALE_VOLTAGE_V / 4096.0f)

//! \brief Defines the current scale factor for the system
//!
#define USER_M1_CURRENT_SF          (USER_M1_ADC_FULL_SCALE_CURRENT_A / 4096.0f)

//! \brief Defines the number of ISR clock ticks per speed controller clock tick
//!
#define USER_M1_NUM_ISR_TICKS_PER_SPEED_TICK        (10)

// the current for running the motor with force open-loop or startup, in A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.3f)           // A

// the current for motor rotor alignment, in A
#define USER_MOTOR1_ALIGN_CURRENT_A        (0.8f)           // A

// the current for start to run motor with closed-loop when the speed is
//  lower than the startup setting speed, in A
#define USER_MOTOR1_STARTUP_CURRENT_A      (0.9f)           // A
/* ^ I/F forcing current.  With 0.5 A the rotor could not follow the I/F ramp:
 * measured 0/10 successful starts at a 40 Hz target, the rotor only reaching
 * 2-18 Hz while the trajectory reached the 33 Hz hand-over, so the speed loop
 * saw a huge error, saturated and tripped the stall protection. */

// the current for running the motor with torque control mode when start the motor, in A.
#define USER_MOTOR1_TORQUE_CURRENT_A       (0.4f)           // A

// the maximum current value of the motor, in A
#define USER_MOTOR1_MAX_CURRENT_A          (1.0f)

// the maximum/base rotation frequency of the motor (Hz)
#define USER_MOTOR1_FREQ_MAX_Hz            (200.0f)         // Hz
// the speed threshold for start the motor, in Hz
#define USER_MOTOR1_SPEED_START_Hz         (8.0f)          // Hz
// the speed threshold for running the motor with force open-loop, in Hz
// Close-loop hand-over speed.  Measured low-speed limit of this eSMO on this
// motor: reliable >= ~25 Hz.  At 20 Hz the speed estimate wobbles (Iq ripple
// 0.18 A), at 15 Hz the loop cannot reach the setpoint, at 12 Hz it loses sync
// (fault 2, speed estimate flips to -44 Hz, Iq ripple 1.08 A).  8 Hz (the old
// value) put the hand-over inside that unreliable band and is the root cause
// of the earlier intermittent loss-of-lock trips.
// Trade-off: the I/F ramp must reach 25 Hz first (~17 s at ACCEL_START 1.5 Hz/s
// plus 2 s align).  Raise ACCEL_START for a faster start.
#define USER_MOTOR1_SPEED_FORCE_Hz         (33.0f)         // Hz

// the acceleration for start the motor, in Hz/s.
// Controls the I/F (forced) ramp rate up to USER_MOTOR1_SPEED_FORCE_Hz, so it
// directly sets the start-up time: 33 Hz / 4 Hz per s ~= 8 s (+2 s align).
#define USER_MOTOR1_ACCEL_START_Hzps       (2.0f)          // Hz/s

// the maximum acceleration for running the motor, in Hz/s
#define USER_MOTOR1_ACCEL_MAX_Hzps         (4.0f)          // Hz/s

//! \brief Defines maximum acceleration for the estimation speed profiles, Hz/sec
//!
#define USER_M1_MAX_ACCEL_Hzps              (2.0f)

// V/f Profile Parameters for open-loop in build level 2
// the low frequency f_low  of V/f profile, in Hz,
// set to 10% of rated motor frequency
#define USER_MOTOR1_FREQ_LOW_Hz            (5.0f)           // Hz

// the high frequency f_high of V/f profile, in Hz,
// set to 100% of rated motor frequency
#define USER_MOTOR1_FREQ_HIGH_Hz           (200.0f)         // Hz, 6000rpm

// the minimum voltage V_min  of V/f profile,
// the value is suggested to set to 15% of rated motor voltage, in Volt.
#define USER_MOTOR1_VOLT_MIN_V             (3.6f)           // Volt

// the maximum voltage,  V_max of V/f profile,
// the value is suggested to set to 100% of rated motor voltage, in Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0f)          // Volt

// Only for eSMO
// PLL (phase-locked loop)
// PID proportional, integral, derivative
// the sliding mode control maximum gain that equals to Ke*fmax/vscale/sqrt(2)*factor(max),
//  tune the factor(0.1~10) based the test status
#define USER_MOTOR1_KSLIDE_MAX             (0.50f)

// (KSLIDE_MIN removed: using a constant maximum sliding gain during bring-up,
//  the reference ramps it by 0.000002/ISR which takes ~12 s to reach max and is
//  far too slow for a fast speed ramp.)
#define USER_MOTOR1_KSLIDE_MIN             (0.50f)

/* Measured by the HF saliency scan with the rotor DC-held (tmp/id_test.py):
 *   Ld = 1.56 mH, Lq = 2.39 mH  -> saliency Lq/Ld = 1.53
 *   L(theta) is a smooth 180deg-periodic curve (verified over 0..150 deg)
 *   -> genuine saliency, HFI is feasible.
 *   Rs = 1.25 ohm (2.5 ohm line-to-line / 2)
 * Runtime-tunable copies are g_foc_rs_ohm / g_foc_ls_h. */
#define USER_MOTOR1_PLL_KP_MAX             (10.0f)

// the PLL control minimum gain that equals to 2*(Damping factor)*(Natural frequency)*factor(min),
// tune the factor(0.1~5) based the test status
#define USER_MOTOR1_PLL_KP_MIN             (1.50f)

// the PLL control gain adjusting coefficient that
// equals to (Kpll_max-Kpll_min)/fscale/fmax
#define USER_MOTOR1_PLL_KP_SF              (5.0f)

// the phase-locked loop control integration gain that
// equals to (Natural frequency)*(Natural frequency)*Ts
#define USER_MOTOR1_PLL_KI                 (2.8125E-06f)    // Not used, reserve

// the threshold of the estimated current error for sliding mode control that
// equals to (motor maximum BEMF voltage / rated voltage), (0.3~0.5 )
#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)

// the parameters of the low-pass filter for the estimated back EMF,
// Kslf equal to (fc*2*PI()*Ts), (0.5~2.5)
#define USER_MOTOR1_BEMF_KSLF_FC_SF        (2.0f)

// the offset coefficient to compensate the error by using
// the low-pass filter that equals to 1.0, or [0.5~1.5]
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)

// the cut-off frequency of the low-pass filter to calculate the estimated speed, (100~400)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

/* Runtime-tunable motor parameters. Defaults come from the macros above, but
 * they can be changed through the debugger and re-applied by restarting the
 * motor (MotorFOC_Init -> MOTOR_vars_Init reads them). Used for tuning Rs/Ls
 * without rebuilding. */
extern volatile float g_foc_rs_ohm;
extern volatile float g_foc_ls_h;

/* Runtime-tunable scale for the eSMO BEMF-LPF phase-lag compensation
 * (thetaOffset). The reference adds a term with slightly different units; the
 * correct factor is found empirically. 0 = no compensation. */
extern volatile float g_foc_thetaoff_sf;

/* eSMO PLL integral gain. The reference left this at 0 and instead relied on a
 * CORDIC phase compensation; enabling the integral term makes the PLL type-II
 * (no steady-state phase lag). Runtime tunable. */
extern volatile float g_foc_pll_ki;

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // end of USER_MTR_H definition
