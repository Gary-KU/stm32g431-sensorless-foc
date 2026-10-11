#include "main.h"
#include "user_vars.h"
#include "user_motor.h"
#include "user_mtr.h"
#include "user_foc.h"
#include "foc_hw.h"
#include "foc_math.h"
#include "motor_foc.h"
#include "stm32g4xx_ll_adc.h"
#include <math.h>

/* ---- board ADC scaling -------------------------------------------------- */
/* Low-side shunt 10 mOhm, external diff amp gain 33k/2k = 16.5, 3.3 V ref:
 * full-scale current = 3.3 / (0.010 * 16.5) = 20 A over 4096 counts. */
#define FOC_CURRENT_FULL_SCALE_A 20.0f
/* Bus divider R37=100k / R45=10k on PB15: full scale = 3.3 * 11 = 36.3 V. */
#define FOC_VBUS_FULL_SCALE_V    36.3f

#define FOC_CURRENT_SF  (FOC_CURRENT_FULL_SCALE_A / 4096.0f)
#define FOC_VBUS_SF     (FOC_VBUS_FULL_SCALE_V / 4096.0f)

/* TIM1 center-aligned PWM. ARR = f_tim / (2 * f_pwm). RCR=1 -> update once per
 * full PWM period, so the ISR runs at exactly PWM_FREQUENCY. */
#define FOC_TIM_CLK_HZ   150000000UL
#define FOC_TIM_ARR      ((uint16_t)(FOC_TIM_CLK_HZ / (2UL * PWM_FREQUENCY)))
#define FOC_ADC_TRIG_CMP ((uint16_t)(FOC_TIM_ARR - 5U)) /* near the top: all low sides on */

#define FOC_ADC1_INJ_U   ADC_CHANNEL_5    /* PB14 */
#define FOC_ADC1_INJ_V   ADC_CHANNEL_14   /* PB11 */
#define FOC_ADC2_BUS_CH  ADC_CHANNEL_15   /* PB15 */
/* Plain channel numbers for the JSQR sequencer fields (the HAL ADC_CHANNEL_x
 * macros are composite values and cannot be shifted into JSQx directly). */
#define FOC_ADC1_INJ_U_NUM 5U
#define FOC_ADC1_INJ_V_NUM 14U

HAL_ADCData_t AdcData;
HAL_PWMData_t PwmData;

volatile uint32_t g_foc_arm = 0;
volatile int32_t  g_foc_command = 0;
volatile float    g_foc_speed_ref_hz = 0.0f;
volatile float    g_foc_vbus_v = 0.0f;
volatile uint32_t g_foc_offset_ready = 0;
volatile uint32_t g_foc_isr_count = 0;
volatile uint32_t g_foc_state = 0;
volatile uint16_t g_foc_ishunt_adc[3];
volatile float    g_foc_ishunt_a[3];
volatile float    g_foc_theta_rad = 0.0f;
volatile float    g_foc_speed_hz = 0.0f;
volatile float    g_foc_id_a = 0.0f;
volatile float    g_foc_iq_a = 0.0f;
volatile float    g_foc_vd_v = 0.0f;
volatile float    g_foc_vq_v = 0.0f;
volatile float    g_foc_duty[3] = {0.5f, 0.5f, 0.5f};
volatile uint32_t g_foc_jsqr_shadow = 0;
volatile uint32_t g_foc_jsqr_after_start = 0;
volatile uint32_t g_foc_adc_cr = 0;
volatile float    g_foc_align_deg = 0.0f;
/* Field weakening is OFF by default.
 *
 * Measured at 100 Hz on a 20 V bus: with the FW loop active the speed has a
 * 4.1 % standard deviation with a dominant 4 Hz oscillation; with the FW
 * disabled it is 1.4 % and the machine still reaches ~100 Hz, because the
 * voltage-limit base speed is already ~105 Hz -- the FW only buys a few percent
 * of extra speed.  The crude "integrator on the voltage error" loop is not
 * damped well enough to be worth that.  Enable with `P fw 1` when the extra top
 * speed under load is needed, and expect the ripple back. */
volatile uint32_t g_foc_fw_enable = 0U;
volatile float    g_foc_fw_gain = 0.0002f;
volatile float    g_foc_fw_max_a = 0.35f;
volatile float    g_foc_fw_id_a = 0.0f;
static float      fw_int = 0.0f;

/* flux observer (mode 1) */
volatile uint32_t g_foc_obs_mode = 0U;
volatile float    g_foc_flux_wc = 30.0f;
volatile float    g_foc_flux_kp = 4.0f;
volatile float    g_foc_flux_ki = 0.02f;
volatile float    g_foc_flux_psi = 0.0175f;   /* rotor flux linkage, Wb */
volatile uint32_t g_foc_mtpa_enable = 1U;
volatile float    g_foc_ld_h = 0.00156f;
volatile float    g_foc_lq_h = 0.00239f;
volatile float    g_foc_mtpa_id_a = 0.0f;
volatile float    g_foc_dtcomp_td_us = 0.0f;
volatile float    g_foc_dtcomp_ith_a = 0.05f;
volatile float    g_foc_iq_pp = 0.0f;
volatile float    g_foc_speed_force_hz = 0.0f;   /* 0 = use compiled default */
volatile float    g_foc_accel_start_hzps = 0.0f; /* 0 = use compiled default */
volatile float    g_foc_start_current_a = 0.0f;  /* 0 = use compiled default */
volatile uint32_t g_foc_spd_fb_filter = 0U;      /* 0 = raw, 1 = filtered */
volatile float    g_foc_handover_errmax = 0.30f; /* |thetaErr| gate for hand-over */
static float      flux_a = 0.0f, flux_b = 0.0f, flux_ui = 0.0f, flux_theta = 0.0f;
volatile float    g_foc_oc_trip_a = 2.5f;   /* debounced trip level, A */
volatile float    g_foc_oc_inst_a = 4.0f;   /* instantaneous trip level, A */
volatile float    g_foc_imax_a = 0.0f;
volatile uint32_t g_foc_fault = 0U;
volatile int32_t  g_foc_stall_ms = 400;
volatile float    g_foc_stop_decel_hzps = 30.0f;
volatile float    g_foc_accel_max_hzps = 0.0f;   /* 0 = use compiled default */
static uint32_t stall_cnt = 0;
static uint8_t  stop_req = 0;
static uint32_t stop_start_ms = 0;
#define FOC_OC_DEBOUNCE 16U                 /* 1 ms at 16 kHz */
static uint32_t oc_cnt = 0;
static uint32_t hfi_sweep_ms = 0;

/* HFI state */
volatile uint32_t g_foc_hfi_enable = 0;
volatile uint32_t g_foc_hfi_stage = 0;
volatile float    g_foc_hfi_v = 2.0f;
volatile float    g_foc_hfi_hold_v = 0.0f;   /* set ~0.63 for the diagnostic */
volatile float    g_foc_hfi_kp = 0.35f;
volatile float    g_foc_hfi_kp_sign = 1.0f;
volatile uint32_t g_foc_hfi_freeze = 0U;
volatile float    g_foc_hfi_theta = 0.0f;
volatile float    g_foc_hfi_theta0 = 0.0f;
volatile float    g_foc_hfi_err = 0.0f;
volatile float    g_foc_hfi_id = 0.0f;
volatile float    g_foc_hfi_iq = 0.0f;
#define FOC_HFI_N 16U                       /* samples per injection cycle */
static uint32_t hfi_k = 0;
static float    hfi_lpf = 0.0f;
static float    hfi_dc_d = 0.0f;
static float    hfi_dc_q = 0.0f;
volatile float    g_foc_rs_ohm = 1.25f;   /* runtime tunable, applied at init */
volatile float    g_foc_ls_h = 0.00198f;
volatile float    g_foc_thetaoff_sf = 0.0f;
volatile float    g_foc_pll_ki = 0.0005f;

/* HF identification (bench) */
volatile uint32_t g_foc_id_enable = 0;
volatile uint32_t g_foc_id_stage = 0;
volatile uint32_t g_foc_id_cnt = 0;
volatile float    g_foc_id_v = 1.5f;      /* injection amplitude, V */
volatile float    g_foc_id_hold_v = 0.63f; /* DC hold ~0.5A on the align axis */
volatile int32_t  g_foc_id_angle_deg = 0;
volatile int16_t  g_foc_id_ampd[16];
volatile int16_t  g_foc_id_ampq[16];
static uint32_t   id_angle_idx = 0;
static float      id_dmin, id_dmax, id_qmin, id_qmax;

static ADC_HandleTypeDef hadc_foc1;
static ADC_HandleTypeDef hadc_foc2;
volatile uint32_t g_foc_isr_cycles = 0U;
volatile uint32_t g_foc_isr_cycles_max = 0U;
volatile uint32_t g_foc_isr_cycles_sum = 0U;

static uint8_t  pwm_on = 0;

/* -------------------------------------------------------------------------- */
void MotorFOC_Brake(void)
{
  /* EG2133: pin LOW -> low side ON. Low sides short the motor (brake). */
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  if ((RCC->AHB2ENR & RCC_AHB2ENR_GPIOAEN) != 0U) {
    GPIOA->BSRR = (uint32_t)(GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10) << 16;
    GPIOA->MODER = (GPIOA->MODER &
                    ~((3UL << 16) | (3UL << 18) | (3UL << 20))) |
                   ((1UL << 16) | (1UL << 18) | (1UL << 20)); /* output */
  }
  pwm_on = 0;
}

/* Fault stop policy.
 *
 * Shorting the phases (MotorFOC_Brake) while the rotor is spinning makes the
 * back-EMF drive a braking current of BEMF/Rs through the low-side switches --
 * at 100 Hz on a 20 V bus that is ~11 V / 1.25 ohm ~= 9 A, which is exactly the
 * kind of abuse that can destroy the bridge. The controlled stop instead keeps
 * the current loop in charge (its output stays clamped to maxCurrent_A) so the
 * braking current is limited. Only short the phases once the rotor is slow.
 *
 * Used by the stall / loss-of-sync fault. Overcurrent keeps an immediate cut
 * for the instantaneous case. */
#define FOC_FAULT_BRAKE_HZ 15.0f

static void FaultStop(void)
{
  if (fabsf(MOTOR_vars.speedFilter_Hz) > FOC_FAULT_BRAKE_HZ && stop_req == 0U) {
    stop_req = 1U;                       /* controlled deceleration */
    stop_start_ms = HAL_GetTick();
  } else {
    MotorFOC_Brake();
    MOTOR_vars.flagRunIdentAndOnLine = false;
    MOTOR_vars.motorState = MOTOR_FAULT_STOP;
  }
}

static void PwmEnable(void)
{
  /* CH1=PA8=W, CH2=PA9=V, CH3=PA10=U on this board. */
  GPIOA->MODER = (GPIOA->MODER &
                  ~((3UL << 16) | (3UL << 18) | (3UL << 20))) |
                 ((2UL << 16) | (2UL << 18) | (2UL << 20)); /* AF */
  TIM1->EGR = TIM_EGR_UG;
  TIM1->SR = ~TIM_SR_UIF;
  TIM1->BDTR |= TIM_BDTR_MOE;
  pwm_on = 1;
}

/* modulation values are phase U/V/W in per-unit, -0.5..0.5 (0.5 => 100% duty) */
static void WritePWM(const float vabc_pu[3])
{
  float u = -vabc_pu[0];          /* U -> CH3 (PA10) */
  float v = -vabc_pu[1];          /* V -> CH2 (PA9)  */
  float w = -vabc_pu[2];          /* W -> CH1 (PA8)  */
  float half = (float)FOC_TIM_ARR * 0.5f;
  uint32_t c1, c2, c3;
  u = (u > 0.5f) ? 0.5f : ((u < -0.5f) ? -0.5f : u);
  v = (v > 0.5f) ? 0.5f : ((v < -0.5f) ? -0.5f : v);
  w = (w > 0.5f) ? 0.5f : ((w < -0.5f) ? -0.5f : w);
  c1 = (uint32_t)((w + 0.5f) * half + 0.5f);
  c2 = (uint32_t)((v + 0.5f) * half + 0.5f);
  c3 = (uint32_t)((u + 0.5f) * half + 0.5f);
  TIM1->CR1 |= TIM_CR1_UDIS;
  TIM1->CCR1 = c1;
  TIM1->CCR2 = c2;
  TIM1->CCR3 = c3;
  TIM1->CR1 &= ~TIM_CR1_UDIS;
  if (!pwm_on) {
    PwmEnable();
  }
}

/* -------------------------------------------------------------------------- */
static void InitGPIO(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PWM pins AF6 = TIM1 */
  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* current shunt inputs + bus divider as analog */
  gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_14 | GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* status LED PC13 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  gpio.Pin = GPIO_PIN_13;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);

  /* PB10 = stop button (active low, internal pull-up) */
  gpio.Pin = GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);
}

static void InitTIM1(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM1_FORCE_RESET();
  __HAL_RCC_TIM1_RELEASE_RESET();

  TIM1->PSC = 0;
  TIM1->ARR = FOC_TIM_ARR;
  TIM1->RCR = REP_COUNTER;                       /* 1 -> 16 kHz ISR */
  TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0;      /* center aligned mode 1 */
  /* CH1/2/3 PWM mode 1 + preload */
  TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE |
                TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
  TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE |
                TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
  /* 50% all channels */
  TIM1->CCR1 = FOC_TIM_ARR / 2;
  TIM1->CCR2 = FOC_TIM_ARR / 2;
  TIM1->CCR3 = FOC_TIM_ARR / 2;
  /* CH4 = ADC trigger, PWM2, fires just before the top of the count where all
   * low sides are on. */
  TIM1->CCR4 = FOC_ADC_TRIG_CMP;
  TIM1->BDTR = TIM_BDTR_OSSI;                    /* no dead time (EG2133 does it) */
  TIM1->EGR = TIM_EGR_UG;
  TIM1->SR = 0;
  /* ADC triggered by CH4 compare */
  TIM1->CR2 = TIM_CR2_MMS_0 | TIM_CR2_MMS_1 | TIM_CR2_MMS_2; /* TRGO = OC4REF */
  TIM1->DIER = TIM_DIER_UIE;                     /* FOC ISR on update event */
  TIM1->CR1 |= TIM_CR1_CEN;
}

/* Software triggered single regular conversion on ADC1. Used from the TIM1
 * update ISR so the shunt current is sampled inside the low-side-on window. */
static inline uint16_t AdcSampleChannel(uint32_t channel_num)
{
  uint32_t guard = 100000U;
  ADC1->SQR1 = ((uint32_t)channel_num << ADC_SQR1_SQ1_Pos);
  ADC1->CR |= ADC_CR_ADSTART;
  while (((ADC1->ISR & ADC_ISR_EOC) == 0U) && (guard != 0U)) {
    guard--;                    /* bounded: never hang the FOC ISR */
  }
  return (uint16_t)ADC1->DR;
}

static void InitADC1(void)
{
  ADC_ChannelConfTypeDef ch = {0};
  ADC_MultiModeTypeDef mm = {0};
  __HAL_RCC_ADC12_CLK_ENABLE();

  hadc_foc1.Instance = ADC1;
  hadc_foc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; /* 170/4 = 42.5 MHz */
  hadc_foc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc_foc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc_foc1.Init.GainCompensation = 0;
  hadc_foc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc_foc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc_foc1.Init.LowPowerAutoWait = DISABLE;
  hadc_foc1.Init.ContinuousConvMode = DISABLE;
  hadc_foc1.Init.NbrOfConversion = 1;
  hadc_foc1.Init.DiscontinuousConvMode = DISABLE;
  hadc_foc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc_foc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc_foc1.Init.DMAContinuousRequests = DISABLE;
  hadc_foc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc_foc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc_foc1) != HAL_OK) { Error_Handler(); }

  mm.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc_foc1, &mm) != HAL_OK) { Error_Handler(); }

  /* Register the sampling time for both shunt channels; SQR1 is rewritten on
   * every sample by AdcSampleChannel(). */
  ch.Rank = ADC_REGULAR_RANK_1;
  ch.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  ch.SingleDiff = ADC_SINGLE_ENDED;
  ch.OffsetNumber = ADC_OFFSET_NONE;
  ch.Channel = FOC_ADC1_INJ_U;
  if (HAL_ADC_ConfigChannel(&hadc_foc1, &ch) != HAL_OK) { Error_Handler(); }
  ch.Channel = FOC_ADC1_INJ_V;
  if (HAL_ADC_ConfigChannel(&hadc_foc1, &ch) != HAL_OK) { Error_Handler(); }

  if (HAL_ADCEx_Calibration_Start(&hadc_foc1, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }

  /* Enable the ADC; conversions are software triggered from the FOC ISR. */
  ADC1->SQR1 = ((uint32_t)FOC_ADC1_INJ_U_NUM << ADC_SQR1_SQ1_Pos);
  LL_ADC_Enable(ADC1);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0U) {
  }
  g_foc_adc_cr = ADC1->CR;
}

static void InitADC2Bus(void)
{
  ADC_ChannelConfTypeDef ch = {0};
  hadc_foc2.Instance = ADC2;
  hadc_foc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc_foc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc_foc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc_foc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc_foc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc_foc2.Init.NbrOfConversion = 1;
  hadc_foc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc_foc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc_foc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc_foc2) != HAL_OK) { Error_Handler(); }
  ch.Channel = FOC_ADC2_BUS_CH;
  ch.Rank = ADC_REGULAR_RANK_1;
  ch.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  ch.SingleDiff = ADC_SINGLE_ENDED;
  ch.OffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADC_ConfigChannel(&hadc_foc2, &ch) != HAL_OK) { Error_Handler(); }
  (void)HAL_ADCEx_Calibration_Start(&hadc_foc2, ADC_SINGLE_ENDED);
}

/* -------------------------------------------------------------------------- */
volatile uint32_t g_foc_jsqr_loop = 0;
volatile int32_t  g_foc_run_ms = 30000;
volatile uint32_t g_foc_main_alive = 0U;      /* bumped by the main loop */
volatile uint32_t g_foc_main_timeout = 0U;    /* main-loop watchdog trips */
static uint32_t   main_alive_seen = 0U;
static uint32_t   main_stall_cnt = 0U;
volatile uint32_t g_foc_log_idx = 0;
volatile int16_t  g_foc_log[256][4];
static uint32_t run_start_ms = 0;
static uint32_t log_last_ms = 0;

void MotorFOC_Init(void)
{
  /* enable the DWT cycle counter for ISR load measurement */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  InitGPIO();
  InitTIM1();
  /* Freeze TIM1 while the debugger has the core halted, so halting stops the
   * PWM and the SWD link stays usable while the motor is energised. */
  DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM1_STOP;
  /* ADC1 (currents) before ADC2, both share the ADC12 common block. */
  InitADC1();
  InitADC2Bus();
  MotorFOC_Brake();

  /* algorithm parameters + controllers */
  MOTOR_vars_Init();

  AdcData.dcBusvoltage_sf = FOC_VBUS_SF;
  AdcData.current_sf = FOC_CURRENT_SF;
  AdcData.offset_I_ad[0] = 2048;
  AdcData.offset_I_ad[1] = 2048;
  AdcData.offset_I_ad[2] = 2048;
  MOTOR_vars.ADCRegularLocked = false;
  MOTOR_vars.offset_complete = 0;
  MOTOR_vars.offset_I_ad_cnt = 0;

  PwmData.period = FOC_TIM_ARR;
  PwmData.minCMPValue = 40;
  PwmData.Vabc_pu.value[0] = 0.0f;
  PwmData.Vabc_pu.value[1] = 0.0f;
  PwmData.Vabc_pu.value[2] = 0.0f;

  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

static void HandleCommand(void)
{
  if (g_foc_command == 2 || g_foc_command == 0) {
    if (g_foc_command == 2) {
      if (g_foc_fault != 0U) {
        /* a real fault (overcurrent/stall) stops immediately */
        MotorFOC_Brake();
        MOTOR_vars.flagRunIdentAndOnLine = false;
        MOTOR_vars.motorState = MOTOR_STOP_IDLE;
        g_foc_fault = 0U;
      } else if (MOTOR_vars.flagRunIdentAndOnLine == true) {
        /* controlled stop: ramp the reference down first */
        if (stop_req == 0U) { stop_req = 1U; stop_start_ms = HAL_GetTick(); }
      } else {
        MotorFOC_Brake();
        MOTOR_vars.motorState = MOTOR_STOP_IDLE;
      }
      g_foc_arm = 0;
    }
  } else {
    if (g_foc_arm == 1U &&
        MOTOR_vars.flagRunIdentAndOnLine == false) {
      g_foc_arm = 0;
      g_foc_fault = 0U;
      oc_cnt = 0U;
      stall_cnt = 0U;
      stop_req = 0U;
      /* Reset every controller integrator. Without this a restart right after
       * a fault inherits a wound-up Id/Iq/speed PI and the current loop drives
       * a large, oscillating current instead of the commanded one. */
      MOTOR_vars.piHandle_Id.Ui = 0.0f;
      MOTOR_vars.piHandle_Iq.Ui = 0.0f;
      MOTOR_vars.piHandle_spd.Ui = 0.0f;
      g_foc_hfi_stage = 0U;
      g_foc_hfi_theta = 0.0f;
      /* Re-apply the runtime-tunable motor parameters (g_foc_rs_ohm /
       * g_foc_ls_h) on every start so they can be tuned via the debugger
       * without re-flashing. */
      MOTOR_vars_Init();
      /* start */
      MOTOR_vars.ADCRegularLocked = true;
      MOTOR_vars.ISRCount = 0;
      MOTOR_vars.stateRunTimeCnt = 0;
      MOTOR_vars.motorState = MOTOR_ALIGNMENT;
      MOTOR_vars.flagRunIdentAndOnLine = true;
      MOTOR_vars.enableSpeedCtrl = false;
      MOTOR_vars.enableCurrentCtrl = false;
      MOTOR_vars.flagEnableCurrentCtrl = true;
      MOTOR_vars.flagEnableSpeedCtrl = true;
      MOTOR_vars.offset_complete = 0;
      MOTOR_vars.offset_I_ad_cnt = 0;
      MOTOR_vars.direction = (g_foc_command > 0) ? 1.0f : -1.0f;
      ESMO_resetParams(&MOTOR_vars.esmoHandle);
      ESMO_resetPLL(&MOTOR_vars.esmoHandle);
      MOTOR_vars.speed_int_Hz = 0.0f;
      MOTOR_vars.trajHandle_spd.targetValue = g_foc_command > 0 ?
                                              g_foc_speed_ref_hz : -g_foc_speed_ref_hz;
      MOTOR_vars.trajHandle_spd.intValue = 0.0f;
      MOTOR_vars.angle_rad = 0.0f;
      run_start_ms = HAL_GetTick();
      log_last_ms = run_start_ms;
      g_foc_log_idx = 0;
      /* start PWM at 50% so offsets can be measured with no torque */
      WritePWM(PwmData.Vabc_pu.value);
    }
  }
}

static void ReadBus(void)
{
  uint16_t raw;
  if (HAL_ADC_Start(&hadc_foc2) != HAL_OK) { return; }
  if (HAL_ADC_PollForConversion(&hadc_foc2, 2) != HAL_OK) {
    HAL_ADC_Stop(&hadc_foc2);
    return;
  }
  raw = (uint16_t)HAL_ADC_GetValue(&hadc_foc2);
  HAL_ADC_Stop(&hadc_foc2);
  g_foc_vbus_v = (float)raw * FOC_VBUS_SF;
  AdcData.VdcBus_V = g_foc_vbus_v;
}

void MotorFOC_Loop1ms(void)
{
  g_foc_main_alive++;
  uint32_t now = HAL_GetTick();
  ReadBus();

  /* hardware stop button (PB10, active low) -> request a controlled stop */
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) {
    g_foc_arm = 0;
    g_foc_command = 2;
  }
  HandleCommand();

  if (MOTOR_vars.flagRunIdentAndOnLine == true) {
    /* ---- stall / loss-of-sync protection --------------------------------
     * Once the speed loop is closed, a current reference pinned at the limit
     * for longer than g_foc_stall_ms means the rotor cannot follow (stall,
     * wrong commutation, mechanical jam): stop before it overheats.
     *
     * Only evaluate this once the trajectory has SETTLED on its target: while a
     * speed change is still ramping the speed loop legitimately saturates (that
     * is how it accelerates), which used to be mistaken for a stall -- so
     * changing the speed made the motor "slowly stop" as the trip engaged. */
    if (MOTOR_vars.motorState >= MOTOR_CL_RUNNING && g_foc_fault == 0U) {
      float traj_err = fabsf(MOTOR_vars.trajHandle_spd.intValue -
                             MOTOR_vars.trajHandle_spd.targetValue);
      if (traj_err > 1.0f) {
        stall_cnt = 0U;                        /* still ramping, not a stall */
      } else if (fabsf(MOTOR_vars.IsRef_A) > 0.9f * MOTOR_vars.maxCurrent_A) {
        if (stall_cnt < 10000U) stall_cnt++;
        if (g_foc_stall_ms > 0 && stall_cnt > (uint32_t)g_foc_stall_ms) {
          g_foc_fault = 2U;
          FaultStop();   /* controlled decel at speed; brake only when slow */
        }
      } else {
        stall_cnt = 0U;
      }
    } else {
      stall_cnt = 0U;
    }

    /* black box: 100 ms grid, 64 entries (~6.4 s) */
    if (g_foc_log_idx < 256U && (now - log_last_ms) >= 200U) {
      float vd = MOTOR_vars.Vdq_out_V.value[0];
      float vq = MOTOR_vars.Vdq_out_V.value[1];
      log_last_ms = now;
      g_foc_log[g_foc_log_idx][0] = (int16_t)MOTOR_vars.motorState;
      if (g_foc_hfi_stage == 1U) {
        g_foc_log[g_foc_log_idx][1] = (int16_t)(g_foc_hfi_theta * 100.0f);
        g_foc_log[g_foc_log_idx][2] = (int16_t)(g_foc_hfi_err * 1000.0f);
        g_foc_log[g_foc_log_idx][3] = (int16_t)(g_foc_hfi_iq * 1000.0f);
      } else {
        g_foc_log[g_foc_log_idx][1] = (int16_t)(MOTOR_vars.speedFilter_Hz * 100.0f);
        g_foc_log[g_foc_log_idx][2] = (int16_t)(MOTOR_vars.Idq_in_A.value[1] * 1000.0f);
        g_foc_log[g_foc_log_idx][3] = (int16_t)(sqrtf(vd * vd + vq * vq) * 10.0f);
      }
      g_foc_log_idx++;
    }
    /* Live speed change: the trajectory target must follow the command in
     * EVERY running state.  It used to be updated only once the loop was
     * closed, so a setpoint change made while the motor was still in the I/F
     * ramp was silently ignored -- e.g. RUN 8 then S 120 left the motor
     * spinning at 8 Hz for ever because the trajectory never reached the
     * 33 Hz hand-over speed. */
    if (MOTOR_vars.flagRunIdentAndOnLine == true) {
      MOTOR_vars.trajHandle_spd.targetValue =
          (MOTOR_vars.direction > 0.0f) ? g_foc_speed_ref_hz : -g_foc_speed_ref_hz;
      if (MOTOR_vars.motorState >= MOTOR_CL_RUNNING) {
        MOTOR_vars.trajHandle_spd.maxDelta =
            ((g_foc_accel_max_hzps > 0.0f) ? g_foc_accel_max_hzps
                                           : MOTOR_vars.accelerationMax_Hzps) *
            MOTOR_vars.ctrlPeriod_sec;
      }
    }
    if (g_foc_run_ms > 0 && (int32_t)(now - run_start_ms) > g_foc_run_ms) {
      if (stop_req == 0U) { stop_req = 1U; stop_start_ms = now; }
    }

    /* ---- controlled stop ------------------------------------------------
     * Ramp the speed reference to zero with the current loop still active so
     * the braking current stays within maxCurrent_A, and only short the phases
     * (MotorFOC_Brake) once the rotor is slow. A direct short at speed is what
     * produced the large, uncontrolled braking current spike. */
    if (stop_req != 0U) {
      MOTOR_vars.trajHandle_spd.maxDelta =
          g_foc_stop_decel_hzps * MOTOR_vars.ctrlPeriod_sec;
      MOTOR_vars.trajHandle_spd.targetValue = 0.0f;
      if (fabsf(MOTOR_vars.speedFilter_Hz) < 3.0f || (now - stop_start_ms) > 5000U) {
        MotorFOC_Brake();
        MOTOR_vars.flagRunIdentAndOnLine = false;
        MOTOR_vars.motorState = MOTOR_NORM_STOP;
        stop_req = 0U;
      }
    }
  }

  if (g_foc_hfi_freeze == 1U && g_foc_hfi_stage == 1U) {
    /* diagnostic: step the frozen angle 0 -> 1.4 rad so the demodulated error
     * can be plotted against the true angle offset (expect a sin(2*dtheta)). */
    if ((now - hfi_sweep_ms) >= 500U) {
      hfi_sweep_ms = now;
      g_foc_hfi_theta0 += 0.2f;
      if (g_foc_hfi_theta0 > 1.5f) g_foc_hfi_theta0 = 0.0f;
      g_foc_hfi_theta = g_foc_hfi_theta0;
    }
  }

  g_foc_jsqr_loop = ADC1->JSQR;   /* diagnostic: watch for unexpected clearing */
  g_foc_state = (uint32_t)MOTOR_vars.motorState;
  g_foc_offset_ready = MOTOR_vars.offset_complete;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,
    (g_foc_fault != 0U) ? ((now / 200U) & 1U ? GPIO_PIN_SET : GPIO_PIN_RESET) :
    ((MOTOR_vars.flagRunIdentAndOnLine == true) ? GPIO_PIN_RESET : GPIO_PIN_SET));
}

/* -------------------------------------------------------------------------- */
void MotorFOC_TimerISR(void)
{
  int32_t adcU, adcV;
  float ia, ib, ic;
  float angleFOC;
  uint32_t cyc0;

  if ((TIM1->SR & TIM_SR_UIF) == 0U) {
    return;
  }
  TIM1->SR = ~TIM_SR_UIF;
  cyc0 = DWT->CYCCNT;
  g_foc_isr_count++;

  if (!pwm_on) {
    return;                     /* outputs not enabled yet */
  }

  /* ---- main-loop watchdog (only armed while the motor is actually driven,
   * so boot / init timing cannot trip it) --------------------------------- */
  if (MOTOR_vars.flagRunIdentAndOnLine == true) {
    if (g_foc_main_alive != main_alive_seen) {
      main_alive_seen = g_foc_main_alive;
      main_stall_cnt = 0U;
    } else if (main_stall_cnt < 0x00FFFFFFU) {
      main_stall_cnt++;
    }
    if (main_stall_cnt > 8000U) {        /* 500 ms at 16 kHz */
      g_foc_main_timeout++;
      g_foc_fault = 3U;                  /* 3 = main loop stalled */
      MotorFOC_Brake();
      MOTOR_vars.flagRunIdentAndOnLine = false;
      MOTOR_vars.motorState = MOTOR_FAULT_STOP;
      return;
    }
  } else {
    main_alive_seen = g_foc_main_alive;
    main_stall_cnt = 0U;
  }
  /* Sample the two low-side shunts while every low side is on (center of the
   * low window, i.e. around the counter overflow). */
  adcU = (int32_t)AdcSampleChannel(FOC_ADC1_INJ_U_NUM);
  adcV = (int32_t)AdcSampleChannel(FOC_ADC1_INJ_V_NUM);
  g_foc_ishunt_adc[0] = (uint16_t)adcU;
  g_foc_ishunt_adc[1] = (uint16_t)adcV;

  if (MOTOR_vars.flagRunIdentAndOnLine == false) {
    return;
  }

  /* ---- HFI: pulsating injection on the estimated d axis ---------------- */
  if (g_foc_hfi_stage == 1U) {
    float ang = g_foc_hfi_theta;
    float ph = 6.2831853f * (float)(hfi_k % FOC_HFI_N) / (float)FOC_HFI_N;
    float c = FOC_Cos(ph);           /* injection: cos(w t) */
    float s = FOC_Sin(ph);           /* demodulation: sin(w t) */
    FOC_AlBe iab;
    FOC_DQ idq;
    float ida = (float)(adcU - AdcData.offset_I_ad[0]) * AdcData.current_sf;
    float idb = (float)(adcV - AdcData.offset_I_ad[1]) * AdcData.current_sf;
    AdcData.I_A.value[0] = ida;
    AdcData.I_A.value[1] = idb;
    AdcData.I_A.value[2] = -(ida + idb);
    FOC_Clarke(ida, idb, &iab);
    FOC_Park(&iab, ang, &idq);
    g_foc_hfi_id = idq.d;
    g_foc_hfi_iq = idq.q;
    /* Demodulate the q-axis HF current. The inductive response lags the
     * injected voltage by 90 deg (both Id and Iq go with sin(w t) while the
     * injection is cos(w t)), so the correlator must use sin(w t).
     * A slow LPF tracks the DC/fundamental part of Id/Iq and is subtracted
     * first, otherwise that large offset swamps the tiny saliency term and
     * drags the tracking loop. */
    hfi_dc_d += 0.002f * (idq.d - hfi_dc_d);
    hfi_dc_q += 0.002f * (idq.q - hfi_dc_q);
    /* The saliency term appears in phase with the injected carrier (it scales
     * with 1/L, not with w*L), so correlate the q-axis HF current with cos.
     * (Correlating with sin gave identically zero.) */
    hfi_lpf += 0.05f * ((idq.q - hfi_dc_q) * c - hfi_lpf);
    g_foc_hfi_err = hfi_lpf;
    /* tracking loop: err ~ sin(2*dtheta) -> drive to zero */
    if (g_foc_hfi_freeze == 0U) {
      g_foc_hfi_theta += g_foc_hfi_kp_sign * g_foc_hfi_kp * hfi_lpf;
      if (g_foc_hfi_theta > 3.14159265f) g_foc_hfi_theta -= 6.2831853f;
      else if (g_foc_hfi_theta < -3.14159265f) g_foc_hfi_theta += 6.2831853f;
    }
    /* inject on the estimated d axis; optional DC hold on axis 0 pins the
     * rotor during the diagnostic so dtheta = theta_h (known). */
    {
      float vh = g_foc_hfi_v * c;
      float va = g_foc_hfi_hold_v + vh * FOC_Cos(ang);
      float vb = vh * FOC_Sin(ang);
      MATH_Vec2 vab2;
      vab2.value[0] = va;
      vab2.value[1] = vb;
      MOTOR_vars.svgenHandle.oneOverDcBus_invV = 1.0f / AdcData.VdcBus_V;
      SVGEN_run(&MOTOR_vars.svgenHandle, &vab2, &PwmData.Vabc_pu);
    }
    WritePWM(PwmData.Vabc_pu.value);
    hfi_k++;
    return;
  }

  /* ---- saliency scan: DC hold on the align axis + HF injection at a set of
   * angles. The DC hold keeps the rotor pinned at theta=0 so the measured
   * "d" / "q" axes are real, not shifted by rotor motion. For a genuine
   * saliency the amplitude vs angle follows a smooth 180 deg-periodic curve;
   * for a measurement artefact it is erratic. */
  if (g_foc_id_stage == 1U) {
    float th = (float)(id_angle_idx * 30U) * 0.017453292f;   /* 0,30,60,... deg elec */
    float s = FOC_Sin(6.2831853f * (float)(g_foc_id_cnt % 16U) / 16.0f);
    FOC_AlBe iab;
    FOC_DQ idq;
    float ida = (float)(adcU - AdcData.offset_I_ad[0]) * AdcData.current_sf;
    float idb = (float)(adcV - AdcData.offset_I_ad[1]) * AdcData.current_sf;
    float vh, va, vb;
    AdcData.I_A.value[0] = ida;
    AdcData.I_A.value[1] = idb;
    AdcData.I_A.value[2] = -(ida + idb);
    FOC_Clarke(ida, idb, &iab);
    FOC_Park(&iab, th, &idq);
    if (g_foc_id_cnt >= 16U && g_foc_id_cnt < 256U) {
      if (idq.d < id_dmin) id_dmin = idq.d;
      if (idq.d > id_dmax) id_dmax = idq.d;
      if (idq.q < id_qmin) id_qmin = idq.q;
      if (idq.q > id_qmax) id_qmax = idq.q;
    }
    g_foc_id_angle_deg = (int32_t)(id_angle_idx * 30U);
    /* stationary-frame voltage = DC hold at 0 deg + HF vector at th */
    vh = g_foc_id_v * s;
    va = g_foc_id_hold_v + vh * FOC_Cos(th);
    vb = vh * FOC_Sin(th);
    {
      MATH_Vec2 vab2;
      vab2.value[0] = va;
      vab2.value[1] = vb;
      MOTOR_vars.svgenHandle.oneOverDcBus_invV = 1.0f / AdcData.VdcBus_V;
      SVGEN_run(&MOTOR_vars.svgenHandle, &vab2, &PwmData.Vabc_pu);
    }
    WritePWM(PwmData.Vabc_pu.value);
    g_foc_id_cnt++;
    if (g_foc_id_cnt >= 272U) {
      g_foc_id_ampd[id_angle_idx] = (int16_t)((id_dmax - id_dmin) * 0.5f * 1000.0f);
      g_foc_id_ampq[id_angle_idx] = (int16_t)((id_qmax - id_qmin) * 0.5f * 1000.0f);
      g_foc_id_cnt = 0U;
      id_dmin = id_qmin = 1.0e9f;
      id_dmax = id_qmax = -1.0e9f;
      id_angle_idx++;
      if (id_angle_idx >= FOC_ID_NANGLES) {
        id_angle_idx = 0U;
        g_foc_id_stage = 3U;
        MotorFOC_Brake();
        MOTOR_vars.flagRunIdentAndOnLine = false;
        MOTOR_vars.motorState = MOTOR_NORM_STOP;
      }
    }
    return;
  }

  if (MOTOR_vars.offset_complete == 0) {
    if (MOTOR_vars.offset_I_ad_cnt == 0) {
      AdcData.offset_I_ad_sum[0] = 0;
      AdcData.offset_I_ad_sum[1] = 0;
      AdcData.offset_I_ad_sum[2] = 0;
    }
    AdcData.offset_I_ad_sum[0] += adcU;
    AdcData.offset_I_ad_sum[1] += adcV;
    AdcData.offset_I_ad_sum[2] += (adcU + adcV) / 2; /* keep sum[2] meaningful */
    MOTOR_vars.offset_I_ad_cnt++;
    if (MOTOR_vars.offset_I_ad_cnt >= 256) {
      AdcData.offset_I_ad[0] = AdcData.offset_I_ad_sum[0] >> 8;
      AdcData.offset_I_ad[1] = AdcData.offset_I_ad_sum[1] >> 8;
      AdcData.offset_I_ad[2] = 2048;
      MOTOR_vars.offset_I_ad_cnt = 0;
      MOTOR_vars.offset_complete = 1;
    }
    WritePWM(PwmData.Vabc_pu.value); /* hold 50% while calibrating */
    return;
  }

  ia = (float)(adcU - AdcData.offset_I_ad[0]) * AdcData.current_sf;
  ib = (float)(adcV - AdcData.offset_I_ad[1]) * AdcData.current_sf;
  ic = -(ia + ib); /* W shunt amp is dead; reconstruct */
  AdcData.I_A.value[0] = ia;
  AdcData.I_A.value[1] = ib;
  AdcData.I_A.value[2] = ic;
  g_foc_ishunt_a[0] = ia;
  g_foc_ishunt_a[1] = ib;
  g_foc_ishunt_a[2] = ic;

  /* ---- overcurrent protection (acts before the FOC math) ---- */
  {
    float iabs = fabsf(ia);
    if (fabsf(ib) > iabs) iabs = fabsf(ib);
    if (fabsf(ic) > iabs) iabs = fabsf(ic);
    g_foc_imax_a = iabs;
    if (iabs > g_foc_oc_trip_a) {
      oc_cnt++;
      if (oc_cnt >= FOC_OC_DEBOUNCE || iabs > g_foc_oc_inst_a) {
        g_foc_fault = 1U;
        if (iabs > g_foc_oc_inst_a) {
          /* hard instantaneous overcurrent: cut now */
          MotorFOC_Brake();
          MOTOR_vars.flagRunIdentAndOnLine = false;
          MOTOR_vars.motorState = MOTOR_FAULT_STOP;
          return;
        }
        FaultStop();     /* sustained overcurrent: decelerate with the loop in charge */
        return;
      }
    } else {
      oc_cnt = 0U;
    }
  }

  /* open-loop angle generator (used for align + I/F start) */
  ANGLE_GEN_run(MOTOR_vars.speed_int_Hz);
  MOTOR_vars.angleGen_rad = MOTOR_vars.angle_rad;

  CLARKE_run(&AdcData.I_A, &MOTOR_vars.Iab_A);

  if (MOTOR_vars.flagRunIdentAndOnLine == true) {
    MOTOR_vars.enableSpeedCtrl = true;
    MOTOR_vars.enableCurrentCtrl = true;
    TRAJ_run(MOTOR_vars.trajHandle_spd.targetValue);
  } else {
    MOTOR_vars.enableSpeedCtrl = false;
    MOTOR_vars.enableCurrentCtrl = false;
  }
  MOTOR_vars.speed_int_Hz = MOTOR_vars.trajHandle_spd.intValue;
  MOTOR_vars.oneOverDcBus_inv = 1.0f / AdcData.VdcBus_V;
  MOTOR_vars.svgenHandle.oneOverDcBus_invV = MOTOR_vars.oneOverDcBus_inv;

  /* eSMO observer */
  MOTOR_vars.esmoHandle.speedRef = MOTOR_vars.esmoHandle.speed_sf * MOTOR_vars.speed_int_Hz;
  ESMO_run(&MOTOR_vars.esmoHandle, AdcData.VdcBus_V, &PwmData.Vabc_pu, &MOTOR_vars.Iab_A);

  /* ---- optional stator-flux observer (g_foc_obs_mode = 1) --------------
   * Integrates (v - R*i) with a leaky integrator to get the stator flux, then
   * subtracts L*i to obtain the rotor flux and takes its angle. Replaces the
   * eSMO angle; the eSMO still runs so the two can be compared. */
  if (g_foc_obs_mode == 1U) {
    float ts = MOTOR_vars.ctrlPeriod_sec;
    float ia = MOTOR_vars.Iab_A.value[0];
    float ib = MOTOR_vars.Iab_A.value[1];
    float va = MOTOR_vars.esmoHandle.Valpha;
    float vb = MOTOR_vars.esmoHandle.Vbeta;
    float raw, err;
    flux_a += ts * (va - g_foc_rs_ohm * ia - g_foc_flux_wc * flux_a);
    flux_b += ts * (vb - g_foc_rs_ohm * ib - g_foc_flux_wc * flux_b);
    raw = atan2f(flux_b - g_foc_ls_h * ib, flux_a - g_foc_ls_h * ia);
    err = raw - flux_theta;
    if (err > 3.14159265f) err -= 6.2831853f;
    else if (err < -3.14159265f) err += 6.2831853f;
    flux_ui += g_foc_flux_ki * err;
    if (flux_ui > 1.0f) flux_ui = 1.0f;
    else if (flux_ui < -1.0f) flux_ui = -1.0f;
    {
      float wpu = flux_ui + g_foc_flux_kp * err;
      if (wpu > 1.0f) wpu = 1.0f;
      else if (wpu < -1.0f) wpu = -1.0f;
      flux_theta += wpu * 6.2831853f * MOTOR_vars.esmoHandle.scaleFreq_Hz * ts;
      if (flux_theta > 3.14159265f) flux_theta -= 6.2831853f;
      else if (flux_theta < -3.14159265f) flux_theta += 6.2831853f;
      MOTOR_vars.esmoHandle.thetaEst = flux_theta;
    }
  }

  /* Before the observer-based loop is closed the BEMF is too small to be
   * trusted, so keep the PLL purely proportional (no integrator wind-up). */
  if (MOTOR_vars.motorState < MOTOR_CL_RUNNING) {
    MOTOR_vars.esmoHandle.pll_ui = 0.0f;
  }
  /* Keep the flux integrator initialised while the rotor is held at theta=0,
   * but let it keep integrating through the I/F ramp so it is already valid at
   * the hand-over (a bare zero initial value needs ~1/wc to converge, which is
   * what stalled the motor right at the switch). */
  if (MOTOR_vars.motorState <= MOTOR_ALIGNMENT) {
    float c0 = FOC_Cos(MOTOR_vars.angleGen_rad);
    float s0 = FOC_Sin(MOTOR_vars.angleGen_rad);
    flux_a = g_foc_ls_h * MOTOR_vars.Iab_A.value[0] + g_foc_flux_psi * c0;
    flux_b = g_foc_ls_h * MOTOR_vars.Iab_A.value[1] + g_foc_flux_psi * s0;
    flux_ui = 0.0f;
    flux_theta = MOTOR_vars.angleGen_rad;
  }

  MOTOR_vars.anglePLLComp_rad = MOTOR_vars.speedPLL_Hz * MOTOR_vars.angleDelayed_sf;
  MOTOR_vars.anglePLL_rad = MATH_incrAngle(MOTOR_vars.esmoHandle.thetaEst,
                                           MOTOR_vars.anglePLLComp_rad);
  SPDFR_run(&MOTOR_vars.spdfrHandle, MOTOR_vars.anglePLL_rad);
  MOTOR_vars.speedPLL_Hz = MOTOR_vars.spdfrHandle.speed_Hz;
  MOTOR_vars.speed_Hz = MOTOR_vars.speedPLL_Hz;
  MOTOR_vars.speedFilter_Hz = MOTOR_vars.speedFilter_Hz * 0.875f + MOTOR_vars.speed_Hz * 0.125f;
  MOTOR_vars.speedAbs_Hz = fabsf(MOTOR_vars.speedFilter_Hz);

  MOTOR_vars.stateRunTimeCnt++;

  if (MOTOR_vars.motorState >= MOTOR_CL_RUNNING) {
    angleFOC = MOTOR_vars.anglePLL_rad;
    ESMO_updateKslide(&MOTOR_vars.esmoHandle);
    if ((MOTOR_vars.motorState == MOTOR_CL_RUNNING) &&
        (MOTOR_vars.stateRunTimeCnt > MOTOR_vars.startupTimeDelay)) {
      /* observer locked and settled -> close the speed loop */
      MOTOR_vars.Idq_out_A.value[0] = 0.0f;
      MOTOR_vars.motorState = MOTOR_CTRL_RUN;
      MOTOR_vars.stateRunTimeCnt = 0;
    }
  } else if (MOTOR_vars.motorState == MOTOR_OL_START) {
    angleFOC = MOTOR_vars.angleGen_rad;
    MOTOR_vars.enableSpeedCtrl = false;
    MOTOR_vars.Idq_out_A.value[0] = 0.0f;
    /* The I/F ramp rate is the start acceleration, not the run acceleration.
     * Previously maxDelta kept its init value here, so ACCEL_START had no
     * effect and the ramp was always the same rate. */
    MOTOR_vars.trajHandle_spd.maxDelta =
        ((g_foc_accel_start_hzps > 0.0f) ? g_foc_accel_start_hzps
                                         : MOTOR_vars.accelerationStart_Hzps) *
        MOTOR_vars.ctrlPeriod_sec;
    /* Both IsRef_A and Idq_out_A[1] must be set: the code further down does
     * Idq_out_A[1] = IsRef_A, so only setting Idq_out_A[1] would be lost. */
    if (MOTOR_vars.speed_int_Hz > 0.0f) {
      MOTOR_vars.IsRef_A = (g_foc_start_current_a > 0.0f) ? g_foc_start_current_a
                                                          : MOTOR_vars.startCurrent_A;
    } else {
      MOTOR_vars.IsRef_A = -(g_foc_start_current_a > 0.0f ? g_foc_start_current_a
                                                          : MOTOR_vars.startCurrent_A);
    }
    MOTOR_vars.Idq_out_A.value[1] = MOTOR_vars.IsRef_A;
    if (fabsf(MOTOR_vars.speed_int_Hz) >=
        ((g_foc_speed_force_hz > 0.0f) ? g_foc_speed_force_hz
                                       : MOTOR_vars.speedForce_Hz)) {
      MOTOR_vars.trajHandle_spd.intValue = MOTOR_vars.speed_int_Hz;
      if (MOTOR_vars.stateRunTimeCnt > MOTOR_vars.forceRunTimeDelay &&
          (g_foc_handover_errmax <= 0.0f ||
           fabsf(MOTOR_vars.esmoHandle.thetaErr) < g_foc_handover_errmax)) {
        MOTOR_vars.motorState = MOTOR_CL_RUNNING;
        MOTOR_vars.stateRunTimeCnt = 0;
        MOTOR_vars.esmoHandle.theta = angleFOC * MATH_ONE_OVER_TWO_PI;
        MOTOR_vars.piHandle_spd.Ui = MOTOR_vars.frswPos_sf * MOTOR_vars.Idq_out_A.value[1];
      }
    }
  } else { /* MOTOR_ALIGNMENT */
    angleFOC = 0.0f;
    MOTOR_vars.enableSpeedCtrl = false;
    MOTOR_vars.Idq_out_A.value[0] = MOTOR_vars.alignCurrent_A;
    MOTOR_vars.Idq_out_A.value[1] = 0.0f;
    MOTOR_vars.trajHandle_spd.intValue = 0.0f;
    MOTOR_vars.esmoHandle.theta = 0.0f;
    MOTOR_vars.angle_rad = 0.0f;
    if (MOTOR_vars.stateRunTimeCnt > MOTOR_vars.alignTimeDelay) {
      if (g_foc_hfi_enable) {
        /* rotor aligned at theta=0 -> start HFI tracking */
        g_foc_hfi_stage = 1U;
        g_foc_hfi_theta = g_foc_hfi_theta0;
        hfi_k = 0U;
        hfi_lpf = 0.0f;
        hfi_dc_d = 0.0f;
        hfi_dc_q = 0.0f;
        MOTOR_vars.stateRunTimeCnt = 0;
        return;
      }
      if (g_foc_id_enable) {
        /* rotor is now aligned at theta=0: start the saliency scan */
        g_foc_id_stage = 1;
        g_foc_id_cnt = 0;
        id_angle_idx = 0;
        id_dmin = id_qmin = 1.0e9f;
        id_dmax = id_qmax = -1.0e9f;
        MOTOR_vars.stateRunTimeCnt = 0;
        return;
      }
      MOTOR_vars.motorState = MOTOR_OL_START;
      MOTOR_vars.stateRunTimeCnt = 0;
      MOTOR_vars.Idq_out_A.value[0] = MOTOR_vars.fluxCurrent_A;
    }
  }

  MOTOR_vars.angleFOC_rad = angleFOC;
  PARK_run(&MOTOR_vars.Iab_A, &MOTOR_vars.Idq_in_A, angleFOC);
  MOTOR_vars.speed_rpm = MOTOR_vars.speed_Hz * MOTOR_vars.hz2Rpm_sf;

  /* speed loop at 1/10 of the ISR rate */
  MOTOR_vars.counterSpeed++;
  if (MOTOR_vars.counterSpeed >= userParamsHandle.numCtrlTicksPerSpeedTick) {
    MOTOR_vars.counterSpeed = 0;
    MOTOR_vars.speed_reg_Hz = (g_foc_spd_fb_filter != 0U) ? MOTOR_vars.speedFilter_Hz
                                                          : MOTOR_vars.speed_Hz;
    /* The speed loop used to be fed the raw PLL speed (speed_Hz). The eSMO
     * speed estimate carries switching-frequency noise and the PI amplified it
     * into a visible ~3% speed ripple. Use the filtered estimate instead; the
     * filter is ~1 ms so the added phase lag is negligible next to the loop
     * period (1/1600 s). */
    if (MOTOR_vars.enableSpeedCtrl == true) {
      MOTOR_vars.Is_ffwd_A = 0.0f;
      PI_run_series(&MOTOR_vars.piHandle_spd, MOTOR_vars.speed_int_Hz,
                    MOTOR_vars.speed_reg_Hz, MOTOR_vars.Is_ffwd_A,
                    (float32_t *)&MOTOR_vars.IsRef_A);
    } else if (MOTOR_vars.motorState >= MOTOR_CL_RUNNING) {
      MOTOR_vars.IsRef_A = (MOTOR_vars.speed_int_Hz > 0.0f) ?
                           MOTOR_vars.IsSet_A : -MOTOR_vars.IsSet_A;
      MOTOR_vars.piHandle_spd.Ui = MOTOR_vars.IsRef_A;
    }
  }
  MOTOR_vars.Idq_out_A.value[1] = MOTOR_vars.IsRef_A;
  MOTOR_vars.IdqRef_A.value[0] = MOTOR_vars.Idq_out_A.value[0] + MOTOR_vars.IdRated_A;
  MOTOR_vars.IdqRef_A.value[1] = MOTOR_vars.Idq_out_A.value[1];

  /* ---- MTPA -----------------------------------------------------------
   * Add a small negative id proportional to iq^2 to exploit the reluctance
   * torque (id = A - sqrt(A^2 + iq^2)). A = psi_r / (2*(Lq-Ld)). */
  if (g_foc_mtpa_enable != 0U && MOTOR_vars.motorState >= MOTOR_CL_RUNNING) {
    float dl = g_foc_lq_h - g_foc_ld_h;
    if (dl > 1.0e-9f) {
      float a = g_foc_flux_psi / (2.0f * dl);
      float iq = MOTOR_vars.IdqRef_A.value[1];
      if (isfinite(iq) && isfinite(a)) {
        float idm = a - sqrtf(a * a + iq * iq);
        /* clamp to a sane negative range so a bad sample cannot poison the
         * current reference (a NaN id saturated the Id PI and tripped OC) */
        if (!isfinite(idm) || idm > 0.0f) idm = 0.0f;
        if (idm < -MOTOR_vars.maxCurrent_A) idm = -MOTOR_vars.maxCurrent_A;
        MOTOR_vars.IdqRef_A.value[0] += idm;
        g_foc_mtpa_id_a = idm;
      } else {
        g_foc_mtpa_id_a = 0.0f;
      }
    }
  } else {
    g_foc_mtpa_id_a = 0.0f;
  }

  /* ---- field weakening ------------------------------------------------ */
  if (g_foc_fw_enable != 0U && MOTOR_vars.motorState >= MOTOR_CL_RUNNING &&
      stop_req == 0U) {   /* drop FW while braking so the stop current stays low */
    float vd = MOTOR_vars.Vdq_out_V.value[0];
    float vq = MOTOR_vars.Vdq_out_V.value[1];
    float vmag = sqrtf(vd * vd + vq * vq);
    float vlim = 0.95f * MOTOR_vars.VsMax_V;
    fw_int -= g_foc_fw_gain * (vmag - vlim);   /* over-voltage -> more negative id */
    /* Note: an earlier revision added a "back off while overspeeding" term here.
     * It created a second feedback path (FW integrator <-> speed) that showed up
     * as an extra oscillation, and it is unnecessary now that fw_max_a is only
     * 0.35 A (a 3 % flux reduction cannot make the machine run away). */
    if (fw_int > 0.0f) fw_int = 0.0f;
    if (fw_int < -g_foc_fw_max_a) fw_int = -g_foc_fw_max_a;
  } else {
    fw_int = 0.0f;
  }
  g_foc_fw_id_a = fw_int;
  if (fw_int != 0.0f) {
    float iqlim;
    MOTOR_vars.IdqRef_A.value[0] += fw_int;
    /* keep the total current vector inside maxCurrent_A */
    iqlim = MOTOR_vars.maxCurrent_A * MOTOR_vars.maxCurrent_A -
            MOTOR_vars.IdqRef_A.value[0] * MOTOR_vars.IdqRef_A.value[0];
    iqlim = (iqlim > 0.0f) ? sqrtf(iqlim) : 0.0f;
    if (MOTOR_vars.IdqRef_A.value[1] > iqlim) MOTOR_vars.IdqRef_A.value[1] = iqlim;
    if (MOTOR_vars.IdqRef_A.value[1] < -iqlim) MOTOR_vars.IdqRef_A.value[1] = -iqlim;
  }

  MOTOR_vars.enableCurrentCtrl = true;

  /* current loop */
  MOTOR_vars.VsMax_V = MOTOR_vars.maxVsMag_pu * AdcData.VdcBus_V;
  MOTOR_vars.piHandle_Id.outMin = -MOTOR_vars.VsMax_V;
  MOTOR_vars.piHandle_Id.outMax = MOTOR_vars.VsMax_V;
  PI_run_series(&MOTOR_vars.piHandle_Id, MOTOR_vars.IdqRef_A.value[0],
                MOTOR_vars.Idq_in_A.value[0], 0.0f,
                (float32_t *)&MOTOR_vars.Vdq_out_V.value[0]);
  {
    float outMax_V = sqrtf(MOTOR_vars.VsMax_V * MOTOR_vars.VsMax_V -
                           MOTOR_vars.Vdq_out_V.value[0] * MOTOR_vars.Vdq_out_V.value[0]);
    MOTOR_vars.piHandle_Iq.outMin = -outMax_V;
    MOTOR_vars.piHandle_Iq.outMax = outMax_V;
  }
  PI_run_series(&MOTOR_vars.piHandle_Iq, MOTOR_vars.IdqRef_A.value[1],
                MOTOR_vars.Idq_in_A.value[1], 0.0f,
                (float32_t *)&MOTOR_vars.Vdq_out_V.value[1]);

  IPARK_run(&MOTOR_vars.Vdq_out_V, &MOTOR_vars.Vab_out_V, angleFOC);

  /* ---- dead-time compensation (alpha-beta frame) ----------------------
   * Clarke of the per-phase sign terms scaled by (Td/Ts)*Vbus.  Applied only
   * when |i| exceeds a threshold so the sign cannot chatter at zero crossing. */
  if (g_foc_dtcomp_td_us > 0.0f) {
    float dv = (g_foc_dtcomp_td_us / 62.5f) * AdcData.VdcBus_V;
    float ith = g_foc_dtcomp_ith_a;
    float sa = 0.0f, sb = 0.0f, sc = 0.0f;
    float ia = AdcData.I_A.value[0];
    float ib = AdcData.I_A.value[1];
    float ic = AdcData.I_A.value[2];
    if (ia >  ith) sa =  1.0f; else if (ia < -ith) sa = -1.0f;
    if (ib >  ith) sb =  1.0f; else if (ib < -ith) sb = -1.0f;
    if (ic >  ith) sc =  1.0f; else if (ic < -ith) sc = -1.0f;
    MOTOR_vars.Vab_out_V.value[0] += dv * (2.0f / 3.0f) * (sa - 0.5f * sb - 0.5f * sc);
    MOTOR_vars.Vab_out_V.value[1] += dv * 0.57735027f * (sb - sc);
  }

  /* ---- Iq ripple tracker (peak-to-peak over ~200 ms = 3200 ISRs) ------- */
  {
    static float iqmin = 1.0e9f, iqmax = -1.0e9f;
    static uint32_t iqn = 0U;
    float iqv = MOTOR_vars.Idq_in_A.value[1];
    if (iqv < iqmin) iqmin = iqv;
    if (iqv > iqmax) iqmax = iqv;
    if (++iqn >= 3200U) {
      g_foc_iq_pp = iqmax - iqmin;
      iqmin = 1.0e9f; iqmax = -1.0e9f; iqn = 0U;
    }
  }

  SVGEN_run(&MOTOR_vars.svgenHandle, &MOTOR_vars.Vab_out_V, &PwmData.Vabc_pu);
  WritePWM(PwmData.Vabc_pu.value);

  /* debug */
  g_foc_theta_rad = angleFOC;
  g_foc_speed_hz = MOTOR_vars.speedFilter_Hz;
  g_foc_id_a = MOTOR_vars.Idq_in_A.value[0];
  g_foc_iq_a = MOTOR_vars.Idq_in_A.value[1];
  g_foc_vd_v = MOTOR_vars.Vdq_out_V.value[0];
  g_foc_vq_v = MOTOR_vars.Vdq_out_V.value[1];
  g_foc_duty[0] = PwmData.Vabc_pu.value[0];
  g_foc_duty[1] = PwmData.Vabc_pu.value[1];
  g_foc_duty[2] = PwmData.Vabc_pu.value[2];

  {
    uint32_t d = DWT->CYCCNT - cyc0;
    g_foc_isr_cycles = d;
    g_foc_isr_cycles_sum += d;
    if (d > g_foc_isr_cycles_max) g_foc_isr_cycles_max = d;
  }
}
