/* foc_serial.c -- host link over USART1 (PB6 = TX, PB7 = RX), 115200 8N1.
 * See foc_serial.h for the protocol. */

#include "foc_serial.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include "motor_foc.h"
#include "user_mtr.h"

#include <string.h>

/* ------------------------------------------------------------------ tuning */

volatile uint32_t g_foc_serial_period_ms = 20U;   /* telemetry period (50 Hz) */
volatile uint32_t g_foc_serial_rx_lines  = 0U;
volatile uint32_t g_foc_serial_rx_drops  = 0U;
volatile uint32_t g_foc_serial_tx_frames = 0U;
volatile uint32_t g_foc_serial_cmd_errors= 0U;

/* ------------------------------------------------------------------ state */

static UART_HandleTypeDef huart1;

#define RX_RING_SZ 256U
static volatile uint8_t  rx_ring[RX_RING_SZ];
static volatile uint16_t rx_head = 0U;      /* written by the ISR */
static volatile uint16_t rx_tail = 0U;      /* read by the task    */
static uint8_t  rx_byte;

#define LINE_SZ 72U
static char     line_buf[LINE_SZ];
static uint16_t line_len = 0U;

static char     tx_buf[160];

/* ------------------------------------------------------- small float tools */

/* Minimal float -> string (no libc printf dependency: microlib/%f is a trap on
 * embedded targets and the string library would pull in a lot of code). */
static int fmt_f(char *p, float v, int dec)
{
  static const long pow10[7] = { 1L, 10L, 100L, 1000L, 10000L, 100000L, 1000000L };
  int  n = 0, neg = 0;
  long scaled, ip, fp, d;

  if (dec < 0) dec = 0;
  if (dec > 6) dec = 6;
  if (v != v) { p[0] = 'n'; p[1] = 'a'; p[2] = 'n'; p[3] = '\0'; return 3; }
  if (v < 0.0f) { neg = 1; v = -v; }
  if (v > 200000.0f) v = 200000.0f;

  scaled = (long)(v * (float)pow10[dec] + 0.5f);
  ip = scaled / pow10[dec];
  fp = scaled % pow10[dec];

  if (neg) p[n++] = '-';
  {
    char t[12];
    int  k = 0;
    if (ip == 0L) t[k++] = '0';
    while (ip > 0L) { t[k++] = (char)('0' + (int)(ip % 10L)); ip /= 10L; }
    while (k > 0) p[n++] = t[--k];
  }
  if (dec > 0) {
    p[n++] = '.';
    d = pow10[dec] / 10L;
    while (d >= 1L) { p[n++] = (char)('0' + (int)((fp / d) % 10L)); d /= 10L; }
  }
  p[n] = '\0';
  return n;
}

/* Minimal string -> float. */
static float str_f(const char *s)
{
  int   neg = 0;
  float v = 0.0f, f = 0.1f;
  while (*s == ' ' || *s == '\t') s++;
  if (*s == '-') { neg = 1; s++; } else if (*s == '+') { s++; }
  while (*s >= '0' && *s <= '9') { v = v * 10.0f + (float)(*s - '0'); s++; }
  if (*s == '.') {
    s++;
    while (*s >= '0' && *s <= '9') { v += (float)(*s - '0') * f; f *= 0.1f; s++; }
  }
  return neg ? -v : v;
}

/* ------------------------------------------------------------- parameters */

typedef struct {
  const char *name;
  void       *ptr;
  uint8_t     is_int;
} Param_t;

static const Param_t g_params[] = {
  { "rs",       (void *)&g_foc_rs_ohm,           0U },
  { "ls",       (void *)&g_foc_ls_h,             0U },
  { "pllki",    (void *)&g_foc_pll_ki,           0U },
  { "thetaoff", (void *)&g_foc_thetaoff_sf,      0U },
  { "force",    (void *)&g_foc_speed_force_hz,   0U },
  { "accel",    (void *)&g_foc_accel_start_hzps, 0U },
  { "starti",   (void *)&g_foc_start_current_a,  0U },
  { "spdfb",    (void *)&g_foc_spd_fb_filter,    1U },
  { "hoerr",    (void *)&g_foc_handover_errmax, 0U },
  { "decel",    (void *)&g_foc_stop_decel_hzps,  0U },
  { "octrip",   (void *)&g_foc_oc_trip_a,        0U },
  { "ocinst",   (void *)&g_foc_oc_inst_a,        0U },
  { "stallms",  (void *)&g_foc_stall_ms,         1U },
  { "runms",    (void *)&g_foc_run_ms,           1U },
  { "fw",       (void *)&g_foc_fw_enable,        1U },
  { "fwgain",   (void *)&g_foc_fw_gain,          0U },
  { "fwmax",    (void *)&g_foc_fw_max_a,         0U },
  { "obs",      (void *)&g_foc_obs_mode,         1U },
  { "fluxwc",   (void *)&g_foc_flux_wc,          0U },
  { "fluxkp",   (void *)&g_foc_flux_kp,          0U },
  { "fluxki",   (void *)&g_foc_flux_ki,          0U },
  { "psi",      (void *)&g_foc_flux_psi,         0U },
  { "mtpa",     (void *)&g_foc_mtpa_enable,      1U },
  { "ld",       (void *)&g_foc_ld_h,             0U },
  { "lq",       (void *)&g_foc_lq_h,             0U },
  { "dt",       (void *)&g_foc_dtcomp_td_us,     0U },
  { "dtith",    (void *)&g_foc_dtcomp_ith_a,     0U },
  { "period",   (void *)&g_foc_serial_period_ms, 1U },
};
#define N_PARAMS (sizeof(g_params) / sizeof(g_params[0]))

/* ------------------------------------------------------------------- TX/RX */

void FOC_SerialWrite(const char *s)
{
  uint16_t n = (uint16_t)strlen(s);
  if (n == 0U) return;
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)s, n, 20U);
}

static void send_telemetry(void)
{
  char *p = tx_buf;
  int   n = 0;

  p[n++] = '$'; p[n++] = 'D';
  p[n++] = ',';
  n += fmt_f(p + n, (float)g_foc_state, 0);        p[n++] = ',';
  n += fmt_f(p + n, (float)g_foc_fault, 0);        p[n++] = ',';
  n += fmt_f(p + n, g_foc_speed_ref_hz, 2);        p[n++] = ',';
  n += fmt_f(p + n, g_foc_speed_hz,     2);        p[n++] = ',';
  n += fmt_f(p + n, g_foc_id_a,         3);        p[n++] = ',';
  n += fmt_f(p + n, g_foc_iq_a,         3);        p[n++] = ',';
  n += fmt_f(p + n, g_foc_vd_v,         2);        p[n++] = ',';
  n += fmt_f(p + n, g_foc_vq_v,         2);        p[n++] = ',';
  n += fmt_f(p + n, g_foc_vbus_v,       2);
  p[n++] = '\r'; p[n++] = '\n'; p[n] = '\0';

  if (HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)n, 20U) == HAL_OK) {
    g_foc_serial_tx_frames++;
  }
}

/* --------------------------------------------------------------- commands */

static const Param_t *find_param(const char *name)
{
  uint32_t i;
  for (i = 0U; i < N_PARAMS; i++) {
    if (strcmp(name, g_params[i].name) == 0) return &g_params[i];
  }
  return (const Param_t *)0;
}

static void ack_param(const char *name, float v)
{
  char *p = tx_buf;
  int   n = 0;
  int   dec = 3;
  float av = (v < 0.0f) ? -v : v;
  /* small tuning values (PLL gains, time constants) need more digits or the
   * ack rounds them away (0.0005 used to echo as 0.001). */
  if (av != 0.0f && av < 0.01f)       dec = 6;
  else if (av != 0.0f && av < 0.1f)   dec = 5;
  else if (av != 0.0f && av < 1.0f)   dec = 4;
  p[n++] = '$'; p[n++] = 'P'; p[n++] = ',';
  while (*name) p[n++] = *name++;
  p[n++] = ',';
  n += fmt_f(p + n, v, dec);
  p[n++] = '\r'; p[n++] = '\n'; p[n] = '\0';
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)n, 20U);
}

/* Split a line into at most 3 whitespace separated tokens, in place. */
static int tokenize(char *s, char *tok[], int max)
{
  int n = 0;
  while (*s && n < max) {
    while (*s == ' ' || *s == '\t') s++;
    if (*s == '\0') break;
    tok[n++] = s;
    while (*s && *s != ' ' && *s != '\t') s++;
    if (*s) *s++ = '\0';
  }
  return n;
}

static void handle_line(char *s)
{
  char *tok[4];
  int   ntok;

  g_foc_serial_rx_lines++;
  ntok = tokenize(s, tok, 4);
  if (ntok == 0) return;

  /* RUN <hz> / R <hz> ------------------------------------------------ */
  if ((strcmp(tok[0], "RUN") == 0) || (strcmp(tok[0], "R") == 0)) {
    if (ntok < 2) { g_foc_serial_cmd_errors++; return; }
    {
      float hz = str_f(tok[1]);
      if (hz >  200.0f) hz =  200.0f;
      if (hz < -200.0f) hz = -200.0f;
      g_foc_speed_ref_hz = hz;
      g_foc_command = (hz >= 0.0f) ? 1 : -1;
      g_foc_arm = 1U;
    }
    return;
  }

  /* S <hz> : reference only ------------------------------------------ */
  if (strcmp(tok[0], "S") == 0) {
    if (ntok < 2) { g_foc_serial_cmd_errors++; return; }
    {
      float hz = str_f(tok[1]);
      if (hz >  200.0f) hz =  200.0f;
      if (hz < -200.0f) hz = -200.0f;
      g_foc_speed_ref_hz = hz;
    }
    return;
  }

  /* STOP / DIS ------------------------------------------------------- */
  if (strcmp(tok[0], "STOP") == 0) { g_foc_command = 2; g_foc_arm = 0U; return; }
  if (strcmp(tok[0], "DIS")  == 0) { g_foc_arm = 0U; return; }
  if (strcmp(tok[0], "?")    == 0) { send_telemetry(); return; }

  /* P <name> <v> : set parameter ------------------------------------- */
  if (strcmp(tok[0], "P") == 0) {
    const Param_t *pp;
    if (ntok < 3) { g_foc_serial_cmd_errors++; return; }
    pp = find_param(tok[1]);
    if (pp == (const Param_t *)0) { g_foc_serial_cmd_errors++; return; }
    {
      float v = str_f(tok[2]);
      if (pp->is_int) {
        int32_t iv = (int32_t)(v + (v >= 0.0f ? 0.5f : -0.5f));
        *(volatile int32_t *)pp->ptr = iv;
        ack_param(tok[1], (float)iv);
      } else {
        *(volatile float *)pp->ptr = v;
        ack_param(tok[1], v);
      }
    }
    return;
  }

  /* anything else: also allow "B<name> <v>"-free raw assignment attempt */
  g_foc_serial_cmd_errors++;
}

/* --------------------------------------------------------------- init/IRQ */

void FOC_SerialInit(void)
{
  GPIO_InitTypeDef gpio;

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  /* PB6 = USART1_TX, PB7 = USART1_RX (AF7) */
  gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_PULLUP;
  gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &gpio);

  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200U;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  (void)HAL_UART_Init(&huart1);

  /* USART1 must be *lower* priority than the 16 kHz FOC ISR (TIM1 = 1). */
  HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    uint16_t next = (uint16_t)((rx_head + 1U) % RX_RING_SZ);
    if (next == rx_tail) {
      g_foc_serial_rx_drops++;          /* ring full: drop the byte */
    } else {
      rx_ring[rx_head] = rx_byte;
      rx_head = next;
    }
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
  }
}

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

/* The HAL references these optional FIFO/wakeup callbacks unconditionally; with
 * no weak definitions in this HAL revision they must be provided here. */
void HAL_UARTEx_RxFifoFullCallback(UART_HandleTypeDef *huart) { (void)huart; }
void HAL_UARTEx_TxFifoEmptyCallback(UART_HandleTypeDef *huart) { (void)huart; }
void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart) { (void)huart; }

/* ------------------------------------------------------------------- task */

void FOC_SerialTask(void)
{
  static uint32_t last_tx = 0U;
  uint32_t now = HAL_GetTick();

  /* drain the ring into lines */
  while (rx_tail != rx_head) {
    char c = (char)rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % RX_RING_SZ);

    if (c == '\n' || c == '\r') {
      if (line_len > 0U) {
        line_buf[line_len] = '\0';
        handle_line(line_buf);
        line_len = 0U;
      }
    } else if (line_len < (LINE_SZ - 1U)) {
      line_buf[line_len++] = c;
    } else {
      line_len = 0U;                    /* overlong line: drop it */
      g_foc_serial_cmd_errors++;
    }
  }

  /* periodic telemetry */
  if ((uint32_t)(now - last_tx) >= g_foc_serial_period_ms) {
    last_tx = now;
    send_telemetry();
  }
}
