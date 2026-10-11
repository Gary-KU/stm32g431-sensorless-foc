/* foc_serial.h -- host link over USART1 (PB6 = TX, PB7 = RX).
 *
 * PA9/PA10 (the other USART1 pins) are taken by TIM1 PWM, so PB6/PB7 is the
 * only usable USART1 mapping on this board.
 *
 * Protocol (ASCII, 115200 8N1):
 *
 *   host -> MCU  (lines terminated by \n or \r)
 *     RUN <hz>      run at <hz> (negative = reverse); arms and starts
 *     R <hz>        same as RUN
 *     S <hz>        set the speed reference only (no start)
 *     STOP          normal controlled stop
 *     DIS           disarm (drop g_foc_arm)
 *     P <name> <v>  set a tuning parameter (see the table in foc_serial.c)
 *     ?             emit one telemetry frame immediately
 *
 *   MCU -> host  (~50 Hz)
 *     $D,<state>,<fault>,<spref>,<speed>,<id>,<iq>,<vd>,<vq>,<vbus>\r\n
 *
 * A second frame type is sent once on request:
 *     $P,<name>,<value>\r\n   (ack for P)
 */
#ifndef FOC_SERIAL_H
#define FOC_SERIAL_H

#include <stdint.h>

/* Initialise USART1 on PB6/PB7 and start the receive interrupt. */
void FOC_SerialInit(void);

/* Call from the main loop as often as possible: drains received lines and
 * emits telemetry at the configured rate. */
void FOC_SerialTask(void);

/* Raw line writer (used for the boot banner and error messages). */
void FOC_SerialWrite(const char *s);

/* Host-link telemetry period in ms (default 20 = 50 Hz). */
extern volatile uint32_t g_foc_serial_period_ms;

/* Counters for diagnostics. */
extern volatile uint32_t g_foc_serial_rx_lines;
extern volatile uint32_t g_foc_serial_rx_drops;
extern volatile uint32_t g_foc_serial_tx_frames;
extern volatile uint32_t g_foc_serial_cmd_errors;

#endif /* FOC_SERIAL_H */
