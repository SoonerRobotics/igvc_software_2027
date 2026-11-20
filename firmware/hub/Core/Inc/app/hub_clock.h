/**
 * hub_clock.h - Boot the CPU on the internal 48 MHz clock, then try the
 * 8 MHz crystal (HSE) that FDCAN needs. D-021 (b1), D-023.
 * See hub_clock.c for the accepted risks.
 */
#ifndef HUB_CLOCK_H
#define HUB_CLOCK_H

/* Set the CPU to HSI48 = 48 MHz WITHOUT touching the crystal.
 * Call in main.c USER CODE SysInit (before the UART/I2C init), in place of
 * the generated SystemClock_Config(). */
void hub_clock_boot(void);

/* Try to start the 8 MHz crystal, waiting at most HSE_STARTUP_TIMEOUT
 * (100 ms). Returns 1 if it runs, 0 if not (then it is switched off again).
 * Call once, after console_init(), so the result can be printed. */
int hub_clock_start_hse(void);

/* Result of the last hub_clock_start_hse(): 1 = crystal running.
 * CAN code checks this before using FDCAN. */
int hub_clock_hse_ok(void);

#endif /* HUB_CLOCK_H */
