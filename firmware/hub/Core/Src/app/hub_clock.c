/**
 * hub_clock.c - Boot on the internal 48 MHz clock, then try the crystal.
 *
 * Why: the generated SystemClock_Config() starts the 8 MHz crystal (HSE)
 * before the UART exists. If the crystal is dead it calls Error_Handler()
 * and the board hangs silently. Here the CPU boots on HSI48 (internal,
 * cannot fail), the console starts, and only then is the crystal tried with
 * a timeout, so a dead crystal is reported as "crystal FAIL - CAN disabled"
 * and UART + I2C keep working. Only FDCAN needs the crystal (D-002, D-016).
 * Decisions: D-021 (option b1), D-023.
 *
 * RISKS (accepted, D-023) - read before changing clocks or regenerating:
 *  1. hub_clock_boot() COPIES the CubeMX clock settings (flash latency 1,
 *     HSI divider 1, SYSCLK source HSI, AHB/APB divider 1). If the clock
 *     tree is changed in CubeMX, update hub_clock_boot() to match by hand;
 *     CubeMX will not do it.
 *  2. The CubeMX ticks "Do Not Generate Function Call" on SystemClock_Config
 *     and MX_FDCAN1_Init (Project Manager > Advanced Settings) are REQUIRED.
 *     After every regenerate, check that main.c does not call them. If it
 *     does, the silent hang on a dead crystal returns, and MX_FDCAN1_Init
 *     would run twice.
 *  3. A crystal that fails AFTER boot is not detected (the Clock Security
 *     System is a later option).
 *  4. A dead crystal adds about 100 ms (HSE_STARTUP_TIMEOUT) to boot.
 */
#include "app/hub_clock.h"
#include "app/hub_config.h"
#include "main.h"

static int s_hse_ok;                     /* result of hub_clock_start_hse() */

void hub_clock_boot(void)
{
  /* Same values as the generated SystemClock_Config(), minus the crystal
   * (risk 1: keep in step with CubeMX by hand). */
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);   /* 1 wait state for 48 MHz */

  osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState            = RCC_HSI_ON;
  osc.HSIDiv              = RCC_HSI_DIV1;      /* HSI48 / 1 = 48 MHz */
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    Error_Handler();                     /* internal oscillator: should never fail */
  }

  clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  clk.SYSCLKDivider  = RCC_SYSCLK_DIV1;
  clk.AHBCLKDivider  = RCC_HCLK_DIV1;
  clk.APB1CLKDivider = RCC_APB1_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

int hub_clock_start_hse(void)
{
#if HUB_TEST_FORCE_HSE_FAIL
  /* Test switch in hub_config.h: act as if the crystal did not start. */
  s_hse_ok = 0;
  return s_hse_ok;
#else
  RCC_OscInitTypeDef osc = {0};

  /* Only the crystal; the running HSI48 is left alone. The HAL waits up to
   * HSE_STARTUP_TIMEOUT (100 ms, stm32c0xx_hal_conf.h) for it to be ready. */
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState       = RCC_HSE_ON;
  if (HAL_RCC_OscConfig(&osc) == HAL_OK)
  {
    s_hse_ok = 1;
  }
  else
  {
    /* Not ready in time: switch it off so a half-working crystal is never
     * used by FDCAN later. */
    __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);
    s_hse_ok = 0;
  }
  return s_hse_ok;
#endif
}

int hub_clock_hse_ok(void)
{
  return s_hse_ok;
}
