#include "blinky_thread.h"

extern bsp_leds_t g_bsp_leds;

void blinky_thread_entry(void* pvParameters)
{
  FSP_PARAMETER_NOT_USED(pvParameters);

  bsp_leds_t leds = g_bsp_leds;

  if (0 == leds.led_count)
    {
      while (1)
        {
          ;  // There are no LEDs on this board
        }
    }

  for (uint32_t i = 0; i < leds.led_count; i++)
    {
      R_BSP_PinClear(BSP_IO_REGION_SAFE, (bsp_io_port_pin_t)leds.p_leds[i]);
    }

  while (1)
    {
      for (uint32_t i = 0; i < leds.led_count; i++)
        {
          R_BSP_PinToggle(BSP_IO_REGION_SAFE, (bsp_io_port_pin_t)leds.p_leds[i]);
        }

      vTaskDelay(configTICK_RATE_HZ);
    }
}
