#include "RT2TZM_FSP_init.h"
#include "hal_data.h"

fsp_err_t rzt2m_fsp_init(void)
{
  fsp_err_t status = FSP_SUCCESS;

  //OI status = usb_hal_init();
  //OI if (FSP_SUCCESS != status)
  //OI   {
  //OI     usb_hal_deinit();
  //OI   }

  status = elc_hal_init();
  if (FSP_SUCCESS != status)
    {
      elc_hal_deinit();
    }

  status = gpt_timers_hal_init();
  if (FSP_SUCCESS != status)
    {
      elc_hal_deinit();
      gpt_timers_hal_deinit();
    }

  status = mtu3_hal_init();
  if (FSP_SUCCESS != status)
    {
      elc_hal_deinit();
      mtu3_hal_deinit();
    }

  status = poe3_hal_init();
  if (FSP_SUCCESS != status)
    {
      elc_hal_deinit();
      poe3_hal_deinit();
    }

  status = adc_hal_init();
  if (FSP_SUCCESS != status)
    {
      elc_hal_deinit();
      adc_hal_deinit();
    }

  status = dsmif_hal_init();
  if (FSP_SUCCESS != status)
    {
      elc_hal_deinit();
      dsmif_hal_deinit();
    }

  status = uart_hal_init();
  if (FSP_SUCCESS != status)
    {
      elc_hal_deinit();
      uart_hal_deinit();
    }

  status = canfd_hal_init();
  if (FSP_SUCCESS != status)
    {
      elc_hal_deinit();
      canfd_hal_deinit();
    }

  status = spi_hal_init();
  if (FSP_SUCCESS != status)
    {
      elc_hal_deinit();
      canfd_hal_deinit();
    }

  status = max17841_init();
  if (FSP_SUCCESS != status)
    {
      elc_hal_deinit();
      max17841_deinit();
    }

  return status;
}
